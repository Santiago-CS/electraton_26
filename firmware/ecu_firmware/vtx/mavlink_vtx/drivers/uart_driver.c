/*
 * uart_driver.c - LPUART transport with optional eDMA, i.MX93 Cortex-M33.
 * See uart_driver.h for the API and the DMA/port pairing rules.
 *
 * Author: Diego
 * Created on: 6, April 2026
 */

#include <string.h>

#include "uart_driver.h"

/*******************************************************************************
 * Instance tables
 ******************************************************************************/

/* Index 0 is LPUART1: the i.MX93 numbers its ports from 1. */
#define UART_INSTANCE_COUNT 8U

/* Numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY (2 here). */
#define UART_DMA_IRQ_PRIORITY 5U

/* eDMA channel vectors: DMA3 has one per channel from DMA3_0_IRQn, DMA4 shares
 * one per channel pair from DMA4_0_1_IRQn. Same mapping as spi_driver.c. */
#define UART_DMA3_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA3_0_IRQn + (ch)))
#define UART_DMA4_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA4_0_1_IRQn + ((ch) / 2U)))

typedef struct {
    LPUART_Type    *base;
    IRQn_Type       irq;
    clock_root_t    clock_root;
    clock_ip_name_t clock_gate;
    bool            in_wakeupmix; /*!< true -> reachable from DMA4, false -> DMA3 */
} uart_hw_t;

static const uart_hw_t s_uart_hw[UART_INSTANCE_COUNT] = {
    {LPUART1, LPUART1_IRQn, kCLOCK_Root_Lpuart1, kCLOCK_Lpuart1, false},
    {LPUART2, LPUART2_IRQn, kCLOCK_Root_Lpuart2, kCLOCK_Lpuart2, false},
    {LPUART3, LPUART3_IRQn, kCLOCK_Root_Lpuart3, kCLOCK_Lpuart3, true},
    {LPUART4, LPUART4_IRQn, kCLOCK_Root_Lpuart4, kCLOCK_Lpuart4, true},
    {LPUART5, LPUART5_IRQn, kCLOCK_Root_Lpuart5, kCLOCK_Lpuart5, true},
    {LPUART6, LPUART6_IRQn, kCLOCK_Root_Lpuart6, kCLOCK_Lpuart6, true},
    {LPUART7, LPUART7_IRQn, kCLOCK_Root_Lpuart7, kCLOCK_Lpuart7, true},
    {LPUART8, LPUART8_IRQn, kCLOCK_Root_Lpuart8, kCLOCK_Lpuart8, true},
};

/* SDK handle storage, one slot per port. */
static lpuart_handle_t      g_lpuartHandles[UART_INSTANCE_COUNT];
static lpuart_edma_handle_t g_lpuartEdmaHandles[UART_INSTANCE_COUNT];
static edma_handle_t        g_lpuartTxEdmaHandles[UART_INSTANCE_COUNT];
static edma_handle_t        g_lpuartRxEdmaHandles[UART_INSTANCE_COUNT];

/* ISR hooks installed by uart_attach_interrupt(). */
static uart_irq_callback_t volatile s_uart_hook[UART_INSTANCE_COUNT];

/* EDMA_Init() resets the whole module, so do it once per eDMA instance. */
static bool s_edma_started[2]; /* [0] = DMA3, [1] = DMA4 */

/*******************************************************************************
 * Helpers
 ******************************************************************************/

static status_t uart_get_instance(const uart_ctrl_t *ctrl, uint32_t *instance)
{
    if ((ctrl == NULL) || (instance == NULL)) {
        return kStatus_InvalidArgument;
    }

    for (uint32_t i = 0U; i < UART_INSTANCE_COUNT; i++) {
        if (s_uart_hw[i].base == ctrl->uart_base) {
            *instance = i;
            return kStatus_Success;
        }
    }

    return kStatus_InvalidArgument;
}

/*
 * The M33's code TCM, where .text and .rodata live on this linker script.
 *
 * The eDMA addresses the M33's *data* TCM at its plain local address - proven
 * on this board with a memory-to-memory copy on both controllers, which is why
 * nothing here translates addresses any more. Whether it can also reach the
 * code TCM aperture was not part of that test, so a .rodata buffer is still
 * refused rather than handed over on an assumption. Copy it into RAM first.
 */
#define UART_CODE_TCM_BEGIN 0x0FFE0000U
#define UART_CODE_TCM_END   0x0FFFFFFFU

/* Checks the buffer is one the eDMA can address. No translation: see above. */
static status_t uart_dma_addr(const void *local, uint8_t **dma_addr)
{
    uint32_t addr;

    if (local == NULL) {
        *dma_addr = NULL;
        return kStatus_Success;
    }

    addr = (uint32_t)(uintptr_t)local;
    if ((addr >= UART_CODE_TCM_BEGIN) && (addr <= UART_CODE_TCM_END)) {
        return kStatus_InvalidArgument;
    }

    *dma_addr = (uint8_t *)(uintptr_t)addr;
    return kStatus_Success;
}

/*
 * Ungate the eDMA. The root is deliberately left alone: kCLOCK_Root_M33 and
 * kCLOCK_Root_WakeupAxi are shared infrastructure that the A55 side is also
 * running on, and both are necessarily already up - we could not be touching
 * the LPUART registers otherwise.
 */
static void uart_enable_dma_clock(const uart_ctrl_t *ctrl)
{
    if (ctrl->dma_base == (EDMA_Type *)DMA3) {
        CLOCK_EnableClock(kCLOCK_Edma1);
    } else {
        CLOCK_EnableClock(kCLOCK_Edma2);
    }
}

/*******************************************************************************
 * Driver Functions
 ******************************************************************************/

void uart_get_default_vtx_config(uart_ctrl_t *ctrl, lpuart_edma_transfer_callback_t callback)
{
    ctrl->uart_base      = VTX_LPUART_BASEADDR;
    ctrl->dma_base       = (EDMA_Type *)VTX_LPUART_DMA_BASEADDR;
    ctrl->tx_dma_channel = VTX_LPUART_TX_DMA_CHANNEL;
    ctrl->rx_dma_channel = VTX_LPUART_RX_DMA_CHANNEL;
    ctrl->tx_dma_request = (uint32_t)VTX_LPUART_TX_DMA_REQUEST;
    ctrl->rx_dma_request = (uint32_t)VTX_LPUART_RX_DMA_REQUEST;
    ctrl->callback       = callback;
    ctrl->baudrate       = VTX_UART_BAUDRATE;
    ctrl->enable_dma     = true;
}

void uart_get_default_gps_config(uart_ctrl_t *ctrl, lpuart_edma_transfer_callback_t callback)
{
    ctrl->uart_base      = GPS_LPUART_BASEADDR;
    ctrl->dma_base       = (EDMA_Type *)GPS_LPUART_DMA_BASEADDR;
    ctrl->tx_dma_channel = GPS_LPUART_TX_DMA_CHANNEL;
    ctrl->rx_dma_channel = GPS_LPUART_RX_DMA_CHANNEL;
    ctrl->tx_dma_request = (uint32_t)GPS_LPUART_TX_DMA_REQUEST;
    ctrl->rx_dma_request = (uint32_t)GPS_LPUART_RX_DMA_REQUEST;
    ctrl->callback       = callback;
    ctrl->baudrate       = GPS_UART_BAUDRATE;
    ctrl->enable_dma     = true;
}

status_t uart_init(uart_ctrl_t *ctrl)
{
    lpuart_config_t config;
    edma_config_t   dmaConfig = {0};
    uint32_t        instance;
    status_t        status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (ctrl->enable_dma) {
        /* An AONMIX eDMA has no request line to a WAKEUPMIX port, or the
         * reverse - a mismatch here would silently never trigger. */
        bool dma_is_dma4 = (ctrl->dma_base == (EDMA_Type *)DMA4);
        if (dma_is_dma4 != s_uart_hw[instance].in_wakeupmix) {
            return kStatus_InvalidArgument;
        }
    }

    /*
     * BOARD_BootClockRUN() on this board only brings up the PLLs, so the port's
     * own root and gate are ours to set. mux 0 is Osc24M, div 1 -> 24 MHz,
     * which gives 0.16 % baud error at 115200.
     */
    const clock_root_config_t uartClkCfg = {
        .clockOff = false,
        .mux      = 0,
        .div      = 1,
    };
    CLOCK_SetRootClock(s_uart_hw[instance].clock_root, &uartClkCfg);
    CLOCK_EnableClock(s_uart_hw[instance].clock_gate);

    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = ctrl->baudrate;
    config.enableTx     = true;
    config.enableRx     = true;

    status = LPUART_Init(ctrl->uart_base, &config, CLOCK_GetIpFreq(s_uart_hw[instance].clock_root));
    if (status != kStatus_Success) {
        return status;
    }

    if (ctrl->enable_dma) {
        uint32_t edma_index = (ctrl->dma_base == (EDMA_Type *)DMA3) ? 0U : 1U;

        uart_enable_dma_clock(ctrl);

        if (!s_edma_started[edma_index]) {
            EDMA_GetDefaultConfig(&dmaConfig);
            EDMA_Init(ctrl->dma_base, &dmaConfig);
            s_edma_started[edma_index] = true;
        }

        /*
         * Scrub both channels BEFORE either handle exists.
         *
         * A warm restart - remoteproc stop then start - resets the M33 but NOT
         * DMA4, so channels this image armed on its previous run still have
         * their DONE/INT flags set. EDMA_Init() does not help: on this SoC its
         * eDMA4 path only writes MP_CSR and never touches per-channel state.
         *
         * The first EDMA_CreateHandle() enables the NVIC line for the whole
         * channel *pair*, and the SDK's dispatcher does
         *
         *     if (channel CH_INT set) EDMA_HandleIRQ(s_EDMAHandle[inst][ch]);
         *
         * with no NULL check. So a stale flag on the other channel of the pair
         * fires an interrupt in the window before its handle is registered and
         * trips "ASSERT ERROR handle != NULL" in EDMA_HandleIRQ. Clearing both
         * channels first closes that window.
         *
         * Deliberately per-channel and not a module-wide reset: DMA4 lives in
         * WAKEUPMIX and is shared with the A55 side, so clearing all 64
         * channels would stamp on whatever Linux has running.
         */
        EDMA_DisableChannelRequest(ctrl->dma_base, ctrl->tx_dma_channel);
        EDMA_ClearChannelStatusFlags(ctrl->dma_base, ctrl->tx_dma_channel,
                                     (uint32_t)kEDMA_DoneFlag | (uint32_t)kEDMA_ErrorFlag |
                                         (uint32_t)kEDMA_InterruptFlag);
        EDMA_DisableChannelRequest(ctrl->dma_base, ctrl->rx_dma_channel);
        EDMA_ClearChannelStatusFlags(ctrl->dma_base, ctrl->rx_dma_channel,
                                     (uint32_t)kEDMA_DoneFlag | (uint32_t)kEDMA_ErrorFlag |
                                         (uint32_t)kEDMA_InterruptFlag);

        /* EDMA_CreateHandle() enables the channel's NVIC interrupt itself. */
        (void)EDMA_CreateHandle(&g_lpuartTxEdmaHandles[instance], ctrl->dma_base, ctrl->tx_dma_channel);
        (void)EDMA_CreateHandle(&g_lpuartRxEdmaHandles[instance], ctrl->dma_base, ctrl->rx_dma_channel);

        /*
         * ...and enables it at reset priority 0, which is numerically BELOW
         * configMAX_SYSCALL_INTERRUPT_PRIORITY (2). The transfer-complete
         * callback is dispatched from this channel ISR, so a callback that
         * calls any ...FromISR() API from priority 0 trips FreeRTOS's
         * vPortValidateInterruptPriority() - and configASSERT here is
         * taskDISABLE_INTERRUPTS() plus an empty for(;;), which freezes the
         * whole system without printing a thing.
         *
         * Missing this is exactly what hung the GPS task the moment its first
         * eDMA chunk completed. spi_driver.c and i2c_driver.c have always set
         * both; this driver only set the peripheral line below, which was
         * harmless for as long as its callbacks only touched a plain flag.
         */
        if (ctrl->dma_base == (EDMA_Type *)DMA3) {
            NVIC_SetPriority(UART_DMA3_CHANNEL_IRQ(ctrl->tx_dma_channel), UART_DMA_IRQ_PRIORITY);
            NVIC_SetPriority(UART_DMA3_CHANNEL_IRQ(ctrl->rx_dma_channel), UART_DMA_IRQ_PRIORITY);
        } else {
            NVIC_SetPriority(UART_DMA4_CHANNEL_IRQ(ctrl->tx_dma_channel), UART_DMA_IRQ_PRIORITY);
            NVIC_SetPriority(UART_DMA4_CHANNEL_IRQ(ctrl->rx_dma_channel), UART_DMA_IRQ_PRIORITY);
        }

        /* On this part the channel index is free and the request source is
         * selected through the channel mux - they are not the same number. */
        EDMA_SetChannelMux(ctrl->dma_base, ctrl->tx_dma_channel, ctrl->tx_dma_request);
        EDMA_SetChannelMux(ctrl->dma_base, ctrl->rx_dma_channel, ctrl->rx_dma_request);

        /*
         * Both directions go through the one eDMA handle. Creating a
         * transactional handle as well would overwrite s_lpuartIsr[] for this
         * port and break whichever was registered first, so it is one or the
         * other - never both.
         */
        LPUART_TransferCreateHandleEDMA(ctrl->uart_base,
                                        &g_lpuartEdmaHandles[instance],
                                        ctrl->callback,
                                        NULL,
                                        &g_lpuartTxEdmaHandles[instance],
                                        &g_lpuartRxEdmaHandles[instance]);

        /* CreateHandleEDMA enables the LPUART's own interrupt at reset
         * priority 0, and that ISR dispatches ctrl->callback. Harmless while
         * the callback stays clear of FreeRTOS, but one ...FromISR() call from
         * priority 0 trips configASSERT and freezes the system, so pin it. */
        NVIC_SetPriority(s_uart_hw[instance].irq, UART_DMA_IRQ_PRIORITY);
    } else {
        LPUART_TransferCreateHandle(ctrl->uart_base,
                                    &g_lpuartHandles[instance],
                                    (lpuart_transfer_callback_t)ctrl->callback,
                                    NULL);
    }

    return kStatus_Success;
}

status_t uart_write_bytes(uart_ctrl_t *ctrl, const uint8_t *data, uint32_t size)
{
    lpuart_transfer_t xfer;
    uint32_t          instance;
    status_t          status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if ((data == NULL) || (size == 0U)) {
        return kStatus_InvalidArgument;
    }

    xfer.dataSize = (size_t)size;

    if (ctrl->enable_dma) {
        /* Rejects a string literal, which lives in code TCM where the eDMA
         * cannot reach it - copy it into a RAM buffer first. */
        status = uart_dma_addr(data, &xfer.data);
        if (status != kStatus_Success) {
            return status;
        }
        return LPUART_SendEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance], &xfer);
    }

    xfer.data = (uint8_t *)(uintptr_t)data;
    return LPUART_TransferSendNonBlocking(ctrl->uart_base, &g_lpuartHandles[instance], &xfer);
}

status_t uart_write(uart_ctrl_t *ctrl, const char *string)
{
    if ((string == NULL) || (*string == '\0')) {
        return kStatus_InvalidArgument;
    }

    return uart_write_bytes(ctrl, (const uint8_t *)string, (uint32_t)strlen(string));
}

status_t uart_rx_dma_count(uart_ctrl_t *ctrl, uint32_t *count)
{
    uint32_t instance;
    status_t status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if ((count == NULL) || !ctrl->enable_dma) {
        return kStatus_InvalidArgument;
    }

    return LPUART_TransferGetReceiveCountEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance], count);
}

status_t uart_read_dma(uart_ctrl_t *ctrl, uint8_t *data, uint32_t size)
{
    lpuart_transfer_t receiveXfer;
    uint32_t          instance;
    status_t          status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if ((!ctrl->enable_dma) || (data == NULL) || (size == 0U)) {
        return kStatus_InvalidArgument;
    }

    status = uart_dma_addr(data, &receiveXfer.data);
    if (status != kStatus_Success) {
        return status;
    }
    receiveXfer.dataSize = (size_t)size;

    /* A receiver error holds the data register stale until the flag is cleared,
     * which would stall the DMA request on the first byte. */
    LPUART_ClearStatusFlags(ctrl->uart_base,
                            (uint32_t)(kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                       kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag));

    return LPUART_ReceiveEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance], &receiveXfer);
}

status_t uart_abort_rx_dma(uart_ctrl_t *ctrl)
{
    uint32_t instance;
    status_t status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (!ctrl->enable_dma) {
        return kStatus_InvalidArgument;
    }

    LPUART_TransferAbortReceiveEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance]);
    return kStatus_Success;
}

status_t uart_set_baudrate(uart_ctrl_t *ctrl, uint32_t baudrate)
{
    uint32_t instance;
    status_t status;

    status = uart_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }
    if (baudrate == 0U) {
        return kStatus_InvalidArgument;
    }

    if (ctrl->enable_dma) {
        LPUART_TransferAbortReceiveEDMA(ctrl->uart_base, &g_lpuartEdmaHandles[instance]);
    }

    status = LPUART_SetBaudRate(ctrl->uart_base, baudrate,
                                CLOCK_GetIpFreq(s_uart_hw[instance].clock_root));
    if (status != kStatus_Success) {
        return status;
    }

    /* Framing/noise errors latched at the previous rate would otherwise be
     * reported against the new one. */
    LPUART_ClearStatusFlags(ctrl->uart_base,
                            (uint32_t)(kLPUART_RxOverrunFlag | kLPUART_NoiseErrorFlag |
                                       kLPUART_FramingErrorFlag | kLPUART_ParityErrorFlag));

    ctrl->baudrate = baudrate;
    return kStatus_Success;
}

status_t uart_attach_interrupt(LPUART_Type *uart_base, uart_irq_callback_t callback)
{
    uart_ctrl_t probe = {.uart_base = uart_base};
    uint32_t    instance;
    status_t    status;

    status = uart_get_instance(&probe, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    s_uart_hook[instance] = callback;
    __DMB();

    EnableIRQ(s_uart_hw[instance].irq);
    return kStatus_Success;
}

/*******************************************************************************
 * Interrupt Service Routines
 *
 * The startup file's weak LPUARTn_IRQHandler tail-calls LPUARTn_DriverIRQHandler,
 * which the SDK LPUART driver provides and which dispatches to whichever handle
 * uart_init() registered. Overriding the vector here lets the user hook run
 * first; the SDK dispatcher still has to be called afterwards or transfers
 * never complete.
 ******************************************************************************/

static void uart_dispatch(uint32_t instance)
{
    uart_irq_callback_t hook = s_uart_hook[instance];

    if (hook != NULL) {
        hook();
    }
}

#define UART_DEFINE_IRQ(n)                        \
    void LPUART##n##_DriverIRQHandler(void);      \
    void LPUART##n##_IRQHandler(void);            \
    void LPUART##n##_IRQHandler(void)             \
    {                                             \
        uart_dispatch((n) - 1U);                  \
        LPUART##n##_DriverIRQHandler();           \
        SDK_ISR_EXIT_BARRIER;                     \
    }

UART_DEFINE_IRQ(1)
UART_DEFINE_IRQ(2)
UART_DEFINE_IRQ(3)
UART_DEFINE_IRQ(4)
UART_DEFINE_IRQ(5)
UART_DEFINE_IRQ(6)
UART_DEFINE_IRQ(7)
UART_DEFINE_IRQ(8)
