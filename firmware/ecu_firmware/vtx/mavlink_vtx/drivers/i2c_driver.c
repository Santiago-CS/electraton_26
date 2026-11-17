/*
 * i2c_driver.c - LPI2C master with optional eDMA, i.MX93 Cortex-M33.
 * See i2c_driver.h for the API and the DMA caveat.
 *
 * Author: Diego
 */

#include <string.h>

#include "i2c_driver.h"

/*******************************************************************************
 * Instance tables
 ******************************************************************************/

/* Index 0 is LPI2C1: the i.MX93 numbers its ports from 1, and the SDK's own
 * instance lookup returns 1..8 from a 9-entry table. */
#define I2C_INSTANCE_COUNT 8U

/* Default DMA completion wait when ctrl->timeout_ms is 0. */
#define I2C_DEFAULT_TIMEOUT_MS 100U

/* NVIC priority for the eDMA channel interrupts. The completion callback gives
 * a semaphore, so this must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY is 2 here). EDMA_CreateHandle()
 * calls EnableIRQ() but never sets a priority, which would leave these at 0 -
 * above the ceiling, where xSemaphoreGiveFromISR() is not allowed. */
#define I2C_DMA_IRQ_PRIORITY 5U

/* eDMA channel vectors are contiguous: DMA3 has one per channel from
 * DMA3_0_IRQn, DMA4 shares one per channel pair from DMA4_0_1_IRQn. */
#define I2C_DMA3_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA3_0_IRQn + (ch)))
#define I2C_DMA4_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA4_0_1_IRQn + ((ch) / 2U)))

typedef struct {
    LPI2C_Type     *base;
    IRQn_Type       irq;          /*!< the peripheral's own line, not the eDMA channel's */
    clock_root_t    clock_root;
    clock_ip_name_t clock_gate;
    bool            in_wakeupmix; /*!< true -> driven by DMA4, false -> DMA3 */
} i2c_hw_t;

static const i2c_hw_t s_i2c_hw[I2C_INSTANCE_COUNT] = {
    {LPI2C1, LPI2C1_IRQn, kCLOCK_Root_Lpi2c1, kCLOCK_Lpi2c1, false},
    {LPI2C2, LPI2C2_IRQn, kCLOCK_Root_Lpi2c2, kCLOCK_Lpi2c2, false},
    {LPI2C3, LPI2C3_IRQn, kCLOCK_Root_Lpi2c3, kCLOCK_Lpi2c3, true},
    {LPI2C4, LPI2C4_IRQn, kCLOCK_Root_Lpi2c4, kCLOCK_Lpi2c4, true},
    {LPI2C5, LPI2C5_IRQn, kCLOCK_Root_Lpi2c5, kCLOCK_Lpi2c5, true},
    {LPI2C6, LPI2C6_IRQn, kCLOCK_Root_Lpi2c6, kCLOCK_Lpi2c6, true},
    {LPI2C7, LPI2C7_IRQn, kCLOCK_Root_Lpi2c7, kCLOCK_Lpi2c7, true},
    {LPI2C8, LPI2C8_IRQn, kCLOCK_Root_Lpi2c8, kCLOCK_Lpi2c8, true},
};

static edma_handle_t              g_lpi2cRxEdmaHandles[I2C_INSTANCE_COUNT];
static edma_handle_t              g_lpi2cTxEdmaHandles[I2C_INSTANCE_COUNT];
static lpi2c_master_edma_handle_t g_i2c_edma_handles[I2C_INSTANCE_COUNT];

/*
 * Result of the last eDMA transfer, written by the completion callback and
 * read back by i2c_master_transfer(). Without this the driver cannot tell a
 * NAK from a completed transfer: the callback is invoked either way.
 */
static volatile status_t s_dma_status[I2C_INSTANCE_COUNT];

/* EDMA_Init() resets a whole controller, so do it once per eDMA instance -
 * a single global flag would skip the second controller entirely. */
static bool s_edma_started[2]; /* [0] = DMA3, [1] = DMA4 */

/*******************************************************************************
 * Helpers
 ******************************************************************************/

static status_t i2c_get_instance(const i2c_ctrl_t *ctrl, uint32_t *instance)
{
    if ((ctrl == NULL) || (instance == NULL)) {
        return kStatus_InvalidArgument;
    }

    for (uint32_t i = 0U; i < I2C_INSTANCE_COUNT; i++) {
        if (s_i2c_hw[i].base == ctrl->i2c_base) {
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
#define I2C_CODE_TCM_BEGIN 0x0FFE0000U
#define I2C_CODE_TCM_END   0x0FFFFFFFU

/* Checks the buffer is one the eDMA can address. No translation: see above. */
static status_t i2c_dma_addr(const void *local, void **dma_addr)
{
    uint32_t addr;

    if (local == NULL) {
        *dma_addr = NULL;
        return kStatus_Success;
    }

    addr = (uint32_t)(uintptr_t)local;
    if ((addr >= I2C_CODE_TCM_BEGIN) && (addr <= I2C_CODE_TCM_END)) {
        return kStatus_InvalidArgument;
    }

    *dma_addr = (void *)(uintptr_t)addr;
    return kStatus_Success;
}

/*
 * Internal DMA completion callback. Gives the per-instance binary semaphore so
 * i2c_master_transfer() can unblock the calling FreeRTOS task.
 *
 * userData is the SemaphoreHandle_t stored in i2c_ctrl_t.dma_semaphore and
 * installed as the userData argument in LPI2C_MasterCreateEDMAHandle.
 */
static void i2c_dma_complete_callback(LPI2C_Type *base,
                                      lpi2c_master_edma_handle_t *handle,
                                      status_t completionStatus,
                                      void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)handle;

    /* The callback fires on failure as well as success, so the status has to
     * be carried out to the caller - a discarded NAK looks exactly like a
     * completed read of a buffer the DMA never wrote. */
    for (uint32_t i = 0U; i < I2C_INSTANCE_COUNT; i++) {
        if (s_i2c_hw[i].base == base) {
            s_dma_status[i] = completionStatus;
            break;
        }
    }

    xSemaphoreGiveFromISR((SemaphoreHandle_t)userData, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 * Default configuration
 ******************************************************************************/

void i2c_get_default_config(i2c_ctrl_t *ctrl)
{
    ctrl->i2c_base        = I2C_MASTER_BASEADDR;
    ctrl->dma_base        = (EDMA_Type *)I2C_MASTER_DMA_BASE;
    ctrl->dma_rx_channel  = I2C_MASTER_DMA_RX_CHANNEL;
    ctrl->dma_tx_channel  = I2C_MASTER_DMA_TX_CHANNEL;
    ctrl->edma_rx_channel = (uint32_t)I2C_RECEIVE_EDMA_CHANNEL;
    ctrl->edma_tx_channel = (uint32_t)I2C_TRANSMIT_EDMA_CHANNEL;
    ctrl->baudrate_bps    = I2C_MASTER_BAUDRATE;
    /* For short register pokes the blocking path is usually faster end to end -
     * set this false if that is all you do. */
    ctrl->enable_dma      = true;
    ctrl->dma_semaphore   = NULL;
    ctrl->timeout_ms      = I2C_DEFAULT_TIMEOUT_MS;
}

void i2c_get_default_baro_config(i2c_ctrl_t *ctrl)
{
    ctrl->i2c_base        = BARO_I2C_BASEADDR;
    ctrl->dma_base        = (EDMA_Type *)BARO_I2C_DMA_BASE;
    ctrl->dma_rx_channel  = BARO_I2C_DMA_RX_CHANNEL;
    ctrl->dma_tx_channel  = BARO_I2C_DMA_TX_CHANNEL;
    ctrl->edma_rx_channel = (uint32_t)BARO_I2C_RX_DMA_REQUEST;
    ctrl->edma_tx_channel = (uint32_t)BARO_I2C_TX_DMA_REQUEST;
    ctrl->baudrate_bps    = BARO_I2C_BAUDRATE;
    ctrl->enable_dma      = true;
    ctrl->dma_semaphore   = NULL;
    ctrl->timeout_ms      = I2C_DEFAULT_TIMEOUT_MS;
}

/*******************************************************************************
 * Driver Functions
 ******************************************************************************/

status_t i2c_init(i2c_ctrl_t *ctrl)
{
    lpi2c_master_config_t masterConfig;
    edma_config_t         userConfig = {0};
    uint32_t              instance;
    status_t              status;

    status = i2c_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (ctrl->enable_dma) {
        /* An AONMIX eDMA has no request line to a WAKEUPMIX port, or the
         * reverse - the channel would simply never trigger. */
        bool dma_is_dma4 = (ctrl->dma_base == (EDMA_Type *)DMA4);
        if (dma_is_dma4 != s_i2c_hw[instance].in_wakeupmix) {
            return kStatus_InvalidArgument;
        }
    }

    /*
     * BOARD_BootClockRUN() on this board only brings up the PLLs, so the port's
     * root is ours to set. mux 0 is Osc24M, div 1 -> 24 MHz, which is what the
     * baud divider works from. LPI2C_MasterInit() ungates the peripheral
     * itself (i.MX93 defines LPI2C_CLOCKS), so the gate here is belt-and-braces.
     */
    const clock_root_config_t i2cClkCfg = {
        .clockOff = false,
        .mux      = 0,
        .div      = 1,
    };
    CLOCK_SetRootClock(s_i2c_hw[instance].clock_root, &i2cClkCfg);
    CLOCK_EnableClock(s_i2c_hw[instance].clock_gate);

    LPI2C_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate_Hz = ctrl->baudrate_bps;

    LPI2C_MasterInit(ctrl->i2c_base, &masterConfig, CLOCK_GetIpFreq(s_i2c_hw[instance].clock_root));

    if (!ctrl->enable_dma) {
        return kStatus_Success;
    }

    uint32_t edma_index = (ctrl->dma_base == (EDMA_Type *)DMA3) ? 0U : 1U;

    /* Ungate the eDMA. The root is deliberately left alone: kCLOCK_Root_M33 and
     * kCLOCK_Root_WakeupAxi are shared infrastructure the A55 side runs on, and
     * both are already up or we could not reach the LPI2C registers.
     * Note DMA3 is "eDMA1" and DMA4 is "eDMA2" in the gate naming. */
    CLOCK_EnableClock((edma_index == 0U) ? kCLOCK_Edma1 : kCLOCK_Edma2);

    if (!s_edma_started[edma_index]) {
        EDMA_GetDefaultConfig(&userConfig);
        EDMA_Init(ctrl->dma_base, &userConfig);
        s_edma_started[edma_index] = true;
    }

    (void)memset(&g_lpi2cRxEdmaHandles[instance], 0, sizeof(g_lpi2cRxEdmaHandles[instance]));
    (void)memset(&g_lpi2cTxEdmaHandles[instance], 0, sizeof(g_lpi2cTxEdmaHandles[instance]));

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
    EDMA_DisableChannelRequest(ctrl->dma_base, ctrl->dma_rx_channel);
    EDMA_ClearChannelStatusFlags(ctrl->dma_base, ctrl->dma_rx_channel,
                                 (uint32_t)kEDMA_DoneFlag | (uint32_t)kEDMA_ErrorFlag |
                                     (uint32_t)kEDMA_InterruptFlag);
    EDMA_DisableChannelRequest(ctrl->dma_base, ctrl->dma_tx_channel);
    EDMA_ClearChannelStatusFlags(ctrl->dma_base, ctrl->dma_tx_channel,
                                 (uint32_t)kEDMA_DoneFlag | (uint32_t)kEDMA_ErrorFlag |
                                     (uint32_t)kEDMA_InterruptFlag);

    /* EDMA_CreateHandle() enables the channel's NVIC line itself. */
    (void)EDMA_CreateHandle(&g_lpi2cRxEdmaHandles[instance], ctrl->dma_base, ctrl->dma_rx_channel);
    (void)EDMA_CreateHandle(&g_lpi2cTxEdmaHandles[instance], ctrl->dma_base, ctrl->dma_tx_channel);

    /* ...but not a priority, and the completion callback calls into FreeRTOS. */
    if (edma_index == 0U) {
        NVIC_SetPriority(I2C_DMA3_CHANNEL_IRQ(ctrl->dma_rx_channel), I2C_DMA_IRQ_PRIORITY);
        NVIC_SetPriority(I2C_DMA3_CHANNEL_IRQ(ctrl->dma_tx_channel), I2C_DMA_IRQ_PRIORITY);
    } else {
        NVIC_SetPriority(I2C_DMA4_CHANNEL_IRQ(ctrl->dma_rx_channel), I2C_DMA_IRQ_PRIORITY);
        NVIC_SetPriority(I2C_DMA4_CHANNEL_IRQ(ctrl->dma_tx_channel), I2C_DMA_IRQ_PRIORITY);
    }

    /* Route the LPI2C RX and TX requests to the chosen channels. On this part
     * the channel index is free and the request source is selected through the
     * channel mux - they are not the same number. */
    EDMA_SetChannelMux(ctrl->dma_base, ctrl->dma_rx_channel, ctrl->edma_rx_channel);
    EDMA_SetChannelMux(ctrl->dma_base, ctrl->dma_tx_channel, ctrl->edma_tx_channel);

    /* Created before the eDMA handle so userData is valid from the first
     * interrupt. Re-initialising a port must not leak the previous one. */
    if (ctrl->dma_semaphore == NULL) {
        ctrl->dma_semaphore = xSemaphoreCreateBinary();
        if (ctrl->dma_semaphore == NULL) {
            return kStatus_Fail;
        }
    }

    LPI2C_MasterCreateEDMAHandle(ctrl->i2c_base,
                                 &g_i2c_edma_handles[instance],
                                 &g_lpi2cRxEdmaHandles[instance],
                                 &g_lpi2cTxEdmaHandles[instance],
                                 i2c_dma_complete_callback,
                                 ctrl->dma_semaphore);

    /*
     * LPI2C_MasterCreateEDMAHandle() also enables the LPI2C's OWN interrupt -
     * the eDMA path uses it to feed the remaining command words and to report
     * errors - and leaves it at reset priority 0. That ISR dispatches our
     * completion callback, which calls xSemaphoreGiveFromISR(), so without this
     * line FreeRTOS's vPortValidateInterruptPriority() trips configASSERT and
     * the whole system freezes silently. Setting the eDMA channel priorities
     * alone is not enough.
     */
    NVIC_SetPriority(s_i2c_hw[instance].irq, I2C_DMA_IRQ_PRIORITY);

    return kStatus_Success;
}

status_t i2c_master_transfer(i2c_ctrl_t      *ctrl,
                             uint8_t          addr7,
                             uint32_t         subaddress,
                             size_t           subaddressSize,
                             lpi2c_direction_t dir,
                             uint8_t         *data,
                             size_t           dataSize)
{
    lpi2c_master_transfer_t xfer;
    uint32_t                instance;
    status_t                status;

    status = i2c_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (subaddressSize > sizeof(xfer.subaddress)) {
        return kStatus_InvalidArgument;
    }
    /* A zero-length read is not a thing on the wire; a zero-length write is
     * (it is how a bus scan probes an address). */
    if ((dir == kLPI2C_Read) && (dataSize == 0U)) {
        return kStatus_InvalidArgument;
    }

    xfer.flags          = (uint32_t)kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = (uint16_t)addr7;
    xfer.direction      = dir;
    xfer.subaddress     = subaddress;
    xfer.subaddressSize = subaddressSize;
    xfer.dataSize       = dataSize;

    if (!ctrl->enable_dma) {
        xfer.data = data;
        return LPI2C_MasterTransferBlocking(ctrl->i2c_base, &xfer);
    }

    /* Rejects a .rodata buffer, which lives in code TCM where the eDMA cannot
     * reach it - copy it into a RAM buffer first. */
    status = i2c_dma_addr(data, &xfer.data);
    if (status != kStatus_Success) {
        return status;
    }

    s_dma_status[instance] = kStatus_Success;

    status = LPI2C_MasterTransferEDMA(ctrl->i2c_base, &g_i2c_edma_handles[instance], &xfer);
    if (status != kStatus_Success) {
        return status;
    }

    uint32_t timeout = (ctrl->timeout_ms != 0U) ? ctrl->timeout_ms : I2C_DEFAULT_TIMEOUT_MS;

    if (xSemaphoreTake(ctrl->dma_semaphore, pdMS_TO_TICKS(timeout)) != pdTRUE) {
        /* Timeout - abort so the bus and the peripheral are left in a clean
         * state rather than mid-transaction with SCL held. */
        (void)LPI2C_MasterTransferAbortEDMA(ctrl->i2c_base, &g_i2c_edma_handles[instance]);
        return kStatus_Timeout;
    }

    /* kStatus_LPI2C_Nak (3302) here means nothing acknowledged the address. */
    return s_dma_status[instance];
}

status_t i2c_write_regs(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len)
{
    /* The SDK transfer struct is not const-qualified on data even for a write. */
    return i2c_master_transfer(ctrl, addr7, (uint32_t)reg, 1U, kLPI2C_Write,
                               (uint8_t *)(uintptr_t)data, len);
}

status_t i2c_read_regs(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transfer(ctrl, addr7, (uint32_t)reg, 1U, kLPI2C_Read, data, len);
}

status_t i2c_probe(i2c_ctrl_t *ctrl, uint8_t addr7)
{
    lpi2c_master_transfer_t xfer;
    uint32_t                instance;
    status_t                status;

    status = i2c_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    /* Address-only write: START, address, STOP. Deliberately on the blocking
     * path whatever enable_dma says - it is a one-off and a zero-length DMA
     * transaction is not worth the risk. */
    xfer.flags          = (uint32_t)kLPI2C_TransferDefaultFlag;
    xfer.slaveAddress   = (uint16_t)addr7;
    xfer.direction      = kLPI2C_Write;
    xfer.subaddress     = 0U;
    xfer.subaddressSize = 0U;
    xfer.data           = NULL;
    xfer.dataSize       = 0U;

    return LPI2C_MasterTransferBlocking(ctrl->i2c_base, &xfer);
}
