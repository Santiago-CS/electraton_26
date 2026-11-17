/*
 * spi_driver.c - LPSPI master with optional eDMA, i.MX93 Cortex-M33.
 * See spi_driver.h for the API and the LPSPI/eDMA pairing rules.
 *
 * Author: Diego
 * Created on: 9, April 2026
 */

#include <string.h>

#include "spi_driver.h"

/*******************************************************************************
 * Instance tables
 ******************************************************************************/

/* Index 0 is LPSPI1: the i.MX93 numbers its ports from 1. Note that the SDK's
 * own LPSPI_GetInstance() returns 1..8 from a 9-entry table, so sizing arrays
 * by FSL_FEATURE_SOC_LPSPI_COUNT and indexing them with it overruns by one. */
#define SPI_INSTANCE_COUNT 8U

/* NVIC priority for the eDMA channel interrupts. The completion callback gives
 * a semaphore, so this must be numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY is 2 here). EDMA_CreateHandle()
 * calls EnableIRQ() but never sets a priority, which would leave these at 0 -
 * above the ceiling, where xSemaphoreGiveFromISR() is not allowed. */
#define SPI_DMA_IRQ_PRIORITY 5U

/* eDMA channel vectors are contiguous: DMA3 has one per channel from
 * DMA3_0_IRQn, DMA4 shares one per channel pair from DMA4_0_1_IRQn. */
#define SPI_DMA3_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA3_0_IRQn + (ch)))
#define SPI_DMA4_CHANNEL_IRQ(ch) ((IRQn_Type)((uint32_t)DMA4_0_1_IRQn + ((ch) / 2U)))

typedef struct {
    LPSPI_Type     *base;
    IRQn_Type       irq;          /*!< the peripheral's own line, not the eDMA channel's */
    clock_root_t    clock_root;
    clock_ip_name_t clock_gate;
    bool            in_wakeupmix; /*!< true -> driven by DMA4, false -> DMA3 */
} spi_hw_t;

static const spi_hw_t s_spi_hw[SPI_INSTANCE_COUNT] = {
    {LPSPI1, LPSPI1_IRQn, kCLOCK_Root_Lpspi1, kCLOCK_Lpspi1, false},
    {LPSPI2, LPSPI2_IRQn, kCLOCK_Root_Lpspi2, kCLOCK_Lpspi2, false},
    {LPSPI3, LPSPI3_IRQn, kCLOCK_Root_Lpspi3, kCLOCK_Lpspi3, true},
    {LPSPI4, LPSPI4_IRQn, kCLOCK_Root_Lpspi4, kCLOCK_Lpspi4, true},
    {LPSPI5, LPSPI5_IRQn, kCLOCK_Root_Lpspi5, kCLOCK_Lpspi5, true},
    {LPSPI6, LPSPI6_IRQn, kCLOCK_Root_Lpspi6, kCLOCK_Lpspi6, true},
    {LPSPI7, LPSPI7_IRQn, kCLOCK_Root_Lpspi7, kCLOCK_Lpspi7, true},
    {LPSPI8, LPSPI8_IRQn, kCLOCK_Root_Lpspi8, kCLOCK_Lpspi8, true},
};

static edma_handle_t             g_lpspiRxEdmaHandles[SPI_INSTANCE_COUNT];
static edma_handle_t             g_lpspiTxEdmaHandles[SPI_INSTANCE_COUNT];
static lpspi_master_handle_t     g_spi_handles[SPI_INSTANCE_COUNT];
static lpspi_master_edma_handle_t g_spi_edma_handles[SPI_INSTANCE_COUNT];

/* Result of the last eDMA transfer; see the note in i2c_driver.c. */
static volatile status_t s_dma_status[SPI_INSTANCE_COUNT];

/* EDMA_Init() resets a whole controller, so do it once per eDMA instance -
 * a single global flag would skip the second controller entirely. */
static bool s_edma_started[2]; /* [0] = DMA3, [1] = DMA4 */

/*******************************************************************************
 * Helpers
 ******************************************************************************/

static status_t spi_get_instance(const spi_ctrl_t *ctrl, uint32_t *instance)
{
    if ((ctrl == NULL) || (instance == NULL)) {
        return kStatus_InvalidArgument;
    }

    for (uint32_t i = 0U; i < SPI_INSTANCE_COUNT; i++) {
        if (s_spi_hw[i].base == ctrl->spi_base) {
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
#define SPI_CODE_TCM_BEGIN 0x0FFE0000U
#define SPI_CODE_TCM_END   0x0FFFFFFFU

/* Checks the buffer is one the eDMA can address. No translation: see above. */
static status_t spi_dma_addr(const void *local, uint8_t **dma_addr)
{
    uint32_t addr;

    if (local == NULL) {
        *dma_addr = NULL; /* half-duplex: the SDK substitutes its own scratch */
        return kStatus_Success;
    }

    addr = (uint32_t)(uintptr_t)local;
    if ((addr >= SPI_CODE_TCM_BEGIN) && (addr <= SPI_CODE_TCM_END)) {
        return kStatus_InvalidArgument;
    }

    *dma_addr = (uint8_t *)(uintptr_t)addr;
    return kStatus_Success;
}

/*
 * Internal DMA completion callback.
 *
 * Called from the eDMA IRQ handler when the LPSPI DMA transfer finishes. Gives
 * the per-instance binary semaphore so spi_master_transfer() can unblock the
 * calling FreeRTOS task instead of busy-polling.
 *
 * userData is the SemaphoreHandle_t stored in spi_ctrl_t.dma_semaphore and
 * installed as the userData argument in LPSPI_MasterTransferCreateHandleEDMA.
 */
static void spi_dma_complete_callback(LPSPI_Type *base,
                                      lpspi_master_edma_handle_t *handle,
                                      status_t completionStatus,
                                      void *userData)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    (void)handle;

    /* The callback fires on failure too, so carry the status out to the
     * caller rather than reporting every transfer as a success. */
    for (uint32_t i = 0U; i < SPI_INSTANCE_COUNT; i++) {
        if (s_spi_hw[i].base == base) {
            s_dma_status[i] = completionStatus;
            break;
        }
    }

    xSemaphoreGiveFromISR((SemaphoreHandle_t)userData, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*******************************************************************************
 * Default configurations
 ******************************************************************************/

void spi_get_defaultconfig_imu(spi_ctrl_t *imu)
{
    imu->spi_base         = IMU_SPI_MASTER_BASEADDR;
    imu->dma_base         = (EDMA_Type *)IMU_SPI_MASTER_DMA_BASE;
    imu->dma_rx_channel   = IMU_SPI_MASTER_DMA_RX_CHANNEL;
    imu->dma_tx_channel   = IMU_SPI_MASTER_DMA_TX_CHANNEL;
    imu->edma_rx_channel  = (uint32_t)IMU_SPI_RECEIVE_EDMA_CHANNEL;
    imu->edma_tx_channel  = (uint32_t)IMU_SPI_TRANSMIT_EDMA_CHANNEL;
    imu->baudrate_u32     = IMU_SPI_TRANSFER_BAUDRATE;
    imu->pcs_for_init     = IMU_SPI_MASTER_PCS_FOR_INIT;
    imu->pcs_for_transfer = (uint32_t)IMU_SPI_MASTER_PCS_FOR_TRANSFER;
    imu->cpol             = IMU_SPI_MASTER_CPOL;
    imu->cpha             = IMU_SPI_MASTER_CPHA;
    imu->byte_swap        = true;
    imu->pcs_continuous   = false;
    imu->enable_dma       = true;
    imu->dma_semaphore    = NULL;
}

/*******************************************************************************
 * Driver Functions
 ******************************************************************************/

status_t spi_init(spi_ctrl_t *ctrl)
{
    lpspi_master_config_t masterConfig;
    edma_config_t         userConfig = {0};
    uint32_t              instance;
    status_t              status;

    status = spi_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (ctrl->enable_dma) {
        /* An AONMIX eDMA has no request line to a WAKEUPMIX port, or the
         * reverse - the channel would simply never trigger. */
        bool dma_is_dma4 = (ctrl->dma_base == (EDMA_Type *)DMA4);
        if (dma_is_dma4 != s_spi_hw[instance].in_wakeupmix) {
            return kStatus_InvalidArgument;
        }
    }

    /*
     * BOARD_BootClockRUN() on this board only brings up the PLLs, so the port's
     * root and gate are ours to set. mux 0 is Osc24M, div 1 -> 24 MHz, which is
     * the LPSPI functional clock the baud divider works from.
     */
    const clock_root_config_t spiClkCfg = {
        .clockOff = false,
        .mux      = 0,
        .div      = 1,
    };
    CLOCK_SetRootClock(s_spi_hw[instance].clock_root, &spiClkCfg);
    CLOCK_EnableClock(s_spi_hw[instance].clock_gate);

    LPSPI_MasterGetDefaultConfig(&masterConfig);
    masterConfig.baudRate = ctrl->baudrate_u32;
    masterConfig.whichPcs = ctrl->pcs_for_init;
    masterConfig.cpol     = ctrl->cpol;
    masterConfig.cpha     = ctrl->cpha;
    masterConfig.pcsToSckDelayInNanoSec        = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.lastSckToPcsDelayInNanoSec    = 1000000000U / (masterConfig.baudRate * 2U);
    masterConfig.betweenTransferDelayInNanoSec = 1000000000U / (masterConfig.baudRate * 2U);

    LPSPI_MasterInit(ctrl->spi_base, &masterConfig, CLOCK_GetIpFreq(s_spi_hw[instance].clock_root));

    if (!ctrl->enable_dma) {
        /* Blocking path - no DMA, no callback needed (pass NULL). */
        LPSPI_MasterTransferCreateHandle(ctrl->spi_base, &g_spi_handles[instance], NULL, NULL);
        return kStatus_Success;
    }

    uint32_t edma_index = (ctrl->dma_base == (EDMA_Type *)DMA3) ? 0U : 1U;

    /* Ungate the eDMA. The root is deliberately left alone: kCLOCK_Root_M33 and
     * kCLOCK_Root_WakeupAxi are shared infrastructure the A55 side runs on, and
     * both are already up or we could not reach the LPSPI registers.
     * Note DMA3 is "eDMA1" and DMA4 is "eDMA2" in the gate naming. */
    CLOCK_EnableClock((edma_index == 0U) ? kCLOCK_Edma1 : kCLOCK_Edma2);

    if (!s_edma_started[edma_index]) {
        EDMA_GetDefaultConfig(&userConfig);
        EDMA_Init(ctrl->dma_base, &userConfig);
        s_edma_started[edma_index] = true;
    }

    (void)memset(&g_lpspiRxEdmaHandles[instance], 0, sizeof(g_lpspiRxEdmaHandles[instance]));
    (void)memset(&g_lpspiTxEdmaHandles[instance], 0, sizeof(g_lpspiTxEdmaHandles[instance]));

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
    (void)EDMA_CreateHandle(&g_lpspiRxEdmaHandles[instance], ctrl->dma_base, ctrl->dma_rx_channel);
    (void)EDMA_CreateHandle(&g_lpspiTxEdmaHandles[instance], ctrl->dma_base, ctrl->dma_tx_channel);

    /* ...but not a priority, and the completion callback calls into FreeRTOS. */
    if (edma_index == 0U) {
        NVIC_SetPriority(SPI_DMA3_CHANNEL_IRQ(ctrl->dma_rx_channel), SPI_DMA_IRQ_PRIORITY);
        NVIC_SetPriority(SPI_DMA3_CHANNEL_IRQ(ctrl->dma_tx_channel), SPI_DMA_IRQ_PRIORITY);
    } else {
        NVIC_SetPriority(SPI_DMA4_CHANNEL_IRQ(ctrl->dma_rx_channel), SPI_DMA_IRQ_PRIORITY);
        NVIC_SetPriority(SPI_DMA4_CHANNEL_IRQ(ctrl->dma_tx_channel), SPI_DMA_IRQ_PRIORITY);
    }

    /* Route the LPSPI RX and TX requests to the chosen channels. On this part
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

    LPSPI_MasterTransferCreateHandleEDMA(ctrl->spi_base,
                                         &g_spi_edma_handles[instance],
                                         spi_dma_complete_callback,
                                         ctrl->dma_semaphore,
                                         &g_lpspiRxEdmaHandles[instance],
                                         &g_lpspiTxEdmaHandles[instance]);

    /* Same trap as the LPI2C driver: CreateHandleEDMA also enables the LPSPI's
     * own interrupt at reset priority 0, and that ISR can dispatch the
     * completion callback. It has not fired on the IMU's transfers, but a
     * callback that touches FreeRTOS from priority 0 would freeze everything. */
    NVIC_SetPriority(s_spi_hw[instance].irq, SPI_DMA_IRQ_PRIORITY);

    return kStatus_Success;
}

status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize)
{
    lpspi_transfer_t masterXfer;
    uint32_t         instance;
    status_t         status;

    status = spi_get_instance(ctrl, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    if (dataSize == 0U) {
        return kStatus_InvalidArgument;
    }

    masterXfer.dataSize    = dataSize;
    masterXfer.configFlags = ctrl->pcs_for_transfer;
    if (ctrl->byte_swap) {
        masterXfer.configFlags |= (uint32_t)kLPSPI_MasterByteSwap;
    }
    if (ctrl->pcs_continuous) {
        masterXfer.configFlags |= (uint32_t)kLPSPI_MasterPcsContinuous;
    }

    if (!ctrl->enable_dma) {
        masterXfer.txData = txData;
        masterXfer.rxData = rxData;
        return LPSPI_MasterTransferBlocking(ctrl->spi_base, &masterXfer);
    }

    /* Rejects a .rodata buffer, which lives in code TCM where the eDMA cannot
     * reach it - copy it into a RAM buffer first. lpspi_transfer_t::txData is
     * const-qualified, so it is filled through a plain pointer. */
    uint8_t *txDmaAddr;
    uint8_t *rxDmaAddr;

    status = spi_dma_addr(txData, &txDmaAddr);
    if (status != kStatus_Success) {
        return status;
    }
    status = spi_dma_addr(rxData, &rxDmaAddr);
    if (status != kStatus_Success) {
        return status;
    }
    masterXfer.txData = txDmaAddr;
    masterXfer.rxData = rxDmaAddr;

    s_dma_status[instance] = kStatus_Success;

    status = LPSPI_MasterTransferEDMA(ctrl->spi_base, &g_spi_edma_handles[instance], &masterXfer);
    if (status != kStatus_Success) {
        return status;
    }

    if (xSemaphoreTake(ctrl->dma_semaphore, pdMS_TO_TICKS(100)) != pdTRUE) {
        /* Timeout - abort so the peripheral is left in a clean state. */
        LPSPI_MasterTransferAbortEDMA(ctrl->spi_base, &g_spi_edma_handles[instance]);
        return kStatus_Timeout;
    }

    return s_dma_status[instance];
}
