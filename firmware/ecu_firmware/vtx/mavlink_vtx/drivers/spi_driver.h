/*
 * spi_driver.h - LPSPI master with optional eDMA, i.MX93 Cortex-M33.
 *
 * Ported from the MCXN947 version. As with the LPUART, there is no LP_FLEXCOMM
 * here: LPSPI1..LPSPI8 are standalone blocks with their own clock roots, gates
 * and vectors, and which eDMA can drive them is fixed by where they sit:
 *
 *   LPSPI1, LPSPI2   -> AONMIX    -> DMA3 (kDma3RequestMuxLPSPI*)
 *   LPSPI3 .. LPSPI8 -> WAKEUPMIX -> DMA4 (kDma4RequestMuxLPSPI*)
 *
 * spi_init() rejects a mismatched pair rather than letting you configure a
 * channel that can never be triggered.
 *
 * Author: Diego
 * Created on: 9, April 2026
 */

#ifndef SPI_DRIVER_H_
#define SPI_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "fsl_clock.h"
#include "fsl_device_registers.h"
#include "fsl_edma.h"
#include "fsl_lpspi.h"
#include "fsl_lpspi_edma.h"

/* FreeRTOS - needed for the DMA completion semaphore */
#include "FreeRTOS.h"
#include "semphr.h"

/* ---------------------------------------------------------------------------
 *  Port assignment
 *
 *  The IMU sits on LPSPI4, which is in WAKEUPMIX and so is driven by DMA4.
 *  Its pads are muxed in BOARD_InitPins() (pin_mux.c); on the FRDM board all
 *  four go straight from the SoC to header P11 with nothing else on them.
 *  These are the same four pads LPSPI5 uses - only the ALT differs - so the
 *  wiring is identical either way:
 *
 *    GPIO_IO18  ball R18  P11 pin 12  ALT5  LPSPI4_PCS0
 *    GPIO_IO19  ball R17  P11 pin 35  ALT5  LPSPI4_SIN   (MISO)
 *    GPIO_IO20  ball T20  P11 pin 38  ALT5  LPSPI4_SOUT  (MOSI)
 *    GPIO_IO21  ball T21  P11 pin 40  ALT5  LPSPI4_SCK
 *
 *  eDMA channels 2 and 3 are used here; 0 and 1 belong to the VTX LPUART.
 * ------------------------------------------------------------------------- */

#define IMU_SPI_TRANSFER_BAUDRATE       2000000U /* 2 MHz */
#define IMU_SPI_MASTER_BASEADDR         (LPSPI4)
#define IMU_SPI_MASTER_PCS_FOR_INIT     (kLPSPI_Pcs0)
#define IMU_SPI_MASTER_PCS_FOR_TRANSFER (kLPSPI_MasterPcs0)
#define IMU_SPI_MASTER_CPOL             kLPSPI_ClockPolarityActiveLow
#define IMU_SPI_MASTER_CPHA             kLPSPI_ClockPhaseSecondEdge
#define IMU_SPI_MASTER_DMA_BASE         DMA4
#define IMU_SPI_MASTER_DMA_RX_CHANNEL   2U
#define IMU_SPI_MASTER_DMA_TX_CHANNEL   3U
#define IMU_SPI_RECEIVE_EDMA_CHANNEL    kDma4RequestMuxLPSPI4Rx
#define IMU_SPI_TRANSMIT_EDMA_CHANNEL   kDma4RequestMuxLPSPI4Tx

typedef struct spi_ctrl_s {
    LPSPI_Type *spi_base;
    EDMA_Type  *dma_base;       /*!< DMA3 or DMA4; EDMA_Type is void on this SoC */
    uint32_t    dma_rx_channel; /*!< eDMA channel index, not the request source */
    uint32_t    dma_tx_channel;
    uint32_t    edma_rx_channel; /*!< kDma4RequestMuxLPSPI3Rx & co. */
    uint32_t    edma_tx_channel;
    uint32_t    baudrate_u32;
    lpspi_clock_phase_t    cpha;
    lpspi_clock_polarity_t cpol;
    lpspi_which_pcs_t      pcs_for_init;
    /*
     * Not an lpspi_which_pcs_t despite the name: kLPSPI_MasterPcs* are already
     * shifted into their place in the transfer's configFlags word, whereas
     * kLPSPI_Pcs* (used for pcs_for_init) are plain indices. The two only
     * coincide for PCS0.
     */
    uint32_t               pcs_for_transfer;
    bool byte_swap;  /*!< kLPSPI_MasterByteSwap: send byte 0 of the buffer first */
    /*
     * kLPSPI_MasterPcsContinuous: hold PCS asserted across the whole transfer.
     * Off by default because it is a per-device decision, but most multi-byte
     * device protocols need it - without TCR[CONT] the LPSPI drops PCS between
     * every 8-bit frame, so a register burst becomes N separate transactions.
     */
    bool pcs_continuous;
    bool enable_dma;
    /*
     * Binary semaphore given by the DMA completion ISR so spi_master_transfer()
     * can block the calling task instead of busy-waiting. Created by spi_init()
     * when enable_dma is true.
     */
    SemaphoreHandle_t dma_semaphore;
} spi_ctrl_t;

/*!
 * @brief Gets the default configuration for the IMU SPI master.
 * @param imu Pointer to a spi_ctrl_t struct that will hold the default configuration.
 */
void spi_get_defaultconfig_imu(spi_ctrl_t *imu);

/*!
 * @brief Clock, configure and open the LPSPI master, bringing up both eDMA
 *        channels when enable_dma is set.
 * @return kStatus_InvalidArgument for an unknown base or a DMA that cannot
 *         reach it, kStatus_Fail if the semaphore could not be created.
 */
status_t spi_init(spi_ctrl_t *ctrl);

/*!
 * @brief Full-duplex transfer. Blocking either way: by polling when enable_dma
 *        is false, or by blocking the calling task on the DMA semaphore when it
 *        is true - so the DMA path must be called from a task, after the
 *        scheduler has started.
 *
 * In DMA mode txData/rxData must live in .data/.bss or shared RAM. A buffer in
 * code TCM (anything const that the linker put in .rodata) is rejected with
 * kStatus_InvalidArgument, because the eDMA has no way to address it.
 */
status_t spi_master_transfer(spi_ctrl_t *ctrl, uint8_t *txData, uint8_t *rxData, size_t dataSize);

#endif /* SPI_DRIVER_H_ */
