/*
 * i2c_driver.h - LPI2C master with optional eDMA, i.MX93 Cortex-M33.
 *
 * Same shape as spi_driver.h / uart_driver.h. As with those, which eDMA can
 * drive a port is fixed by where the port sits:
 *
 *   LPI2C1, LPI2C2   -> AONMIX    -> DMA3 (kDma3RequestMuxLPI2C*)
 *   LPI2C3 .. LPI2C8 -> WAKEUPMIX -> DMA4 (kDma4RequestMuxLPI2C*)
 *
 * i2c_init() rejects a mismatched pair rather than letting you configure a
 * channel that can never be triggered.
 *
 * The eDMA path here leans on something the UART and SPI paths do not:
 * fsl_lpi2c_edma.c feeds the DMA from memory inside its own handle - the I2C
 * command sequence from handle->commandBuffer, the scatter-gather chain from
 * handle->tcds[] - at their plain local addresses. That is fine on this board:
 * a memory-to-memory test showed both eDMA controllers address M33 TCM at the
 * local address, so no translation is needed anywhere and the SDK's own
 * assumption holds.
 *
 * ---------------------------------------------------------------------------
 *
 * Author: Diego
 */

#ifndef I2C_DRIVER_H_
#define I2C_DRIVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsl_clock.h"
#include "fsl_edma.h"
#include "fsl_lpi2c.h"
#include "fsl_lpi2c_edma.h"

/* FreeRTOS - needed for the DMA completion semaphore */
#include "FreeRTOS.h"
#include "semphr.h"

/* ---------------------------------------------------------------------------
 *  Default port
 *
 *  LPI2C4 on GPIO_IO02/IO03 (ALT1) - the sensor board's magnetometer bus.
 *  The TMP_SENSOR_BOARD schematic calls it I2C6 (its numbering is i.MX95), and
 *  the FRDM netlist calls the same two pins I2C4_SDA / I2C4_SCL, which agrees
 *  with the i.MX93 instance:
 *
 *    GPIO_IO02  ball K20  P11 pin 3  ALT1  LPI2C4_SDA
 *    GPIO_IO03  ball K21  P11 pin 5  ALT1  LPI2C4_SCL
 *
 *  Each runs to the header through a 0 ohm series link (R2668 / R2672) with
 *  nothing else on the net - no pull-ups anywhere on the FRDM side.
 *
 *  BUS PULL-UPS: the sensor board's 3.3 V-side pull-ups for this bus, R54
 *  (SDA) and R58 (SCL), 2.2k each, are marked DNP. Its 1.8 V-side pair
 *  (R20/R11) is fitted, but a MOSFET level translator needs both - with only
 *  the low side fitted nothing ever pulls the host line high. Populate R54 and
 *  R58, or add equivalent pull-ups to 3V3, or this bus will not run.
 *
 *  LPI2C3 reaches GPIO_IO00/IO01 instead, which is the schematic's I2C5 - the
 *  BMP581 barometer, whose DNP pull-ups are R55/R57.
 *
 *  eDMA channels 4 and 5 are used here: 0/1 belong to the VTX LPUART and 2/3
 *  to the IMU LPSPI.
 * ------------------------------------------------------------------------- */

/*
 * When the pads are running on their weak internal pull-ups instead of the
 * board's missing 2.2k resistors (I2C_INTERNAL_PULLUP in pin_mux.c), 400 kHz
 * has no chance of meeting the rise time. Drop both buses to 100 kHz so the
 * diagnostic is actually testing the bus and not the edge rate.
 */
#ifndef I2C_INTERNAL_PULLUP
#define I2C_INTERNAL_PULLUP 0
#endif

#define I2C_MASTER_BASEADDR        (LPI2C4)
#if I2C_INTERNAL_PULLUP
#define I2C_MASTER_BAUDRATE        100000U /* weak pull-ups: see above */
#else
#define I2C_MASTER_BAUDRATE        400000U /* 400 kHz fast mode */
#endif
#define I2C_MASTER_DMA_BASE        DMA4
#define I2C_MASTER_DMA_RX_CHANNEL  4U
#define I2C_MASTER_DMA_TX_CHANNEL  5U
#define I2C_RECEIVE_EDMA_CHANNEL   kDma4RequestMuxLPI2C4Rx
#define I2C_TRANSMIT_EDMA_CHANNEL  kDma4RequestMuxLPI2C4Tx

/* ---------------------------------------------------------------------------
 *  Barometer port
 *
 *  LPI2C3 on GPIO_IO00/IO01 (ALT1) - the schematic's I2C5, carrying the BMP581.
 *
 *    GPIO_IO00  P11 pin 27  ALT1  LPI2C3_SDA   (schematic I2C5_SDA)
 *    GPIO_IO01  P11 pin 28  ALT1  LPI2C3_SCL   (schematic I2C5_SCL)
 *
 *  Two things to know about this bus that the magnetometer's does not share:
 *
 *  1. The GPS is on it too. The Pixhawk-standard GPS connector carries DDC on
 *     pins 4 and 5, and those are this same pair - so a u-blox module left in
 *     its default state is a second device at 0x42. Harmless (the addresses do
 *     not collide with the BMP581's 0x46), but it means a bus scan here will
 *     normally find the GPS as well, and a wedged GPS can hold the lines down.
 *
 *  2. 100 kHz, not 400 kHz. See the pull-up note below - this bus is expected
 *     to run on marginal pull-ups at first, and the slower rate gives the rise
 *     times somewhere to hide.
 *
 *  BUS PULL-UPS: same problem as LPI2C4, different resistors. The 3.3 V-side
 *  pair for this bus is R57 (SCL) and R55 (SDA), 2.2k each, and both are DNP.
 *  The 1.8 V side (R13/R22) is fitted. A MOSFET translator needs both ends, so
 *  until R55/R57 are populated nothing pulls the host side high and every
 *  transfer NAKs. See BARO_I2C_INTERNAL_PULLUP in pin_mux.c for a solder-free
 *  way to try the driver before then.
 *
 *  eDMA channels 8 and 9: 0/1 VTX LPUART, 2/3 IMU LPSPI, 4/5 magnetometer
 *  LPI2C, 6/7 GPS LPUART. DMA4 has 64 channels, so there is plenty of room.
 * ------------------------------------------------------------------------- */

#define BARO_I2C_BASEADDR        (LPI2C3)
#define BARO_I2C_BAUDRATE        100000U /* 100 kHz - see the pull-up note */
#define BARO_I2C_DMA_BASE        DMA4
#define BARO_I2C_DMA_RX_CHANNEL  8U
#define BARO_I2C_DMA_TX_CHANNEL  9U
#define BARO_I2C_RX_DMA_REQUEST  kDma4RequestMuxLPI2C3Rx
#define BARO_I2C_TX_DMA_REQUEST  kDma4RequestMuxLPI2C3Tx

typedef struct i2c_ctrl_s {
    LPI2C_Type *i2c_base;
    EDMA_Type  *dma_base;       /*!< DMA3 or DMA4; unused when enable_dma is false */
    uint32_t    dma_rx_channel; /*!< eDMA channel index, not the request source */
    uint32_t    dma_tx_channel;
    uint32_t    edma_rx_channel; /*!< kDma4RequestMuxLPI2C4Rx & co. */
    uint32_t    edma_tx_channel;
    uint32_t    baudrate_bps;
    bool        enable_dma;
    /*
     * Binary semaphore given by the DMA completion ISR so i2c_master_transfer()
     * can block the calling task instead of busy-waiting. Created by i2c_init()
     * when enable_dma is true.
     */
    SemaphoreHandle_t dma_semaphore;
    uint32_t          timeout_ms; /*!< DMA completion wait; 0 selects 100 ms */
} i2c_ctrl_t;

/* Fill ctrl with the magnetometer defaults (LPI2C4, 400 kHz, eDMA). */
void i2c_get_default_config(i2c_ctrl_t *ctrl);

/* Fill ctrl with the barometer defaults (LPI2C3, 100 kHz, eDMA). */
void i2c_get_default_baro_config(i2c_ctrl_t *ctrl);

/*!
 * @brief Clock, configure and open the LPI2C master, bringing up both eDMA
 *        channels when enable_dma is set.
 * @return kStatus_InvalidArgument for an unknown base or a DMA that cannot
 *         reach it, kStatus_Fail if the semaphore could not be created.
 */
status_t i2c_init(i2c_ctrl_t *ctrl);

/*!
 * @brief One I2C transaction: START, 7-bit address, optional sub-address,
 *        data, STOP.
 *
 * Blocking either way - by polling when enable_dma is false, or by blocking
 * the calling task on the DMA semaphore when it is true, which means the DMA
 * path must be called from a task after the scheduler has started.
 *
 * In DMA mode data must live in .data/.bss or shared RAM; a buffer in code TCM
 * (anything const the linker put in .rodata) is rejected with
 * kStatus_InvalidArgument because the eDMA cannot address it.
 *
 * @param addr7          7-bit slave address, unshifted.
 * @param subaddress     register address, sent MSB first; ignored if size is 0.
 * @param subaddressSize 0..4 bytes.
 * @param dir            kLPI2C_Write or kLPI2C_Read.
 */
status_t i2c_master_transfer(i2c_ctrl_t      *ctrl,
                             uint8_t          addr7,
                             uint32_t         subaddress,
                             size_t           subaddressSize,
                             lpi2c_direction_t dir,
                             uint8_t         *data,
                             size_t           dataSize);

/* Convenience wrappers over i2c_master_transfer() for 8-bit register maps. */
status_t i2c_write_regs(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, const uint8_t *data, size_t len);
status_t i2c_read_regs(i2c_ctrl_t *ctrl, uint8_t addr7, uint8_t reg, uint8_t *data, size_t len);

/*!
 * @brief Address-only write, for bus scans. Always runs on the blocking path -
 *        it is a one-off and a zero-length DMA transfer is not worth the risk.
 * @return kStatus_Success if the device ACKed its address.
 */
status_t i2c_probe(i2c_ctrl_t *ctrl, uint8_t addr7);

#endif /* I2C_DRIVER_H_ */
