/*
 * Copyright 2021 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _APP_H_
#define _APP_H_

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*${macro:start}*/
#define VTX_LPUART              LPUART6
#define VTX_LPUART_CLOCK_ROOT   kCLOCK_Root_Lpuart6
#define VTX_LPUART_CLOCK_GATE   kCLOCK_Lpuart6
#define VTX_LPUART_CLK_FREQ     CLOCK_GetIpFreq(VTX_LPUART_CLOCK_ROOT)
#define VTX_LPUART_IRQn         LPUART6_IRQn
#define VTX_LPUART_IRQHandler   LPUART6_IRQHandler

/* The IMU link. Must agree with IMU_SPI_MASTER_BASEADDR in drivers/spi_driver.h. */
#define IMU_LPSPI               LPSPI4
#define IMU_LPSPI_CLOCK_ROOT    kCLOCK_Root_Lpspi4
#define IMU_LPSPI_CLOCK_GATE    kCLOCK_Lpspi4
#define IMU_LPSPI_CLK_FREQ      CLOCK_GetIpFreq(IMU_LPSPI_CLOCK_ROOT)
#define IMU_LPSPI_IRQn          LPSPI4_IRQn
/*${macro:end}*/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*${prototype:start}*/
void BOARD_InitHardware(void);
/*${prototype:end}*/

#endif /* _APP_H_ */
