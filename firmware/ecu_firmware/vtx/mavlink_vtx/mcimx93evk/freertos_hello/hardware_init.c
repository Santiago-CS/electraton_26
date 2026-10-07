/*
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
/*${header:start}*/
#include "app.h"
#include "board.h"
#include "pin_mux.h"
/*${header:end}*/

/*${function:start}*/
void BOARD_InitHardware(void)
{
    /* clang-format off */

    /* mux 0 is Osc24M, div 1 -> 24 MHz. BOARD_BootClockRUN() only brings up
     * the PLLs on this board, so every peripheral root is ours to set. */
    const clock_root_config_t lpuartClkCfg = {
        .clockOff = false,
        .mux = 0,
        .div = 1
    };

    const clock_root_config_t lpspiClkCfg = {
        .clockOff = false,
        .mux = 0,
        .div = 1
    };

    /* clang-format on */
    BOARD_InitBootPins();
    BOARD_BootClockRUN();

    CLOCK_SetRootClock(VTX_LPUART_CLOCK_ROOT, &lpuartClkCfg);
    CLOCK_EnableClock(VTX_LPUART_CLOCK_GATE);

    /*
     * The IMU link on LPSPI4. LPSPI_MasterInit() already ungates the peripheral
     * by itself - i.MX93 defines LPSPI_CLOCKS, unlike LPTMR - so the gate call
     * below is belt-and-braces. The root is the part that matters: nothing else
     * configures it, and CLOCK_GetIpFreq() reads it back to size the baud
     * divider, so leaving it unset gives the wrong SCK or none at all.
     */
    CLOCK_SetRootClock(IMU_LPSPI_CLOCK_ROOT, &lpspiClkCfg);
    CLOCK_EnableClock(IMU_LPSPI_CLOCK_GATE);

    BOARD_InitDebugConsole();
}
/*${function:end}*/
