
# Copyright 2026 NXP
#
# SPDX-License-Identifier: BSD-3-Clause

cmake_minimum_required(VERSION 3.22.0)

cmake_path(APPEND conf_file_path ${CMAKE_CURRENT_LIST_DIR} ${board} prj.conf)
set(CONF_FILE ${conf_file_path} ${CONF_FILE})
include(${SdkRootDirPath}/cmake/extension/mcux.cmake)

project(mavlink_vtx LANGUAGES C CXX ASM PROJECT_BOARD_PORT_PATH ${board_root}/${board}/freertos_examples/freertos_hello)

include(${SdkRootDirPath}/CMakeLists.txt)


include(${board}/board_files.cmake)

# debug_console_lite forces the signed flag off in its radix converter unless
# this is set (fsl_debug_console.c: "#if !(PRINTF_ADVANCED_ENABLE > 0) neg = 0;"),
# so every %d prints as unsigned and -19 comes out as 4294967277.
mcux_add_configuration(
    CC "-DPRINTF_ADVANCED_ENABLE=1"
    CX "-DPRINTF_ADVANCED_ENABLE=1"
)

# DIAGNOSTIC, enabled deliberately: run both I2C buses on the pads' internal
# pull-ups, because the board's 3.3 V-side resistors (R58/R54 magnetometer,
# R57/R55 barometer) are all DNP. Also drops both buses to 100 kHz, since weak
# pull-ups cannot meet 400 kHz rise times. See the long note in pin_mux.c.
#
# REMOVE THIS once the resistors are populated - it is not a substitute for
# them, and leaving it on masks a real hardware fix with a marginal bus.
mcux_add_configuration(
    CC "-DI2C_INTERNAL_PULLUP=1"
    CX "-DI2C_INTERNAL_PULLUP=1"
)

# GPS navigation rate. Default (flag absent, or 0) is 10 Hz with all
# constellations enabled. Setting it to 1 switches Galileo, BeiDou and GLONASS
# off to reach 25 Hz, which costs satellite count and fix geometry - see the
# comment on GPS_SINGLE_GNSS_25HZ in sensors/gps_m10.h before enabling it.
#
# mcux_add_configuration(
#     CC "-DGPS_SINGLE_GNSS_25HZ=1"
#     CX "-DGPS_SINGLE_GNSS_25HZ=1"
# )

mcux_add_source(
    SOURCES main.c
            drivers/uart_driver.c
            drivers/spi_driver.c
            drivers/i2c_driver.c
            drivers/gpio_driver.c
            sensors/icm_42686_p.c
            sensors/bmm350.c
            sensors/bmp581.c
            sensors/gps_m10.c
            sensors/gps_ubx.c
            comms/mavlink_min.c
)

mcux_add_include(
    INCLUDES .
             drivers/
             sensors/
             comms/
)
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)

