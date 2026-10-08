
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

# Project copy of the SDK linker script, so that .bss/.heap/.stack carry their
# real load address. With the SDK's own, remoteproc refuses to start the image
# once it grows past ~106 KB of code - see the note at the top of the copy.
mcux_remove_armgcc_linker_script(
    BASE_PATH ${SdkRootDirPath}
    TARGETS debug release
    LINKER devices/i.MX/i.MX93/MIMX9352/gcc/MIMX9352xxxxM_ram.ld
)
mcux_add_armgcc_linker_script(
    BASE_PATH ${CMAKE_CURRENT_LIST_DIR}
    TARGETS debug release
    LINKER mcimx93evk/freertos_hello/MIMX9352_cm33_ram.ld
)

# debug_console_lite forces the signed flag off in its radix converter unless
# this is set (fsl_debug_console.c: "#if !(PRINTF_ADVANCED_ENABLE > 0) neg = 0;"),
# so every %d prints as unsigned and -19 comes out as 4294967277.
mcux_add_configuration(
    CC "-DPRINTF_ADVANCED_ENABLE=1"
    CX "-DPRINTF_ADVANCED_ENABLE=1"
)

# Internal pad pull-ups on both I2C buses, in parallel with the board's 2.2k
# (R54/R58, R55/R57 are now fitted). Also drops both buses to 100 kHz - see
# i2c_driver.h. Measured on the magnetometer bus (LPI2C4):
#   400 kHz, board resistors only        locked up after ~400 reads
#   400 kHz, + internal pull-ups         locked up after ~230 reads
#   200 kHz, + internal pull-ups         locked up after ~550 reads
#   100 kHz, + internal pull-ups         clean
# I2C_MASTER_BAUDRATE can override the speed from here if that is retested.
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
            sensors/gps_m10.c
            sensors/gps_ubx.c
            comms/mavlink_min.c
            comms/pmac_link.c
            nav/ahrs.c
            mcimx93evk/freertos_hello/rsc_table.c
)

mcux_add_include(
    INCLUDES .
             drivers/
             sensors/
             comms/
             nav/
)
mcux_convert_binary(BINARY ${APPLICATION_BINARY_DIR}/${MCUX_SDK_PROJECT_NAME}.bin)

