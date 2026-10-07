/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "board.h"
#include "app.h"

/* User includes */
#include "uart_driver.h"
#include "spi_driver.h"
#include "icm_42686_p.h"
#include "i2c_driver.h"
#include "bmm350.h"
#include "bmp581.h"
#include "mavlink_min.h"
#include "gps_m10.h"





/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* Task priorities. */
#define  imu_task_PRIORITY     (configMAX_PRIORITIES - 2)
#define  mag_task_PRIORITY     (configMAX_PRIORITIES - 3)
#define  gps_task_PRIORITY     (configMAX_PRIORITIES - 4)
#define  baro_task_PRIORITY    (configMAX_PRIORITIES - 5)

/*
 * Periodic data output per task. Init banners and failures are never
 * suppressed - losing those costs diagnosis - only the once-per-sample lines,
 * which exist to prove a sensor works and are noise once it has.
 */
#define  GPS_PRINT_FIX         0
#define  BARO_PRINT_DATA       0

/*
 * The two I2C sensors are off until R54/R58 and R55/R57 are fitted. Both buses
 * are proven to work only on the pads' weak internal pull-ups, which gives the
 * barometer about 1 corrupt read in 125 and the magnetometer no usable data at
 * all - see STATUS.md section 4.1. Set this back to 1 once the resistors are
 * populated; nothing else needs to change.
 */
#define  I2C_SENSORS_ENABLED   0

/* ---------------------------------------------------------------------------
 *  IMU -> VTX telemetry stream
 *
 *  Raw IMU samples pushed out of LPUART6 as fixed-size binary frames. The
 *  point is to measure what the VTX link will actually carry, so the frame
 *  carries a sequence number and a timestamp: without the sequence number the
 *  receiving end cannot tell a dropped frame from a slow one, which is the
 *  only measurement that matters.
 *
 *  Binary rather than ASCII on purpose. The same sample as text is about 40
 *  bytes against 24 here, and at 200 Hz that is the difference between 42% and
 *  70% of a 115200 link - which would measure the encoding, not the link.
 *
 *  Wire format, little-endian throughout, 24 bytes:
 *
 *    0   u8   0xA5            sync
 *    1   u8   0x5A            sync
 *    2   u16  seq             wraps; gaps at the receiver are lost frames
 *    4   u32  t_us            microseconds, monotonic
 *    8   i16  accel[3]        raw LSB, X Y Z
 *    14  i16  gyro[3]         raw LSB, X Y Z
 *    20  i16  temp            raw LSB
 *    22  u16  crc             CRC-16/CCITT-FALSE over bytes 2..21
 *
 *  Raw LSB counts, not scaled units: the receiver applies the full-scale map
 *  it was told about, and sending raw keeps this frame independent of the
 *  range the IMU happens to be configured for. At FS_SEL=1 that is 2048 LSB/g
 *  and 16.4 LSB/dps - see sensors/icm_42686_p.h.
 * ------------------------------------------------------------------------- */
/*
 * What goes on the VTX link.
 *
 * 1 = MAVLink v2, which is what the datalink actually wants. It is not a
 *     transparent serial pipe: it parses frames and discards bytes that are
 *     not valid MAVLink, which is why the custom frame below never reached
 *     the ground station. Speaking MAVLink also means QGroundControl shows
 *     the board as a vehicle.
 * 0 = the custom 24-byte raw frame. Kept because flipping this is the
 *     experiment that proves the paragraph above: if MAVLink arrives and raw
 *     does not, the datalink is filtering.
 */
#define  VTX_STREAM_MAVLINK    1

/* SCALED_IMU every Nth sample. At 200 Hz, 4 gives 50 Hz and ~1.7 kB/s. */
#define  VTX_IMU_DIVISOR       4U
#define  VTX_HEARTBEAT_MS      1000U

#define  VTX_IMU_FRAME_BYTES   24U
#define  VTX_FRAME_SYNC0       0xA5U
#define  VTX_FRAME_SYNC1       0x5AU
/*
 * Queue depth. One frame is 208 us on the wire at 115200 and a new one arrives
 * every 5 ms, so this is pure insurance against a scheduling hiccup rather
 * than a working buffer. Overruns are counted, never silently dropped.
 */
#define  VTX_TX_SLOTS          16U /* one task iteration can now queue
                                     * HEARTBEAT + SCALED_IMU + GPS_RAW_INT */
/* Big enough for the longest frame either format produces: GPS_RAW_INT is 42
 * bytes on the wire against the raw frame's 24. */
#define  VTX_SLOT_BYTES        48U
#define  VTX_REPORT_MS         1000U
#define  GPS_RX_CHUNK_BYTES    64U  /* eDMA receive granularity */
#define  GPS_POLL_MS            5U  /* worst-case added latency for the tail of a
                                     * burst, since the task consumes partial
                                     * chunks - see gps_drain() */
#define  GPS_TX_TIMEOUT_MS    200U  /* a 42-byte frame at 9600 baud is 44 ms */
#define  GPS_ACK_TIMEOUT_MS   500U  /* the receiver answers a VALSET promptly;
                                     * this is slack, not a measured figure */
/*
 * A ring, not a ping-pong. At 115200 a 32-byte chunk completes every 2.78 ms,
 * so two buffers and a polled task could not keep up during a burst - the ISR
 * overwrote buffers mid-read and the parser saw spliced sentences. Eight
 * buffers is 256 bytes, about 22 ms of slack, and the task is woken by a
 * notification rather than polling so it drains promptly.
 */
#define  GPS_RX_BUFFERS         8U
#define  GPS_BAUD_TRY_MS     1500U  /* > 1 navigation epoch, so a silent second
                                     * is not mistaken for a wrong baud rate */

/*******************************************************************************
 * Global variables
 ******************************************************************************/
uart_ctrl_t vtx_uart;
spi_ctrl_t  imu_spi;
i2c_ctrl_t  mag_i2c;
i2c_ctrl_t  baro_i2c;
uart_ctrl_t gps_uart;

/*
 * A ring of receive buffers. uart_read_dma() completes on a byte count and
 * stops, so anything arriving before the next arm is dropped on the floor -
 * at 115200 baud that is a byte every 87 us. Re-arming from the task left a
 * whole scheduler tick of dead air; the ISR swaps buffers and re-arms
 * immediately instead, which shrinks the gap to a few microseconds.
 */
static uint8_t           s_gps_rx[GPS_RX_BUFFERS][GPS_RX_CHUNK_BYTES];
static volatile uint8_t  s_gps_head;   /* buffer the eDMA is writing, advanced by the ISR */
static volatile uint8_t  s_gps_tail;   /* next buffer for the parser, advanced by the task */
static volatile uint32_t s_gps_chunks;
static volatile uint32_t s_gps_rearm_fail;
static volatile uint32_t s_gps_ring_overflow;
static TaskHandle_t      s_gps_task_handle;
static volatile bool     s_gps_tx_done;
/*
 * How much of the buffer at s_gps_tail the parser has already been given.
 * Owned by the task; the ISR only clears it, and only on the overflow path
 * where it force-advances the tail past a buffer the count no longer describes.
 */
static volatile uint32_t s_gps_partial;

/* Set by the eDMA callback when a queued RX block has landed. */
static volatile bool s_vtx_rx_done;

/*
 * VTX transmit queue. The producer (imu_task) fills a slot and kicks; the
 * TxIdle callback chains straight into the next slot from interrupt context,
 * so the link stays saturated without the task having to wait on anything.
 */
static uint8_t           s_vtx_tx[VTX_TX_SLOTS][VTX_SLOT_BYTES];
static uint16_t          s_vtx_tx_len[VTX_TX_SLOTS];
#if VTX_STREAM_MAVLINK
/* This node's MAVLink identity. sysid 1 is the conventional first vehicle,
 * compid 1 is MAV_COMP_ID_AUTOPILOT1 - which is what makes a ground station
 * treat us as the flight controller rather than a peripheral. */
static mavlink_node_t    s_mav = {.sysid = 1U, .compid = 1U, .seq = 0U};
#endif
static volatile uint8_t  s_vtx_tx_head;  /* next slot the producer will fill */
static volatile uint8_t  s_vtx_tx_tail;  /* slot currently on the wire */
static volatile bool     s_vtx_tx_busy;
static volatile uint32_t s_vtx_frames_sent;
static volatile uint32_t s_vtx_bytes_sent;
static volatile uint32_t s_vtx_frames_dropped; /* queue was full */
static volatile uint32_t s_vtx_tx_errors;      /* uart_write_bytes refused */

/*
 * The latest GPS solution, published by gps_task for imu_task to transmit.
 *
 * imu_task stays the only thing that writes the VTX queue and the only thing
 * that touches the MAVLink sequence counter, so neither needs a lock. The
 * price is this hand-off: gps_task fills a complete snapshot and copies it in
 * under a critical section, imu_task copies it out the same way. Both copies
 * are a few dozen bytes, so the critical sections are a few cycles.
 *
 * Already in MAVLink's units, so the decisions about what counts as "no fix"
 * or "course unknown" are made once, next to the GPS data they depend on.
 */
typedef struct {
    uint32_t updates;   /*!< changes on every new solution; 0 = none yet */
    uint8_t  fix_type;  /*!< GPS_FIX_TYPE_* */
    uint8_t  sats;
    int32_t  lat_1e7;
    int32_t  lon_1e7;
    int32_t  alt_mm;    /*!< above mean sea level */
    uint16_t eph;       /*!< DOP x100, MAVLINK_U16_UNKNOWN if none */
    uint16_t vel_cm_s;
    uint16_t cog_cdeg;  /*!< MAVLINK_U16_UNKNOWN when too slow to mean anything */
} gps_snapshot_t;

static gps_snapshot_t s_gps_snap;

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void imu_task(void *pvParameters);
#if I2C_SENSORS_ENABLED
static void mag_task(void *pvParameters);
#endif
static void gps_task(void *pvParameters);
#if I2C_SENSORS_ENABLED
static void baro_task(void *pvParameters);
#endif
static void gps_uart_callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);
static void vtx_uart_callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData);


/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * Microsecond clock
 *
 * The FreeRTOS tick is 5 ms here (configTICK_RATE_HZ is 200), so tick counts
 * cannot timestamp a 200 Hz stream - one tick IS the sample period. The M33's
 * DWT cycle counter gives real microseconds instead.
 *
 * DWT is not guaranteed usable: it can be left locked, and on some parts it
 * only runs with a debugger attached. So the enable is checked by looking for
 * the counter actually moving, and there is a tick-based fallback. Which one
 * is in use gets printed, because a silently wrong timebase would make every
 * rate figure below a lie.
 ******************************************************************************/

static bool s_dwt_ok;

static void micros_init(void)
{
    uint32_t before;

    /* Unlock, enable the trace block, then the cycle counter itself. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    /* No DWT->LAR in this CMSIS revision - the lock register is not modelled
     * for this core, so there is nothing to unlock here. If CYCCNT turns out
     * not to run, the fallback below covers it. */
    DWT->CYCCNT = 0U;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;

    before = DWT->CYCCNT;
    for (volatile uint32_t i = 0U; i < 100U; i++) {
        /* burn a few cycles */
    }
    s_dwt_ok = (DWT->CYCCNT != before);
}

/*
 * CYCCNT is 32 bits, which at this board's ~200 MHz wraps every 21.5 seconds.
 * Returning it raw made MAVLink's time_boot_ms jump backwards to zero every
 * 21 s - plainly visible in a capture as "21.46s" followed by "0.01s" - and a
 * ground station is entitled to do something unhelpful with a clock that goes
 * backwards. Accumulate the wraps into 64 bits instead.
 *
 * Single-caller by design: imu_task is the only thing that asks the time, at
 * 200 Hz, which is far more often than the 21 s wrap period. If a second
 * context ever calls this, the carry needs a lock.
 */
static uint32_t s_cyc_last;
static uint64_t s_cyc_high;

static uint64_t micros64(void)
{
    uint32_t now;

    if (!s_dwt_ok) {
        return (uint64_t)xTaskGetTickCount() * (1000000ULL / configTICK_RATE_HZ);
    }

    now = DWT->CYCCNT;
    if (now < s_cyc_last) {
        s_cyc_high += 0x100000000ULL;
    }
    s_cyc_last = now;

    return (s_cyc_high + now) / (uint64_t)(SystemCoreClock / 1000000U);
}

#if !VTX_STREAM_MAVLINK
static uint32_t micros(void)
{
    return (uint32_t)micros64();
}
#endif

/*******************************************************************************
 * VTX frame
 ******************************************************************************/

#if !VTX_STREAM_MAVLINK
/* CRC-16/CCITT-FALSE: init 0xFFFF, poly 0x1021, no reflection, no final xor. */
static uint16_t vtx_crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint32_t i = 0U; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (uint8_t b = 0U; b < 8U; b++) {
            crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((uint16_t)(crc << 1) ^ 0x1021U)
                                          : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/*
 * Serialise explicitly rather than memcpy a packed struct: the wire format is
 * then stated in one place and does not depend on the compiler's padding or on
 * the core's endianness.
 */
static void vtx_build_imu_frame(uint8_t *out, uint16_t seq, uint32_t t_us,
                                const icm42686_data_t *s)
{
    uint16_t crc;
    uint32_t i = 0U;

    out[i++] = VTX_FRAME_SYNC0;
    out[i++] = VTX_FRAME_SYNC1;
    out[i++] = (uint8_t)(seq & 0xFFU);
    out[i++] = (uint8_t)(seq >> 8);
    out[i++] = (uint8_t)(t_us & 0xFFU);
    out[i++] = (uint8_t)((t_us >> 8) & 0xFFU);
    out[i++] = (uint8_t)((t_us >> 16) & 0xFFU);
    out[i++] = (uint8_t)((t_us >> 24) & 0xFFU);

    for (uint32_t a = 0U; a < 3U; a++) {
        out[i++] = (uint8_t)((uint16_t)s->accel_raw[a] & 0xFFU);
        out[i++] = (uint8_t)((uint16_t)s->accel_raw[a] >> 8);
    }
    for (uint32_t a = 0U; a < 3U; a++) {
        out[i++] = (uint8_t)((uint16_t)s->gyro_raw[a] & 0xFFU);
        out[i++] = (uint8_t)((uint16_t)s->gyro_raw[a] >> 8);
    }
    out[i++] = (uint8_t)((uint16_t)s->temp_raw & 0xFFU);
    out[i++] = (uint8_t)((uint16_t)s->temp_raw >> 8);

    /* Over everything between the sync bytes and the checksum. */
    crc = vtx_crc16(&out[2], i - 2U);
    out[i++] = (uint8_t)(crc & 0xFFU);
    out[i++] = (uint8_t)(crc >> 8);
}
#endif /* !VTX_STREAM_MAVLINK */


/*!
 * @brief Start the next queued frame if the port is idle.
 *
 * Called from the task. The TxIdle callback chains the following frames itself
 * from interrupt context, so this only ever starts a stopped stream - hence
 * the critical section, which keeps the busy flag and the queue consistent
 * against that callback.
 */
static void vtx_tx_kick(void)
{
    taskENTER_CRITICAL();
    if (!s_vtx_tx_busy && (s_vtx_tx_tail != s_vtx_tx_head))
    {
        if (uart_write_bytes(&vtx_uart, s_vtx_tx[s_vtx_tx_tail],
                             s_vtx_tx_len[s_vtx_tx_tail]) == kStatus_Success)
        {
            s_vtx_tx_busy = true;
        }
        else
        {
            s_vtx_tx_errors++;
        }
    }
    taskEXIT_CRITICAL();
}

/*! @brief Queue one already-built frame. Drops the newest and counts it when
 *         the queue is full - dropping the oldest would reorder the sequence
 *         numbers, which is the one thing the receiver uses to detect loss.
 *
 *  Single producer today (imu_task). If a second task ever queues frames, the
 *  head advance and the MAVLink sequence counter both need a lock. */
static void vtx_queue_frame(const uint8_t *frame, uint16_t len)
{
    uint8_t next = (uint8_t)((s_vtx_tx_head + 1U) % VTX_TX_SLOTS);

    if ((len == 0U) || (len > VTX_SLOT_BYTES))
    {
        s_vtx_tx_errors++;
        return;
    }
    if (next == s_vtx_tx_tail)
    {
        s_vtx_frames_dropped++;
        return;
    }

    (void)memcpy(s_vtx_tx[s_vtx_tx_head], frame, len);
    s_vtx_tx_len[s_vtx_tx_head] = len;
    s_vtx_tx_head = next;

    vtx_tx_kick();
}

/*! @brief Queue one IMU sample in whichever format is selected. */
static void vtx_queue_imu_sample(const icm42686_data_t *s)
{
    uint8_t  frame[VTX_SLOT_BYTES];
    uint16_t len;

#if VTX_STREAM_MAVLINK
    len = mavlink_scaled_imu(frame, sizeof(frame), &s_mav,
                             (uint32_t)(micros64() / 1000ULL),
                             s->accel_mg, s->gyro_mdps, NULL);
#else
    static uint16_t seq;

    vtx_build_imu_frame(frame, seq++, micros(), s);
    len = VTX_IMU_FRAME_BYTES;
#endif

    vtx_queue_frame(frame, len);
}


/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitHardware();

    uart_get_default_vtx_config(&vtx_uart, vtx_uart_callback);
    if (uart_init(&vtx_uart) != kStatus_Success)
    {
        PRINTF("uart_init failed!\r\n");
        while (1)
            ;
    }
    /* Not a string literal: those live in code TCM, which the eDMA cannot read. */
    static char vtx_banner[] = "VTX LPUART6 up\r\n";
    (void)uart_write(&vtx_uart, vtx_banner);

    micros_init();

    /* configMINIMAL_STACK_SIZE + 300: icm42686_data_t plus the burst buffer is
     * a few hundred bytes of frame, and PRINTF is not cheap on stack either. */
    if (xTaskCreate(imu_task, "imu_task", configMINIMAL_STACK_SIZE + 300, NULL, imu_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("IMU task creation failed!.\r\n");
        while (1)
            ;
    }

#if I2C_SENSORS_ENABLED
    if (xTaskCreate(mag_task, "mag_task", configMINIMAL_STACK_SIZE + 300, NULL, mag_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("MAG task creation failed!.\r\n");
        while (1)
            ;
    }
#endif

    /* +500: gps_m10_t carries an 84-byte line buffer and a field pointer array
     * on the parsing path, on top of PRINTF. */
    if (xTaskCreate(gps_task, "gps_task", configMINIMAL_STACK_SIZE + 500, NULL, gps_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("GPS task creation failed!.\r\n");
        while (1)
            ;
    }

#if I2C_SENSORS_ENABLED
    /* +300: bmp581_altitude_cm() calls powf, and PRINTF is the usual tenant. */
    if (xTaskCreate(baro_task, "baro_task", configMINIMAL_STACK_SIZE + 300, NULL, baro_task_PRIORITY, NULL) !=
        pdPASS)
    {
        PRINTF("BARO task creation failed!.\r\n");
        while (1)
            ;
    }
#endif
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Brings up the ICM-42686-P and prints a sample every IMU_SAMPLE_RATE_MS.
 *
 * Runs the LPSPI in polled mode rather than eDMA: a 15-byte transaction at
 * 2 MHz is about 60 us every 100 ms, so DMA buys nothing here, and it keeps the
 * sample path clear of the TCM-aliasing question that the eDMA path still has
 * open. Flip enable_dma to true once that is settled.
 *
 * pcs_continuous must be on either way - the burst read is one CS assertion.
 */
static void imu_task(void *pvParameters)
{
    icm42686_t      imu;
    icm42686_data_t sample;
    status_t        status;

    (void)pvParameters;

    spi_get_defaultconfig_imu(&imu_spi);
    /*
     * eDMA on LPSPI4 / DMA4 channels 2 and 3. Safe now that the memory-to-memory
     * test showed both controllers address M33 TCM at the plain local address,
     * so the SDK's software TCDs and our .bss buffers are all reachable.
     *
     * pcs_continuous stays on either way: the burst read is one CS assertion,
     * and without TCR[CONT] the LPSPI drops CS between every byte.
     */
    imu_spi.enable_dma     = true;
    imu_spi.pcs_continuous = true;

    status = spi_init(&imu_spi);
    if (status != kStatus_Success)
    {
        PRINTF("IMU: spi_init failed (%d)\r\n", (int)status);
        vTaskDelete(NULL);
    }

    PRINTF("IMU: LPSPI4 %s\r\n", imu_spi.enable_dma ? "eDMA (DMA4 ch2/ch3)" : "polled");

    icm42686_get_default_config(&imu, &imu_spi);
    /*
     * 200 Hz rather than the driver's 100 Hz default: one FreeRTOS tick is
     * 5 ms here, so 200 Hz is the fastest the task can be scheduled without a
     * data-ready interrupt, and matching the sensor ODR to the task period
     * avoids aliasing one against the other.
     */
    imu.accel_odr = ACCEL_ODR_200Hz;
    imu.gyro_odr  = GYRO_ODR_200Hz;

    status = icm42686_init(&imu);
    if (status != kStatus_Success)
    {
        uint8_t id = 0U;
        (void)icm42686_who_am_i(&imu, &id);
        PRINTF("IMU: init failed (%d), WHO_AM_I=0x%02X expected 0x%02X\r\n",
               (int)status, id, ICM42686_WHO_AM_I_VAL);
        vTaskDelete(NULL);
    }

    PRINTF("IMU: ICM-42686-P ready, %u Hz sampled -> VTX LPUART6 as %s at %u Hz (%s clock)\r\n",
           (unsigned int)(1000U / IMU_SAMPLE_RATE_MS),
           VTX_STREAM_MAVLINK ? "MAVLink SCALED_IMU" : "raw frames",
           (unsigned int)(1000U / IMU_SAMPLE_RATE_MS / VTX_IMU_DIVISOR),
           s_dwt_ok ? "DWT" : "tick");

    uint32_t   reads = 0U;
    uint32_t   fails = 0U;
    TickType_t next_report = xTaskGetTickCount() + pdMS_TO_TICKS(VTX_REPORT_MS);
#if VTX_STREAM_MAVLINK
    TickType_t next_beat   = xTaskGetTickCount();
    uint32_t   gps_sent    = 0U; /* s_gps_snap.updates last transmitted */
#endif
    uint32_t   last_sent = 0U;
    uint32_t   last_bytes = 0U;

    while (1)
    {
        status = icm42686_read_data(&imu, &sample);
        reads++;

        if (status == kStatus_Success)
        {
            /* Every sample is read, but only every Nth is transmitted: a GCS
             * has no use for 200 Hz and the link does. */
            if ((reads % VTX_IMU_DIVISOR) == 0U)
            {
                vtx_queue_imu_sample(&sample);
            }

            /* Milli-units as integers: PRINTF_FLOAT_ENABLE is off in this
             * build, so %f would print garbage. */
            // PRINTF("IMU a[mg] %6d %6d %6d  g[mdps] %7d %7d %7d  t[mC] %6d\r\n",
            //        (int)sample.accel_mg[0], (int)sample.accel_mg[1], (int)sample.accel_mg[2],
            //        (int)sample.gyro_mdps[0], (int)sample.gyro_mdps[1], (int)sample.gyro_mdps[2],
            //        (int)sample.temp_mc);
        }
        else
        {
            /*
             * Worth distinguishing while the DMA path is on trial:
             * kStatus_Timeout (4) means the transfer was accepted but the
             * completion callback never fired - the eDMA did not finish.
             * kStatus_LPSPI_Busy means a previous transfer never cleared.
             */
            fails++;
            PRINTF("IMU: read failed, status=%d (%u of %u reads)\r\n",
                   (int)status, (unsigned int)fails, (unsigned int)reads);
        }

#if VTX_STREAM_MAVLINK
        /*
         * HEARTBEAT at 1 Hz. Without it a ground station will not show a
         * vehicle at all, however much sensor data arrives - it is the message
         * that says "something is here". Sent from this task so there is a
         * single producer for the transmit queue and the MAVLink sequence
         * counter needs no lock.
         */
        if ((int32_t)(xTaskGetTickCount() - next_beat) >= 0)
        {
            uint8_t  beat[VTX_SLOT_BYTES];
            uint16_t len;

            next_beat += pdMS_TO_TICKS(VTX_HEARTBEAT_MS);
            len = mavlink_heartbeat(beat, sizeof(beat), &s_mav,
                                    MAV_TYPE_GENERIC, MAV_STATE_ACTIVE);
            vtx_queue_frame(beat, len);
        }

        /*
         * GPS_RAW_INT whenever gps_task has published a new solution - 10 Hz
         * with the receiver configured, 1 Hz on factory NMEA. Sent from here,
         * not from gps_task, to keep a single producer; see s_gps_snap.
         */
        {
            gps_snapshot_t snap;

            taskENTER_CRITICAL();
            snap = s_gps_snap;
            taskEXIT_CRITICAL();

            if ((snap.updates != 0U) && (snap.updates != gps_sent))
            {
                uint8_t  frame[VTX_SLOT_BYTES];
                uint16_t len;

                gps_sent = snap.updates;
                len = mavlink_gps_raw_int(frame, sizeof(frame), &s_mav, micros64(),
                                          snap.fix_type, snap.sats,
                                          snap.lat_1e7, snap.lon_1e7, snap.alt_mm,
                                          snap.eph, (uint16_t)MAVLINK_U16_UNKNOWN,
                                          snap.vel_cm_s, snap.cog_cdeg);
                vtx_queue_frame(frame, len);
            }
        }
#endif

        if ((int32_t)(xTaskGetTickCount() - next_report) >= 0)
        {
            uint32_t sent  = s_vtx_frames_sent;
            uint32_t bytes = s_vtx_bytes_sent;
            uint32_t fps   = sent - last_sent;
            uint32_t bps   = bytes - last_bytes;
            /* Line capacity in bytes/s: 10 bits on the wire per byte with one
             * start and one stop bit, so 115200 baud carries 11520 B/s. */
            uint32_t capacity = vtx_uart.baudrate / 10U;

            next_report += pdMS_TO_TICKS(VTX_REPORT_MS);
            last_sent  = sent;
            last_bytes = bytes;

            PRINTF("VTX %u fps  %u B/s  %u%% of %u baud  [sent=%u drop=%u err=%u reads=%u fail=%u]\r\n",
                   (unsigned int)fps, (unsigned int)bps,
                   (unsigned int)((bps * 100U) / capacity),
                   (unsigned int)vtx_uart.baudrate,
                   (unsigned int)sent, (unsigned int)s_vtx_frames_dropped,
                   (unsigned int)s_vtx_tx_errors,
                   (unsigned int)reads, (unsigned int)fails);
        }

        vTaskDelay(pdMS_TO_TICKS(IMU_SAMPLE_RATE_MS));
    }
}

#if I2C_SENSORS_ENABLED
/*!
 * @brief Brings up the BMM350 magnetometer and prints a sample every
 *        BMM350_SAMPLE_RATE_MS.
 *
 * Uses the LPI2C4 defaults from i2c_driver.h. Values are RAW 24-bit counts:
 * converting to microtesla needs Bosch's OTP compensation, which this driver
 * does not implement - see the scope note in sensors/bmm350.h.
 *
 * On a failure to find the part it scans the bus, because the first thing you
 * want to know is whether anything is acknowledging at all.
 */
static void mag_task(void *pvParameters)
{
    bmm350_t      mag;
    bmm350_data_t sample;
    status_t      status;
    uint32_t      reads = 0U;
    uint32_t      fails = 0U;

    (void)pvParameters;

    i2c_get_default_config(&mag_i2c);

    status = i2c_init(&mag_i2c);
    if (status != kStatus_Success)
    {
        PRINTF("MAG: i2c_init failed (%d)\r\n", (int)status);
        vTaskDelete(NULL);
    }

    PRINTF("MAG: LPI2C4 %s @ %u Hz\r\n",
           mag_i2c.enable_dma ? "eDMA (DMA4 ch4/ch5)" : "polled",
           (unsigned int)mag_i2c.baudrate_bps);

    bmm350_get_default_config(&mag, &mag_i2c);

    status = bmm350_init(&mag);
    if (status != kStatus_Success)
    {
        uint8_t id = 0U;
        (void)bmm350_chip_id(&mag, &id);
        PRINTF("MAG: init failed (%d), CHIP_ID=0x%02X expected 0x%02X at addr 0x%02X\r\n",
               (int)status, id, BMM350_CHIP_ID_VAL, mag.addr7);

        /* Nothing answered where we expected it - say who is on the bus, if
         * anyone. All-silent points at the pull-ups; something at another
         * address points at ADSEL or the wrong bus. */
        PRINTF("MAG: scanning bus...\r\n");
        for (uint8_t a = 0x08U; a < 0x78U; a++)
        {
            if (i2c_probe(&mag_i2c, a) == kStatus_Success)
            {
                PRINTF("MAG:   ACK at 0x%02X\r\n", a);
            }
        }
        PRINTF("MAG: scan done\r\n");
        vTaskDelete(NULL);
    }

    PRINTF("MAG: BMM350 ready\r\n");

    while (1)
    {
        status = bmm350_read_data(&mag, &sample);
        reads++;

        if (status == kStatus_Success)
        {
            PRINTF("MAG raw x %8d  y %8d  z %8d   t %8d\r\n",
                   (int)sample.mag_raw[0], (int)sample.mag_raw[1], (int)sample.mag_raw[2],
                   (int)sample.temp_raw);
        }
        else
        {
            fails++;
            PRINTF("MAG: read failed, status=%d (%u of %u reads)\r\n",
                   (int)status, (unsigned int)fails, (unsigned int)reads);
        }

        vTaskDelay(pdMS_TO_TICKS(BMM350_SAMPLE_RATE_MS));
    }
}
#endif /* I2C_SENSORS_ENABLED */


#if I2C_SENSORS_ENABLED
/*!
 * @brief Reads the BMP581 barometer on LPI2C3 and prints pressure and altitude.
 *
 * Expect this to fail until R55 and R57 are populated - see the barometer
 * section of i2c_driver.h. A failure here looks exactly like the magnetometer's
 * on the other bus, and for the same reason, so the diagnostics are the same:
 * 900 (kStatus_LPI2C_Busy) means the lines are being held low and the bus never
 * even starts, 902 (kStatus_LPI2C_Nak) means it is clocking and nothing is
 * answering.
 *
 * On failure it scans the bus, which is more informative here than on LPI2C4:
 * the GPS DDC sits on this same pair, so a healthy bus with a default-
 * configured u-blox attached should ACK at 0x42 whether or not the barometer
 * answers at 0x46. Finding 0x42 but not 0x46 would point at the sensor board;
 * finding neither points at the pull-ups.
 */
static void baro_task(void *pvParameters)
{
    bmp581_t      baro;
    bmp581_data_t sample;
    status_t      status;
    uint32_t      reads = 0U;
    uint32_t      fails = 0U;
    /* Captured on the first good reading. Absolute altitude is only as good as
     * the sea-level reference, but the change from here is trustworthy. */
    uint32_t ref_pa    = 0U;
    bool     have_ref  = false;

    (void)pvParameters;

    i2c_get_default_baro_config(&baro_i2c);

    status = i2c_init(&baro_i2c);
    if (status != kStatus_Success)
    {
        PRINTF("BARO: i2c_init failed (%d)\r\n", (int)status);
        vTaskDelete(NULL);
    }

    PRINTF("BARO: LPI2C3 %s @ %u Hz\r\n",
           baro_i2c.enable_dma ? "eDMA (DMA4 ch8/ch9)" : "polled",
           (unsigned int)baro_i2c.baudrate_bps);

    bmp581_get_default_config(&baro, &baro_i2c);

    status = bmp581_init(&baro);
    if (status != kStatus_Success)
    {
        uint8_t id = 0U;

        (void)bmp581_chip_id(&baro, &id);
        PRINTF("BARO: init failed (%d), CHIP_ID=0x%02X expected 0x%02X/0x%02X at addr 0x%02X\r\n",
               (int)status, id, BMP581_CHIP_ID_VAL, BMP581_CHIP_ID_VAL_ALT, baro.addr7);

        PRINTF("BARO: scanning bus...\r\n");
        for (uint8_t a = 0x08U; a < 0x78U; a++)
        {
            if (i2c_probe(&baro_i2c, a) == kStatus_Success)
            {
                PRINTF("BARO:   ACK at 0x%02X%s\r\n", a,
                       (a == 0x42U) ? "  (u-blox GPS DDC)" : "");
            }
        }
        PRINTF("BARO: scan done\r\n");
        vTaskDelete(NULL);
    }

    PRINTF("BARO: BMP581 ready, osr_p=%ux osr_t=%ux effective, odr %s\r\n",
           (unsigned int)(1U << baro.osr_p_eff), (unsigned int)(1U << baro.osr_t_eff),
           baro.odr_valid ? "valid" : "TOO FAST for this oversampling");

    while (1)
    {
        status = bmp581_read_data(&baro, &sample);
        reads++;

        if (status == kStatus_Success)
        {
            int32_t alt_cm = bmp581_altitude_cm(sample.press_pa, BMP581_SEA_LEVEL_PA);

            if (!have_ref)
            {
                ref_pa   = sample.press_pa;
                have_ref = true;
            }

#if BARO_PRINT_DATA
            PRINTF("BARO %u.%02u Pa  %d.%03u C  alt %d.%02u m",
                   (unsigned int)(sample.press_c_pa / 100U),
                   (unsigned int)(sample.press_c_pa % 100U),
                   (int)(sample.temp_m_c / 1000),
                   (unsigned int)((sample.temp_m_c < 0 ? -sample.temp_m_c : sample.temp_m_c) % 1000),
                   (int)(alt_cm / 100),
                   (unsigned int)((alt_cm < 0 ? -alt_cm : alt_cm) % 100));

            {
                /* Relative to the first reading - the number a barometer is
                 * actually good for. */
                int32_t d_cm = bmp581_altitude_cm(sample.press_pa, BMP581_SEA_LEVEL_PA) -
                               bmp581_altitude_cm(ref_pa, BMP581_SEA_LEVEL_PA);

                PRINTF("  (%c%d.%02u m from start)\r\n",
                       (d_cm < 0) ? '-' : '+',
                       (int)((d_cm < 0 ? -d_cm : d_cm) / 100),
                       (unsigned int)((d_cm < 0 ? -d_cm : d_cm) % 100));
            }
#else
            /* Still tracked, so turning BARO_PRINT_DATA back on needs no other
             * change - and so the reference is taken from the first good
             * reading rather than whenever printing happens to be enabled. */
            (void)alt_cm;
            (void)ref_pa;
#endif
        }
        else
        {
            fails++;
            /* kStatus_Fail here is the driver's range check rejecting a
             * corrupted transfer, which on this bus is a different and much
             * more common story than a NAK - say which. */
            PRINTF("BARO: read failed, status=%d%s (%u of %u reads) raw p=0x%06X t=0x%06X\r\n",
                   (int)status,
                   (status == kStatus_Fail) ? " (out of range: corrupt transfer)" : "",
                   (unsigned int)fails, (unsigned int)reads,
                   (unsigned int)sample.press_raw,
                   (unsigned int)((uint32_t)sample.temp_raw & 0xFFFFFFU));
        }

        vTaskDelay(pdMS_TO_TICKS(BMP581_SAMPLE_RATE_MS));
    }
}
#endif /* I2C_SENSORS_ENABLED */


/*!
 * @brief eDMA completion callback for the GPS LPUART. Interrupt context.
 */
static void gps_uart_callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    BaseType_t woken = pdFALSE;
    uint8_t    next;

    if (status == kStatus_LPUART_TxIdle)
    {
        /* A configuration frame has gone out. The task waits on this before
         * reusing the frame buffer or expecting the ACK. */
        s_gps_tx_done = true;
        if (s_gps_task_handle != NULL)
        {
            vTaskNotifyGiveFromISR(s_gps_task_handle, &woken);
            portYIELD_FROM_ISR(woken);
        }
        return;
    }

    if (status != kStatus_LPUART_RxIdle)
    {
        return;
    }

    /* s_gps_rx[s_gps_head] is now full. Publish it and move on. */
    next = (uint8_t)((s_gps_head + 1U) % GPS_RX_BUFFERS);
    if (next == s_gps_tail)
    {
        /* Ring full: drop the oldest rather than stall the receiver. Losing a
         * whole chunk costs one sentence; overwriting one the parser is mid-way
         * through splices two together, which is far worse. */
        s_gps_ring_overflow++;
        s_gps_tail    = (uint8_t)((s_gps_tail + 1U) % GPS_RX_BUFFERS);
        s_gps_partial = 0U; /* it described the buffer we just skipped */
    }
    s_gps_head = next;
    s_gps_chunks++;

    /*
     * Safe to re-arm from here: LPUART_ReceiveEDMACallback() calls
     * LPUART_TransferAbortReceiveEDMA() before invoking this callback, so the
     * handle is already idle.
     */
    if (uart_read_dma(&gps_uart, s_gps_rx[s_gps_head], GPS_RX_CHUNK_BYTES) != kStatus_Success)
    {
        s_gps_rearm_fail++;
    }

    if (s_gps_task_handle != NULL)
    {
        vTaskNotifyGiveFromISR(s_gps_task_handle, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/*!
 * @brief Hand the parser everything that has arrived - whole chunks and the
 *        part of the chunk the eDMA is still filling.
 *
 * Draining only completed chunks strands the tail of every burst. A receiver
 * emitting one 100-byte NAV-PVT per fix leaves a few bytes short of a chunk
 * boundary each time, and those bytes then wait for the *next* fix to top the
 * chunk up - a whole navigation period of latency, which at 10 Hz is 100 ms
 * added to every position. Reading the eDMA's progress count and consuming the
 * partial chunk brings that down to one GPS_POLL_MS tick.
 */
static void gps_drain(gps_m10_t *gps)
{
    uint8_t head;

    /* Completed chunks. s_gps_partial describes the buffer at the tail, which
     * may have completed since the last call - the count is still the number
     * of bytes of it already consumed, so resume from there. */
    while (s_gps_tail != s_gps_head)
    {
        uint8_t  idx  = s_gps_tail;
        uint32_t from = s_gps_partial;

        if (from < GPS_RX_CHUNK_BYTES)
        {
            (void)gps_m10_feed(gps, &s_gps_rx[idx][from], GPS_RX_CHUNK_BYTES - from);
        }
        s_gps_partial = 0U;
        s_gps_tail    = (uint8_t)((idx + 1U) % GPS_RX_BUFFERS);
    }

    /* Then the chunk still being filled. */
    head = s_gps_head;
    if (s_gps_tail == head)
    {
        uint32_t landed;

        if (uart_rx_dma_count(&gps_uart, &landed) == kStatus_Success)
        {
            if (landed > GPS_RX_CHUNK_BYTES)
            {
                landed = GPS_RX_CHUNK_BYTES;
            }

            /*
             * Re-read the head: if the ISR completed this chunk and re-armed
             * while we were reading the count, that count belongs to the next
             * buffer and must not be applied to this one. Dropping it is safe -
             * the loop above picks the completed chunk up on the next call.
             */
            if ((s_gps_head == head) && (landed > s_gps_partial))
            {
                (void)gps_m10_feed(gps, &s_gps_rx[head][s_gps_partial],
                                   landed - s_gps_partial);
                s_gps_partial = landed;
            }
        }
    }
}

/*! @brief Put a UBX frame on the wire and wait for the eDMA to finish with it. */
static status_t gps_send_frame(const uint8_t *frame, uint16_t len)
{
    status_t status;

    s_gps_tx_done = false;

    status = uart_write_bytes(&gps_uart, frame, len);
    if (status != kStatus_Success)
    {
        return status;
    }

    for (uint32_t waited = 0U; waited < GPS_TX_TIMEOUT_MS; waited += GPS_POLL_MS)
    {
        if (s_gps_tx_done)
        {
            return kStatus_Success;
        }
        vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
    }

    return kStatus_Timeout;
}

/*!
 * @brief Configure the receiver for fast position-only output.
 *
 * Each UBX-CFG-VALSET is acknowledged, so this reports per step whether the
 * receiver accepted it. A NAK means the configuration key is wrong for the
 * installed firmware rather than that the hardware is broken - see the warning
 * at the top of gps_ubx.h, and cross-check the key against the interface
 * description or against what u-center emits.
 *
 * The receive stays armed throughout, which is what lets the ACKs arrive.
 *
 * @return true only if every step was ACK'd.
 */
static bool gps_configure(gps_m10_t *gps)
{
    /* Static, not stack: the eDMA reads the frame directly out of here. */
    static gps_m10_cfg_step_t s_steps[GPS_CFG_STEP_MAX];
    uint32_t n;
    bool     all_ok = true;

    n = gps_m10_build_fast_config(s_steps, GPS_CFG_STEP_MAX, GPS_NAV_PERIOD_MS,
                                  GPS_SINGLE_GNSS_25HZ != 0);
    if (n == 0U)
    {
        PRINTF("GPS: could not build config frames (bad key size, or a sub-100 ms\r\n"
               "     period requested without GPS_SINGLE_GNSS_25HZ)\r\n");
        return false;
    }

    PRINTF("GPS: configuring for %u ms nav period (%u Hz), NAV-PVT only, %s\r\n",
           (unsigned int)GPS_NAV_PERIOD_MS, (unsigned int)(1000U / GPS_NAV_PERIOD_MS),
           (GPS_SINGLE_GNSS_25HZ != 0) ? "GPS/SBAS/QZSS only" : "multi-GNSS");

    for (uint32_t i = 0U; i < n; i++)
    {
        status_t status;
        bool     answered = false;

        gps_ubx_expect_ack(&gps->ubx, UBX_CLASS_CFG, UBX_ID_VALSET);

        status = gps_send_frame(s_steps[i].frame, s_steps[i].len);
        if (status != kStatus_Success)
        {
            PRINTF("GPS:   %-9s send failed (%d)\r\n", s_steps[i].name, (int)status);
            all_ok = false;
            continue;
        }

        for (uint32_t waited = 0U; waited < GPS_ACK_TIMEOUT_MS; waited += GPS_POLL_MS)
        {
            gps_drain(gps);
            if (!gps_ubx_cfg_pending(&gps->ubx))
            {
                answered = true;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
        }

        if (!answered)
        {
            /* No ACK at all is a different fault from a NAK: the frame never
             * reached the receiver, so suspect TX wiring or the baud rate
             * before suspecting the keys. */
            PRINTF("GPS:   %-9s no ACK within %u ms (TX wiring? baud?)\r\n",
                   s_steps[i].name, (unsigned int)GPS_ACK_TIMEOUT_MS);
            all_ok = false;
        }
        else if (!gps->ubx.ack_ok)
        {
            PRINTF("GPS:   %-9s NAK - key rejected, verify the UBX_KEY_* ids\r\n",
                   s_steps[i].name);
            all_ok = false;
        }
        else
        {
            PRINTF("GPS:   %-9s ACK (%u bytes)\r\n", s_steps[i].name,
                   (unsigned int)s_steps[i].len);

            if (s_steps[i].settle_ms != 0U)
            {
                /* The constellation change restarts the GNSS subsystem. Keep
                 * draining while it re-acquires so the parser stays in sync
                 * with whatever the receiver emits across the restart. */
                PRINTF("GPS:   %-9s settling %u ms\r\n", s_steps[i].name,
                       (unsigned int)s_steps[i].settle_ms);
                for (uint32_t waited = 0U; waited < s_steps[i].settle_ms; waited += GPS_POLL_MS)
                {
                    gps_drain(gps);
                    vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
                }
            }
        }
    }

    return all_ok;
}

/*!
 * @brief Publish the current solution for the VTX stream, if it changed.
 *
 * Called by gps_task after every drain. Cheap when nothing new has arrived,
 * which is most calls - the GPS produces 10 solutions a second and this runs
 * on every 64-byte receive chunk.
 */
static void gps_publish(const gps_m10_t *gps)
{
    static uint32_t       last;
    const gps_m10_data_t *d       = &gps->data;
    uint32_t              updates = d->pvt_count + d->gga_count;
    gps_snapshot_t        snap;

    if (updates == last)
    {
        return;
    }
    last = updates;

    snap.updates = updates;
    snap.sats    = d->satellites;
    snap.lat_1e7 = d->lat_1e7;
    snap.lon_1e7 = d->lon_1e7;
    snap.alt_mm  = d->alt_mm;

    if (d->from_ubx)
    {
        /*
         * Read the UBX fix type directly: the shared struct folds it into
         * GGA's quality codes, which lose the 2-D / 3-D distinction a GCS
         * displays. gnssFixOK is what makes a solution usable at all.
         */
        uint8_t ft = gps->ubx.pvt.fix_type;

        if (!gps->ubx.pvt.fix_ok)
        {
            snap.fix_type = GPS_FIX_TYPE_NO_FIX;
        }
        else if (ft == (uint8_t)kUbxFix2D)
        {
            snap.fix_type = GPS_FIX_TYPE_2D_FIX;
        }
        else if ((ft == (uint8_t)kUbxFix3D) || (ft == (uint8_t)kUbxFixGnssDeadRe))
        {
            snap.fix_type = GPS_FIX_TYPE_3D_FIX;
        }
        else
        {
            snap.fix_type = GPS_FIX_TYPE_NO_FIX; /* dead reckoning or time only */
        }

        /*
         * NAV-PVT has PDOP but no HDOP, and GPS_RAW_INT wants HDOP. PDOP is
         * always >= HDOP, so this over-states the uncertainty rather than
         * under-stating it. Not hdop_1e2: that came from GGA, and with NMEA
         * switched off it is frozen at whatever the last GGA before the
         * configuration said.
         */
        snap.eph = ((d->pdop_1e2 != 0U) && (d->pdop_1e2 < MAVLINK_U16_UNKNOWN))
                       ? (uint16_t)d->pdop_1e2 : (uint16_t)MAVLINK_U16_UNKNOWN;
    }
    else
    {
        snap.fix_type = !d->position_valid ? GPS_FIX_TYPE_NO_FIX
                        : (d->fix_quality == (uint8_t)kGpsFixDgnss) ? GPS_FIX_TYPE_DGPS
                                                                     : GPS_FIX_TYPE_3D_FIX;
        snap.eph = ((d->hdop_1e2 != 0U) && (d->hdop_1e2 < MAVLINK_U16_UNKNOWN))
                       ? (uint16_t)d->hdop_1e2 : (uint16_t)MAVLINK_U16_UNKNOWN;
    }

    {
        uint32_t v = d->speed_mm_s / 10U;

        snap.vel_cm_s = (v > 65534U) ? (uint16_t)65534U : (uint16_t)v;
    }

    /*
     * Below about half a metre a second a GNSS course over ground is noise -
     * on the bench it wanders through the whole compass. MAVLink has a value
     * for "unknown", so use it rather than publish a confident-looking
     * random heading.
     */
    if ((d->speed_mm_s < 500U) || (d->course_1e3 >= 360000U))
    {
        snap.cog_cdeg = (uint16_t)MAVLINK_U16_UNKNOWN;
    }
    else
    {
        snap.cog_cdeg = (uint16_t)(d->course_1e3 / 10U);
    }

    taskENTER_CRITICAL();
    s_gps_snap = snap;
    taskEXIT_CRITICAL();
}

/*!
 * @brief Drives the u-blox M10 on LPUART3: finds the baud rate, configures it
 *        for fast position-only output, then reports the fix.
 *
 * Three stages:
 *
 *  1. Find the line rate. The receiver's configured baud is the one thing that
 *     cannot be read off the wire - an undersampled stream loses the
 *     information - so the rates are swept until something decodes. Either
 *     format counts: a module straight from the factory streams NMEA, but one
 *     already configured by a previous run of this firmware emits UBX only, and
 *     locking solely on NMEA would fail on a warm restart.
 *
 *  2. Configure. UBX-CFG-VALSET sets the navigation period, enables NAV-PVT and
 *     silences the NMEA sentences nothing here decodes. Written to RAM only, so
 *     a power cycle restores the factory settings.
 *
 *  3. Report. NAV-PVT carries position, velocity, time and the accuracy
 *     estimates in one frame, so there is no waiting for a second sentence to
 *     pair position with velocity.
 *
 * The receive is a ring of eDMA chunks that the task drains partially - see
 * gps_drain() for why draining only completed chunks would add a whole
 * navigation period of latency.
 */
static void gps_task(void *pvParameters)
{
    /* Static: gps_m10_t carries an 84-byte NMEA line buffer, a 96-byte UBX
     * payload buffer and the decoded state, which is more than this task's
     * stack wants to hold. */
    static gps_m10_t gps;
    status_t   status;
    TickType_t next_report;
    uint32_t   last_updates = 0U;
    uint32_t   last_pvt     = 0U;

    (void)pvParameters;

    gps_m10_init(&gps);
    s_gps_task_handle = xTaskGetCurrentTaskHandle();

    uart_get_default_gps_config(&gps_uart, gps_uart_callback);

    status = uart_init(&gps_uart);
    if (status != kStatus_Success)
    {
        PRINTF("GPS: uart_init failed (%d)\r\n", (int)status);
        vTaskDelete(NULL);
    }

    PRINTF("GPS: LPUART3 %s, probing baud rates\r\n",
           gps_uart.enable_dma ? "eDMA (DMA4 ch6/ch7)" : "polled");

    /*
     * Sweep the plausible line rates until sentences decode. The receiver's
     * configured rate is the one thing we cannot read off the wire - an
     * undersampled stream loses the information - so measuring beats guessing.
     * Ordered by how likely each is on a module that has seen a flight stack.
     */
    static const uint32_t k_bauds[] = {115200U, 38400U, 9600U, 57600U, 230400U};
    bool locked = false;

    for (uint32_t attempt = 0U; (attempt < ARRAY_SIZE(k_bauds)) && !locked; attempt++)
    {
        bool dumped = false;

        status = uart_set_baudrate(&gps_uart, k_bauds[attempt]);
        if (status != kStatus_Success)
        {
            PRINTF("GPS: set baud %u failed (%d)\r\n", (unsigned int)k_bauds[attempt], (int)status);
            continue;
        }

        gps_m10_init(&gps);
        s_gps_head    = 0U;
        s_gps_tail    = 0U;
        s_gps_partial = 0U; /* or the first chunk of this attempt is fed from
                             * the previous attempt's offset */

        status = uart_read_dma(&gps_uart, s_gps_rx[s_gps_head], GPS_RX_CHUNK_BYTES);
        if (status != kStatus_Success)
        {
            PRINTF("GPS: arm failed at %u baud (%d)\r\n", (unsigned int)k_bauds[attempt], (int)status);
            continue;
        }

        for (uint32_t waited = 0U; (waited < GPS_BAUD_TRY_MS) && !locked; waited += GPS_POLL_MS)
        {
            if (s_gps_tail != s_gps_head)
            {
                if (!dumped)
                {
                    dumped = true;
                    PRINTF("GPS: %6u baud, first chunk: \"", (unsigned int)k_bauds[attempt]);
                    for (uint32_t i = 0U; i < GPS_RX_CHUNK_BYTES; i++)
                    {
                        uint8_t c = s_gps_rx[s_gps_tail][i];
                        PRINTF("%c", ((c >= 0x20U) && (c < 0x7FU)) ? (char)c : '.');
                    }
                    PRINTF("\"\r\n");
                }

                gps_drain(&gps);

                /* Two good messages, not one, so a lucky checksum on garbage
                 * does not lock in the wrong rate. A UBX frame counts as well
                 * as an NMEA sentence: an already-configured module emits
                 * nothing but UBX. */
                if ((gps.data.sentences_ok >= 2U) || (gps.data.ubx_frames_ok >= 2U))
                {
                    locked = true;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(GPS_POLL_MS));
        }

        PRINTF("GPS: %6u baud -> %u nmea, %u ubx, %u bytes, %u csum_err%s\r\n",
               (unsigned int)k_bauds[attempt],
               (unsigned int)gps.data.sentences_ok,
               (unsigned int)gps.data.ubx_frames_ok,
               (unsigned int)gps.data.bytes_seen,
               (unsigned int)(gps.data.checksum_errors + gps.data.ubx_csum_errors),
               locked ? "   <-- locked" : "");
    }

    if (!locked)
    {
        PRINTF("GPS: no rate decoded; continuing at %u baud\r\n", (unsigned int)gps_uart.baudrate);
    }

    /*
     * Configure even if the sweep never locked. If the receiver is actually
     * there and only the decode failed, the ACK for the first VALSET proves it
     * both ways round - and if it is not there, "no ACK" says so plainly.
     */
    if (!gps_configure(&gps))
    {
        PRINTF("GPS: configuration incomplete - the receiver keeps its previous\r\n"
               "     output settings, so NMEA decoding below may still work\r\n");
    }

    next_report = xTaskGetTickCount() + pdMS_TO_TICKS(GPS_PRINT_RATE_MS);

    while (1)
    {
        /* Woken by the ISR on every chunk; the timeout only keeps the periodic
         * report running when the receiver goes quiet. */
        (void)ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(GPS_POLL_MS));

        gps_drain(&gps);
        gps_publish(&gps);

        if ((int32_t)(xTaskGetTickCount() - next_report) >= 0)
        {
            next_report += pdMS_TO_TICKS(GPS_PRINT_RATE_MS);
            {
            const gps_m10_data_t *d = gps_m10_data(&gps);
            /* Both formats count as an update, so the stall check does not
             * fire just because NMEA has been switched off. */
            uint32_t updates = d->sentences_ok + d->pvt_count;

            if (updates == last_updates)
            {
                /* Nothing decoded since the last report. bytes_seen separates
                 * "no wiring" from "bytes arriving but not valid NMEA" - the
                 * latter is almost always the wrong baud rate. */
                /*
                 * The LPUART's own error flags are the decisive clue: framing
                 * or noise errors mean bytes are arriving at the wrong baud
                 * rate, overrun means we are not draining fast enough.
                 */
                uint32_t uf = LPUART_GetStatusFlags(GPS_LPUART_BASEADDR);

                PRINTF("GPS: no messages (bytes=%u csum_err=%u/%u chunks=%u rearm_fail=%u"
                       " | OR=%u FE=%u NE=%u PE=%u)\r\n",
                       (unsigned int)d->bytes_seen, (unsigned int)d->checksum_errors,
                       (unsigned int)d->ubx_csum_errors,
                       (unsigned int)s_gps_chunks, (unsigned int)s_gps_rearm_fail,
                       (unsigned int)((uf & (uint32_t)kLPUART_RxOverrunFlag) != 0U),
                       (unsigned int)((uf & (uint32_t)kLPUART_FramingErrorFlag) != 0U),
                       (unsigned int)((uf & (uint32_t)kLPUART_NoiseErrorFlag) != 0U),
                       (unsigned int)((uf & (uint32_t)kLPUART_ParityErrorFlag) != 0U));
            }
            else
            {
                int32_t  lat = d->lat_1e7;
                int32_t  lon = d->lon_1e7;
                uint32_t la  = (uint32_t)((lat < 0) ? -lat : lat);
                uint32_t lo  = (uint32_t)((lon < 0) ? -lon : lon);

                /* NAV-PVT frames since the last report, which with a 1 s
                 * report period is the achieved navigation rate in Hz - the
                 * number that says whether the configuration actually took. */
                unsigned int rate_hz = (unsigned int)(d->pvt_count - last_pvt);

#if GPS_PRINT_FIX
                PRINTF("GPS %04u-%02u-%02u %02u:%02u:%02u fix=%u sats=%u %s  %s %u Hz"
                       "  [pvt=%u gga=%u rmc=%u csum=%u/%u ovf=%u]\r\n",
                       (unsigned int)d->year, d->month, d->day,
                       d->hour, d->minute, d->second,
                       d->fix_quality, d->satellites,
                       d->position_valid ? "VALID" : "no-fix",
                       d->from_ubx ? "UBX" : "NMEA", rate_hz,
                       (unsigned int)d->pvt_count,
                       (unsigned int)d->gga_count, (unsigned int)d->rmc_count,
                       (unsigned int)d->checksum_errors,
                       (unsigned int)d->ubx_csum_errors,
                       (unsigned int)s_gps_ring_overflow);
                PRINTF("    lat %c%u.%07u  lon %c%u.%07u  alt %d mm  spd %u mm/s\r\n",
                       (lat < 0) ? '-' : '+', (unsigned int)(la / 10000000U), (unsigned int)(la % 10000000U),
                       (lon < 0) ? '-' : '+', (unsigned int)(lo / 10000000U), (unsigned int)(lo % 10000000U),
                       (int)d->alt_mm, (unsigned int)d->speed_mm_s);
                if (d->from_ubx)
                {
                    /* NMEA carries no accuracy estimate, so this line only
                     * means anything once NAV-PVT is the source. */
                    PRINTF("    hAcc %u mm  vAcc %u mm  pDOP %u.%02u\r\n",
                           (unsigned int)d->h_acc_mm, (unsigned int)d->v_acc_mm,
                           (unsigned int)(d->pdop_1e2 / 100U),
                           (unsigned int)(d->pdop_1e2 % 100U));
                }
#else
                (void)rate_hz;
                (void)lat; (void)lon; (void)la; (void)lo;
#endif
            }

            last_updates = updates;
            last_pvt     = d->pvt_count;
            }
        }
    }
}

/*!
 * @brief eDMA completion callback for the VTX LPUART. Runs in interrupt context.
 */
static void vtx_uart_callback(LPUART_Type *base, lpuart_edma_handle_t *handle, status_t status, void *userData)
{
    (void)base;
    (void)handle;
    (void)userData;

    if (status == kStatus_LPUART_RxIdle)
    {
        s_vtx_rx_done = true;
        return;
    }

    if (status == kStatus_LPUART_TxIdle)
    {
        /*
         * Only frames this queue put on the wire count. The startup banner is
         * sent through the same port with uart_write(), and without this guard
         * its completion advanced s_vtx_tx_tail past a slot that was never
         * filled - which both inflated the frame count and put one slot of
         * stale bytes on the link. Visible as "sent" exceeding "reads".
         */
        if (!s_vtx_tx_busy)
        {
            return;
        }

        s_vtx_frames_sent++;
        s_vtx_bytes_sent += s_vtx_tx_len[s_vtx_tx_tail]; /* this slot's real length */

        s_vtx_tx_tail = (uint8_t)((s_vtx_tx_tail + 1U) % VTX_TX_SLOTS);

        /*
         * Chain straight into the next frame from here. Safe for the same
         * reason the GPS receive re-arm is: LPUART_TransferEdmaHandleIRQ()
         * sets txState back to idle before invoking this callback, so the
         * handle will accept a new transfer. Doing it here rather than in the
         * task is what keeps the stream gap-free - a task round trip would
         * cost a 5 ms tick between every frame.
         *
         * No FreeRTOS call in this path, so nothing here depends on the NVIC
         * priority - though uart_init() pins it to 5 regardless.
         */
        if (s_vtx_tx_tail != s_vtx_tx_head)
        {
            /*
             * The slot's own length, never a constant. This line once sent a
             * fixed 24 bytes - a leftover from the raw-frame format - so every
             * frame that went out by chaining rather than by vtx_tx_kick() was
             * cut short. GPS_RAW_INT lost its last 18 bytes, checksum included,
             * and mavfwd on the VTX discarded it without trace. A GPS frame is
             * chained whenever it lands in the same loop pass as a SCALED_IMU,
             * one time in four, which was all of the 4.1% telemetry loss.
             */
            if (uart_write_bytes(&vtx_uart, s_vtx_tx[s_vtx_tx_tail],
                                 s_vtx_tx_len[s_vtx_tx_tail]) != kStatus_Success)
            {
                s_vtx_tx_errors++;
                s_vtx_tx_busy = false;
            }
        }
        else
        {
            s_vtx_tx_busy = false;
        }
    }
}
