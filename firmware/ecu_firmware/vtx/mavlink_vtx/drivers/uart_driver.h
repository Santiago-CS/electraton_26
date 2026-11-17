/*
 * uart_driver.h - LPUART transport with optional eDMA, i.MX93 Cortex-M33.
 *
 * Ported from the MCXN947 version. The i.MX93 has no LP_FLEXCOMM wrapper:
 * LPUART1..LPUART8 are standalone blocks with their own clock roots, gates and
 * interrupt vectors, so instance 1 is LPUART1 and there is no LPUART0.
 *
 * Which eDMA you may use is fixed by where the port lives:
 *   LPUART1, LPUART2            -> AONMIX    -> DMA3 (kDma3RequestMux*)
 *   LPUART3 .. LPUART8          -> WAKEUPMIX -> DMA4 (kDma4RequestMux*)
 * There is no path from DMA3 to a WAKEUPMIX LPUART, so the VTX port on
 * LPUART6 has to use DMA4.
 *
 * Author: Diego
 * Created on: 6, April 2026
 */

#ifndef UART_DRIVER_H_
#define UART_DRIVER_H_

#include <stdbool.h>
#include <stdint.h>

#include "fsl_clock.h"
#include "fsl_edma.h"
#include "fsl_lpuart.h"
#include "fsl_lpuart_edma.h"

/* ANSI Color Codes */
#define ANSI_RESET   "\x1b[0m"
#define ANSI_RED     "\x1b[31m"
#define ANSI_GREEN   "\x1b[32m"
#define ANSI_YELLOW  "\x1b[33m"
#define ANSI_BLUE    "\x1b[34m"
#define ANSI_MAGENTA "\x1b[35m"
#define ANSI_CYAN    "\x1b[36m"
#define ANSI_WHITE   "\x1b[37m"

/* ANSI Style Codes */
#define ANSI_BOLD    "\x1b[1m"
#define ANSI_CLS     "\x1b[2J\x1b[H"

/* The telemetry port: LPUART6 on GPIO_IO04/IO05, so DMA4. */
#define VTX_LPUART_BASEADDR        LPUART6
#define VTX_LPUART_DMA_BASEADDR    DMA4
#define VTX_LPUART_TX_DMA_REQUEST  kDma4RequestMuxLPUART6Tx
#define VTX_LPUART_RX_DMA_REQUEST  kDma4RequestMuxLPUART6Rx
#define VTX_LPUART_TX_DMA_CHANNEL  0U
#define VTX_LPUART_RX_DMA_CHANNEL  1U
#define VTX_UART_BAUDRATE          115200U

/* ---------------------------------------------------------------------------
 *  The GPS port: LPUART3 on GPIO_IO14/IO15 (ALT1), so DMA4 as well.
 *
 *  Matches the sensor board, which carries UART3_TX on header pin 8 and
 *  UART3_RX on pin 10 through to the SM10B connector and the u-blox M10. On
 *  the FRDM both pads go straight from the SoC to P11:
 *
 *    GPIO_IO14  ball P20  P11 pin 8   ALT1  LPUART3_TX  (host out -> GPS in)
 *    GPIO_IO15  ball P21  P11 pin 10  ALT1  LPUART3_RX  (GPS out -> host in)
 *
 *  eDMA channels 6 and 7: 0/1 are the VTX LPUART, 2/3 the IMU LPSPI, 4/5 the
 *  magnetometer LPI2C.
 * ------------------------------------------------------------------------- */
#define GPS_LPUART_BASEADDR        LPUART3
#define GPS_LPUART_DMA_BASEADDR    DMA4
#define GPS_LPUART_TX_DMA_REQUEST  kDma4RequestMuxLPUART3Tx
#define GPS_LPUART_RX_DMA_REQUEST  kDma4RequestMuxLPUART3Rx
#define GPS_LPUART_TX_DMA_CHANNEL  6U
#define GPS_LPUART_RX_DMA_CHANNEL  7U
/*
 * u-blox M10 leaves the factory at 38400, but a module that has been
 * configured (or came off a flight-controller stack) is commonly 115200.
 * gps_task sweeps the plausible rates at startup, so this is only the
 * first one it tries.
 */
#define GPS_UART_BAUDRATE          115200U

/* Hook run at the head of the port's ISR, before the SDK dispatcher. */
typedef void (*uart_irq_callback_t)(void);

typedef struct uart_s {
    LPUART_Type *uart_base;
    EDMA_Type   *dma_base;       /*!< DMA3 or DMA4; unused when enable_dma is false */
    uint32_t     tx_dma_channel; /*!< eDMA channel index, not the request source */
    uint32_t     rx_dma_channel;
    uint32_t     tx_dma_request; /*!< kDma4RequestMuxLPUART6Tx & co. */
    uint32_t     rx_dma_request;
    lpuart_edma_transfer_callback_t callback;
    uint32_t     baudrate;
    bool         enable_dma;
} uart_ctrl_t;

/* Fill ctrl with the VTX_* defaults above. */
void uart_get_default_vtx_config(uart_ctrl_t *ctrl, lpuart_edma_transfer_callback_t callback);

/* Fill ctrl with the GPS_* defaults above. */
void uart_get_default_gps_config(uart_ctrl_t *ctrl, lpuart_edma_transfer_callback_t callback);

/*
 * Clock, configure and open the port, and when enable_dma is set bring up both
 * eDMA channels. Returns kStatus_InvalidArgument for a base this driver does
 * not know, or a DMA base that cannot reach it.
 */
status_t uart_init(uart_ctrl_t *ctrl);

/*
 * Queue a NUL-terminated string. Non-blocking; returns kStatus_LPUART_TxBusy if
 * the previous send has not finished. In DMA mode the buffer must live in
 * .data/.bss or shared RAM - a string literal is in code TCM, which the eDMA
 * cannot read, and is rejected with kStatus_InvalidArgument.
 */
status_t uart_write(uart_ctrl_t *ctrl, const char *string);

/*
 * Queue an explicit byte count. Same rules as uart_write(), but safe for binary
 * payloads: a UBX frame carries length and checksum bytes that are routinely
 * 0x00, so strlen() would truncate it at the first one.
 *
 * Non-blocking. The buffer must stay valid until the transfer completes, which
 * the port's callback reports as kStatus_LPUART_TxIdle.
 */
status_t uart_write_bytes(uart_ctrl_t *ctrl, const uint8_t *data, uint32_t size);

/* Arm an eDMA receive of exactly size bytes. The callback fires with
 * kStatus_LPUART_RxIdle when it completes. */
status_t uart_read_dma(uart_ctrl_t *ctrl, uint8_t *data, uint32_t size);

/*
 * How many bytes of the armed eDMA receive have landed so far.
 *
 * uart_read_dma() only calls back once the whole block has arrived, which
 * strands the tail of a burst until the next burst tops the block up - a full
 * navigation period of added latency when a GNSS receiver sends one frame per
 * fix. Reading the progress count lets a caller consume a partial block instead
 * of waiting for it to fill.
 *
 * Returns kStatus_NoTransferInProgress when no receive is armed.
 */
status_t uart_rx_dma_count(uart_ctrl_t *ctrl, uint32_t *count);

/*
 * Abort an in-flight eDMA RX transfer. Used by error-recovery paths (e.g. when
 * a telemetry response never arrives within its expected window). The caller is
 * responsible for re-arming with uart_read_dma() when it wants to receive again.
 */
status_t uart_abort_rx_dma(uart_ctrl_t *ctrl);

/*!
 * @brief Change the line rate on an already-initialised port.
 *
 * Aborts any in-flight eDMA receive, reprograms the divider and clears the
 * receiver's error flags - a baud change mid-stream otherwise leaves a framing
 * error latched from the old rate. The caller re-arms the receive.
 */
status_t uart_set_baudrate(uart_ctrl_t *ctrl, uint32_t baudrate);

/* Install a hook called at the top of this port's ISR. */
status_t uart_attach_interrupt(LPUART_Type *uart_base, uart_irq_callback_t callback);

#endif /* UART_DRIVER_H_ */
