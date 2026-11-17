/*
 * lptmr_driver.h - periodic LPTMR timer with a user callback.
 *
 *  Created on: Nov 22, 2025
 *      Author: diego
 */

#ifndef LPTMR_DRIVER_H_
#define LPTMR_DRIVER_H_

#include "fsl_common.h"
#include "fsl_lptmr.h"

/*
 * Rate the counter itself ticks at: the 32 kHz source selected by PSR[PCS] =
 * kLPTMR_PrescalerClock_1, with the prescaler bypassed. This is not the CCM
 * root rate - the root only feeds the peripheral's register bus.
 */
#define LPTMR_COUNTER_CLOCK_HZ 32000U

/* Invoked from interrupt context on every compare match. */
typedef void (*timer_callback_t)(void *args);

typedef struct timer_s {
    LPTMR_Type *lptmr_base; /*!< LPTMR1 or LPTMR2 */
    uint32_t    frequency;  /*!< desired compare-match rate in Hz, 1..LPTMR_COUNTER_CLOCK_HZ */
    void       *user_ctx;   /*!< handed back to the callback; may be NULL */
} timer_ctrl_t;

/*
 * Bring up the port's clock, configure it for periodic compare and program the
 * period from timer->frequency. Returns kStatus_InvalidArgument for an unknown
 * base or a frequency of 0 / above LPTMR_COUNTER_CLOCK_HZ.
 */
status_t timer_init(timer_ctrl_t *timer);

/* Install the callback and enable the interrupt. Safe to call before or after
 * timer_start(). */
status_t timer_attach_callback(timer_ctrl_t *timer, timer_callback_t callback);

status_t timer_start(timer_ctrl_t *timer);
status_t timer_stop(timer_ctrl_t *timer);

#endif /* LPTMR_DRIVER_H_ */
