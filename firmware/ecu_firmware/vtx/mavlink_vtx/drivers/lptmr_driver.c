/*
 * lptmr_driver.c - periodic LPTMR timer with a user callback.
 * See lptmr_driver.h for the API.
 *
 *  Created on: Nov 22, 2025
 *      Author: diego
 */

#include "lptmr_driver.h"

/* i.MX93 has two instances; index 0 is LPTMR1. */
#define TIMER_INSTANCE_COUNT 2U

/*
 * NVIC priority. It must be numerically greater than or equal to
 * configMAX_SYSCALL_INTERRUPT_PRIORITY (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY
 * is 2 on this board) so the callback may use the FreeRTOS ...FromISR() API.
 */
#define TIMER_IRQ_PRIORITY 5U

typedef struct {
    LPTMR_Type     *base;
    IRQn_Type       irq;
    clock_root_t    clock_root;
    clock_ip_name_t clock_gate;
} timer_hw_t;

static const timer_hw_t s_timer_hw[TIMER_INSTANCE_COUNT] = {
    {LPTMR1, LPTMR1_IRQn, kCLOCK_Root_Lptmr1, kCLOCK_Lptmr1},
    {LPTMR2, LPTMR2_IRQn, kCLOCK_Root_Lptmr2, kCLOCK_Lptmr2},
};

/* Written by timer_attach_callback(), read by the ISR. */
static timer_callback_t volatile s_callback[TIMER_INSTANCE_COUNT];
static void *volatile            s_callback_ctx[TIMER_INSTANCE_COUNT];

static status_t timer_get_instance(const timer_ctrl_t *timer, uint32_t *instance)
{
    if ((timer == NULL) || (instance == NULL)) {
        return kStatus_InvalidArgument;
    }

    for (uint32_t i = 0U; i < TIMER_INSTANCE_COUNT; i++) {
        if (s_timer_hw[i].base == timer->lptmr_base) {
            *instance = i;
            return kStatus_Success;
        }
    }

    return kStatus_InvalidArgument;
}

status_t timer_init(timer_ctrl_t *timer)
{
    lptmr_config_t lptmrConfig;
    uint32_t       instance;
    uint32_t       period_ticks;
    status_t       status;

    status = timer_get_instance(timer, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    /*
     * One tick is the shortest period the hardware can express, and
     * LPTMR_SetTimerPeriod() programs CMR = ticks - 1, so a frequency that
     * divides down to 0 ticks would arm a full-range compare (hours) instead of
     * failing visibly.
     */
    if ((timer->frequency == 0U) || (timer->frequency > LPTMR_COUNTER_CLOCK_HZ)) {
        return kStatus_InvalidArgument;
    }

    /*
     * BOARD_BootClockRUN() on this board only initialises the PLLs - it leaves
     * every peripheral root and gate alone - and LPTMR_Init() ungates nothing
     * because i.MX93 does not define LPTMR_CLOCKS. Without this the first LPTMR
     * register write stalls on the bus and the core never returns.
     *
     * The root feeds the register interface only; the counter runs from the
     * 32 kHz source picked below via PSR[PCS].
     */
    const clock_root_config_t rootCfg = {
        .clockOff = false,
        .mux      = 0, /* Osc24M */
        .div      = 1,
    };
    CLOCK_SetRootClock(s_timer_hw[instance].clock_root, &rootCfg);
    CLOCK_EnableClock(s_timer_hw[instance].clock_gate);

    /* Defaults first, so a field added by a future SDK is still initialised. */
    LPTMR_GetDefaultConfig(&lptmrConfig);
    lptmrConfig.timerMode            = kLPTMR_TimerModeTimeCounter;
    lptmrConfig.enableFreeRunning    = false; /* counter resets on compare match */
    lptmrConfig.bypassPrescaler      = true;
    lptmrConfig.prescalerClockSource = kLPTMR_PrescalerClock_1; /* 32 kHz */
    lptmrConfig.value                = kLPTMR_Prescale_Glitch_0;

    LPTMR_Init(timer->lptmr_base, &lptmrConfig);

    /* Integer division: the achieved rate is LPTMR_COUNTER_CLOCK_HZ /
     * period_ticks, which only equals timer->frequency when it divides 32000. */
    period_ticks = LPTMR_COUNTER_CLOCK_HZ / timer->frequency;
    LPTMR_SetTimerPeriod(timer->lptmr_base, period_ticks);

    return kStatus_Success;
}

status_t timer_attach_callback(timer_ctrl_t *timer, timer_callback_t callback)
{
    uint32_t instance;
    status_t status;

    status = timer_get_instance(timer, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    /* Publish the callback before anything can dispatch it. */
    s_callback_ctx[instance] = timer->user_ctx;
    s_callback[instance]     = callback;
    __DMB();

    /* A compare match left pending by an earlier run would fire the moment the
     * IRQ is enabled, so drop it first. */
    LPTMR_ClearStatusFlags(timer->lptmr_base, kLPTMR_TimerCompareFlag);

    NVIC_SetPriority(s_timer_hw[instance].irq, TIMER_IRQ_PRIORITY);
    LPTMR_EnableInterrupts(timer->lptmr_base, kLPTMR_TimerInterruptEnable);
    EnableIRQ(s_timer_hw[instance].irq);

    return kStatus_Success;
}

status_t timer_start(timer_ctrl_t *timer)
{
    uint32_t instance;
    status_t status;

    status = timer_get_instance(timer, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    LPTMR_StartTimer(timer->lptmr_base);
    return kStatus_Success;
}

status_t timer_stop(timer_ctrl_t *timer)
{
    uint32_t instance;
    status_t status;

    status = timer_get_instance(timer, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    LPTMR_StopTimer(timer->lptmr_base);

    /* Drop any match that landed as we stopped, otherwise the next
     * timer_start() takes an immediate spurious interrupt. TIE is left on so
     * start/stop stay symmetric and the callback survives a stop. */
    LPTMR_ClearStatusFlags(timer->lptmr_base, kLPTMR_TimerCompareFlag);

    return kStatus_Success;
}

static void timer_irq_handler(uint32_t instance)
{
    timer_callback_t callback;

    LPTMR_ClearStatusFlags(s_timer_hw[instance].base, kLPTMR_TimerCompareFlag);

    callback = s_callback[instance];
    if (callback != NULL) {
        callback(s_callback_ctx[instance]);
    }

    /* The write buffer has to drain before we leave, or the flag clear can land
     * after the handler returns and the interrupt is re-entered. */
    __DSB();
    __ISB();
}

void LPTMR1_IRQHandler(void)
{
    timer_irq_handler(0U);
}

void LPTMR2_IRQHandler(void)
{
    timer_irq_handler(1U);
}
