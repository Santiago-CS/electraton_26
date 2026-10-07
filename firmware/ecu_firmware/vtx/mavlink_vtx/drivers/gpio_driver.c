/*
 * gpio_driver.c - RGPIO driver for the i.MX93 Cortex-M33.
 * See gpio_driver.h for the API and what differs from the MCXN947 version.
 *
 * Author: Diego
 * Created on: 10, April 2026
 */

#include <string.h>

#include "gpio_driver.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

#define GPIO_INSTANCE_COUNT 4U  /* GPIO1..GPIO4; index 0 is GPIO1 */
#define GPIO_PINS_PER_PORT  32U
#define GPIO_IRQ_SEL_COUNT  2U  /* this part brings out two lines per instance */

/*
 * NVIC priority. Numerically >= configMAX_SYSCALL_INTERRUPT_PRIORITY
 * (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY is 2 here) so a callback may
 * use the FreeRTOS ...FromISR() API.
 */
#define GPIO_IRQ_PRIORITY 5U

typedef struct {
    RGPIO_Type     *base;
    clock_ip_name_t clock_gate;
    IRQn_Type       irq[GPIO_IRQ_SEL_COUNT];
} gpio_hw_t;

static const gpio_hw_t s_gpio_hw[GPIO_INSTANCE_COUNT] = {
    {GPIO1, kCLOCK_Gpio1, {GPIO1_0_IRQn, GPIO1_1_IRQn}},
    {GPIO2, kCLOCK_Gpio2, {GPIO2_0_IRQn, GPIO2_1_IRQn}},
    {GPIO3, kCLOCK_Gpio3, {GPIO3_0_IRQn, GPIO3_1_IRQn}},
    {GPIO4, kCLOCK_Gpio4, {GPIO4_0_IRQn, GPIO4_1_IRQn}},
};

/*
 * One callback per pin, not one per port. The MCXN947 version stored a single
 * handler for a whole instance and installed it on both lines, so two pins on
 * the same port could not have different handlers.
 */
static gpio_callback_t volatile s_callback[GPIO_INSTANCE_COUNT][GPIO_PINS_PER_PORT];
static void *volatile            s_callback_ctx[GPIO_INSTANCE_COUNT][GPIO_PINS_PER_PORT];

/*******************************************************************************
 * Helpers
 ******************************************************************************/

static status_t gpio_get_instance(const gpio_ctrl_t *gpio, uint32_t *instance)
{
    if ((gpio == NULL) || (instance == NULL) || (gpio->pin >= GPIO_PINS_PER_PORT)) {
        return kStatus_InvalidArgument;
    }

    for (uint32_t i = 0U; i < GPIO_INSTANCE_COUNT; i++) {
        if (s_gpio_hw[i].base == gpio->gpio_base) {
            *instance = i;
            return kStatus_Success;
        }
    }

    return kStatus_InvalidArgument;
}

/*******************************************************************************
 * Functions
 ******************************************************************************/

status_t gpio_init(gpio_ctrl_t *gpio)
{
    rgpio_pin_config_t pinConfig;
    uint32_t           instance;
    status_t           status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    /* GPIO has no CCM root of its own on this part, only an LPCG gate. */
    CLOCK_EnableClock(s_gpio_hw[instance].clock_gate);

    pinConfig.pinDirection = (gpio->dir == gpio_output) ? kRGPIO_DigitalOutput : kRGPIO_DigitalInput;
    pinConfig.outputLogic  = gpio->output_default;

    RGPIO_PinInit(gpio->gpio_base, gpio->pin, &pinConfig);

    /*
     * PIDR gates the input buffer and comes up with every pin disabled, and
     * RGPIO_PinInit() does not touch it - so without this an input reads
     * nothing at all. Enabled for outputs too, so the level can be read back.
     */
    RGPIO_EnablePortInput(gpio->gpio_base, 1UL << gpio->pin, true);

    return kStatus_Success;
}

status_t gpio_set_output(gpio_ctrl_t *gpio, uint8_t val)
{
    uint32_t instance;
    status_t status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    RGPIO_PinWrite(gpio->gpio_base, gpio->pin, val);
    return kStatus_Success;
}

uint32_t gpio_read_input(gpio_ctrl_t *gpio)
{
    uint32_t instance;

    if (gpio_get_instance(gpio, &instance) != kStatus_Success) {
        return 0U;
    }

    return RGPIO_PinRead(gpio->gpio_base, gpio->pin);
}

status_t gpio_toggle_output(gpio_ctrl_t *gpio)
{
    uint32_t instance;
    status_t status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    RGPIO_PortToggle(gpio->gpio_base, 1UL << gpio->pin);
    return kStatus_Success;
}

status_t gpio_clear_interrupt_flag(gpio_ctrl_t *gpio)
{
    uint32_t instance;
    status_t status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    RGPIO_ClearPinsInterruptFlags(gpio->gpio_base, gpio->irq_sel, 1UL << gpio->pin);
    return kStatus_Success;
}

status_t gpio_attach_interrupt(gpio_ctrl_t *gpio, gpio_callback_t callback, rgpio_interrupt_config_t trigger)
{
    uint32_t instance;
    status_t status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }
    if ((uint32_t)gpio->irq_sel >= GPIO_IRQ_SEL_COUNT) {
        return kStatus_InvalidArgument; /* only outputs 0 and 1 reach the NVIC */
    }

    /* Publish the callback before anything can dispatch it. */
    s_callback_ctx[instance][gpio->pin] = gpio->user_ctx;
    s_callback[instance][gpio->pin]     = callback;
    __DMB();

    /* Drop an edge captured before we were listening, or it fires immediately. */
    RGPIO_ClearPinsInterruptFlags(gpio->gpio_base, gpio->irq_sel, 1UL << gpio->pin);

    /* ICR[pin] carries both the trigger and which line it drives. */
    RGPIO_SetPinInterruptConfig(gpio->gpio_base, gpio->pin, gpio->irq_sel, trigger);

    NVIC_SetPriority(s_gpio_hw[instance].irq[gpio->irq_sel], GPIO_IRQ_PRIORITY);
    EnableIRQ(s_gpio_hw[instance].irq[gpio->irq_sel]);

    return kStatus_Success;
}

status_t gpio_detach_interrupt(gpio_ctrl_t *gpio)
{
    uint32_t instance;
    status_t status;

    status = gpio_get_instance(gpio, &instance);
    if (status != kStatus_Success) {
        return status;
    }

    /* Silence the pin first, then drop the callback - not the other way round,
     * or an edge in between would dispatch through a NULL pointer. The line
     * itself stays enabled: other pins may still be using it. */
    RGPIO_SetPinInterruptConfig(gpio->gpio_base, gpio->pin, gpio->irq_sel, kRGPIO_InterruptOrDMADisabled);
    RGPIO_ClearPinsInterruptFlags(gpio->gpio_base, gpio->irq_sel, 1UL << gpio->pin);

    s_callback[instance][gpio->pin]     = NULL;
    s_callback_ctx[instance][gpio->pin] = NULL;

    return kStatus_Success;
}

/*******************************************************************************
 * Interrupt Service Routines
 *
 * The vectors are NOT named GPIOn_m_IRQHandler. The startup file leaves the
 * GPIO entries as ReservedNN_IRQHandler, where NN is the vector index, i.e.
 * IRQn + 16:
 *
 *   GPIO1_0 (IRQn 10)  -> Reserved26     GPIO1_1 (IRQn 11)  -> Reserved27
 *   GPIO2_0 (IRQn 57)  -> Reserved73     GPIO2_1 (IRQn 58)  -> Reserved74
 *   GPIO3_0 (IRQn 59)  -> Reserved75     GPIO3_1 (IRQn 60)  -> Reserved76
 *   GPIO4_0 (IRQn 189) -> Reserved205    GPIO4_1 (IRQn 190) -> Reserved206
 *
 * Defining GPIO2_0_IRQHandler instead would compile, link and never once be
 * called. Check startup_MIMX9352_cm33.S if these numbers ever look wrong.
 ******************************************************************************/

static void gpio_dispatch(uint32_t instance, rgpio_interrupt_sel_t sel)
{
    RGPIO_Type *base = s_gpio_hw[instance].base;
    uint32_t    flags;

    flags = RGPIO_GetPinsInterruptFlags(base, sel);

    /* Clear first: a level-sensitive source that is still asserted re-arms
     * immediately, which is wanted, but clearing after the callback would also
     * discard an edge that arrived while the callback ran. */
    RGPIO_ClearPinsInterruptFlags(base, sel, flags);

    while (flags != 0U) {
        uint32_t pin = (uint32_t)__builtin_ctz(flags);

        flags &= ~(1UL << pin);

        gpio_callback_t callback = s_callback[instance][pin];
        if (callback != NULL) {
            callback(s_callback_ctx[instance][pin]);
        }
    }
}

#define GPIO_DEFINE_IRQ(vector, instance, sel)   \
    void vector(void);                           \
    void vector(void)                            \
    {                                            \
        gpio_dispatch((instance), (sel));        \
        SDK_ISR_EXIT_BARRIER;                    \
    }

GPIO_DEFINE_IRQ(Reserved26_IRQHandler,  0U, kRGPIO_InterruptOutput0) /* GPIO1_0 */
GPIO_DEFINE_IRQ(Reserved27_IRQHandler,  0U, kRGPIO_InterruptOutput1) /* GPIO1_1 */
GPIO_DEFINE_IRQ(Reserved73_IRQHandler,  1U, kRGPIO_InterruptOutput0) /* GPIO2_0 */
GPIO_DEFINE_IRQ(Reserved74_IRQHandler,  1U, kRGPIO_InterruptOutput1) /* GPIO2_1 */
GPIO_DEFINE_IRQ(Reserved75_IRQHandler,  2U, kRGPIO_InterruptOutput0) /* GPIO3_0 */
GPIO_DEFINE_IRQ(Reserved76_IRQHandler,  2U, kRGPIO_InterruptOutput1) /* GPIO3_1 */
GPIO_DEFINE_IRQ(Reserved205_IRQHandler, 3U, kRGPIO_InterruptOutput0) /* GPIO4_0 */
GPIO_DEFINE_IRQ(Reserved206_IRQHandler, 3U, kRGPIO_InterruptOutput1) /* GPIO4_1 */
