/*!
 * @file gpio_driver.h
 * @brief GPIO driver for the i.MX93 Cortex-M33.
 *
 * Ported from the MCXN947 version. Three things are fundamentally different
 * here and they shape the whole API:
 *
 * 1. There is no PORT peripheral. Pad muxing and the electrical settings
 *    (pull, drive strength, slew, open-drain, SION) live in IOMUXC, and the
 *    IOMUXC macros are indexed by pad name, not by (instance, pin) - so there
 *    is no way for this driver to reach them. Mux the pad to its ALT0 GPIO
 *    function in BOARD_InitPins() first; this driver only drives the logic.
 *
 * 2. The IP is RGPIO, not Kinetis GPIO: RGPIO_Type, four instances GPIO1..GPIO4
 *    (there is no GPIO0 or GPIO5), and the input buffer is gated by PIDR and
 *    comes up disabled - a pin that is not explicitly enabled reads nothing.
 *
 * 3. Each instance drives two NVIC lines, and each pin picks which one through
 *    ICR[pin].IRQS. The vectors are *not* called GPIOn_m_IRQHandler - the
 *    startup file leaves them as ReservedNN_IRQHandler, so defining the
 *    obvious name would link cleanly and never fire. See gpio_driver.c.
 *
 * Which instance a pad belongs to is fixed by the pad, e.g. the GPIO_IO00..
 * GPIO_IO29 pads are GPIO2_IO00..GPIO2_IO29 at ALT0.
 *
 * @author Diego
 * @date April 10, 2026
 *
 * @example
 * @code
 * gpio_ctrl_t imu_int = {
 *     .gpio_base = GPIO2,
 *     .pin       = 17,
 *     .dir       = gpio_input,
 * };
 * gpio_init(&imu_int);
 *
 * void my_callback(void *arg) { ... }
 * gpio_attach_interrupt(&imu_int, my_callback, kRGPIO_InterruptFallingEdge);
 * @endcode
 */

#ifndef GPIO_DRIVER_H_
#define GPIO_DRIVER_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsl_clock.h"
#include "fsl_common.h"
#include "fsl_rgpio.h"

/*! @brief GPIO pin direction enumeration */
typedef enum gpio_direction_s {
    gpio_input = 0, /*!< Configure pin as input */
    gpio_output,    /*!< Configure pin as output */
} gpio_direction_t;

/*! @brief Pin interrupt callback. Runs in interrupt context. */
typedef void (*gpio_callback_t)(void *arg);

/*!
 * @brief GPIO control structure for one pin.
 *
 * Populate gpio_base, pin and dir before calling gpio_init(). The rest may be
 * left zeroed: that selects a default output level of 0 and interrupt output 0.
 */
typedef struct gpio_ctrl_s {
    RGPIO_Type *gpio_base;      /*!< GPIO1, GPIO2, GPIO3 or GPIO4 */
    uint8_t     pin;            /*!< Pin number, 0..31 */
    gpio_direction_t dir;       /*!< gpio_input or gpio_output */
    uint8_t     output_default; /*!< Initial level for an output; ignored for an input */
    /*!
     * Which of the instance's two NVIC lines this pin's interrupt goes to.
     * kRGPIO_InterruptOutput0 or ...Output1 - the part exposes only those two.
     * Splitting pins across the two lines lets you give them different NVIC
     * priorities; leave it 0 if you do not care.
     */
    rgpio_interrupt_sel_t irq_sel;
    void       *user_ctx;       /*!< Passed back to the callback; may be NULL */
} gpio_ctrl_t;

/*!
 * @brief Ungate the instance, set the direction and enable the input buffer.
 *
 * Does NOT touch the pad mux - do that in BOARD_InitPins().
 * @return kStatus_InvalidArgument for an unknown base or a pin above 31.
 */
status_t gpio_init(gpio_ctrl_t *gpio);

/*! @brief Drive an output pin. */
status_t gpio_set_output(gpio_ctrl_t *gpio, uint8_t val);

/*! @brief Read the pad state. Works on outputs too, since gpio_init() leaves
 *         the input buffer enabled either way. */
uint32_t gpio_read_input(gpio_ctrl_t *gpio);

/*! @brief Invert an output pin. */
status_t gpio_toggle_output(gpio_ctrl_t *gpio);

/*!
 * @brief Install a per-pin interrupt callback and enable the line.
 *
 * Unlike the MCXN947 version this is per pin, not per port: two pins on the
 * same instance can carry different callbacks.
 *
 * @param trigger e.g. kRGPIO_InterruptFallingEdge, kRGPIO_InterruptEitherEdge.
 */
status_t gpio_attach_interrupt(gpio_ctrl_t *gpio, gpio_callback_t callback, rgpio_interrupt_config_t trigger);

/*! @brief Disable this pin's interrupt and drop its callback. */
status_t gpio_detach_interrupt(gpio_ctrl_t *gpio);

/*! @brief Clear a pin's interrupt flag. The ISR already does this; exposed for
 *         callers that want to discard a stale edge before attaching. */
status_t gpio_clear_interrupt_flag(gpio_ctrl_t *gpio);

#endif /* GPIO_DRIVER_H_ */
