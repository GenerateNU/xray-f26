/*
 * pcb_blink - proves the custom PCB was flashed and the firmware is running.
 *
 * On boot: 3 quick flashes, then a steady 1 Hz blink.
 * The LED pin comes from the motorboard board file (alias led0 -> PC5).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>

static const struct gpio_dt_spec status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

int main(void)
{
    if (!gpio_is_ready_dt(&status_led)) {
        return 0;
    }
    gpio_pin_configure_dt(&status_led, GPIO_OUTPUT_INACTIVE);

    /* 3 quick flashes = "just booted" */
    for (int i = 0; i < 6; i++) {
        gpio_pin_toggle_dt(&status_led);
        k_msleep(100);
    }

    /* Steady heartbeat = "firmware is running" */
    while (1) {
        gpio_pin_toggle_dt(&status_led);
        k_msleep(500);
    }

    return 0;
}
