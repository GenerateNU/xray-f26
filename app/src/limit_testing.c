#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "homing.h"

// is this axis's limit switch pressed? 1 = pressed, 0 = not
int axis_limit_pressed(const struct axis *axis) {
    return gpio_pin_get_dt(&axis->limit);
}