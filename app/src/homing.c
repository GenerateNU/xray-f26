#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "homing.h"

// home this axis
#define HOME_DIR         0       // DIR value that moves TOWARD the switch (CHECK THIS!)
#define HOME_FAST_US     1000    // fast approach speed (step interval)
#define HOME_MAX_STEPS   8000    // timeout: give up after this many steps (small for testing!)

// is this axis's limit switch pressed? 1 = pressed, 0 = not
int axis_limit_pressed(const struct axis *axis) {
    return gpio_pin_get_dt(&axis->limit);
}


int home_axis(const struct axis *axis) {
    limit_clear(axis);                               // start with the flag down

    // 1. FAST approach: move toward the switch until it clicks
    int done = axis_move(axis, HOME_MAX_STEPS, HOME_DIR, HOME_FAST_US);

    if (done >= HOME_MAX_STEPS) {                    // went the whole way, no click
        printk("ERROR! %s homing timeout\n", axis->name);
        safe_trigger();
        return -1;
    }

    //todo check in with zach and see the whereabouts about the distance sensor and 
    // try to integrate that into the error one above so tht we can double check for our safety system

    printk("%s hit the switch after %d steps\n", axis->name, done);
    return 0;
}