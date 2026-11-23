#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include "rot_motors.h"

/*
    TEST: both motors + both limit switches
      R1 spins until LIM_R1 (PC13) reads HIGH -> R1 ENABLE (PC3) goes LOW
      R2 spins until LIM_R2 (PB9)  reads HIGH -> R2 ENABLE (PB5) goes LOW
    Each switch only stops its own motor. Both motors spin at the same time.

    Each switch is watched two ways:
      - an interrupt (stops the motor the instant the pin goes HIGH)
      - a backup check on every step (in case an interrupt is ever missed)

    status LED (PC5):
        on solid   = at least one motor still spinning
        fast blink = both motors stopped (limit hit, or couldn't start)
        slow blink = setup failed
*/
#define R1_AXIS            (&r1_axis)
#define R2_AXIS            (&r2_axis)
#define R1_TOWARD_LIMIT    1        // change to 0 if R1 spins away from its switch
#define R2_TOWARD_LIMIT    1        // change to 0 if R2 spins away from its switch

#define CRUISE_INTERVAL_US 20000    // 50 full steps/s = 15 RPM
#define START_INTERVAL_US  40000    // slower at the very start, then speeds up
#define RAMP_STEPS         15       // steps spent speeding up at the start
#define STEP_PULSE_US      10       // short, clean STEP pulse

#define USER_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec armed_motor = GPIO_DT_SPEC_GET(USER_NODE, armed_motor_gpios);
static const struct gpio_dt_spec fw_status_led = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);

/* one callback per switch (they're on different ports: PC13 and PB9) */
static struct gpio_callback r1_limit_cb;
static struct gpio_callback r2_limit_cb;

/* set when a motor should stop (limit hit, or it couldn't start) */
static volatile bool r1_done = false;
static volatile bool r2_done = false;

static void setup_failed(void)
{
    while (true) {                      // slow blink forever = setup failed
        gpio_pin_toggle_dt(&fw_status_led);
        k_msleep(500);
    }
}

/* runs the instant LIM_R1 goes HIGH: stop R1 only */
static void r1_limit_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_set_dt(&R1_AXIS->ena, 0);   // R1 ENABLE (PC3) LOW = R1 driver disabled
    r1_done = true;
}

/* runs the instant LIM_R2 goes HIGH: stop R2 only */
static void r2_limit_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    gpio_pin_set_dt(&R2_AXIS->ena, 0);   // R2 ENABLE (PB5) LOW = R2 driver disabled
    r2_done = true;
}

/* hook a limit switch up to its interrupt function */
static void setup_limit(const struct rot_axis *axis, struct gpio_callback *cb,
                        gpio_callback_handler_t isr)
{
    gpio_pin_interrupt_configure_dt(&axis->limit, GPIO_INT_EDGE_TO_ACTIVE);  // fire on LOW -> HIGH
    gpio_init_callback(cb, isr, BIT(axis->limit.pin));
    gpio_add_callback(axis->limit.port, cb);
}

/*
    get one motor ready to spin. returns true if it can move.
    if its switch is already HIGH, or the driver won't turn on, that motor
    just stays still and the other one keeps testing.
*/
static bool start_axis(const struct rot_axis *axis, int dir)
{
    if (gpio_pin_get_dt(&axis->limit) == 1) {
        printk("%s: limit switch already HIGH at startup, not moving\n", axis->name);
        return false;
    }
    if (rot_axis_enable(axis) != 0) {
        printk("%s: failed to enable, not moving\n", axis->name);
        return false;
    }
    gpio_pin_set_dt(&axis->dir, dir);
    return true;
}

/* how long to wait for step i: slow at the start, speeding up to cruise */
static uint32_t step_interval_us(int i)
{
    if (i >= RAMP_STEPS) {
        return CRUISE_INTERVAL_US;
    }
    return START_INTERVAL_US - (START_INTERVAL_US - CRUISE_INTERVAL_US) * i / RAMP_STEPS;
}

int main(void)
{
    gpio_pin_configure_dt(&fw_status_led, GPIO_OUTPUT_INACTIVE);

    // ARMED_MOTOR (PA10) high before anything else
    if (!gpio_is_ready_dt(&armed_motor)) setup_failed();
    gpio_pin_configure_dt(&armed_motor, GPIO_OUTPUT_ACTIVE);
    k_msleep(5);

    // set up both motors' pins (asleep, disabled)
    if (rot_axis_setup(R1_AXIS) != 0) setup_failed();
    if (rot_axis_setup(R2_AXIS) != 0) setup_failed();

    // turn on each motor that can move
    r1_done = !start_axis(R1_AXIS, R1_TOWARD_LIMIT);
    r2_done = !start_axis(R2_AXIS, R2_TOWARD_LIMIT);

    // both limit switches watch for HIGH from now on
    setup_limit(R1_AXIS, &r1_limit_cb, r1_limit_isr);
    setup_limit(R2_AXIS, &r2_limit_cb, r2_limit_isr);

    k_busy_wait(1000);   // DIR settles before the first step
    printk("Spinning until each limit switch reads HIGH...\n");
    gpio_pin_set_dt(&fw_status_led, 1);   // solid on = spinning

    int i = 0;                       // step counter, used for the speed-up ramp
    int r1_steps = 0, r2_steps = 0;
    bool r1_reported = r1_done, r2_reported = r2_done;

    // keep going while at least one motor is still spinning
    while (!r1_done || !r2_done) {
        // backup check: read the switches directly every step too,
        // in case an interrupt is ever missed
        if (!r1_done && gpio_pin_get_dt(&R1_AXIS->limit) == 1) {
            gpio_pin_set_dt(&R1_AXIS->ena, 0);   // R1 ENABLE (PC3) LOW
            r1_done = true;
        }
        if (!r2_done && gpio_pin_get_dt(&R2_AXIS->limit) == 1) {
            gpio_pin_set_dt(&R2_AXIS->ena, 0);   // R2 ENABLE (PB5) LOW
            r2_done = true;
        }

        bool step_r1 = !r1_done;
        bool step_r2 = !r2_done;
        uint32_t interval = step_interval_us(i);

        // one step for each motor that's still going, at the same time
        if (step_r1) gpio_pin_set_dt(&R1_AXIS->step, 1);
        if (step_r2) gpio_pin_set_dt(&R2_AXIS->step, 1);
        k_busy_wait(STEP_PULSE_US);
        gpio_pin_set_dt(&R1_AXIS->step, 0);
        gpio_pin_set_dt(&R2_AXIS->step, 0);
        k_busy_wait(interval - STEP_PULSE_US);

        if (step_r1) r1_steps++;
        if (step_r2) r2_steps++;
        i++;

        // report each limit hit once, outside the interrupt
        if (r1_done && !r1_reported) {
            printk("R1 LIMIT HIT after %d steps: R1 ENABLE is LOW\n", r1_steps);
            r1_reported = true;
        }
        if (r2_done && !r2_reported) {
            printk("R2 LIMIT HIT after %d steps: R2 ENABLE is LOW\n", r2_steps);
            r2_reported = true;
        }
    }

    // backup: make sure both ENABLE pins are LOW
    gpio_pin_set_dt(&R1_AXIS->ena, 0);
    gpio_pin_set_dt(&R2_AXIS->ena, 0);
    printk("Both motors stopped (R1: %d steps, R2: %d steps)\n", r1_steps, r2_steps);

    while (true) {                      // fast blink forever = both stopped
        gpio_pin_toggle_dt(&fw_status_led);
        k_msleep(100);
    }

    return 0;
}