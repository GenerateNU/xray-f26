#ifndef ROT_MOTORS_H
#define ROT_MOTORS_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* what is in each rotational motor (same idea as struct axis in motors.h) */
struct rot_axis {
    const char *name;
    struct gpio_dt_spec step;
    struct gpio_dt_spec dir;
    struct gpio_dt_spec ena;
    struct gpio_dt_spec sleep;
    struct gpio_dt_spec nfault;
    struct gpio_dt_spec limit;
    struct spi_dt_spec  spi;     /* SPI type, not GPIO! */
    uint8_t trq_dac;             /* current dial for THIS motor (0-255), see rot_motors.c */
};

/* the two rotational motors on the Motor Board */
extern const struct rot_axis r1_axis;
extern const struct rot_axis r2_axis;

int rot_axis_setup(const struct rot_axis *axis);

int rot_axis_enable(const struct rot_axis *axis);

int rot_axis_move(const struct rot_axis *axis, int steps, int dir, uint32_t step_interval_us);

int rot_axis_move_deg(const struct rot_axis *axis, double degrees, uint32_t step_interval_us);

#endif