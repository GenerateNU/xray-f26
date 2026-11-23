#ifndef MOTORS_H
#define MOTORS_H

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

/* what is in each axis */
struct axis {
    const char *name;
    struct gpio_dt_spec step;
    struct gpio_dt_spec dir;
    struct gpio_dt_spec ena;
    struct gpio_dt_spec sleep;
    struct gpio_dt_spec nfault;
    struct gpio_dt_spec limit;
    struct spi_dt_spec  spi;     /* SPI type, not GPIO! */
};


/* THE TRIPLE DIPPER  */
extern const struct axis x_axis;
extern const struct axis y_axis;
extern const struct axis z_axis;

int axis_setup(const struct axis *axis);

int motor_power(void);

int axis_enable(const struct axis *axis);

int axis_move(const struct axis *axis, int steps, int dir, uint32_t step_interval_us);

#endif