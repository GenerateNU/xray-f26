#ifndef IMU_TILT_H
#define IMU_TILT_H

#include <zephyr/kernel.h>

/* Shared lock for the I2C bus (IMU, and the distance sensor later) */
extern struct k_mutex i2c_bus_mutex;

/* Set up the IMU: sample rate and data-ready interrupt. Returns 0 on success. */
int imu_tilt_init(void);

/* Take the IMU's current angle as 0 deg. Call with the tilt axis at its start position. */
int imu_set_zero(void);

/*
 * Wake the IMU thread, wait for it to collect fresh data-ready samples, and
 * write the measured tilt angle (deg, relative to the zero) into *tilt_deg.
 * Blocks until done. Returns 0 on success.
 */
int imu_measure_tilt(double *tilt_deg);

#endif