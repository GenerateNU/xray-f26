/*
 * imu.c - ISM330DHCX bring-up test (tilt only)
 *
 * Reads the SparkFun Micro 6DoF IMU - ISM330DHCX (Qwiic) over I2C using
 * Zephyr's built-in st,ism330dhcx driver and logs the acceleration (m/s^2)
 * and the tilt angles calculated from it. Only tilt is needed, so only the
 * accelerometer is used.
 *
 * This is a polling test to see what data the IMU gives back.
 * Interrupt-driven reads and the step-verification logic come later,
 * once the tilt motor code exists.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(imu, LOG_LEVEL_INF);

#define IMU_SAMPLE_RATE_HZ  104    /* how often the IMU measures internally */
#define IMU_LOG_PERIOD_MS   2000    /* how often we read and log (5 times per second) */
#define IMU_THREAD_STACK    2048
#define IMU_THREAD_PRIORITY 7      /* below the main state machine and motor control */

#define RAD_TO_DEG (180.0 / 3.14159265358979323846)

/* The ISM330DHCX node comes from app.overlay (ism330dhcx@6b on arduino_i2c) */
static const struct device *const imu = DEVICE_DT_GET_ONE(st_ism330dhcx);

static void imu_thread(void *a, void *b, void *c)
{
    struct sensor_value acc[3];
    struct sensor_value rate = { .val1 = IMU_SAMPLE_RATE_HZ, .val2 = 0 };

    /* Zephyr has already tried to start the driver at boot; this checks it worked */
    if (!device_is_ready(imu)) {
        LOG_ERR("ISM330DHCX not ready: check wiring and I2C address (0x6B)");
        return;
    }

    /*
     * The accelerometer is off at power-up. Setting a sample rate switches
     * it on. The range is left at its power-up default of +/-2 g, which is
     * plenty, since gravity is 1 g and the tilt axis moves slowly.
     */
    if (sensor_attr_set(imu, SENSOR_CHAN_ACCEL_XYZ,
                        SENSOR_ATTR_SAMPLING_FREQUENCY, &rate)) {
        LOG_ERR("ISM330DHCX configuration failed");
        return;
    }
    LOG_INF("ISM330DHCX ready (accelerometer, %d Hz)", IMU_SAMPLE_RATE_HZ);

    while (1) {
        /* Read one new sample over I2C */
        if (sensor_sample_fetch_chan(imu, SENSOR_CHAN_ACCEL_XYZ)) {
            LOG_WRN("ISM330DHCX read failed");
            k_msleep(IMU_LOG_PERIOD_MS);
            continue;
        }

        /* Zephyr gives acceleration in m/s^2 */
        sensor_channel_get(imu, SENSOR_CHAN_ACCEL_XYZ, acc);

        double ax = sensor_value_to_double(&acc[0]);
        double ay = sensor_value_to_double(&acc[1]);
        double az = sensor_value_to_double(&acc[2]);

        /*
         * Tilt from gravity. When the board is still, the accelerometer only
         * feels gravity, so the direction of that 9.8 m/s^2 tells us how the
         * board is tilted. Roll = tilt around X, pitch = tilt around Y.
         * Only one of these will match the tilt motor's axis, depending
         * on how the IMU is mounted. The reading is only accurate while
         * the board is still, which matches reading it after a move ends.
         */
        double roll = atan2(ay, az) * RAD_TO_DEG;
        double pitch = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;

        LOG_INF("accel x=%6.2f y=%6.2f z=%6.2f m/s^2 | roll=%6.1f pitch=%6.1f deg",
                ax, ay, az, roll, pitch);

        k_msleep(IMU_LOG_PERIOD_MS);
    }
}

K_THREAD_DEFINE(imu_tid, IMU_THREAD_STACK, imu_thread, NULL, NULL, NULL,
                IMU_THREAD_PRIORITY, 0, 0);
