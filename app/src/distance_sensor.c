#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include "VL53L4CD_api.h"
#include "VL53L4CD_calibration.h"

LOG_MODULE_REGISTER(distance_sensor, LOG_LEVEL_INF);

#define MODE_INTERRUPT 0
#define MODE_POLLING   1
#define MODE_CALIBRATE 2

#define SENSOR_MODE MODE_POLLING

#define SENSOR_ADDR            0x29
#define THRESH_LOW_MM          24
#define THRESH_HIGH_MM         50
#define OFFSET_MM              0
#define CAL_TARGET_MM          100
#define CAL_SAMPLES            20
#define SENSOR_THREAD_STACK    2048
#define SENSOR_THREAD_PRIORITY 0

static const struct gpio_dt_spec irq_pin = GPIO_DT_SPEC_GET(DT_PATH(zephyr_user), irq_gpios);
static struct gpio_callback irq_cb;
K_SEM_DEFINE(sensor_sem, 0, 1);

static void sensor_isr(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    k_sem_give(&sensor_sem);
}

static int sensor_init_common(void)
{
    uint16_t id = 0;

    if (VL53L4CD_GetSensorId(SENSOR_ADDR, &id) || id != 0xEBAA) {
        LOG_ERR("Sensor not found (id 0x%04X)", id);
        return -1;
    }
    LOG_INF("Sensor found (id 0x%04X)", id);

    if (VL53L4CD_SensorInit(SENSOR_ADDR)) {
        LOG_ERR("Sensor init failed");
        return -1;
    }

    if (SENSOR_MODE != MODE_CALIBRATE) {
        VL53L4CD_SetOffset(SENSOR_ADDR, OFFSET_MM);
        LOG_INF("Offset applied: %d mm", OFFSET_MM);
    }

    VL53L4CD_SetRangeTiming(SENSOR_ADDR, 50, 0);
    return 0;
}

static void run_calibration(void)
{
    int16_t offset = 0;

    LOG_INF("Place a flat target at %d mm. Starting in 5 s...", CAL_TARGET_MM);
    k_msleep(5000);

    if (VL53L4CD_CalibrateOffset(SENSOR_ADDR, CAL_TARGET_MM, &offset, CAL_SAMPLES)) {
        LOG_ERR("Calibration failed");
        return;
    }
    LOG_INF("Calibration done. Set OFFSET_MM to %d", offset);
}

static void run_polling(void)
{
    VL53L4CD_ResultsData_t r;
    uint8_t ready = 0;

    VL53L4CD_StartRanging(SENSOR_ADDR);
    LOG_INF("Polling mode started");

    while (1) {
        VL53L4CD_CheckForDataReady(SENSOR_ADDR, &ready);
        if (ready) {
            VL53L4CD_GetResult(SENSOR_ADDR, &r);
            VL53L4CD_ClearInterrupt(SENSOR_ADDR);
            LOG_INF("Distance: %u mm (status %u)", r.distance_mm, r.range_status);
        }
        k_msleep(1000);
    }
}

static void run_interrupt(void)
{
    VL53L4CD_ResultsData_t r;

    VL53L4CD_SetDetectionThresholds(SENSOR_ADDR, THRESH_LOW_MM, THRESH_HIGH_MM, 0);

    gpio_pin_configure_dt(&irq_pin, GPIO_INPUT);
    gpio_pin_interrupt_configure_dt(&irq_pin, GPIO_INT_EDGE_TO_ACTIVE);
    gpio_init_callback(&irq_cb, sensor_isr, BIT(irq_pin.pin));
    gpio_add_callback(irq_pin.port, &irq_cb);

    VL53L4CD_StartRanging(SENSOR_ADDR);
    LOG_INF("Interrupt mode started (threshold %d mm)", THRESH_LOW_MM);

    while (1) {
        k_sem_take(&sensor_sem, K_FOREVER);
        VL53L4CD_GetResult(SENSOR_ADDR, &r);
        VL53L4CD_ClearInterrupt(SENSOR_ADDR);

        if (r.range_status == 0 && r.distance_mm < THRESH_LOW_MM) {
            LOG_INF("Object detected: %u mm", r.distance_mm);
        } else {
            LOG_WRN("Interrupt with unconfirmed reading: %u mm (status %u)",
                    r.distance_mm, r.range_status);
        }
    }
}

static void sensor_thread(void *a, void *b, void *c)
{
    if (sensor_init_common()) {
        return;
    }

    if (SENSOR_MODE == MODE_CALIBRATE) {
        run_calibration();
    } else if (SENSOR_MODE == MODE_POLLING) {
        run_polling();
    } else {
        run_interrupt();
    }
}

K_THREAD_DEFINE(sensor_tid, SENSOR_THREAD_STACK, sensor_thread, NULL, NULL, NULL,
                SENSOR_THREAD_PRIORITY, 0, 0);
