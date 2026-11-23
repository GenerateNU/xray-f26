/*
 * rot_motor_test.c - the two rotational motors (R1, R2) on the Motor Board
 *
 * Same structure as motors.c for X/Y/Z on the Main Board:
 *   - SPI is only used at startup, to set up the driver chip (stepping mode,
 *     current, outputs on). It replaces the DIP switches on the DM542T.
 *   - Moving the motor is still STEP / DIR pins, exactly like the DM542T.
 *
 * Currently set to FULL STEPPING (200 steps per rev, 1.8 deg per step).
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <math.h>
#include "rot_motors.h"

/*
    step timing:
    pulses must be long enough for the driver to see them
    DIR needs to be set a little before stepping
*/
#define DIR_SETUP_US          1000
#define STEP_PULSE_US         10      // how long STEP stays high each step (short, clean pulse)

/*
    speed limits + acceleration ramp
    every move starts slow, speeds up over RAMP_STEPS, cruises, then slows down
    over the last RAMP_STEPS before stopping. Starting or stopping at full speed
    is what makes a stepper jerk, stall, or skip.

    (full stepping: 200 steps per rev)
      20000 us per step =  50 steps/s = 15 RPM   (start / stop speed)
       8000 us per step = 125 steps/s = 37.5 RPM
       4000 us per step = 250 steps/s = 75 RPM   (fastest allowed)
*/
#define START_INTERVAL_US     20000   // speed at the very start and end of every move
#define MIN_STEP_INTERVAL_US  4000    // max speed cap: anything faster is slowed to this
#define RAMP_STEPS            15      // steps spent speeding up, and again slowing down
#define SETTLE_MS             200     // pause after every move so the motor stops shaking

/* driver registers (same chip and same values as the X/Y/Z drivers) */
#define DRV_REG_FAULT    0x00   // FAULT register, bit 7 = any fault
#define DRV_REG_CTRL1    0x04   // EN_OUT lives in bit 7
#define DRV_REG_CTRL2    0x05   // stepping mode in bits 0-3
#define DRV_REG_CTRL9    0x0C   // EN_AUTO (auto microstepping) in bit 0
#define DRV_REG_CTRL10   0x0D   // ISTSL: holding current
#define DRV_REG_CTRL11   0x0E   // TRQ_DAC: the current dial (0-255)
#define DRV_REG_CTRL12   0x0F   // EN_STSL (bit 7) + TSTSL_FALL (bits 6-3): standstill power saving
#define DRV_REG_CTRL13   0x10   // TSTSL_DLY (bits 7-2) + VREF_INT_EN (bit 1)
#define DRV_READ_BIT     (1u << 14)   // bit 14 = 1 means read

#define CTRL1_DEFAULT    0x0F   // TOFF / DECAY datasheet defaults
#define EN_OUT_BIT       0x80   // turns the motor outputs on inside the chip
#define MICROSTEP_FULL   0x01   // full stepping with 71% current (0x00 = 100% in BOTH coils = much hotter)
#define EN_AUTO_OFF      0x00   // auto microstepping off, so it really does full steps
#define CTRL13_VREF_INT  0x12   // internal 3.3V reference on (bit 1) + TSTSL_DLY = 4 -> 64 ms delay
#define CTRL12_STSL_ON   0x88   // EN_STSL = 1 (bit 7) + TSTSL_FALL = 1 ms per current step (bits 6-3)
#define FAULT_ANY_BIT    0x80

/*
    current for the rotational motors (datasheet 7.3.8 / 7.3.9, internal 3.3 V reference):
        current (A) = 3.3 V x (TRQ_DAC + 1) / 256 / KV
        KV = 0.66 V/A for the DDW package -> 5 A at TRQ_DAC = 255
        KV = 0.33 V/A for the DDV package -> 10 A at TRQ_DAC = 255

    the old 0x80 was ~2.5 A (DDW) or ~5 A (DDV): way above the tilt motor's 1.5 A,
    which is why the motor and driver got hot.

    0x25 = 14.8%  -> ~0.74 A on DDW, ~1.48 A on DDV  (safe on either package)
    !! ask the EE which package is on the board. If it's DDW, raise it to 0x4B (~1.48 A).
*/
#define ROT_TRQ_DAC      0x25

/*
    holding current: once the motor stops, the driver lowers the current to this
    (standstill power saving). 75% of the run current, so the tilt still holds
    the RadEye against gravity. Lower it if the motor holds fine; raise it if it sags.
*/
#define ROT_ISTSL        ((ROT_TRQ_DAC * 3) / 4)

/*
    motor --> 200 full steps per rev, full stepping = 200 steps per rev
    GEAR_RATIO = motor turns per output turn (1.0 = direct drive)
*/
#define FULL_STEPS_PER_REV 200
#define MICROSTEPS         1      // full stepping: 1 pulse = 1 full step = 1.8 deg
#define STEPS_PER_REV      (FULL_STEPS_PER_REV * MICROSTEPS)
#define GEAR_RATIO         1.0
#define STEPS_PER_DEG      (STEPS_PER_REV * GEAR_RATIO / 360.0)   // 0.56 steps per degree

/* SPI settings: 16-bit messages, most significant bit first, mode 1 (same as X/Y/Z) */
#define DRI_SPI_SET (SPI_OP_MODE_MASTER | SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/*
GRAB DEM DEVICE TREE OVERLAY!!!! (pins come from app.overlay)
*/
#define USER_NODE DT_PATH(zephyr_user)


/* ---------- talking to the driver over SPI ---------- */

static int drv_write_reg(const struct rot_axis *axis, uint8_t addy, uint8_t value)
{
    // bits 13-8 = register address, bits 7-0 = the value. bits 14 & 15 stay 0 = write
    uint16_t tx = ((addy & 0x3F) << 8) | value;

    struct spi_buf buf = { .buf = &tx, .len = sizeof(tx) };       // one message
    struct spi_buf_set tx_set = { .buffers = &buf, .count = 1 };  // a stack of 1 message

    // Zephyr pulls this driver's CS pin low, clocks out the 16 bits, then lets CS go high
    return spi_write_dt(&axis->spi, &tx_set);
}

static int drv_read_reg(const struct rot_axis *axis, uint8_t addy, uint8_t *value)
{
    // bit 14 = 1 means read, bits 13-8 = address, bits 7-0 = 0
    uint16_t tx = DRV_READ_BIT | ((addy & 0x3F) << 8);
    uint16_t rx = 0;                                              // the driver's reply lands here

    struct spi_buf tx_buf = { .buf = &tx, .len = sizeof(tx) };
    struct spi_buf rx_buf = { .buf = &rx, .len = sizeof(rx) };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    // send and receive at the same time: SPI always swaps bits both ways
    int ret = spi_transceive_dt(&axis->spi, &tx_set, &rx_set);

    *value = rx & 0xFF;   // the register's value is in the bottom 8 bits
    return ret;
}

/* the "DIP switches", but set over SPI */
static int drv_configure(const struct rot_axis *axis)
{
    uint8_t value;

    // full stepping (like setting the DM542T to 200 pulses/rev)
    if (drv_write_reg(axis, DRV_REG_CTRL2, MICROSTEP_FULL) != 0) {
        printk("ERROR! %s failed to set full stepping \n", axis->name);
        return -1;
    }

    // auto microstepping OFF, so the chip really does full steps
    if (drv_write_reg(axis, DRV_REG_CTRL9, EN_AUTO_OFF) != 0) {
        printk("ERROR! %s failed to set CTRL9 \n", axis->name);
        return -1;
    }

    drv_write_reg(axis, DRV_REG_CTRL11, axis->trq_dac);   // moving current (like SW1-SW3)
    drv_write_reg(axis, DRV_REG_CTRL10, ROT_ISTSL);       // lower holding current (like SW4 off)
    drv_write_reg(axis, DRV_REG_CTRL13, CTRL13_VREF_INT); // internal reference + 64 ms standstill delay
    drv_write_reg(axis, DRV_REG_CTRL12, CTRL12_STSL_ON);  // standstill power saving ON:
                                                          // 64 ms after the last step, current drops
                                                          // from TRQ_DAC to ISTSL automatically

    // read the current back AFTER writing it: if it matches, SPI to this driver works
    drv_read_reg(axis, DRV_REG_CTRL11, &value);
    if (value != axis->trq_dac) {
        printk("ERROR! %s current readback 0x%02X (SPI not working?) \n", axis->name, value);
        return -1;
    }

    printk("%s is configured (full stepping, 71%% current, standstill saving on) \n", axis->name);
    return 0;
}


/* ---------- the two motors (pins + SPI from app.overlay) ---------- */

const struct rot_axis r1_axis = {
    .name = "R1",
    .step = GPIO_DT_SPEC_GET(USER_NODE, r1_step_gpios),
    .dir = GPIO_DT_SPEC_GET(USER_NODE, r1_dir_gpios),
    .ena = GPIO_DT_SPEC_GET(USER_NODE, r1_ena_gpios),
    .sleep = GPIO_DT_SPEC_GET(USER_NODE, r1_sleep_gpios),
    .nfault = GPIO_DT_SPEC_GET(USER_NODE, r1_nfault_gpios),
    .limit = GPIO_DT_SPEC_GET(USER_NODE, r1_limit_gpios),
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(driver_r1), DRI_SPI_SET),
    .trq_dac = ROT_TRQ_DAC,
};

const struct rot_axis r2_axis = {
    .name = "R2",
    .step = GPIO_DT_SPEC_GET(USER_NODE, r2_step_gpios),
    .dir = GPIO_DT_SPEC_GET(USER_NODE, r2_dir_gpios),
    .ena = GPIO_DT_SPEC_GET(USER_NODE, r2_ena_gpios),
    .sleep = GPIO_DT_SPEC_GET(USER_NODE, r2_sleep_gpios),
    .nfault = GPIO_DT_SPEC_GET(USER_NODE, r2_nfault_gpios),
    .limit = GPIO_DT_SPEC_GET(USER_NODE, r2_limit_gpios),
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(driver_r2), DRI_SPI_SET),
    .trq_dac = ROT_TRQ_DAC,
};


/* ---------- pin setup ---------- */

int rot_axis_setup(const struct rot_axis *axis)
{
    if (!gpio_is_ready_dt(&axis->step) ||
        !gpio_is_ready_dt(&axis->dir) ||
        !gpio_is_ready_dt(&axis->ena) ||
        !gpio_is_ready_dt(&axis->sleep) ||
        !gpio_is_ready_dt(&axis->nfault) ||
        !gpio_is_ready_dt(&axis->limit)) {
        printk("ERROR! %s GPIO PORT IS NOT READY\n", axis->name);
        return -1;
    }

    if (!spi_is_ready_dt(&axis->spi)) {
        printk("ERROR! %s SPI PORT IS NOT READY\n", axis->name);
        return -1;
    }

    // outputs start OFF (asleep, disabled, no step)
    gpio_pin_configure_dt(&axis->step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->ena, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->sleep, GPIO_OUTPUT_INACTIVE);

    // inputs: fault from the driver, limit switch
    gpio_pin_configure_dt(&axis->nfault, GPIO_INPUT);
    gpio_pin_configure_dt(&axis->limit, GPIO_INPUT);

    printk("%s is ready \n", axis->name);
    return 0;
}


/* ---------- wake up + configure + turn on ---------- */

int rot_axis_enable(const struct rot_axis *axis)
{
    uint8_t value;

    gpio_pin_set_dt(&axis->sleep, 1);   // WAKE UPPP
    k_msleep(2);                        // give the chip time to wake before SPI

    if (drv_configure(axis) != 0) {
        printk("ERROR! %s failed to configure \n", axis->name);
        return -1;
    }

    // nFAULT is active low, so 1 here means FAULT
    if (gpio_pin_get_dt(&axis->nfault) == 1) {
        printk("ERROR! %s has a fault \n", axis->name);
        return -1;
    }

    drv_write_reg(axis, DRV_REG_CTRL1, EN_OUT_BIT | CTRL1_DEFAULT);   // outputs on inside the chip
    gpio_pin_set_dt(&axis->ena, 1);                                  // and the physical ENABLE pin

    // read everything back to make sure the driver really took the settings
    drv_read_reg(axis, DRV_REG_CTRL1, &value);
    if (value != (EN_OUT_BIT | CTRL1_DEFAULT)) {
        printk("ERROR! %s CTRL1 readback 0x%02X \n", axis->name, value);
        return -1;
    }
    drv_read_reg(axis, DRV_REG_CTRL11, &value);
    if (value != axis->trq_dac) {
        printk("ERROR! %s current readback 0x%02X \n", axis->name, value);
        return -1;
    }
    drv_read_reg(axis, DRV_REG_CTRL13, &value);
    if (value != CTRL13_VREF_INT) {
        printk("ERROR! %s CTRL13 readback 0x%02X \n", axis->name, value);
        return -1;
    }
    drv_read_reg(axis, DRV_REG_FAULT, &value);
    if (value & FAULT_ANY_BIT) {
        printk("ERROR! %s FAULT register 0x%02X \n", axis->name, value);
        return -1;
    }

    printk("%s is enabled \n", axis->name);
    return 0;
}


/* ---------- full low-power: sleep ---------- */

/*
    turns the motor completely OFF: outputs off, driver asleep (uses ~3 uA).
    !! the motor no longer holds its position. On the tilt axis, gravity can
    !! swing the RadEye down, so only use this when the axis is parked/supported.
    call rot_axis_enable() again to wake it up (it re-sends all the settings).
*/
void rot_axis_sleep(const struct rot_axis *axis)
{
    gpio_pin_set_dt(&axis->ena, 0);     // ENABLE pin off
    gpio_pin_set_dt(&axis->sleep, 0);   // nSLEEP low = sleep mode
    printk("%s is asleep \n", axis->name);
}


/* ---------- moving ---------- */

/*
    how long to wait for step number i out of total steps.
    near the start and end of the move it's slower (bigger interval),
    in the middle it's the cruise speed you asked for.
*/
static uint32_t ramp_interval_us(int i, int total, uint32_t cruise_us)
{
    int from_edge = MIN(i, total - 1 - i);   // how many steps from the nearest end of the move

    if (cruise_us >= START_INTERVAL_US || from_edge >= RAMP_STEPS) {
        return cruise_us;                    // already slow, or in the middle: cruise speed
    }

    // slide from START_INTERVAL_US (slow) down to cruise_us (fast) over RAMP_STEPS
    return START_INTERVAL_US - (START_INTERVAL_US - cruise_us) * from_edge / RAMP_STEPS;
}

// steps = how many steps, dir = 1 or 0, step_interval_us = cruise time per step (bigger = slower)
int rot_axis_move(const struct rot_axis *axis, int steps, int dir, uint32_t step_interval_us)
{
    // max speed cap: never go faster than MIN_STEP_INTERVAL_US per step
    if (step_interval_us < MIN_STEP_INTERVAL_US) {
        printk("%s: %u us per step is too fast, using %u us\n",
               axis->name, step_interval_us, MIN_STEP_INTERVAL_US);
        step_interval_us = MIN_STEP_INTERVAL_US;
    }

    // set the direction while the motor is stopped, then wait before the first step
    gpio_pin_set_dt(&axis->dir, dir);
    k_busy_wait(DIR_SETUP_US);

    for (int i = 0; i < steps; i++) {
        uint32_t interval = ramp_interval_us(i, steps, step_interval_us);

        gpio_pin_set_dt(&axis->step, 1);              // short, clean pulse
        k_busy_wait(STEP_PULSE_US);
        gpio_pin_set_dt(&axis->step, 0);
        k_busy_wait(interval - STEP_PULSE_US);        // rest of the step time
    }

    // let the motor fully stop and stop shaking before anything else happens
    // (most important before a direction change)
    k_msleep(SETTLE_MS);

    return steps;
}

// same as rot_axis_move, but in degrees. + is one way, - is the other
int rot_axis_move_deg(const struct rot_axis *axis, double degrees, uint32_t step_interval_us)
{
    int steps = (int)(fabs(degrees) * STEPS_PER_DEG + 0.5);   // degrees -> steps, rounded
    int dir = (degrees >= 0) ? 1 : 0;

    return rot_axis_move(axis, steps, dir, step_interval_us);
}