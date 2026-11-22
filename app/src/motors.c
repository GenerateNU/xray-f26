// #include <stdio.h>
#include "motors.h"
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>


/*
    This is for the DM542T timing: 
    pulses must be >2.5 us 
    DIR needs to be set  >+ 5 us before stepping 
*/
#define X_STEP_PULSE_US 5
#define DIR_STEPUP_US 1000 // confirm after ehcking data sheet
#define MIN_STEP_INTERVAL_US 10 


// driver registers 
#define DRV_REG_CTRL2 0x05 //MICROSTEPPING BITS 0-3 + STEP/DIR pins we're leaving at 0
//#define DRV_REG_CTRL9 0x0c // EN_AUTO in bit 0 (AUTOMICROSTEPPING!!!)
//#define EN_OL_BIT 0x80 // bit 7 in CTRL9
#define DRV_READ_BIT (1u << 14) // bit 14 = 1 means read 

// EN_PUT
#define DRV_REG_CTRL1 0x04
#define CTRL1_DEFAULT   0x0F   /* TOFF=19us, DECAY=smart tune ripple control (datasheet defaults) */
#define EN_OUT_BIT      0x80  

#define MICROSTEP_1_4 0x04
#define EN_AUTO_BIT 0x01

/* 
Z AXIS 
#define DRV_REG_CTRL11   0x0E   // TRQ_DAC: the current dial (0-255)
#define DRV_REG_CTRL13   0x10   // VREF_INT_EN lives in bit 1
#define TRQ_DAC_VALUE    0xC9 
#define DRV_REG_CTRL10   0x0D   // ISTSL: holding current
#define ISTSL_VALUE      TRQ_DAC_VALUE   // same as moving current (for now)
#define CTRL13_VREF_INT  0x12   // internal 3.3V reference on (bit 1), rest at default


*/
// VREF / current
#define DRV_REG_CTRL11   0x0E   // TRQ_DAC: the current dial (0-255)
#define DRV_REG_CTRL13   0x10   // VREF_INT_EN lives in bit 1
#define TRQ_DAC_VALUE    0xC9 
//#define TRQ_DAC_VALUE    0x97 
#define DRV_REG_CTRL10   0x0D   // ISTSL: holding current
#define ISTSL_VALUE      TRQ_DAC_VALUE   // same as moving current (for now)
#define CTRL13_VREF_INT  0x12   // internal 3.3V reference on (bit 1), rest at default


/*
    motor --> 200 full steps per rev x microstepping ( to match the uh dm542t dip switches)
*/
#define FULL_STEPS_PER_REV 200 
#define MICROSTEPS 4
#define STEPS_PER_REV (FULL_STEPS_PER_REV * MICROSTEPS)


/* 
    for SPI

   
*/
#define DRI_SPI_SET (SPI_OP_MODE_CONTROLLER | SPI_WORD_SET(16) | SPI_TRANSFER_MSB | SPI_MODE_CPHA)

/* 
GRAB DEM DEVICE TREE OVERLAY!!!! 
*/

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec motion_en = GPIO_DT_SPEC_GET(USER_NODE, motion_enable_gpios);
static const struct gpio_dt_spec motor_arm = GPIO_DT_SPEC_GET(USER_NODE, motor_arm_gpios);
static const struct gpio_dt_spec estop_status = GPIO_DT_SPEC_GET(USER_NODE, estop_pin_gpios);
static const struct gpio_dt_spec arm_status = GPIO_DT_SPEC_GET(USER_NODE, motor_arm_status_gpios);


static int drv_write_reg(const struct axis *axis, uint8_t addy, uint8_t value) 
{
    // build the addy into bits 13-8, value into bits 7-0
    // rmbr:: bits 14 & 15 stay @ 0 for write
    // the << 8 is sliding it 8 slots left into bits 13-8, then drop the value into bottom 8 bits
    uint16_t tx = ((addy & 0x3F) << 8) | value;

    // wrap these presents 
    struct spi_buf buf = { .buf = &tx, .len = sizeof(tx) }; // single msg at &tx and how long it is
    struct spi_buf_set tx_set = { .buffers = &buf, .count = 1 }; // stack of msg, but just 1 msg

    // send the presents 
    return spi_write_dt(&axis->spi, &tx_set);
}
static int drv_read_reg(const struct axis *axis, uint8_t addy, uint8_t *value) {

    // we build. we read CTRL2. 
    uint16_t tx = DRV_READ_BIT | ((addy & 0x3F) << 8); // bit 14 = 1 means read, bits 13-8 = addy, bits 7-0 = 0
    uint16_t rx = 0; // where the presents will be received

    //wrap the presents, but now for 2. one for tx and one for rx 
    struct spi_buf tx_buf = { .buf = &tx, .len = sizeof(tx) };
    struct spi_buf rx_buf = { .buf = &rx, .len = sizeof(rx) };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    // spi_transsceive dt swaps notes. done? rx holds the driver's reply 
    int ret = spi_transceive_dt(&axis->spi, &tx_set, &rx_set);

    *value = rx & 0xFF; // grab the info inside of value of this addy
    return ret;
}

static int drv_configure(const struct axis *axis) {

    uint8_t value;

    // set the microstepping to 1/4
    if (drv_write_reg(axis, DRV_REG_CTRL2, MICROSTEP_1_4) != 0) {
        printk("ERROR! %s axis failed to set microstepping \n", axis->name);
        return -1;
    }


    /* 
    
    if (drv_write_reg(axis, DRV_REG_CTRL9, EN_AUTO_BIT) != 0) {
        printk("ERROR! %s axis failed to set auto microstepping \n", axis->name);
        return -1;
    }


    if (drv_write_reg(axis, DRV_REG_CTRL9, EN_AUTO_BIT) != 0) {
        printk("ERROR! %s axis failed to set open-loop \n", axis->name);
        return -1;

    }


    // set the auto microstepping bit 
    
    // set the open-loop bit
    // open load detection is off for now
    // aka if u wanna turn it on, jyst add EN_OL_Bit ||


    drv_read_reg(axis, DRV_REG_CTRL9, &value);
    if (value != EN_AUTO_BIT) {
        printk("ERROR! %s CTRL9 readback 0x%02X \n", axis->name, value);
        return -1;
    }
    
    */



    drv_write_reg(axis, DRV_REG_CTRL11, TRQ_DAC_VALUE);    // 1. set the current
    drv_write_reg(axis, DRV_REG_CTRL10, ISTSL_VALUE);      // holding current (same for now)
    drv_write_reg(axis, DRV_REG_CTRL13, CTRL13_VREF_INT);  // 2. turn on the internal reference

    printk("%s axis is configured \n", axis->name);
    return 0;
}


const struct axis x_axis = {
    .name = "X",
    .step   = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), step_gpios),
    .dir    = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), dir_gpios),
    .ena    = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), ena_gpios),
    .limit  = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), limit_gpios),
    .sleep  = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), sleep_gpios),
    .nfault = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_x), nfault_gpios),
    .spi    = SPI_DT_SPEC_GET(DT_NODELABEL(driver_x), DRI_SPI_SET),
};

const struct axis y_axis = {
    .name = "Y",
    .step = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), step_gpios),
    .dir = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), dir_gpios),
    .ena = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), ena_gpios),
    .limit = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), limit_gpios),
    .sleep = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), sleep_gpios),
    .nfault = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_y), nfault_gpios),
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(driver_y), DRI_SPI_SET),
};

const struct axis z_axis = {
    .name = "Z",
    .step = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), step_gpios),
    .dir = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), dir_gpios),
    .ena = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), ena_gpios),
    .limit = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), limit_gpios),
    .sleep = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), sleep_gpios),
    .nfault = GPIO_DT_SPEC_GET(DT_NODELABEL(driver_z), nfault_gpios),
    .spi = SPI_DT_SPEC_GET(DT_NODELABEL(driver_z), DRI_SPI_SET),
};

bool safe_setup(const struct axis *axis) {
    
    if (gpio_pin_get_dt(&axis->nfault) == 1) {
        printk("ERROR! %s axis has a fault \n", axis->name);
        return 0;
    }


    return 1;
}

// THIS IS OUR NEW LIKE PIN SETUP 

int axis_setup(const struct axis *axis) {
    
    if (!gpio_is_ready_dt(&axis->step) ||
        !gpio_is_ready_dt(&axis->dir) ||
        !gpio_is_ready_dt(&axis->ena) ||
        !gpio_is_ready_dt(&axis->limit) ||
        !gpio_is_ready_dt(&axis->sleep) ||
        !gpio_is_ready_dt(&axis->nfault)) {
            printk("ERROR! GPIO PORT IS NOT READY\n");
            return -1;
        }

    if (!spi_is_ready_dt(&axis->spi)) {
        printk("ERROR! SPI PORT IS NOT READY\n");
        return -1;
    }

    gpio_pin_configure_dt(&axis->step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->ena, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&axis->sleep, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&axis->nfault, GPIO_INPUT);
    gpio_pin_configure_dt(&axis->limit, GPIO_INPUT);

    printk("%s axis is ready \n", axis->name);

    return 0;

};

/* 

int motor_power(void) {

    if (!gpio_is_ready_dt(&motion_en) || !gpio_is_ready_dt(&motor_arm)) {
        printk("ERROR! MOTOR POWER IS NOT READY\n");
        return -1;
    }

    gpio_pin_configure_dt(&estop_status, GPIO_INPUT); 
    gpio_pin_configure_dt(&arm_status, GPIO_INPUT);
    gpio_pin_configure_dt(&motion_en, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&motor_arm, GPIO_OUTPUT_INACTIVE);

     printk("estop status: %d, armed status: %d\n",
           gpio_pin_get_dt(&estop_status), gpio_pin_get_dt(&arm_status));
    
    gpio_pin_set_dt(&motion_en, 1); // ENABLE MOTION
    k_msleep(5); // wait for the enable to take effect
    gpio_pin_set_dt(&motor_arm, 1); // ARM MOTOR
    k_msleep(5);

    return 0;
}


*/

int motor_power(void) {
    if (!gpio_is_ready_dt(&motion_en) || !gpio_is_ready_dt(&motor_arm) ||
        !gpio_is_ready_dt(&estop_status) || !gpio_is_ready_dt(&arm_status)) {
        printk("ERROR! MOTOR POWER PINS NOT READY\n");
        return -1;
    }

    gpio_pin_configure_dt(&estop_status, GPIO_INPUT);
    gpio_pin_configure_dt(&arm_status, GPIO_INPUT);
    gpio_pin_configure_dt(&motion_en, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&motor_arm, GPIO_OUTPUT_INACTIVE);   // starts LOW

    // 1. give permission
    gpio_pin_set_dt(&motion_en, 1);
    k_msleep(5);

    // 2. is the e-stop released? (HIGH = OK)
    if (gpio_pin_get_dt(&estop_status) != 1) {
        printk("ERROR! E-STOP pressed or not connected\n");
        return -1;
    }

    // 3. arm: LOW -> HIGH edge latches U6
    gpio_pin_set_dt(&motor_arm, 0);
    k_msleep(1);
    gpio_pin_set_dt(&motor_arm, 1);
    k_msleep(5);

    // 4. did it actually arm?
    if (gpio_pin_get_dt(&arm_status) != 1) {
        printk("ERROR! board did not arm\n");
        return -1;
    }

    printk("MOTORS ARMED\n");
    return 0;
}


int axis_enable(const struct axis *axis) {
    gpio_pin_set_dt(&axis->sleep, 1); //WAKE UPPP
    k_msleep(2);

    if (drv_configure(axis) != 0) {
        printk("ERROR! %s axis failed to configure \n", axis->name);
        return -1;
    }

    // we're using an active low so 1 --> FAULT.
    if (gpio_pin_get_dt(&axis->nfault) == 1) { 
        printk("ERROR! %s axis has a fault \n", axis->name);
        return -1;
    }


    gpio_pin_set_dt(&axis->ena, 1); // ENABLE MOTOR physcial wire from STM 32--ENable pin on driver
    drv_write_reg(axis, 0x04, 0x8F);    //EN_OUT on (aka a switch inside the chip)

    uint8_t value;

    drv_read_reg(axis, 0x04, &value);              // CTRL1: outputs on?
    if (value != 0x8F) return -1;

    drv_read_reg(axis, DRV_REG_CTRL11, &value);    // current setting
    if (value != TRQ_DAC_VALUE) return -1;

    drv_read_reg(axis, DRV_REG_CTRL13, &value);    // internal reference on?
    if (value != CTRL13_VREF_INT) return -1;

    drv_read_reg(axis, 0x00, &value);              // FAULT register
    if (value & 0x80) return -1;                   // bit 7 = any fault

    
    printk("%s axis is enabled \n", axis->name);
    return 0;
};

//MOVE THE AXIS 

// which motor? (x,y,z?), 
// yo btw steps is j how many steps the motor takes (in microsteps) 

int axis_move (const struct axis *axis, int steps, int dir, uint32_t interval_us) {

    //spi_cs_control(&axis->spi, 1, 0); // set the CS pin to inactive (high) and 0 delay

    // make sure its not going too fast 
    if (interval_us < MIN_STEP_INTERVAL_US) {
        printk("ERROR: %s interval too small (%u us)\n", axis->name, interval_us);
        return 0; 
    };
    //set direction to whatever dir is @
    // we reach a specific motor's pin, direction
    gpio_pin_set_dt(&axis->dir, dir);
    k_busy_wait(DIR_STEPUP_US);

    // pulsing both high and low 
    for (int i = 0; i < steps; i ++) {
        gpio_pin_set_dt(&axis->step, 1);
        k_busy_wait(interval_us / 2);
        gpio_pin_set_dt(&axis->step, 0);
        k_busy_wait(interval_us - interval_us / 2);
    };

    return steps;
};