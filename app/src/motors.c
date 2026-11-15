// #include <stdio.h>
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>


/ This is accessing the DeviceTree node through its label (can see the entire devicetree in generated zephyr.dts file in the build directory)
#define LED2 DT_ALIAS(led0)

/*
    This is for the DM542T timing: 
    pulses must be >2.5 us 
    DIR needs to be set  >+ 5 us before stepping 
*/
#define X_STEP_PULSE_US 5
#define DIR_STEPUP_US 1000
#define MIN_STEP_INTERNVAL_US 10 


/*
    motor --> 200 full steps per rev x microstepping ( to match the uh dm542t dip switches)
*/
#define FULL_STEPS_PER_REV 200 
#define MICROSTEPS 4 
#define STEPS_PER_REV (FULL_STEPS_PER_REV * MICROSTEPS)

/* 
    for SPI
*/
#define DRI_SPI_SET (SPI_OP_MODE_MASTER | SPI_WORD_SET(16) | SPI_TRANSFER_MSB)


static const struct spi_dt_spec driver_x = SPI_DT_SPEC_GET(SPI3, driver_x);
static const struct spi_dt_spec driver_y = SPI_DT_SPEC_GET(USER_NODE, driver_y);
static const struct spi_dt_spec driver_z = SPI_DT_SPEC_GET(USER_NODE, driver_z);


/* 
GRAB DEM DEVICE TREE OVERLAY!!!! 
*/

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec x_step_pin = GPIO_DT_SPEC_GET(USER_NODE, x_step_gpios);
static const struct gpio_dt_spec y_step_pin = GPIO_DT_SPEC_GET(USER_NODE, y_step_gpios);
static const struct gpio_dt_spec z_step_pin = GPIO_DT_SPEC_GET(USER_NODE, z_step_gpios);

static const struct gpio_dt_spec x_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, x_dir_gpios);
static const struct gpio_dt_spec y_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, y_dir_gpios);
static const struct gpio_dt_spec z_dir_pin = GPIO_DT_SPEC_GET(USER_NODE, z_dir_gpios);

static const struct gpio_dt_spec x_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, x_ena_gpios);
static const struct gpio_dt_spec y_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, y_ena_gpios);
static const struct gpio_dt_spec z_ena_pin = GPIO_DT_SPEC_GET(USER_NODE, z_ena_gpios);

static const struct gpio_dt_spec x_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, x_limit_gpios);
static const struct gpio_dt_spec y_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, y_limit_gpios);
static const struct gpio_dt_spec z_limit_pin = GPIO_DT_SPEC_GET(USER_NODE, z_limit_gpios);

static const struct gpio_dt_spec x_sleep_pin = GPIO_DT_SPEC_GET(USER_NODE, x_sleep_gpios);
static const struct gpio_dt_spec y_sleep_pin = GPIO_DT_SPEC_GET(USER_NODE, y_sleep_gpios);
static const struct gpio_dt_spec z_sleep_pin = GPIO_DT_SPEC_GET(USER_NODE, z_sleep_gpios);

static const struct gpio_dt_spec x_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, x_nfault_gpios);
static const struct gpio_dt_spec y_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, y_nfault_gpios);
static const struct gpio_dt_spec z_nfault_pin = GPIO_DT_SPEC_GET(USER_NODE, z_nfault_gpios);


static int setup_pins(void) {

    /* 
        TODO :: REVISE THE CODE STRUCTURE.
    */
    
    // if these gpio pins are NOT ready, then we say that it snot ready 
    if (!gpio_is_ready_dt(&x_step_pin) ||
        !gpio_is_ready_dt(&y_step_pin) ||
        !gpio_is_ready_dt(&z_step_pin)) {
            printk("ERROR! STEP PINS ARE NOT READY\n");
            return -1;
        }

    if (!gpio_is_ready_dt(&x_dir_pin) ||
        !gpio_is_ready_dt(&y_dir_pin) ||
        !gpio_is_ready_dt(&z_dir_pin)) {
            printk("ERROR! DIR PINS ARE NOT READY\n");
            return -1;
        }

    if (!gpio_is_ready_dt(&x_ena_pin) ||
        !gpio_is_ready_dt(&y_ena_pin) ||
        !gpio_is_ready_dt(&z_ena_pin)) {
            printk("ERROR! ENA PINS ARE NOT READY\n");
            return -1;
        }

    if (!gpio_is_ready_dt(&x_limit_pin) ||
        !gpio_is_ready_dt(&y_limit_pin) ||
        !gpio_is_ready_dt(&z_limit_pin)) {
            printk("ERROR! LIMIT PINS ARE NOT READY\n");
            return -1;
        }

    if (!gpio_is_ready_dt(&x_sleep_pin) ||
        !gpio_is_ready_dt(&y_sleep_pin) ||
        !gpio_is_ready_dt(&z_sleep_pin)) {
            printk("ERROR! SLEEP PINS ARE NOT READY\n");
            return -1;
        }

    if (!gpio_is_ready_dt(&x_nfault_pin) ||
        !gpio_is_ready_dt(&y_nfault_pin) ||
        !gpio_is_ready_dt(&z_nfault_pin)) {
            printk("ERROR! NFAULT PINS ARE NOT READY\n");
            return -1;
        }

    if (!spi_is_ready_dt(&driver_x) || 
        !spi_is_ready_dt(&driver_y) ||
        !spi_is_ready_dt(&driver_z)) {
            printk("ERROR! DRIVER PINS ARE NOT READY\n");
            return -1;
        }

        // if these gpio pin is ready, then we set it as inactive 
        gpio_pin_configure_dt(&x_step_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_step_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_step_pin, GPIO_OUTPUT_INACTIVE);

        gpio_pin_configure_dt(&x_dir_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_dir_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_dir_pin, GPIO_OUTPUT_INACTIVE);

        gpio_pin_configure_dt(&x_ena_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_ena_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_ena_pin, GPIO_OUTPUT_INACTIVE);

        gpio_pin_configure_dt(&x_limit_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_limit_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_limit_pin, GPIO_OUTPUT_INACTIVE);

        gpio_pin_configure_dt(&x_sleep_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_sleep_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_sleep_pin, GPIO_OUTPUT_INACTIVE);

        gpio_pin_configure_dt(&x_nfault_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&y_nfault_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&z_nfault_pin, GPIO_OUTPUT_INACTIVE);

        spi_pin_configure_dt(&driver_x, )



        //this dt func merges the gpio_input w the overlay flags. 
        // aka the input + pull-up + active low 
        //gpio_pin_configure_dt(&limit_pin, GPIO_INPUT);

        // fire pew pew an interupt the moment this pin becomes active (aka the limit switch is hit)
        //gpio_pin_interrupt_configure_dt(&limit_pin, GPIO_INT_EDGE_TO_ACTIVE);

        // so this fills out the hall pass 
        // BIT(limit_pin.pin) turns pin number 14 --> pin mask 
        // aka j a number with the bit 14 switched on  --> on the pass. 
        // limit_cb_darta is like oh when pin 14 is pressed, call limit_isr.
        //gpio_init_callback(&limit_cb_data, limit_isr, BIT(limit_pin.pin));
        // hands hall pass to port B driver. 
        //gpio_add_callback(limit_pin.port, &limit_cb_data);

        printk("PINS ARE READY\n");
        return 0; 
};

