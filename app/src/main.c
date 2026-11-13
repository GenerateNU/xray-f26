// #include <stdio.h>
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>

// This is accessing the DeviceTree node through its label (can see the entire devicetree in generated zephyr.dts file in the build directory)
#define LED2 DT_ALIAS(led0)

/*
    This is for the DM542T timing: 
    pulses must be >2.5 us 
    DIR needs to be set  >+ 5 us before stepping 
*/
#define STEP_PULSE_US 5 
#define DIR_STEPUP_US 1000
#define MIN_STEP_INTERNVAL_US 10 

//pwm
#define DIR_CHANGE_SETTLE_US  1000 

/*
    motor --> 200 full steps per rev x microstepping ( to match the uh dm542t dip switches)
*/
#define FULL_STEPS_PER_REV 200 
#define MICROSTEPS 4 
#define STEPS_PER_REV (FULL_STEPS_PER_REV * MICROSTEPS)

// once there is a gantry: steps per mm = STEPS_PER_REV / mm per rev (<-- this is essentially the lenght of the thing)

/* 
GRAB DEM DEVICE TREE OVERLAY!!!! 
*/

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec step_pin = GPIO_DT_SPEC_GET(USER_NODE, step_gpios);
static const struct gpio_dt_spec dir_pin = GPIO_DT_SPEC_GET(USER_NODE, dir_gpios);
static const struct gpio_dt_spec ena_pin = GPIO_DT_SPEC_GET(USER_NODE, ena_gpios);


//static uint32_t stop_cycles;

// 'gpios' refers to the property name within the led0 node, which is itself inside the leds parent node in the devicetree
// you can see this^^ in code if you search for the 'zephyr/boards/nordic/nrf54l15dk/nrf54l15dk_common.dtsi' file
static const struct gpio_dt_spec led_2 = GPIO_DT_SPEC_GET(LED2, gpios);


//static volatile bool limit_hit = false; 

// this is WHEN the switch was a hit
//static volatile uint32_t limit_hit_cycles; 

// zephyr uses this to keep track of the ISR --> hall pass
//static struct gpio_callback limit_cb_data;

/*

static void limit_isr(const struct device *dev, 
                    struct gpio_callback *cb, 
                        uint32_t pins) {
    limit_hit = true; 

    limit_hit_cycles = k_cycle_get_32(); 
};

*/


static int setup_pins(void) {

    
    // if these gpio pins are NOT ready, then we say that it snot ready 
    if (!gpio_is_ready_dt(&step_pin) ||
        !gpio_is_ready_dt(&dir_pin) ||
        !gpio_is_ready_dt(&ena_pin)) {
            printk("ERROR! GPIO PORT IS NOT READY\n");
            return -1;
        }

        // if these gpio pin is ready, then we set it as inactive 
        gpio_pin_configure_dt(&step_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&dir_pin, GPIO_OUTPUT_INACTIVE);
        gpio_pin_configure_dt(&ena_pin, GPIO_OUTPUT_ACTIVE);


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

/* 
Legit only for one step 
*/
static void step_once(void) {
    gpio_pin_set_dt(&step_pin, 1);
    // spin cpu for 5 us so that the pulse is long enough for the driver
    k_busy_wait(STEP_PULSE_US);
    // STEP goes back down to low one stop? DONE. 
    gpio_pin_set_dt(&step_pin, 0);
};


/* 
move the motor for n steps

steps --> how many steps
dir --> which way 
the other one --> how fast (a postitive whole number)
*/
static int move_steps(int steps, int dir, uint32_t step_interval_us) {

    if (step_interval_us < MIN_STEP_INTERNVAL_US) {
        printk("ERROR! STEP INTERVAL TOO SMALL. \n");
        return 0; 
    };
    gpio_pin_set_dt(&dir_pin, dir);
    k_busy_wait(DIR_STEPUP_US);



    /* 4. the for-loop below starts pulsing */


    // repeat steps time. i is the counter.
    for (int i = 0; i < steps; i ++) {
        /*
        if (limit_hit) {
            stop_cycles = k_cycle_get_32();  // timestamp fo when it hit & stopped 
            return i; 
        };
        */
    
        step_once();
        k_busy_wait(step_interval_us - STEP_PULSE_US);

    };

    return steps;
};


/* 
 --------------TESTS!!-----------------------
*/

/*
static void test_slow_steps(void) {
    printk("\n [TEST 01] 10 slow steps, 1 step/sec");
    for (int i = 1; i <= 50; i ++) {
        // reset the limit hit flag
        //limit_hit = false;
        //ticky tock. one tick is 16 pulses 
        int done = move_steps(MICROSTEPS, 1, 1000);
        printk("[TEST 01] tick %d: %d / %d microsteps \n", i, done, MICROSTEPS);
        k_msleep(1000);
    };
};
*/




static void test_one_rev(void) {
    const uint32_t interval_us = 500;

    printk("\n[TEST 02] 1 rev forward, 1 back, 4 times\n");

    for (int round = 1; round <= 4; round++) {
        //limit_hit = false;

        int fwd = move_steps(STEPS_PER_REV, 1, interval_us);
        k_msleep(500);

        int back = move_steps(STEPS_PER_REV, 0, interval_us);
        printk("[TEST 02] round %d: fwd %d, back %d, expected %d each\n",
               round, fwd, back, STEPS_PER_REV);
        k_msleep(500);
    };
};

/* 

static void test_speed_sweep(void) {

    static const uint32_t intervals_us[] = {2000, 1000, 500, 300 , 200};

    // steps_per_sec * 5, 1, 1000 us/step -> 12 RPM | 0.73 A, 17.56
    // steps_per_sec * 5, 1, 500 us/step --> 24 RPM |  0.853 A, 20.47
    // steps_per_sec * 5, 1, 300 us/step --> 0.7 RPM |  0.765 A,  18.67
    // steps_per_sec * 5, 1, 200 us/step --> 60 RPM | 0.491 A, 11.78
    

    int num_speeds = sizeof(intervals_us) / sizeof(intervals_us[0]);

    printk("\n [TEST 03] SPEED SWEEEP --> 2s at each speed, will proceed from slow to fast");
    printk("[TEST 03] watch for rough spots or stalling! \n ");

    for (int s = 0; s < num_speeds; s++) {
        uint32_t steps_per_sec = 1000000 / intervals_us[s];
        uint32_t rpm = steps_per_sec * 60 / STEPS_PER_REV;
        int steps = steps_per_sec * 10;

        printk("[TEST 03] %u us/step = %u steps/s = %u RPM\n", intervals_us[s], steps_per_sec, rpm);

        //limit_hit = false;

        int done = move_steps(steps, 1, intervals_us[s]); 
        if (done < steps) {
            printk("[TEST 03] aborted \n");
            return;
        };

        k_msleep(1000);

        printk("[TEST 03] DONE");

    }
}

*/



/*
static void test_constant_speed(void){
    printk("\n [TEST 04] SPEED SWEEEP --> 2s at each speed, will proceed from slow to fast");

    static const uint32_t intervals_us[] = {800};
    // about 60 RPM
    int num_speeds = sizeof(intervals_us) / sizeof(intervals_us[0]);

    for (int s = 0; s < num_speeds; s++) {
        uint32_t steps_per_sec = 1000000 / intervals_us[s];
        uint32_t rpm = steps_per_sec * 60 / STEPS_PER_REV;
        int steps = steps_per_sec * 15;

        printk("[TEST 03] %u us/step = %u steps/s = %u RPM\n", intervals_us[s], steps_per_sec, rpm);

        //limit_hit = false;

        int done = move_steps(steps, 1, intervals_us[s]); 
        if (done < steps) {
            printk("[TEST 03] aborted \n");
            return;
        };

    }
}
*/


int main(void)
{
    /*

    ISRs: 
        ISRs will be active thorughout the code and will be alive in any part of the states. 
        used for: limit switches, e-stop, and nfault handling 


    CASE HOMING 
        we begin the homing state, where we calibrate our motors. 
        calibrate from the zmotor --> x motor --> y motor.

        check: are all the motors homed in n time? 
            no? ERROR. 
        
    <EXIT WHEN> HOMING SUCCESS --> CASE IDLE

    CASE IDLE
    we wait until we are given an command from the app 
    we will be using an ISR to look out for a command from the app. 

    <EXIT WHEN> TASK GIVEN --> CASE TASK 

    CASE TASK
        CASE SCAN/JOG:  
            [COMMAND PARSER]
                turns raw serial messages into strucutred formats: 
                    SCAN: 
                        SCAN + face list
                    JOG: 
                        JOG + axis/direction

            <EXIT WHEN> all messages are turned into a structured format --> MOTION COMMANDS

            [MOTION COMAMNDS] 
                SCAN: 
                    turns the structured command into target positions (mm -> steps).
                    
                JOG: 
                    this part translates arrow keys --> movement of the motors 

            <EXIT WHEN> all motion commands are clarified --> MOTION PLANNER

            [MOTION PLANNER] 
                SCAN: 
                    takes in the return values from the motion commands are fed into the motion planner. 
                    Uses that info + raster sweep to complete the scans. 
                    FOR MULTIPLE FACES SCAN: calculates the order of the faces that needs to be scanned. 
                    ALSO, coordiantes multi-axis scans 

                JOG: 
                    simple single axis movement corresponding to the commands given from the app. 
            
            <EXIT WHEN> there is a full & complete motion plan for the motors --> MOTION EXECUTOR
                    

            [MOTION EXECUTOR]
                SCAN & JOG: 
                    takes in the return values from the motion planner and actually executes the motion.
                    configures the DRV8462 at startup, sends commands to the zephyr's built in stepper motor api that sends the signals to
                    our motor drivers. 

            <EXIT WHEN> all motion commands are executed --> CASE TASK COMPLETED


        CASE TASK COMPLETED
            we set a time limit (2 minutes) and poll to make sure that we don't get any additional commands from the app. 
        
        <EXIT WHEN> either: 
            1. we get a new command from the app --> CASE TASK
            2. we don't get any new commands from the app after time limit--> CASE STOP MACHINE

        CASE ERROR 
            split into 2 seperate conditions
                CASE CRITICAL
                (e-stop/limit switch/nfault)
                    machine is shut down and the user is notified of the error.
                
                <EXIT WHEN> user does a full reset --> CASE HOMING
                
                CASE NON-CRITICAL
                (radiation)
                    machine is not shut down, but the user is notified of the error.
                
                <EXIT WHEN> problamtic area is rescanned, and is normal --> CASE TASk

        CASE STOP MACHINE
            motor z is lowered down and the power is shut down 
    */
    

    // led_0.port is used because you need to check if the gpio controller itself is ready to be used, if you just do led_0.pin you will get a number like '9'
   /* LED on = new firmware is running */
    if (!device_is_ready(led_2.port)) {
        return 0;
    }
    gpio_pin_configure_dt(&led_2, GPIO_OUTPUT_ACTIVE);

    /* TESTING THE MOTORS */
    if (setup_pins() != 0) {
        printk("ERROR! PINS ARE NOT READY\n");
        return 0;
    }

    k_msleep(2000);
    //test_slow_steps();
    test_one_rev();
    //test_speed_sweep();
    //test_constant_speed();
    //about 62 rpm
    //test_run_seconds(10, 300);

    
    printk("\n ALL TESTS ARE DONE! \n");

    return 0;

};