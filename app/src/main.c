// #include <stdio.h>
#include <zephyr/kernel.h>
#include "motors.h"
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>


#define FULL_STEPS_PER_REV 200 
#define MICROSTEPS 4 
#define STEPS_PER_REV (FULL_STEPS_PER_REV * MICROSTEPS)

#define USER_NODE DT_PATH(zephyr_user)
static const struct gpio_dt_spec fw_status_pin = GPIO_DT_SPEC_GET(USER_NODE, fwe_status_led_gpios);



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
    


    if (axis_setup(&x_axis) != 0) {
        return 0;
    }
    if (axis_setup(&y_axis) != 0) {
        return 0;
    }
    if (axis_setup(&z_axis) != 0) {
        return 0;
    }

    if (axis_enable(&z_axis) != 0) {
        return 0;
    }

    axis_move(&z_axis, 800, 1, 1000);
    axis_move(&z_axis, 800, 0, 1000);

    gpio_pin_configure_dt(&fw_status_pin, GPIO_OUTPUT_INACTIVE);

    while (true) {
        gpio_pin_toggle_dt(&fw_status_pin);   // flip the LED on/off
        k_msleep(500);                        // wait half a second
    }


    return 0;

}
