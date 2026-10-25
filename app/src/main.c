// #include <stdio.h>
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>

// This is accessing the DeviceTree node through its label (can see the entire devicetree in generated zephyr.dts file in the build directory)
#define LED2 DT_ALIAS(led0)

// 'gpios' refers to the property name within the led0 node, which is itself inside the leds parent node in the devicetree
// you can see this^^ in code if you search for the 'zephyr/boards/nordic/nrf54l15dk/nrf54l15dk_common.dtsi' file
static const struct gpio_dt_spec led_2 = GPIO_DT_SPEC_GET(LED2, gpios);

int main(void)
{
    /*

    ISRs: 
        ISRs will be active thorughout the code and will be alive(?) in any part of the states. 
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

            <EXIT WHEN> all messages are turned into a strucutrued format --> MOTION COMMANDS

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
            we set a time limit (2 minutes?) and poll to make sure that we don't get any additional commands from the app. 
        
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
    if (!device_is_ready(led_2.port))
    {
        return 0;
    }

    int state = 0;

    int ret;
    // configuration of led_0 pin
    ret = gpio_pin_configure_dt(&led_2, GPIO_OUTPUT);

    if (ret != 0)
    {
        return 0;
    }

    while (true)
    {
        state = !state;
        ret = gpio_pin_set_dt(&led_2, state);
        if (ret != 0)
        {
            return 0;
        }
        printk("LED is now in state: %d", state);

        k_msleep(400);
    }

    return 0;
}
