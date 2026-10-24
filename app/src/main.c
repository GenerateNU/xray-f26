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
    while (true) {
        /* switch (state) {
            case STATE_IDLE:
                wait for command from the app to start scanning.
            case STATE_TESTING:
                test trying to read data
                the data will be stored in a buffer.
            case STATE_MONITORING:
                move motors and scan for radiation, continue to read data
                move motors left to right and then right to left row by row.
                when done with one face, rotate to another face.
            case STATE_RESCAN:
                reconfirm radiation level reading,
                move motors to scan the area agai.
                after confirmation, switch back to STATE_MONITORING
            case STATE_CLEANUP:
                clean up after done with scanning, 
                move motors to home whith limit siwtches.
                send report to the app
                switch to STATE_IDLE
           }
        */
    }
}
