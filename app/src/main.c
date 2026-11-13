// #include <stdio.h>
#include <zephyr/kernel.h>
// zephyr/device.h is included eventually through kernel.h
#include <zephyr/drivers/gpio.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#define RADIATIONINDEX 1
#define CEHCKSUMINDEX 2

#define STX 0x02
#define ETX 0x03
/// ask about this and overlays
static const struct device *uart_feather = DEVICE_DT_GET(DT_NODELABEL(usart1));

//single buffer for now
RING_BUF_DECLARE(bufA, 128);
// buffer with only radiation data
RING_BUF_DECLARE(bufRad, 20);
K_SEM_DEFINE(semFeather, 0, 1);

volatile uint8_t threshold = 1; // need to figure out how the app can change this value.

void feather_rx_uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf[20];
    int num_bytes;

    if (!uart_irq_update(dev)) {
        return;
    }

    // keep putting into buffer, give semaphore
    while (uart_irq_rx_ready(dev)) {
        num_bytes = uart_fifo_read(dev, buf, sizeof(buf));
        if (num_bytes > 0) {
            ring_buf_put(&bufA, buf, num_bytes);
            k_sem_give(&semFeather);
        }
    }
}

int parse_packet(uint8_t frame[]) {
    // the frame is structured as STX radiation checksum ETX
    // checksum is raditation value plus STX (2)
    uint8_t checksumVal = frame[CEHCKSUMINDEX]; 
    uint8_t checksum = (STX + frame[RADIATIONINDEX]);
    uint8_t radiation = frame[RADIATIONINDEX];

    if (checksumVal != checksum) {
        return -1;
    }

    if (radiation >= threshold ) {
        // activate interupt.
        // irq_enable(IRQ_Radiation);
    }

    printk("%d\n", radiation);

    // single buffer for now
    ring_buf_put(&bufRad, &radiation, sizeof(radiation));

    /* double buffer
    if (ring_buf_space_get(&bufA) == 0) {
         printk("Buffer B");
        ring_buf_put(&bufB, radiation, 1);
    } else {
        printk("Buffer A");
        ring_buf_put(&bufA, radiation, 1);
    }
    */
    return 0;
}

void buf_reader(void *p1, void *p2, void *p3) {
    uint8_t buf[32];
    uint8_t packet[4];
    int packetLen = 0;

    while (1) {
        k_sem_take(&semFeather, K_FOREVER);

        int num_bytes;
        while ((num_bytes = ring_buf_get(&bufA, buf, sizeof(buf))) > 0) {
            for (int i = 0; i < num_bytes; i++) {
                uint8_t b = buf[i];

                if (packetLen == 0) {          // waiting for start
                    if (b == STX) {
                        packet[packetLen++] = b;
                    }
                    continue;
                }

                packet[packetLen++] = b;       // STX, rad, checksum, ETX
                if (packetLen == 4) {
                    if (b == ETX) {
                        parse_packet(packet);  // checksum checked inside
                    }
                    packetLen = 0;             // reset for next STX
                }
            }
        }
    }
}

K_THREAD_DEFINE(reader, 1024, buf_reader, NULL, NULL, NULL, 5, 0, 0);

int feather_serial_isr_init(void) {
    // setup
    if (!device_is_ready(uart_feather)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_feather, feather_rx_uart_cb, NULL);
    uart_irq_rx_enable(uart_feather);

    return 0;
}

// This is accessing the DeviceTree node through its label (can see the entire devicetree in generated zephyr.dts file in the build directory)
#define LED2 DT_ALIAS(led0)

// 'gpios' refers to the property name within the led0 node, which is itself inside the leds parent node in the devicetree
// you can see this^^ in code if you search for the 'zephyr/boards/nordic/nrf54l15dk/nrf54l15dk_common.dtsi' file
static const struct gpio_dt_spec led_2 = GPIO_DT_SPEC_GET(LED2, gpios);

int main(void)
{
    feather_serial_isr_init();

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
       k_msleep(100);
    }
}
