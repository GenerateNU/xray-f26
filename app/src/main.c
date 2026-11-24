#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include "feather_uart.h"

#include "motors.h"

#define STX 0x02
#define ETX 0x03

#define ACK 0x06
#define NAK 0x15

RING_BUF_DECLARE(buf_pc, 20);
K_SEM_DEFINE(pc_send_sem, 0, 1);

typedef struct {
    int len;
    uint8_t frame[10];
} tx_frame;

typedef enum {
    IDLE,
    GET_SCAN_DATA,
    SEND_SCAN_DATA,
} pc_mcu_communication_t;

volatile pc_mcu_communication_t pc_mcu_communication_state = IDLE;

K_MSGQ_DEFINE(pc_tx_q, sizeof(tx_frame), 8, 1);

static const struct device *uart_pc = DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

volatile bool MOTOR_START = false;

// callback for when data arrives on the rx line
void pc_rx_uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf;
    int num_bytes;

    uart_irq_update(dev);

    while (uart_irq_rx_ready(dev)) {
        num_bytes = uart_fifo_read(dev, &buf, sizeof(buf));
        if (num_bytes <= 0) {
            break;              // FIFO empty: exit instead of looping forever
        }
        
        if (num_bytes > 0) {
            printk("%u", buf);
            if (buf == STX) {
                // move motors
                MOTOR_START = true;
            }
        }
    }
}

int pc_serial_isr_init() {
    // setup
    if (!device_is_ready(uart_pc)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_pc, pc_rx_uart_cb, NULL);

    uart_irq_rx_enable(uart_pc);
    return 0;
}

int main(void)
{
    printk("starting\n");

    pc_serial_isr_init();

    axis_setup(&z_axis);
    axis_enable(&z_axis);
    motor_power();

    while (true) {
        if (MOTOR_START) {
            axis_move(&z_axis, 1600, 1, 500);
        }

        printk("starting\n");
        k_msleep(5000);
    }
}