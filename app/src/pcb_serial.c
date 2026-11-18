#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include "feather_uart.h"

#define STX 0x02
#define ETX 0x03

#define ACK 0x06
#define NAK 0x15

RING_BUF_DECLARE(buf_pc, 20);
K_SEM_DEFINE(pc_send_sem, 0, 1);

static const struct device *uart_pc = DEVICE_DT_GET(DT_NODELABEL(usart2)); // will change

// callback for when data arrives on the rx line
void pc_rx_uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf[20];
    int num_bytes;

    uart_irq_update(dev);

    // keep putting data into buffer, give semaphore
    while (uart_irq_rx_ready(dev)) {
        num_bytes = uart_fifo_read(dev, buf, sizeof(buf));
        if (num_bytes > 0) {
            ring_buf_put(&buf_pc, buf, num_bytes);
            k_sem_give(&pc_send_sem);
        }
    }
}

// mcu response during idle state
int pc_rx_wake_response() {
    uint8_t buf[1];

    // take semaphore and send "ACK" when "@" is recieved.
    while (1) {
        k_sem_take(&pc_send_sem, K_FOREVER);
        ring_buf_get(&buf_pc, buf, sizeof(buf));

        if (buf[0] == '@') {
            uart_poll_out(uart_pc, ACK);
            return 0;
        }
    }
}

// parse through data that the app sends. The frame is in works, so it's gonna change.
int pc_rx_buf_parser() {
    // will chnage
    uint8_t buf[20];
    uint8_t n;
    uint8_t packet[4];
    int packetLen = 0;

    k_sem_take(&pc_send_sem, K_FOREVER);

    n = ring_buf_get(&buf_pc, buf, sizeof(buf));

    bool startFound = false;

    if (n > 1) {
        for (int i = 0; i < n; i++) {
            if (buf[i] == STX) {
                startFound = true;
                packet[packetLen] = buf[i];
                packetLen++;
                continue;
            }

            if (startFound && (packetLen < 4)) {
                packet[packetLen] = buf[i];
                packetLen++;
            }

            if (buf[i] == ETX && startFound && (packetLen == 4)) {
                parse_packet(packet);
                startFound = false;
                packetLen = 0;
            }

            if (packetLen > 4) {
                packetLen = 0;
                startFound = false;
            }
        }
    }
    return 0;
}

// tx callback
void pc_tx_uart_cb(const struct device *dev, void *user_data) {
    // position, face, timestamp (ill get it from the pc), value
    uint8_t packet[20];
    int packet_len;

    int num_bytes;

    uart_irq_update(dev);

    while (uart_irq_tx_ready(dev)) {
        /*
            packet[0] = STX
            uint_t rad;
            rad = ring_buf_get(&bufRad, &rad, 1)
            packet[1] = rad
            packet[2] = X Position
            packet[3] = Y Position
            Packet[4] = Z Position
            Packet[5] = face
            Packet[6] = limit swtich value
            ...
            packet[...] = EXT
        */
        num_bytes = uart_fifo_fill(dev, packet, sizeof(packet));
    }
}

typedef enum {
    IDLE,
    GET_SCAN_DATA,
    SEND_SCAN_DATA,
} pc_mcu_communication_t;

volatile pc_mcu_communication_t pc_mcu_communication_state = IDLE;

void switch_pc_mcu_communication_state(pc_mcu_communication_t state) {
    pc_mcu_communication_state = state;
}

void pc_mcu_communication_thread() {
    while (true) {
        switch (pc_mcu_communication_state) {
            case IDLE:
                if (pc_rx_wake_response() == 0) {
                    switch_pc_mcu_communication_state(GET_SCAN_DATA);
                }
                break;
            case GET_SCAN_DATA:
                if (pc_rx_buf_parser() == 0) {
                    switch_pc_mcu_communication_state(SEND_SCAN_DATA);
                }
                break;
            case SEND_SCAN_DATA:
                send_scan_data_to_pc();
                /*
                    if (END_OF_SCAN) {
                        switch_pc_mcu_communication_state(IDLE);
                    }
                */
                break;
            default:
                break;
        }
    }
}

int send_scan_data_to_pc() {
    if (!ring_buf_is_empty(&bufRad)) {
        uart_irq_tx_enable(uart_pc);
    }
    return 0;
}

int pc_serial_isr_init() {
    // setup
    if (!device_is_ready(uart_pc)) {
        return -1;
    }

    uart_irq_callback_user_data_set(uart_pc, pc_rx_uart_cb, NULL);
    uart_irq_callback_user_data_set(uart_pc, pc_tx_uart_cb, NULL);

    uart_irq_rx_enable(uart_pc);
    uart_irq_tx_disable(uart_pc);

    return 0;
}