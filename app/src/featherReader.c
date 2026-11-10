#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#define STX 0x02
#define ETX 0x03

#define RADIATIONINDEX 1
#define CEHCKSUMINDEX 2

/// ask about this and overlays
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(usart1));

RING_BUF_DECLARE(bufDump, 200);
RING_BUF_DECLARE(bufA, 10);
RING_BUF_DECLARE(bufB, 10);
K_SEM_DEFINE(semFeather, 0, 1);

int uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf[32];
    int n;

    if (!uart_irq_update(dev)) {
        return 1;
    }

    while (uart_irq_rx_ready(dev)) {
        n = uart_fifo_read(dev, buf, sizeof(buf));
        if (n > 0) {
            ring_buf_put(&bufDump, buf, n);
            k_sem_give(&semFeather);
        }
    }
}

int bufDumpReader() {
    uint8_t buf[100];
    uint8_t n;
    uint8_t packet[4];
    int packetLen = 0;

    k_sem_take(&semFeather, K_FOREVER);

    n = ring_buf_get(&bufDump, buf, sizeof(buf));

    bool startFound = false;

    if (n > 1) {
        for (int i = 0; i < n; i++) {
            if (buf[i] == STX) {
                startFound = true;
                packet[packetLen] = buf[i];
                packetLen++;
                continue;
            }

            if (startFound) {
                packet[packetLen] = buf[i];
                packetLen++;
            }

            if (buf[i] == ETX && startFound && (packetLen == 4)) {
                parsePacket(packet);
                k_sem_give(&semFeather);
                startFound = false;
                packetLen = 0;
            }
        }
    }

    return 0;
}

int parsePacket(uint8_t frame[]) {
    int checksumVal = frame[CEHCKSUMINDEX]; 
    int checksum = STX + frame[RADIATIONINDEX];
    int radiation = frame[RADIATIONINDEX];

    if (checksumVal != checksum) {
        return 1;
    }

    printk(radiation);

    if (ring_buf_space_get(&bufA) == 0) {
         printk("Buffer B");
        ring_buf_put(&bufB, radiation, 1);
    } else {
        printk("Buffer A");
        ring_buf_put(&bufA, radiation, 1);
    }

    return 0;
}

int featherSerialISR() {
    if (!device_is_ready(uart)) {
        //return?
        return 1;
    }

    uart_irq_callback_user_data_set(uart, uart_cb, NULL);
    uart_irq_rx_enable(uart);

    bufDumpReader();
}