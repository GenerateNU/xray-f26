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
static const struct device *uart = DEVICE_DT_GET(DT_NODELABEL(lpuart1));

//RING_BUF_DECLARE(bufDump, 200);

//single buffer for now 5 frames, 20 bytes, 4 bytes each
RING_BUF_DECLARE(bufA, 20);
//RING_BUF_DECLARE(bufB, 10);
// buffer with only radiation data
RING_BUF_DECLARE(bufRad, 20);
K_SEM_DEFINE(semFeather, 0, 1);

float threshold = 1; // need to figure out how the app can change this value.

void uart_cb(const struct device *dev, void *user_data) {
    uint8_t buf[20];
    int n;

    if (!uart_irq_update(dev)) {
        return;
    }

    // keep putting into buffer, give semaphore when buffer is full
    while (uart_irq_rx_ready(dev)) {
        n = uart_fifo_read(dev, buf, sizeof(buf));
        if (n > 0) {
            ring_buf_put(&bufA, buf, n);
            if (ring_buf_space_get(&bufA) == 0) {
                k_sem_give(&semFeather);
                // i lose data if only part of the frame makes it...
                // maybe it's better to immediately parse when theres data in the buffer
                // test it out. prob won't matter if it's moving really slow. 
            }
        }
    }
}

int parsePacket(uint8_t frame[]) {
    // the frame is structured as STX radiation checksum ETX
    // checksum is raditation value plus STX (2)
    uint8_t checksumVal = frame[CEHCKSUMINDEX]; 
    uint8_t checksum = (STX + frame[RADIATIONINDEX]);
    uint8_t radiation = frame[RADIATIONINDEX];

    if (checksumVal != checksum) {
        return 1;
    }

    if (radiation >= threshold ) {
        // activate interupt.
        // irq_enable(IRQ_Radiation);
    }

    printk("%d", radiation);

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

void bufReader() {
    uint8_t buf[20];
    uint8_t n;
    uint8_t packet[4];
    int packetLen = 0;

    k_sem_take(&semFeather, K_FOREVER);

    n = ring_buf_get(&bufA, buf, sizeof(buf));

    bool startFound = false;

    if (n > 1) {
        for (int i = 0; i < n; i++) {
            if (buf[i] == STX) {
                startFound = true;
                packet[packetLen] = buf[i];
                packetLen++;
                continue;
            }

            if (startFound && (packetLen <= 4)) {
                packet[packetLen] = buf[i];
                packetLen++;
            }

            if (buf[i] == ETX && startFound && (packetLen == 4)) {
                parsePacket(packet);
                k_sem_give(&semFeather);
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

// is this right...
K_THREAD_DEFINE(reader, 1024, bufReader,
		NULL, NULL, NULL, 5, 0, 0);

int featherSerialISR() {
    // setup
    if (!device_is_ready(uart)) {
        //return?
        return 1;
    }

    uart_irq_callback_user_data_set(uart, uart_cb, NULL);
    uart_irq_rx_enable(uart);
}

// fill the buffer. once filled. give semaphore.
// take semaphore, parse through buffer to get frame, then get radiation, check checksum, check radiation interupt. fill another buffer with just radiation data. 
