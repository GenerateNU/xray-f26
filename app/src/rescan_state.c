#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include "pcb_serial.h"
#include "feather_uart.h"

bool rescan() {
    uint8_t threshold_msg = {STX, '!', '!', ETX};
    add_pc_tx_msg(threshold_msg, 4);
    uint8_t rad;

    int rounds = 5;
    int readings = 5;

    int over_threshold = 0;

    for (int i = 0; i < rounds; i++) {
        for (int j = 0; j < readings; j++) {
            k_msgq_get(&rescan_q, &rad, K_FOREVER);
            
            if (rad > threshold) {
                over_threshold++;
                break;
            }
        }
    }

    if (over_threshold > 2) {
        return true;
    }
    else {
        return false;
    }
    // move motors or just stay in place?
}