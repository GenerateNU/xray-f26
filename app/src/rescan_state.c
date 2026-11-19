#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/ring_buffer.h>

#include "pcb_serial.h"
#include "feather_uart.h"


void rescan() {
    uint8_t threshold_msg = {STX, '!', '!', ETX};
    
    add_pc_tx_msg(threshold_msg, 4);
}