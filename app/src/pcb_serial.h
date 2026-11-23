#ifndef PCB_SERIAL_H
#define PCB_SERIAL_H

static const struct device *uart_pc;
int add_pc_tx_msg(uint8_t data[], uint8_t len);

#endif