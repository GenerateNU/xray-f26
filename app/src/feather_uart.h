#ifndef FEATHER_UART_H
#define FEATHER_UART_H

extern struct k_msgq rescan_q;
extern struct k_msgq pc_tx_q;
extern volatile uint8_t threshold;

#define STX 0x02
#define ETX 0x03

#endif