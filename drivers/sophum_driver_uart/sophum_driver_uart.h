#ifndef __SOPHUM_DRIVER_UART_H__
#define __SOPHUM_DRIVER_UART_H__


#include "driver/uart.h"

#define UART1_TXD_PIN      4
#define UART1_RXD_PIN      5
#define UART1_RTS_PIN      UART_PIN_NO_CHANGE
#define UART1_CTS_PIN      UART_PIN_NO_CHANGE

#define UART1_BAUD_RATE    19200
#define UART1_DATA_BITS    UART_DATA_8_BITS
#define UART1_PARITY_EN    UART_PARITY_DISABLE
#define UART1_STOP_BITS    UART_STOP_BITS_1
#define UART1_FLOW_CTRL_EN UART_HW_FLOWCTRL_DISABLE
#define UART1_BUFF_SIZE    1024

extern void UART1_Init(uint32_t baud_rate, uint8_t data_bits, uint8_t parity_en,
                       uint8_t  stop_bits, uint8_t flow_ctrl_en);

#endif