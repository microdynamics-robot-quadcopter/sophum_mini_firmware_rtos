#include "sophum_driver_uart.h"


void UART1_Init(uint32_t baud_rate, uint8_t data_bits, uint8_t parity_en,
                uint8_t  stop_bits, uint8_t flow_ctrl_en)
{
    uart_config_t uart1_config = {
        .baud_rate = baud_rate,
        .data_bits = data_bits,
        .parity    = parity_en,
        .stop_bits = stop_bits,
        .flow_ctrl = flow_ctrl_en
    };

    uart_param_config(UART_NUM_1, &uart1_config);
    uart_set_pin(UART_NUM_1, UART1_TXD_PIN, UART1_RXD_PIN, UART1_RTS_PIN, UART1_CTS_PIN);
    uart_driver_install(UART_NUM_1, UART1_BUFF_SIZE * 2, 0, 0, NULL, 0);
}