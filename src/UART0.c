#include "UART0.h"

void uart0_init(void)
{
    uart_config_t uart0_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART parameters
    uart_set_pin(UART_NUM_0, UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_param_config(UART_NUM_0, &uart0_config);
    uart_driver_install(UART_NUM_0, 1024 * 2, 0, 0, NULL, 0);
    // Set RS485 half duplex mode
    uart_set_mode(UART_NUM_0, UART_MODE_UART);
}