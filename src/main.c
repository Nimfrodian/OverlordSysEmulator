#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

#include "MODBUS.h"
#include "UART2.h"

void uartWrite(const char* input)
{
    int len = strlen(input);
    uart_write_bytes(UART_NUM_0, input, len);
}

void uart_init() {
    uart2_init();

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

    uart_config_t uart1_config = {
        .baud_rate = 256000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
    };
    // Configure UART1 parameters
    uart_set_pin(UART_NUM_1, GPIO_NUM_19, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_param_config(UART_NUM_1, &uart1_config);
    uart_driver_install(UART_NUM_1, 1024 * 2, 0, 0, NULL, 0);
    // Set RS485 half duplex mode
    uart_set_mode(UART_NUM_1, UART_MODE_RS485_HALF_DUPLEX);


}

void uart_task() {
    while (1) {
        modbus2_runnable();
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}


void app_main() {

    uart_init();
    uart_write_bytes(UART_NUM_0, "Uart init\n", 11);
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
    while(1)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
