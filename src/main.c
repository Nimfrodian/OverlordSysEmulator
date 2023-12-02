#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

#include "MODBUS.h"
#include "UART0.h"
#include "UART1.h"
#include "UART2.h"


void uart_init()
{
    uart0_init();
    uart1_init();
    uart2_init();

    uart_write_bytes(UART_NUM_0, "Uart init\n", 11);
}

void uart_task()
{
    while (1) {
        modbus1_runnable();
        modbus2_runnable();
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

void app_main()
{
    uart_init();
    xTaskCreate(uart_task, "uart_task", 2048, NULL, 10, NULL);
    while(1)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
    }
}
