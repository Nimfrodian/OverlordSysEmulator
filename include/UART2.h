#ifndef UART2_H
#define UART2_H

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "MODBUS.h"

#define MB_2_TIMEOUT_MS    (20U)

void uart2_init(void);
void modbus2_runnable(void);

#endif