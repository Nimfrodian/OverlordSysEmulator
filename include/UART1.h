#ifndef UART1_H
#define UART1_H

#include "freertos/FreeRTOS.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "MODBUS.h"

#define MB_1_TIMEOUT_MS    (20U)

typedef struct
{
    uint8_t moduleID;       // board ID
    uint8_t functionCode;   // function code, usually 0x0F
    uint8_t startAddressHi; // Usually 0
    uint8_t startAddressLo; // Usually 0
    uint8_t numOfRelaysHi;    // Usually 8
    uint8_t numOfRelaysLo;    // Usually 8
    uint8_t followingBytes; // Usually 1
    uint8_t newRlySts;      // New relays status as mask
    uint8_t CrcLo;          // CRC
    uint8_t CrcHi;          // CRC
} mb1MasterMsgData;

typedef struct
{
    union
    {
        mb1MasterMsgData dataNames;
        uint8_t dataArr[10];
    } mbData;
} mb1MasterMsg;

typedef struct
{
    uint8_t moduleID;           // board ID
    uint8_t functionCode;       // usually 0x0F
    uint8_t addressHi;          // Usually 0x00
    uint8_t addressLo;          // Usually 0x00
    uint8_t relaysHi;           // ASSUMPTION: proboably uint16_t mask of relays set
    uint8_t relaysLo;           // ASSUMPTION: proboably uint16_t mask of relays set
    uint8_t CrcLo;              // CRC
    uint8_t CrcHi;              // CRC
} mb1SlaveMsgData;

typedef struct
{
    union
    {
        mb1SlaveMsgData dataNames;
        uint8_t dataArr[8];
    } mbData;
} mb1SlaveMsg;

void uart1_init(void);
void modbus1_runnable(void);

#endif