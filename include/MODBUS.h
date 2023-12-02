#ifndef MODBUS_H
#define MODBUS_H

#include "freertos/FreeRTOS.h"

typedef struct
{
    uint8_t slaveAddress;       // 8-bit value representing the slave being addressed (1 to 247), 0 is reserved for the broadcast address. The Eastron Digital meters do not support the broadcast address.
    uint8_t functionCode;       // 8-bit value telling the addressed slave what action is to be performed. (3, 4, 8 or 16 are valid for Eastron Digital meter)
    uint8_t startAddressHi;     // The top (most significant) eight bits of a 16-bit number specifying the start address of the data being requested.
    uint8_t startAddressLo;     // The bottom (least significant) eight bits of a 16-bit number specifying the start address of the data being requested. As registers are used in pairs and start at zero, then this must be an even number.
    uint8_t NumOfPointsHi;      // The top (most significant) eight bits of a 16-bit number specifying the number of registers being requested
    uint8_t NumOfPointsLo;      // The bottom (least significant) eight bits of a 16-bit number specifying the number of registers being requested. As registers are used in pairs, then this must be an even number
    uint8_t CrcLo;              // The bottom (least significant) eight bits of a 16-bit number representing the error check value
    uint8_t CrcHi;              // The top (most significant) eight bits of a 16-bit number representing the error check value.
} mb2MasterMsgData;

typedef struct
{
    union
    {
        mb2MasterMsgData dataNames;
        uint8_t dataArr[8];
    } mbData;
} mb2MasterMsg;

typedef struct
{
    uint8_t slaveAddress;       // 8-bit value representing the address of slave that is responding
    uint8_t functionCode;       // 8-bit value which, when a copy of the function code in the query, indicates that the slave recognised the query and has responded
    uint8_t byteCount;          // 8-bit value indicating the number of data bytes contained within this response
    uint8_t FirstRegisterHi;    // The top (most significant) eight bits of a 16-bit number representing the first register requested in the query
    uint8_t FirstRegisterLo;    // The bottom (least significant) eight bits of a 16-bit number representing the first register requested in the query
    uint8_t SecondRegisterHi;   // The top (most significant) eight bits of a 16-bit number representing the second register requested in the query
    uint8_t SecondRegisterLo;   // The bottom (least significant) eight bits of a 16-bit number representing the second register requested in the query
    uint8_t CrcLo;              // The bottom (least significant) eight bits of a 16-bit number representing the error check value
    uint8_t CrcHi;              // The top (most significant) eight bits of a 16-bit number representing the error check value
} mb2SlaveMsgData;

typedef struct
{
    union
    {
        mb2SlaveMsgData dataNames;
        uint8_t dataArr[9];
    } mbData;
} mb2SlaveMsg;

enum
{
    MB_MSG_RECEIVED = 0,
    MB_MSG_NOT_RECEIVED,
};

#endif