#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>


void uartWrite(const char* input)
{
    int len = strlen(input);
    uart_write_bytes(UART_NUM_0, input, len);
}

#define UART1_NUM UART_NUM_1
#define UART2_NUM UART_NUM_2

#define BUF_SIZE (1024)

#define MB_2_TIMEOUT_MS    (20U)

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

// prepares the master message based on input data
// function is continually called
int modbus2Receiver(int UartNum, mb2MasterMsg* MasterMsg)
{
    int toReturn = MB_MSG_NOT_RECEIVED;
    static unsigned long long int timeoutCntr_ms[UART_NUM_MAX] = {0};
    static int dataArrIndx[UART_NUM_MAX] = {0};

    // receive the message
    int length = 0;
    uart_get_buffered_data_len(UartNum, (size_t*)&length);

    if (length > 0)
    {
        uart_read_bytes(UartNum, &MasterMsg->mbData.dataArr[dataArrIndx[UartNum]], 1, 20 / portTICK_RATE_MS);  // read one byte
        //char printable[100] = {0};
        //sprintf(printable, "%d: 0x%X\n", dataArrIndx[UartNum], MasterMsg->mbData.dataArr[dataArrIndx[UartNum]]);
        //uartWrite(printable);
        dataArrIndx[UartNum]++;  // increment the counter

        if (1 == dataArrIndx[UartNum])  // if first byte then start countdown
        {
            timeoutCntr_ms[UartNum] = esp_timer_get_time() / 1000;
            //char printable[100] = {0};
            //sprintf(printable, "Timeout started at %llu\n", timeoutCntr_ms[UartNum]);
            //uartWrite(printable);
        }
        else
        {
            unsigned long long int timeSinceFirstByte_ms = (esp_timer_get_time() / 1000) - timeoutCntr_ms[UartNum];

            if (8 == dataArrIndx[UartNum])  // the last byte was received
            {
                dataArrIndx[UartNum] = 0;
                timeoutCntr_ms[UartNum] = 0;
                toReturn = MB_MSG_RECEIVED;
                //char printable[100] = {0};
                //sprintf(printable, "Last byte received at %llu, taking %llu ms\n", (esp_timer_get_time() / 1000), timeSinceFirstByte_ms);
                //uartWrite(printable);
            }
            else if (MB_2_TIMEOUT_MS < timeSinceFirstByte_ms)  // timeout occured
            {
                //char printable[100] = {0};
                //sprintf(printable, "Timeout occured at %llu\n", timeoutCntr_ms[UartNum]);
                //uartWrite(printable);
                for (int i = 0; i < 8; i++)
                {
                    MasterMsg->mbData.dataArr[i] = 0x00;
                }
                dataArrIndx[UartNum] = 0;
                timeoutCntr_ms[UartNum] = 0;
            }
        }
    }

    return toReturn;
}

void modbus2ParserAndComposer(mb2MasterMsg* MasterMsg, mb2SlaveMsg* SlaveMsg)
{
    const float SDM120M_READ_VOLTAGE_V = 230.1;
    const float SDM120M_READ_CURRENT_A = 0.1;
    const float SDM120M_ACTIVE_POWER_W = 23.24;
    const float SDM120M_APPARENT_POWER_VA = -1.0f;
    const float SDM120M_REACTIVE_POWER_VAr = -1.0f;
    const float SDM120M_POWER_FACTOR = -1.0f;
    const float SDM120M_FREQUENCY = 50.01;
    const float SDM120M_IMPORT_ACTIVE_ENERGY_kWh = -1.0f;
    const float SDM120M_EXPORT_ACTIVE_ENERGY_kWh = -1.0f;
    const float SDM120M_IMPORT_REACTIVE_ENERGY_kVArh = -1.0f;
    const float SDM120M_EXPORT_REACTIVE_ENERGY_kVArh = -1.0f;
    const float SDM120M_TOTAL_SYSTEM_POWER_DEMAND_W = -1.0f;
    const float SDM120M_MAXIMUM_TOTAL_SYSTEM_POWER_DEMAND_W = -1.0f;
    const float SDM120M_IMPORT_SYSTEM_POWER_DEMAND_W = -1.0f;
    const float SDM120M_MAXIMUM_IMPORT_SYSTEM_POWER_DEMAND_W = -1.0f;
    const float SDM120M_EXPORT_SYSTEM_POWER_DEMAND_W = -1.0f;
    const float SDM120M_MAXIMUM_EXPORT_SYSTEM_POWER_DEMAND = -1.0f;
    const float SDM120M_CURRENT_DEMAND_A = -1.0f;
    const float SDM120M_MAXIMUM_CURRENT_DEMAND_A = -1.0f;
    const float SDM120M_TOTAL_ACTIVE_ENERGY_kWh = 23.1;
    const float SDM120M_TOTAL_REACTIVE_ENERGY_kVArh = -1.0f;

    uint32_t sendable = 0x00;
    uint16_t address = (MasterMsg->mbData.dataNames.startAddressHi << 8) | (MasterMsg->mbData.dataNames.startAddressLo << 0);
    switch(address)
    {
        case 0x0000:
        {
            float tempVar = SDM120M_READ_VOLTAGE_V + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0006:
        {
            float tempVar = SDM120M_READ_CURRENT_A + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x000C:
        {
            float tempVar = SDM120M_ACTIVE_POWER_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0012:
        {
            float tempVar = SDM120M_APPARENT_POWER_VA + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0018:
        {
            float tempVar = SDM120M_REACTIVE_POWER_VAr + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x001E:
        {
            float tempVar = SDM120M_POWER_FACTOR + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0046:
        {
            float tempVar = SDM120M_FREQUENCY + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0048:
        {
            float tempVar = SDM120M_IMPORT_ACTIVE_ENERGY_kWh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x004A:
        {
            float tempVar = SDM120M_EXPORT_ACTIVE_ENERGY_kWh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x004C:
        {
            float tempVar = SDM120M_IMPORT_REACTIVE_ENERGY_kVArh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x004E:
        {
            float tempVar = SDM120M_EXPORT_REACTIVE_ENERGY_kVArh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0054:
        {
            float tempVar = SDM120M_TOTAL_SYSTEM_POWER_DEMAND_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0056:
        {
            float tempVar = SDM120M_MAXIMUM_TOTAL_SYSTEM_POWER_DEMAND_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0058:
        {
            float tempVar = SDM120M_IMPORT_SYSTEM_POWER_DEMAND_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x005A:
        {
            float tempVar = SDM120M_MAXIMUM_IMPORT_SYSTEM_POWER_DEMAND_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x005C:
        {
            float tempVar = SDM120M_EXPORT_SYSTEM_POWER_DEMAND_W + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x005E:
        {
            float tempVar = SDM120M_MAXIMUM_EXPORT_SYSTEM_POWER_DEMAND + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0102:
        {
            float tempVar = SDM120M_CURRENT_DEMAND_A + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0108:
        {
            float tempVar = SDM120M_MAXIMUM_CURRENT_DEMAND_A + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0156:
        {
            float tempVar = SDM120M_TOTAL_ACTIVE_ENERGY_kWh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        case 0x0158:
        {
            float tempVar = SDM120M_TOTAL_REACTIVE_ENERGY_kVArh + 1.0f;
            sendable = *((const uint32_t*) &tempVar);
            break;
        }
        default:
        {
            sendable = 0xFFFFFFFF;
            break;
        }
    }

    // compose the message based on master message
    SlaveMsg->mbData.dataNames.slaveAddress = MasterMsg->mbData.dataNames.slaveAddress;
    SlaveMsg->mbData.dataNames.functionCode = MasterMsg->mbData.dataNames.functionCode;
    SlaveMsg->mbData.dataNames.byteCount = 4;
    SlaveMsg->mbData.dataNames.FirstRegisterHi = (sendable >> 24) & 0xFF;
    SlaveMsg->mbData.dataNames.FirstRegisterLo = (sendable >> 16) & 0xFF;
    SlaveMsg->mbData.dataNames.SecondRegisterHi = (sendable >> 8) & 0xFF;
    SlaveMsg->mbData.dataNames.SecondRegisterLo = (sendable >> 0) & 0xFF;
    SlaveMsg->mbData.dataNames.CrcLo = 0;
    SlaveMsg->mbData.dataNames.CrcHi = 0;


    //uint8_t* ptr = &MasterMsg->mbData.dataArr[0];
    //char printable[100] = {0};
    //sprintf(printable, "Master: {%X, %X, %X, %X, %X, %X, %X, %X}\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7]);
    //uartWrite(printable);
    //ptr = &SlaveMsg->mbData.dataArr[0];
    //sprintf(printable, "Slave: {%X, %X, %X, %X, %X, %X, %X, %X, %X}\n\n", ptr[0], ptr[1], ptr[2], ptr[3], ptr[4], ptr[5], ptr[6], ptr[7], ptr[8]);
    //uartWrite(printable);
}

void modbus2Responder(mb2SlaveMsg* SlaveMsg)
{
    uart_write_bytes(UART_NUM_2, (const char*) SlaveMsg->mbData.dataArr, 9);
}


void uart_init() {
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
    uart_driver_install(UART_NUM_0, BUF_SIZE * 2, 0, 0, NULL, 0);
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
    uart_set_pin(UART1_NUM, GPIO_NUM_19, GPIO_NUM_18, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_param_config(UART1_NUM, &uart1_config);
    uart_driver_install(UART1_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
    // Set RS485 half duplex mode
    uart_set_mode(UART1_NUM, UART_MODE_RS485_HALF_DUPLEX);


    uart_config_t uart2_config = {
            .baud_rate = 9600,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 122,
            .source_clk = UART_SCLK_APB,
    };
    // Configure MODBUS 1 pins
    uart_set_pin(UART_NUM_2, GPIO_NUM_17, GPIO_NUM_16, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Set MODBUS 1 parameters
    uart_param_config(UART_NUM_2, &uart2_config);
    // Install MODBUS 1 driver
    uart_driver_install(UART_NUM_2, 1024, 1024, 0, NULL, 0);
}

void uart_task() {
    while (1) {
        static mb2MasterMsg masterMsg = {0};
        int mb2MsgRdy = modbus2Receiver(UART_NUM_2, &masterMsg);
        if (MB_MSG_RECEIVED == mb2MsgRdy)
        {
            mb2SlaveMsg slaveMsg = {0};
            modbus2ParserAndComposer(&masterMsg, &slaveMsg);
            modbus2Responder(&slaveMsg);
        }

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
