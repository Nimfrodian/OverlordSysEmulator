#include "UART1.h"
#include "string.h"


static void uartWrite(const char* input)
{
    int len = strlen(input);
    uart_write_bytes(UART_NUM_0, input, len);
}

void uart1_init(void)
{
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

// prepares the master message based on input data
// function is continually called
static int modbus1Receiver(int UartNum, mb1MasterMsg* MasterMsg)
{
    int toReturn = MB_MSG_NOT_RECEIVED;
    static unsigned long long int timeoutCntr_ms[UART_NUM_MAX] = {0};
    static int dataArrIndx[UART_NUM_MAX] = {0};

    // receive the message
    int length = 0;
    uart_get_buffered_data_len(UartNum, (size_t*)&length);

    while (length--)
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

            if (10 == dataArrIndx[UartNum])  // the last byte was received
            {
                dataArrIndx[UartNum] = 0;
                timeoutCntr_ms[UartNum] = 0;
                toReturn = MB_MSG_RECEIVED;
                //char printable[100] = {0};
                //sprintf(printable, "Last byte received at %llu, taking %llu ms\n", (esp_timer_get_time() / 1000), timeSinceFirstByte_ms);
                //uartWrite(printable);
            }
            else if (MB_1_TIMEOUT_MS < timeSinceFirstByte_ms)  // timeout occured
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

static void modbus1ParserAndComposer(mb1MasterMsg* MasterMsg, mb1SlaveMsg* SlaveMsg)
{
    // compose the message based on master message
    SlaveMsg->mbData.dataNames.moduleID = MasterMsg->mbData.dataNames.moduleID;
    SlaveMsg->mbData.dataNames.functionCode = MasterMsg->mbData.dataNames.functionCode;
    SlaveMsg->mbData.dataNames.addressHi = 0;
    SlaveMsg->mbData.dataNames.addressLo = 0;
    SlaveMsg->mbData.dataNames.relaysHi = MasterMsg->mbData.dataNames.numOfRelaysHi;
    SlaveMsg->mbData.dataNames.relaysLo = MasterMsg->mbData.dataNames.numOfRelaysLo;
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

static void modbus1Responder(mb1SlaveMsg* SlaveMsg)
{
    uart_write_bytes(UART_NUM_1, (const char*) SlaveMsg->mbData.dataArr, 8);
}

void modbus1_runnable(void)
{
    static mb1MasterMsg masterMsg = {0};
    int mb1MsgRdy = modbus1Receiver(UART_NUM_1, &masterMsg);
    if (MB_MSG_RECEIVED == mb1MsgRdy)
    {
        mb1SlaveMsg slaveMsg = {0};
        modbus1ParserAndComposer(&masterMsg, &slaveMsg);
        modbus1Responder(&slaveMsg);
    }
}