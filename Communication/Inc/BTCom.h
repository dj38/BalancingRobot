#ifndef BTCom_H
#define BTCom_H

#define BAUDRATE_1200 "AT+BAUD1"
#define BAUDRATE_2400 "AT+BAUD2"
#define BAUDRATE_4800 "AT+BAUD3"
#define BAUDRATE_9600 "AT+BAUD4"   //Default value
#define BAUDRATE_19200 "AT+BAUD5"
#define BAUDRATE_38400 "AT+BAUD6"
#define BAUDRATE_57600 "AT+BAUD7"
#define BAUDRATE_115200 "AT+BAUD8"

//#define BT_Debug
#define BAUDRATE_Debug 115200

#include <string>       // std::string
#include <queue>        // std::queue is a FIFO list
#include "SerialCommand.h"
#include "SerialBuffer.h"

class BTCom : public SerialBuffer
{
public:

    BTCom(UART_HandleTypeDef *huart);
    BTCom(const SerialBuffer&);
    bool setBaudRate(int baudRate);
    void sendBTData(const std::string message);
    SerialCommand getSerialCommand();
    int getCommandsStackSize();
    void RxCpltCallback();

private:
    bool convertBuffer2Commands(); // parse buffer to check if new commands have been received, returns true when new commands are found
    void SerialInCallback();
    std::queue<SerialCommand> m_commands;
};

#endif
