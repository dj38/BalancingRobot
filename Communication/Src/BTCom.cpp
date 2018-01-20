#include "BTCom.h"

#ifdef BT_Debug
Serial BTdebug(USBTX, USBRX); // tx, rx
#endif
//Serial blue(TX_Pin_BlueTooth,RX_Pin_BlueTooth);          // TX, RX

using namespace std;

BTCom::BTCom(UART_HandleTypeDef *huart,std::string name) : SerialBuffer(huart,name)
{
#ifdef BT_Debug
    BTdebug.baud(BAUDRATE_Debug);
#endif
}

void BTCom::RxCpltCallback() {
	SerialBuffer::RxCpltCallback();
    if (*m_rxBuffer.rbegin()==';') {
        m_commands.push(SerialCommand(m_rxBuffer));
        m_rxBuffer.clear();
    }
}

void BTCom::sendBTData(const string message)
{
    *this << message;
}
/*
string BTCom::getBuffer()
{
    return(m_rxBuffer);
}*/

bool BTCom::convertBuffer2Commands()
{
    if (m_rxBuffer.length()>0) {
        size_t pos = m_rxBuffer.find(";");
//    if ((pos!=string::npos) && (m_rxBuffer[pos] == ';')) {
#ifdef BT_Debug
        BTdebug.printf("buffer is  %s\n",m_rxBuffer.c_str());
        BTdebug.printf("character is %s \n",m_rxBuffer[pos]);
#endif
        if (pos!=string::npos) {
            string stringCommand=m_rxBuffer.substr(0,pos+1); // extract string command
            m_rxBuffer.erase(0,pos+1);             // remove trailing ;
            m_commands.push(SerialCommand(stringCommand));
            return(true);
        }
    }
    return(false);
}

SerialCommand BTCom::getSerialCommand()
{
#ifdef BT_Debug
//       BTdebug.printf("buffer is  %s\n",m_rxBuffer.c_str());
//      BTdebug.printf("buffer is  \n");
#endif
//   while(convertBuffer2Commands()); // append any new command in buffer to m_commands
    if(m_commands.empty()) {
        return(SerialCommand("")); // returns NOP command
    }
    SerialCommand command=m_commands.front();
    m_commands.pop();
    return(command);
}

int BTCom::getCommandsStackSize()
{
    return(m_commands.size());
}

bool BTCom::setBaudRate(int baudRate)
{
    switch(baudRate) {
        case 1200 : {
            *this << BAUDRATE_1200;
            break;
        }
        case 2400 : {
            *this << BAUDRATE_2400;
            break;
        }
        case 4800 : {
            *this << BAUDRATE_4800;
            break;
        }
        case 9600 : {
            *this << BAUDRATE_9600;
            break;
        }
        case 19200 : {
            *this << BAUDRATE_19200;
            break;
        }
        case 38400 : {
            *this << BAUDRATE_38400;
            break;
        }
        case 57600 : {
            *this << BAUDRATE_57600;
            break;
        }
        case 115200 : {
            *this << BAUDRATE_115200;
            break;
        }
        default: {
#ifdef BT_Debug
            BTdebug.printf("%d is not a valid baudrate\n",baudRate);
#endif
            return(false);
        }
    }
    return(true);
}
