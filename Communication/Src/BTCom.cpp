#include "BTCom.h"

using namespace std;

BTCom::BTCom(UART_HandleTypeDef *huart) : SerialBuffer(huart)
{
}

BTCom::BTCom(const SerialBuffer& buffer) : SerialBuffer(buffer) {
}

void BTCom::RxCpltCallback() {
	SerialBuffer::RxCpltCallback();
	std::string* rxBuffer=this->rxBuffer();
    if (*(rxBuffer->rbegin())==';') {
        m_commands.push(SerialCommand(*rxBuffer));
        rxBuffer->clear();
    }
}

void BTCom::sendBTData(const string message)
{
    *this << message;
}

bool BTCom::convertBuffer2Commands()
{
	std::string* rxBuffer=this->rxBuffer();
	if (rxBuffer->length()>0) {
        size_t pos = rxBuffer->find(";");
        if (pos!=string::npos) {
            string stringCommand=rxBuffer->substr(0,pos+1); // extract string command
            rxBuffer->erase(0,pos+1);             // remove trailing ;
            m_commands.push(SerialCommand(stringCommand));
            return(true);
        }
    }
    return(false);
}

SerialCommand BTCom::getSerialCommand()
{
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
            return(false);
        }
    }
    return(true);
}
