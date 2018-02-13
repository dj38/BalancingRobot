/*
 * SerialBuffer.cpp
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */

#include "SerialBuffer.h"

struct SerialBuffer::Srep { // representation
	int n;  // number of objects pointing on this rep
	UART_HandleTypeDef *m_huart;
	uint8_t m_halRxBuffer[1];
	std::queue<uint8_t> m_pendingTxBuffer;
	bool m_ongoingTransmit;
	uint8_t m_halTxBuffer[1];
	std::string m_rxBuffer;

	Srep(UART_HandleTypeDef *huart) {
		n=1;
		m_huart=huart;
		m_ongoingTransmit=false;
	}

private:  // To avoid any copy of Srep
	Srep(const Srep&);
	Srep& operator=(const Srep&);

};

SerialBuffer::SerialBuffer(UART_HandleTypeDef *huart) {
	// TODO : fix if at time of definition usart is not initialized (beginning of main.cpp)
	// TODO : Check buffer overflow
	rep=new Srep(huart);
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

SerialBuffer::SerialBuffer(const SerialBuffer& buffer) {
	buffer.rep->n++;
	rep=buffer.rep;   //representation is shared between 2 objects
}

SerialBuffer& SerialBuffer::operator=(const SerialBuffer& buffer) {
	buffer.rep->n++;
	if(--rep->n==0) delete rep;
	rep=buffer.rep;   //representation is shared between 2 objects
	return *this;
}

SerialBuffer::~SerialBuffer() {
	if(--rep->n==0) delete rep;
}

void SerialBuffer::startRX() {
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

void SerialBuffer::write(std::string const& str) {
	for(uint i=0;i<str.size();i++) {
		rep->m_pendingTxBuffer.push((uint8_t)str[i]);
	}
	transmit();
}

void SerialBuffer::TxCpltCallback() {
	rep->m_ongoingTransmit=false;
	transmit();
}

void SerialBuffer::RxCpltCallback() {
	rep->m_rxBuffer.push_back((char)rep->m_halRxBuffer[0]);
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

uint16_t SerialBuffer::getTxBufferSize() {
	return(rep->m_pendingTxBuffer.size());
}

void SerialBuffer::write(uint32_t uint32) {  //TODO could be rewritten with templates ??
	uint8_t buf[4];
	buf[3]=(uint8_t)(uint32&0xff);
	buf[2]=(uint8_t)((uint32>>8)&0xff);
	buf[1]=(uint8_t)((uint32>>16)&0xff);
	buf[0]=(uint8_t)((uint32>>24)&0xff);
	for (int i = 0; i < 4; ++i) { // send MSB first
		rep->m_pendingTxBuffer.push(buf[i]);
	}
	transmit();
}

void SerialBuffer::write(uint16_t uint16) {
	uint8_t buf[2];
	buf[1]=(uint8_t)(uint16&0xff);
	buf[0]=(uint8_t)((uint16>>8)&0xff);
	for (int i = 0; i < 2; ++i) { // send MSB first
		rep->m_pendingTxBuffer.push(buf[i]);
	}
	transmit();
}

void SerialBuffer::write(uint8_t uint8) {
	rep->m_pendingTxBuffer.push(uint8);
	transmit();
}

void SerialBuffer::transmit() {
	if(rep->m_ongoingTransmit) return;
	if(rep->m_pendingTxBuffer.empty())
		return;
	// TODO : an interrupt is generated on each transmitted symbol
	rep->m_ongoingTransmit=true;
	rep->m_halTxBuffer[0]=rep->m_pendingTxBuffer.front();
	rep->m_pendingTxBuffer.pop();
	HAL_UART_Transmit_IT(rep->m_huart,rep->m_halTxBuffer,1);
}

std::string *SerialBuffer::rxBuffer() {
	return(&rep->m_rxBuffer);
}

SerialBuffer& operator<<(SerialBuffer & serial,std::string const& str) {
	serial.write(str);
	return(serial);
}

SerialBuffer& operator<<(SerialBuffer & serial,float const& flt){
	char c[50]; //size of the number
	sprintf(c, "%g", flt);
	serial.write(c);
	return(serial);
}


