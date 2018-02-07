/*
 * SerialBuffer.cpp
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */

#include "SerialBuffer.h"
//#include <sstream>
//#include <limits>
//#include <iomanip>

SerialBuffer::SerialBuffer(UART_HandleTypeDef *huart,std::string name) :
	m_huart(huart), m_name(name), m_ongoingTransmit(false) {
	// TODO : fix it at time of definition usart is not initialized (beginning of main.cpp)
	// TODO : Check buffer overflow
	HAL_UART_Receive_IT(m_huart,m_halRxBuffer,1);
}

void SerialBuffer::startRX() {
	HAL_UART_Receive_IT(m_huart,m_halRxBuffer,1);
}

void SerialBuffer::write(std::string const& str) {
	for(uint i=0;i<str.size();i++) {
		m_pendingTxBuffer.push((uint8_t)str[i]);
	}
	transmit();
}

void SerialBuffer::TxCpltCallback() {
	m_ongoingTransmit=false;
	transmit();
}

void SerialBuffer::RxCpltCallback() {
	m_rxBuffer.push_back((char)m_halRxBuffer[0]);
	HAL_UART_Receive_IT(m_huart,m_halRxBuffer,1);
}

uint16_t SerialBuffer::getTxBufferSize() {
	return(m_pendingTxBuffer.size());
}

void SerialBuffer::write(uint32_t uint32) {
	uint8_t buf[4];
	buf[3]=(uint8_t)(uint32&0xff);
	buf[2]=(uint8_t)((uint32>>8)&0xff);
	buf[1]=(uint8_t)((uint32>>16)&0xff);
	buf[0]=(uint8_t)((uint32>>24)&0xff);
	for (int i = 0; i < 4; ++i) { // send MSB first
		m_pendingTxBuffer.push(buf[i]);
	}
	transmit();
}

void SerialBuffer::write(uint16_t uint16) {
	uint8_t buf[2];
	buf[1]=(uint8_t)(uint16&0xff);
	buf[0]=(uint8_t)((uint16>>8)&0xff);
	for (int i = 0; i < 2; ++i) { // send MSB first
		m_pendingTxBuffer.push(buf[i]);
	}
	transmit();
}

void SerialBuffer::write(uint8_t uint8) {
	m_pendingTxBuffer.push(uint8);
	transmit();
}

void SerialBuffer::transmit() {
	if(m_ongoingTransmit) return;
	if(m_pendingTxBuffer.empty())
		return;
	// TODO : an interrupt is generated on each transmitted symbol
	m_ongoingTransmit=true;
	m_halTxBuffer[0]=m_pendingTxBuffer.front();
	m_pendingTxBuffer.pop();
	HAL_UART_Transmit_IT(m_huart,m_halTxBuffer,1);
}

std::string *SerialBuffer::rxBuffer() {
	return(&m_rxBuffer);
}

SerialBuffer::~SerialBuffer() {
}

SerialBuffer& operator<<(SerialBuffer & serial,std::string const& str) {
/*	std::ostringstream buffer(std::ostringstream::out);
	buffer << str ;
	std::string strrr=buffer.str();
	serial.write(strrr);*/
	// TODO check simplified implementation
	serial.write(str);
	return(serial);
}

SerialBuffer& operator<<(SerialBuffer & serial,float const& flt){
/*	std::ostringstream buffer(std::ostringstream::out);
	buffer.setf(std::ios_base::fixed | std::ios_base::scientific,std::ios_base::floatfield);
	buffer << std::setprecision(4);
	buffer << flt;
	std::string str=buffer.str();*/
//	float f = 1.123456789;
	char c[50]; //size of the number
	sprintf(c, "%g", flt);
	serial.write(c);
	return(serial);
}


