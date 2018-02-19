/*
 * IOSerialStream.cpp
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */

#include "IOSerialStream.h"

struct IOSerialStream::Srep { // representation
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

IOSerialStream::IOSerialStream(UART_HandleTypeDef *huart) {
	// TODO : fix if at time of definition usart is not initialized (beginning of main.cpp)
	// TODO : Check buffer overflow
	rep=new Srep(huart);
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

IOSerialStream::IOSerialStream(const IOSerialStream& buffer) {
	buffer.rep->n++;
	rep=buffer.rep;   //representation is shared between 2 objects
}

IOSerialStream& IOSerialStream::operator=(const IOSerialStream& buffer) {
	buffer.rep->n++;
	if(--rep->n==0) delete rep;
	rep=buffer.rep;   //representation is shared between 2 objects
	return *this;
}

IOSerialStream::~IOSerialStream() {
	if(--rep->n==0) delete rep;
}

void IOSerialStream::startRX() {
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

void IOSerialStream::write(std::string const& str) {
	for(uint i=0;i<str.size();i++) {
		rep->m_pendingTxBuffer.push((uint8_t)str[i]);
	}
	transmit();
}

void IOSerialStream::TxCpltCallback() {
	rep->m_ongoingTransmit=false;
	transmit();
}

void IOSerialStream::RxCpltCallback() {
	rep->m_rxBuffer.push_back((char)rep->m_halRxBuffer[0]);
	HAL_UART_Receive_IT(rep->m_huart,rep->m_halRxBuffer,1);
}

uint16_t IOSerialStream::getTxBufferSize() {
	return(rep->m_pendingTxBuffer.size());
}

void IOSerialStream::write(uint32_t uint32) {  //TODO could be rewritten with templates ??
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

void IOSerialStream::write(uint16_t uint16) {
	uint8_t buf[2];
	buf[1]=(uint8_t)(uint16&0xff);
	buf[0]=(uint8_t)((uint16>>8)&0xff);
	for (int i = 0; i < 2; ++i) { // send MSB first
		rep->m_pendingTxBuffer.push(buf[i]);
	}
	transmit();
}

void IOSerialStream::write(uint8_t uint8) {
	rep->m_pendingTxBuffer.push(uint8);
	transmit();
}

void IOSerialStream::transmit() {
	if(rep->m_ongoingTransmit) return;
	if(rep->m_pendingTxBuffer.empty())
		return;
	// TODO : an interrupt is generated on each transmitted symbol
	rep->m_ongoingTransmit=true;
	rep->m_halTxBuffer[0]=rep->m_pendingTxBuffer.front();
	rep->m_pendingTxBuffer.pop();
	HAL_UART_Transmit_IT(rep->m_huart,rep->m_halTxBuffer,1);
}

std::string *IOSerialStream::rxBuffer() {
	return(&rep->m_rxBuffer);
}

IOSerialStream& IOSerialStream::operator<<(std::string const& str) {
	write(str);
	return(*this);
}

IOSerialStream& IOSerialStream::operator<<(float const& flt){
	char c[50]; //size of the number
	sprintf(c, "%g", flt);
	write(c);
	return(*this);
}

IOStreamList& IOStreamList::operator<<(std::string const& str) {
	for(std::vector<IOStream*>::iterator it=m_streamList.begin();it!=m_streamList.end();++it)
		**it<< str;
	return(*this);
}

IOStreamList& IOStreamList::operator<<(float const& flt) {
	for(std::vector<IOStream*>::iterator it=m_streamList.begin();it!=m_streamList.end();++it)
		**it<< flt;
	return(*this);
}

IOStreamList& IOStreamList::attachStream(IOStream& stream) {
	m_streamList.push_back(&stream);
	return(*this);
}

IOStreamList::~IOStreamList() {

}
