/*
 * SerialBuffer.h
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */
#ifndef SERIALBUFFER_H_
#define SERIALBUFFER_H_

#include <queue>
#include <string>
#include "stm32f4xx_hal.h"

#define __ENDL "\n\r"

class SerialBuffer {
public:
	SerialBuffer(UART_HandleTypeDef *huart,std::string name="");
	void write(uint32_t uint32);
	void write(uint16_t uint16);
	void write(uint8_t uint8);
	void write(std::string const& str);
	virtual ~SerialBuffer();
	void TxCpltCallback();
	virtual void RxCpltCallback();
	std::string *rxBuffer();
	void startRX();
	uint16_t getTxBufferSize();

protected:
	std::string m_rxBuffer;
	uint8_t m_halRxBuffer[1];
	UART_HandleTypeDef *m_huart;

private:
	std::queue<uint8_t> m_pendingTxBuffer;
	std::string m_name;
	bool m_ongoingTransmit;
	uint8_t m_halTxBuffer[1];
	void transmit();
};

SerialBuffer& operator<<(SerialBuffer & serial,std::string const& str);
SerialBuffer& operator<<(SerialBuffer & serial,float const& flt);

#endif /* SERIALBUFFER_H_ */
