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
	SerialBuffer(UART_HandleTypeDef *huart);
	SerialBuffer(const SerialBuffer&);
	SerialBuffer& operator=(const SerialBuffer&);
	void write(uint32_t uint32);
	void write(uint16_t uint16);
	void write(uint8_t uint8);
	void write(std::string const& str);
	virtual ~SerialBuffer();
	void TxCpltCallback();
	virtual void RxCpltCallback();
	void startRX();
	uint16_t getTxBufferSize();

protected:
	std::string *rxBuffer();

private:
	struct Srep; // representation of class
	Srep* rep;
	void transmit();
};

SerialBuffer& operator<<(SerialBuffer & serial,std::string const& str);
SerialBuffer& operator<<(SerialBuffer & serial,float const& flt);

#endif /* SERIALBUFFER_H_ */
