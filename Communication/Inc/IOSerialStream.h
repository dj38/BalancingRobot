/*
 * IOSerialStream.h
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */
#ifndef IOSerialStream_H_
#define IOSerialStream_H_

#include <queue>
#include <string>
#include "stm32f4xx_hal.h"

#define __ENDL "\n\r"

class IOSerialStream {
public:
	IOSerialStream(UART_HandleTypeDef *huart);
	IOSerialStream(const IOSerialStream&);
	IOSerialStream& operator=(const IOSerialStream&);
	void write(uint32_t uint32);
	void write(uint16_t uint16);
	void write(uint8_t uint8);
	void write(std::string const& str);
	virtual ~IOSerialStream();
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

IOSerialStream& operator<<(IOSerialStream & serial,std::string const& str);
IOSerialStream& operator<<(IOSerialStream & serial,float const& flt);

#endif /* IOSerialStream_H_ */
