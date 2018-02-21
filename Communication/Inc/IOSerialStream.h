/*
 * IOSerialStream.h
 *
 *  Created on: 1 avr. 2017
 *      Author: joel
 */
#ifndef IOSerialStream_H_
#define IOSerialStream_H_

#include <queue>
#include <vector>
#include <string>
#include "stm32f4xx_hal.h"

#define __ENDL "\n\r"

class IOStream {
public:
	virtual IOStream& operator<<(std::string const& str)=0;
	virtual IOStream& operator<<(float const& flt)=0;
	virtual ~IOStream() {};
protected:
	virtual std::string *rxBuffer()=0;
};

class IOStreamList {
public:
	IOStreamList& attachStream(IOStream&);
	~IOStreamList();
	template <class T> IOStreamList& operator<<(T const& message) {
		for(std::vector<IOStream*>::iterator it=m_streamList.begin();it!=m_streamList.end();++it)
			**it<< message;
		return(*this);
	}
private:
	std::vector<IOStream*> m_streamList;
};

class IOSerialStream: public IOStream {
public:
	IOSerialStream(UART_HandleTypeDef *huart);
	IOSerialStream(const IOSerialStream&);
	IOSerialStream& operator=(const IOSerialStream&);
	virtual ~IOSerialStream();
	void TxCpltCallback();
	virtual void RxCpltCallback();
	void startRX();
	uint16_t getTxBufferSize();

	IOSerialStream& operator<<(std::string const& str);
	IOSerialStream& operator<<(float const& flt);

protected:
	std::string *rxBuffer();

private:
	struct Srep; // representation of class
	Srep* rep;
	void transmit();
	void write(uint32_t uint32);
	void write(uint16_t uint16);
	void write(uint8_t uint8);
	void write(std::string const& str);
};

#endif /* IOSerialStream_H_ */