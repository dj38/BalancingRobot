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
private:
	std::vector<IOStream*> m_streamList;
	bool m_streamEnable;
public:
	IOStreamList() : m_streamEnable(true) {};
	IOStreamList& attachStream(IOStream&);
	~IOStreamList();
	template <class T> IOStreamList& operator<<(T const& message) {
		if(m_streamEnable)
			for(std::vector<IOStream*>::iterator it=m_streamList.begin();it!=m_streamList.end();++it)
				**it<< message;
		return(*this);
	};
	inline void enableStream(bool enable) {m_streamEnable=enable;};
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
	void write(std::string const& str);
};

#endif /* IOSerialStream_H_ */
