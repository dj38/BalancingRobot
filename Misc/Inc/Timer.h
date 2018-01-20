/*
 * Timer.h
 *
 *  Created on: 2 oct. 2017
 *      Author: joel
 */

#ifndef TIMER_H_
#define TIMER_H_

#include "tim.h"

class Timer {
public:
	Timer();
	virtual ~Timer();
	void reset();
	void start();
	void stop();
	uint32_t read_us();
	float read();
	static void initHandlers(TIM_HandleTypeDef *htimMSB,TIM_HandleTypeDef *htimLSB);

private:
	static TIM_HandleTypeDef *m_htimMSB;
	static TIM_HandleTypeDef *m_htimLSB;
	uint32_t m_offsetOrRunTime; // offset when timer is running, RunTime when stopped
	uint8_t m_running;
};

class TimeOut : public Timer {
public:
	TimeOut();
	virtual ~TimeOut();

	void setTimeOut(const uint32_t timeOut_us);
	uint8_t timeoutCheck();
	uint8_t repeatedTimeoutCheck();
private:
	//uint32_t m_timeoutTime;
	uint32_t m_timeoutDuration;
	//uint8_t m_timeoutRunning;
};

#endif /* TIMER_H_ */
