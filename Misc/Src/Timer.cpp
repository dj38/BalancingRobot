/*
 * Timer.cpp
 *
 *  Created on: 2 oct. 2017
 *      Author: joel
 */

#include "Timer.h"

TIM_HandleTypeDef * Timer::m_htimMSB;
TIM_HandleTypeDef * Timer::m_htimLSB;


// This function must be called before any timer usage
void Timer::initHandlers(TIM_HandleTypeDef *htimMSB,TIM_HandleTypeDef *htimLSB) {
	m_htimMSB=htimMSB;
	m_htimLSB=htimLSB;
}

Timer::Timer() : m_offsetOrRunTime(0), m_running(0) {
}

Timer::~Timer() {
}

void Timer::start() {
	if (!m_running) {
		m_running=1;
		m_offsetOrRunTime=read_us();
	}
}

void Timer::stop() {
	if (m_running) {
		m_offsetOrRunTime=read_us();
		m_running=0;
	}
}

void Timer::reset() {
	if (m_running)
		m_offsetOrRunTime=read_us()+m_offsetOrRunTime;
	else
		m_offsetOrRunTime=0;
}
extern uint8_t counterrr;

uint32_t Timer::read_us() {
	if (m_running) {
		uint16_t MSB;
		uint16_t LSB;
		do {
			MSB=__HAL_TIM_GET_COUNTER(m_htimMSB);
			LSB=__HAL_TIM_GET_COUNTER(m_htimLSB);
		} while(MSB!=__HAL_TIM_GET_COUNTER(m_htimMSB));
		return ((((MSB & 0xffff) << 16 ) | (LSB & 0xffff))-m_offsetOrRunTime);
	};
	return(m_offsetOrRunTime);
}

float Timer::read() {
	return((float)read_us()/1e6);
}

// TimeOut Class definition
TimeOut::TimeOut() : Timer() {
	reset();
}

TimeOut::~TimeOut(){
}

void TimeOut::setTimeOut(const uint32_t timeOut_us) {
	m_timeoutDuration=timeOut_us;
	start();
}

uint8_t TimeOut::timeoutCheck()  {
	if (read_us()>=m_timeoutDuration) {
		stop();
		reset();
		return(1);
	}
	return(0);
}

uint8_t TimeOut::repeatedTimeoutCheck()  {
	if (read_us()>=m_timeoutDuration) {
		reset();
		return(1);
	}
	return(0);
}

