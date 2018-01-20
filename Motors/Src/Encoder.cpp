/*
 * Encoder.cpp
 *
 *  Created on: 8 oct. 2017
 *      Author: joel
 */

#include "Encoder.h"

const float Encoder::wheelDiameter = WHEEL_DIAMETER;
const float Encoder::pi = 3.141592654;
const float Encoder::pulsesPerRev = 2*32*30;
const float Encoder::distPerPulse = wheelDiameter*pi/pulsesPerRev;

Encoder::Encoder(TIM_HandleTypeDef * htimEncoder) : m_htimEncoder(htimEncoder) {
	m_timer.start();
	encoderReset();
	HAL_TIM_Encoder_Start(m_htimEncoder,TIM_CHANNEL_ALL);
	m_speed=m_lastIncr=0;
}

Encoder::~Encoder() {
}

float Encoder::pushNewEncoder() {
	uint8_t previousDataIndex=m_dataIndex;
	m_dataIndex=(m_dataIndex+1)%NB_ENCODER_SAMPLES;
	m_encoderValues[m_dataIndex]=__HAL_TIM_GET_COUNTER(m_htimEncoder);
	m_encoderTimes[m_dataIndex]=m_timer.read();

	uint8_t oldestDataIndex=(m_dataIndex+1)%NB_ENCODER_SAMPLES;
    float deltaT = m_encoderTimes[m_dataIndex]-m_encoderTimes[oldestDataIndex];
    if(deltaT>0) {
    	m_lastIncr = distPerPulse*float(m_encoderValues[m_dataIndex]-m_encoderValues[previousDataIndex]);
        m_speed = distPerPulse*float(m_encoderValues[m_dataIndex]-m_encoderValues[oldestDataIndex])/deltaT;
    } else m_speed=0;
    return(m_speed);
}

void Encoder::encoderReset() {
	__HAL_TIM_SET_COUNTER(m_htimEncoder,0);
	float now=m_timer.read();
	for(int i=0;i<NB_ENCODER_SAMPLES;i++) {
		m_encoderValues[i]=0;
		m_encoderTimes[i]=now;
	};
	m_dataIndex=0;
}

float Encoder::getSpeed() {
	return (m_speed);
}

float Encoder::getLastIncrement() {
	return(m_lastIncr);
}
