/*
 * Encoder.h
 *
 *  Created on: 8 oct. 2017
 *      Author: joel
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#define WHEEL_DIAMETER 0.09 // wheel diameter in meters
#define NB_ENCODER_SAMPLES 2 // must be lower than 256

#include "Timer.h"

class Encoder {
public:
	Encoder(TIM_HandleTypeDef * htimEncoder);
	virtual ~Encoder();
    /** Calculates new position based on ...
         * @param now is current time in us
     */
    float pushNewEncoder(); //now is time in us
    /** Reset speed
     */
    void encoderReset();
    /**      */
    float getSpeed();
    /**
     * return the last distance increment
     */
    float getLastIncrement();

private:

	float m_speed;
	float m_lastIncr;

	TIM_HandleTypeDef *m_htimEncoder;
	int32_t m_encoderValues[NB_ENCODER_SAMPLES];
	float m_encoderTimes[NB_ENCODER_SAMPLES];
	uint8_t m_dataIndex;
	Timer m_timer;

    static const float wheelDiameter;
    static const float pi;
    static const float pulsesPerRev;
    static const float distPerPulse;
};

#endif /* ENCODER_H_ */
