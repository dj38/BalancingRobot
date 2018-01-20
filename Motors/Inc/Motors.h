/*
 * Motors.h
 *
 *  Created on: 10 oct. 2017
 *      Author: joel
 */

#ifndef MOTORS_H_
#define MOTORS_H_

#include "Motor.h"
#include "Encoder.h"

class Motors {
public:
    typedef struct {
        float x,y,azimut; // azimut is angle vs y axis. Initial angle is 0 (robot oriented as y axis)
    } position_t;

	Motors(Motor * leftMotor,Motor * rightMotor,Encoder * leftEnc,Encoder * rightEnc);
	virtual ~Motors();
    void computeSpeed();
    void reset();
    void setPWM(float leftPWM,float rightPWM);
    void setPWM(float leftRightPWM);
    float getSpeed();
    void calibrate();
    float getSpeedLeft();
    float getSpeedRight();

	Motor*& getLeftMotor() {
		return m_leftMotor;
	}

	Motor*& getRightMotor() {
		return m_rightMotor;
	}

	const position_t& getPosition() const {
		return m_position;
	}

private:
    static const float wheelsSpacing;
    position_t m_position;        // robot position
    Motor * m_leftMotor,* m_rightMotor;
    Encoder * m_leftEnc,* m_rightEnc;
};

#endif /* MOTORS_H_ */
