/*
 * Motors.cpp
 *
 *  Created on: 10 oct. 2017
 *      Author: joel
 */

#include "Motors.h"
#include "math.h"
#define PI 3.14159265
#define radToDegrees 180.0/PI

const float Motors::wheelsSpacing = 0.183; // space between the two wheels (m)

Motors::Motors(Motor * leftMotor,Motor * rightMotor,Encoder * leftEnc,Encoder * rightEnc) :
	m_leftMotor(leftMotor),m_rightMotor(rightMotor),m_leftEnc(leftEnc),m_rightEnc(rightEnc) {
   	m_position.theta=m_position.x=m_position.y=0;
}

Motors::~Motors() {
}

void Motors::computeSpeed() {
	m_leftEnc->pushNewEncoder();
	m_rightEnc->pushNewEncoder();
	float leftInc=m_leftEnc->getLastIncrement();
	float rightInc=m_rightEnc->getLastIncrement();
	/*
	float dist=(m_leftEnc->getLastIncrement()+m_rightEnc->getLastIncrement())/2;
	float dthetaOver2=asin((m_rightEnc->getLastIncrement()-m_leftEnc->getLastIncrement())/(2*wheelsSpacing));
	*/
	float dist=(leftInc+rightInc)/2;
	float dthetaOver2=asinf((rightInc-leftInc)/(2*wheelsSpacing));
	m_position.theta+=dthetaOver2; // TODO : ajuster la rotation : pour un tour, theta indique ~350 pour l'instant
	m_position.x-=dist*sinf(m_position.theta);
	m_position.y+=dist*cosf(m_position.theta);
	m_position.theta+=dthetaOver2;
}

void Motors::setPWM(float leftPWM, float rightPWM) {
	m_leftMotor->setMotorPower(leftPWM);
	m_rightMotor->setMotorPower(rightPWM);
}

void Motors::setPWM(float leftRightPWM) {
	m_leftMotor->setMotorPower(leftRightPWM);
	m_rightMotor->setMotorPower(leftRightPWM);
}

void Motors::reset() {
	m_leftMotor->setMotorPower(0);
	m_rightMotor->setMotorPower(0);
	m_leftEnc->encoderReset();
	m_rightEnc->encoderReset();
   	m_position.theta=m_position.x=m_position.y=0;
}

float Motors::getSpeed() {
	return((m_leftEnc->getSpeed()+m_rightEnc->getSpeed())/2);
}

float Motors::getSpeedLeft() {
	return(m_leftEnc->getSpeed());
}

float Motors::getSpeedRight() {
	return(m_rightEnc->getSpeed());
}
