/*
 * Motor.h
 *
 *  Created on: 26 sept. 2017
 *      Author: joel
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "tim.h"

class Motor {
public:
	typedef enum {
		MOTOR_FORWARD=0,
		MOTOR_BACKWARD=1
	} motorDirection_e;

	Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort, const uint16_t dirPin, const int32_t freq=1000,const motorDirection_e direction=MOTOR_FORWARD);
	virtual ~Motor();
	void pwmFreq(const int32_t freq);
	void setMotorPower(const float power,const uint8_t pforce=0);
	void setMotorPolarity(const motorDirection_e direction);
private:
	void pwm(float pwm);
	TIM_HandleTypeDef *m_htim;
	uint32_t m_channel;
	GPIO_TypeDef *m_dirPort;
	uint16_t m_dirPin;
	motorDirection_e m_motorDirection;
	float m_power;
};

#endif /* MOTOR_H_ */
