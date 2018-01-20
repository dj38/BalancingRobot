/*
 * Motor.cpp
 *
 *  Created on: 26 sept. 2017
 *      Author: joel
 */

#include "Motor.h"
#include "math.h"

Motor::Motor(TIM_HandleTypeDef *htim, uint32_t channel, GPIO_TypeDef *dirPort, const uint16_t dirPin, const int32_t freq,const motorDirection_e direction) :
	m_htim(htim), m_channel(channel), m_dirPort(dirPort), m_dirPin(dirPin), m_motorDirection(direction), m_power(0)
{
	pwmFreq(freq);
	setMotorPower(0);
	HAL_TIM_PWM_Start(m_htim,m_channel);
}

Motor::~Motor()
{

}

void Motor::setMotorPower(const float power,const uint8_t pforce)
{
	if(pforce||(power!=m_power)) {
		if (((power>0)&&(m_motorDirection))||((power<0)&&!(m_motorDirection))) {
			HAL_GPIO_WritePin(m_dirPort,m_dirPin,GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(m_dirPort,m_dirPin,GPIO_PIN_SET);
		};
		pwm(fabsf(power));
		m_power=power;
	};
}

void Motor::setMotorPolarity(const motorDirection_e direction)
{
	m_motorDirection=direction;
	setMotorPower(m_power,1);
}

void Motor::pwmFreq(const int32_t freq)
{
	uint32_t RCC_HCLKFreq=HAL_RCC_GetHCLKFreq();
	uint64_t maxAutoReload=65535;
	if ((m_htim->Instance == TIM2)||(m_htim->Instance == TIM5)) {
		maxAutoReload=4294967295;
	}
	uint64_t denom=(maxAutoReload+1)*freq;
	uint32_t prescaler=RCC_HCLKFreq/denom;
	if (RCC_HCLKFreq%denom == 0) {
		prescaler--;
	}
	denom=(prescaler+1)*freq;
	uint64_t autoReload=RCC_HCLKFreq/denom;
	if (RCC_HCLKFreq%denom == 0) {
		autoReload--;
	}
	__HAL_TIM_SET_AUTORELOAD(m_htim,autoReload);
	__HAL_TIM_SET_PRESCALER(m_htim,prescaler);
}

void Motor::pwm(float pwm)
{
	pwm=fminf(fmaxf(pwm,0),1); //pwm value must be within the [0;1] range
	__HAL_TIM_SET_COMPARE(m_htim,m_channel,pwm*__HAL_TIM_GET_AUTORELOAD(m_htim));
}
