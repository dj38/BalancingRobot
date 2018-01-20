#include "Regulation.h"
#include <math.h>

//Regulation::Regulation(float angleP, float angleI, float angleD, float speedP, float speedI, float speedD, float angleOffset, regulMode_e regulMode, float controlAngleLimit) : m_pidAngle(angleP,angleI,angleD), m_pidSpeed(speedP,speedI,speedD)
Regulation::Regulation(float angleP, float angleI, float angleD, float speedP, float speedI, float speedD, float angleOffset, regulMode_e regulMode, float controlAngleLimit) : m_pidAngle(angleP,angleI,angleD), m_pidSpeed(speedP,speedI,speedD)
{
    m_controlAngleLimit=controlAngleLimit;
    m_angleOffset=angleOffset;
//    float m_turningLimit;       // Set the maximum turning value
    m_regulMode=regulMode;    // Set the regulation mode
    m_motorsON=false;
    m_robotStanding=false;
    m_pidAngle.setBias(0.0);
    m_pidAngle.setInputLimits(m_angleOffset-15.0f,m_angleOffset+15.0f);
    m_pidAngle.setOutputLimits(-1,1);
    m_pidAngle.reset();
    m_pidSpeed.setBias(m_angleOffset);
    m_pidSpeed.setInputLimits(-1.5,1.5);
    m_pidSpeed.setOutputLimits(m_angleOffset-m_controlAngleLimit,m_angleOffset+m_controlAngleLimit);
    m_pidSpeed.reset();
    m_rightMotorPWM=m_leftMotorPWM=0;
    m_targetAngle=m_targetSpeed=0;
   	m_measuredAngle=-90;
   	m_joystickY=m_joystickX=0;
   	m_measuredSpeed=0;
   	m_joystickRotationGain=1;
   ///// ToDo check initialization of following attributes
   	m_controlSpeedLimit=0;
   	m_turningLimit=0;
    m_joystickAngleGain=1;
    m_joystickSpeedGain=0.25;
}

// now is time in us
void Regulation::update(int now,float measuredAngle, float measuredSpeed)
{
    m_measuredAngle=measuredAngle;
    m_measuredSpeed=measuredSpeed;
    if(checkRobotIsVertical()) {
        m_pidAngle.reset();
        m_pidSpeed.reset();
        setJoyStickValue(0,0);
    }
    if(m_regulMode==angle) {
        m_targetAngle=m_angleOffset+m_joystickY*m_joystickAngleGain;
    }
    if(m_regulMode==speed) {
        m_targetSpeed=m_joystickY*m_joystickSpeedGain;
        m_pidSpeed.setSetPoint(m_targetSpeed);
        m_pidSpeed.setProcessValue(m_measuredSpeed);
        m_targetAngle=-m_pidSpeed.compute(now);
        //m_targetAngle=m_pidSpeed.compute(now);
    }
    m_pidAngle.setSetPoint(m_targetAngle);
    m_pidAngle.setProcessValue(m_measuredAngle);
    float pwm=m_pidAngle.compute(now)*m_motorsON*m_robotStanding;

    float deltaPwm=m_joystickRotationGain*m_joystickX;
    float pwmMargin=1.0-fabsf(pwm); //checks are done on deltaPWM to ensure stability then rotation
    if (deltaPwm < -pwmMargin) {
    	deltaPwm = -pwmMargin;
    } else if (deltaPwm > pwmMargin) {
		deltaPwm = pwmMargin;
	}
    m_leftMotorPWM=pwm+deltaPwm;
    m_rightMotorPWM=pwm-deltaPwm;
}

void Regulation::setAngleOffset(float angleOffset)
{
    m_angleOffset=angleOffset;
}

void Regulation::setRegulationMode( regulMode_e regulMode)
{
    m_regulMode=regulMode;
}

void Regulation::setControlAngleLimit(float controlAngleLimit)
{
	//m_pidSpeed.setPTuning(m_pidSpeed.getPParam()*m_controlAngleLimit/controlAngleLimit);
	m_controlAngleLimit=controlAngleLimit;
	//m_pidSpeed.setOutputLimits(m_angleOffset-m_controlAngleLimit,m_angleOffset+m_controlAngleLimit);
}

void Regulation::setControlSpeedLimit(float controlSpeedLimit)
{
    m_controlSpeedLimit=controlSpeedLimit;
}

void Regulation::setMotorsState(bool motorsON)
{
    m_motorsON=motorsON;
}

bool  Regulation::checkRobotIsVertical()
{
    bool stateChanged=false;
    if (m_robotStanding) {
        float angleVSVertical=m_measuredAngle-m_angleOffset;
        if ((angleVSVertical<-15.0f)||(angleVSVertical>15.0f)) {
            m_robotStanding=false;
            stateChanged=true;
        }
    } else {
        float measuredAngleVSTarget=m_measuredAngle-m_angleOffset;
        if ((measuredAngleVSTarget>-0.5f)&&(measuredAngleVSTarget<0.5f)) {
            m_robotStanding=true;
            stateChanged=true;
        }
    }
    return(stateChanged);
}

void Regulation::setJoyStickValue(float x, float y)
{
    m_joystickX=x;
    m_joystickY=y;
}

void Regulation::setJoyStickXValue(float x)
{
    m_joystickX=x;
}

void Regulation::setJoyStickYValue(float y)
{
    m_joystickY=y;
}

void Regulation::setJoystickSpeedGain(float joystickSpeedGain)
{
    m_joystickSpeedGain=joystickSpeedGain;
}

void Regulation::setJoystickAngleGain(float joystickAngleGain)
{
    m_joystickAngleGain=joystickAngleGain;
}

void Regulation::setJoystickRotationGain(float joystickRotationGain)
{
    m_joystickRotationGain=joystickRotationGain;
}

float Regulation::getLeftMotorPWM()
{
    return(m_leftMotorPWM);
}

float Regulation::getRightMotorPWM()
{
    return(m_rightMotorPWM);
}

PID *Regulation::getPIDAngle()
{
    return(&m_pidAngle);
}

PID *Regulation::getPIDSpeed()
{
    return(&m_pidSpeed);
}
