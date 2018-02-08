#include "Regulation.h"
#include <math.h>

Regulation::Regulation(float angleP, float angleI, float angleD, float speedP, float speedI, float speedD, float angleOffset, regulMode_e regulMode, float controlAngleLimit) :
	m_pidAngle(angleP,angleI,angleD), m_pidSpeed(speedP,speedI,speedD), m_pidAzimut(0,0,0)
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
    m_pidAzimut.setBias(0.0);
    m_pidAzimut.setInputLimits(-360,720); // TODO input limits to be defined (How to handle modulo on regulation?...)
    m_pidAzimut.setOutputLimits(-1,1);
    m_pidAzimut.reset();

    m_rightMotorPWM=m_leftMotorPWM=0;
    m_angle.target=m_speed.target=0;
   	m_angle.measure=-90;
   	m_joystickY=m_joystickX=0;
   	m_speed.measure=0;
   	m_joystickRotationGain=1;
   ///// ToDo check initialization of following attributes
   	m_controlSpeedLimit=0;
   	m_turningLimit=0;
    m_joystickAngleGain=1;
    m_joystickSpeedGain=0.25;
    m_azimutPIDEnable=false;
    m_joystickAzimutSpeedGain=360; // in degrees/s
}

// now is time in us
void Regulation::update(int now,float measuredAngle, float measuredSpeed, float measuredAzimut)
{
    m_angle.measure=measuredAngle;
    m_speed.measure=measuredSpeed;
    m_azimut.measure=measuredAzimut;
    if(checkRobotIsVertical()) {
        m_pidAngle.reset();
        m_pidSpeed.reset();
        setJoyStickValue(0,0);
    }
    if(m_regulMode==angle) {
        m_angle.target=m_angleOffset+m_joystickY*m_joystickAngleGain;
    }
    if(m_regulMode==speed) {
        m_speed.target=m_joystickY*m_joystickSpeedGain;
        m_pidSpeed.setSetPoint(m_speed.target);
        m_pidSpeed.setProcessValue(m_speed.measure);
        m_angle.target=-m_pidSpeed.compute(now);
    }
    m_pidAngle.setSetPoint(m_angle.target);
    m_pidAngle.setProcessValue(m_angle.measure);
    float pwm=m_pidAngle.compute(now)*m_motorsON*m_robotStanding;

    float deltaPwm=0;
    if(m_azimutPIDEnable){
    	// reminder now and previous time are given in us...
    	turnBy(m_joystickAzimutSpeedGain*m_joystickX*float(now-m_pidAzimut.getPreviousTime())/1000000.0f);
    	// apply offset on target and measure such that measure remains in [0; 360]
    	if(m_azimut.measure>=360.0) {
    		m_azimut.measure-=360.0;
    		m_azimut.target-=360.0;
    	} else if(m_azimut.measure<0.0) {
    		m_azimut.measure+=360.0;
    		m_azimut.target+=360.0;
    	}
    	m_pidAzimut.setSetPoint(m_azimut.target);
    	m_pidAzimut.setProcessValue(m_azimut.measure);
    	deltaPwm=m_pidAzimut.compute(now);
    } else {
        deltaPwm=m_joystickRotationGain*m_joystickX;
    }
    float pwmMargin=1.0-fabsf(pwm); //checks are done on deltaPWM to ensure stability then rotation
    if (deltaPwm < -pwmMargin) {
    	deltaPwm = -pwmMargin;
    } else if (deltaPwm > pwmMargin) {
		deltaPwm = pwmMargin;
	}
    m_leftMotorPWM=pwm+deltaPwm;
    m_rightMotorPWM=pwm-deltaPwm;
}

void Regulation::turnBy(float rotation)
{
	m_azimut.target+=rotation;
}

void Regulation::enableAzimutRegulation(bool azimutRegulEnable)
{
    if(azimutRegulEnable && !(m_azimutPIDEnable)) {
    	m_azimut.target=m_azimut.measure; // when azimut regulation mode is enabled target is set to measure
    	m_pidAzimut.reset();
    }
	m_azimutPIDEnable=azimutRegulEnable;
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
        float angleVSVertical=m_angle.measure-m_angleOffset;
        if ((angleVSVertical<-15.0f)||(angleVSVertical>15.0f)) {
            m_robotStanding=false;
            stateChanged=true;
        }
    } else {
        float measuredAngleVSTarget=m_angle.measure-m_angleOffset;
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
