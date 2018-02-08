
#ifndef REGULATION_H_
#define REGULATION_H_

#include "PID.h"

class Regulation
{
public:
    enum regulMode_e {
        angle = 0,
        pos,
        speed
    };
    typedef struct {
        float target,measure;
    } PID_t;

    Regulation(float angleP, float angleI, float angleD, float speedP, float speedI, float speedD, float angleOffset=0.0, regulMode_e regulMode=angle, float controlAngleLimit=15);
    void update(int now, float measuredAngle, float measuredSpeed=0, float measuredAzimut=0);
    void setRegulationMode( regulMode_e regulMode);
    void enableAzimutRegulation(bool azimutRegulEnable);
    void setControlAngleLimit(float controlAngleLimit);
    void setControlSpeedLimit(float controlSpeedLimit);
    void setMotorsState(bool motorsON);
    void setJoyStickValue(float x, float y);
    void setJoyStickXValue(float x);
    void setJoyStickYValue(float y);
    void setJoystickSpeedGain(float joystickSpeedGain);
    void setJoystickAngleGain(float joystickAngleGain);
    void setJoystickRotationGain(float joystickRotationGain);
    void setAngleOffset(float angleOffset);
    void turnBy(float rotation);
    PID *getPIDAngle();
    PID *getPIDSpeed();
    float getLeftMotorPWM(); 
    float getRightMotorPWM(); 
    float getAngleOffset() const { return m_angleOffset; }
	regulMode_e getRegulMode() const { return m_regulMode; }
	float getTargetSpeed() const { return m_speed.target; }
	float getTargetAngle() const { return m_angle.target; }
	float getMeasuredAngle() const { return m_angle.measure; }
	float getMeasuredSpeed() const { return m_speed.measure; }
	float getControlAngleLimit() const { return m_controlAngleLimit; }

private:
    bool  checkRobotIsVertical();   // returns true if robot "standing state" (standing or horizontal) has changed
    PID   m_pidAngle;           // PID object to handle Angle regulation
    PID   m_pidSpeed;           // PID object to handle Speed regulation
    PID   m_pidAzimut;           // PID object to handle azimut regulation (robot direction)
    PID_t m_speed;              // PID variables for speed
    PID_t m_angle;              // PID variables for angle
    PID_t m_azimut;             // PID variables for azimut
    float m_angleOffset;        // robot angle when it is vertical (~steady state)
    float m_controlAngleLimit;  // Set the maximum tilting angle of the robot
    float m_controlSpeedLimit;  // Set the maximum speed limit
    float m_turningLimit;       // Set the maximum turning value
    regulMode_e m_regulMode;    // Set the regulation mode (angle/speed/position)
    bool  m_azimutPIDEnable;	//
    bool  m_motorsON;           // Allows user to stop motors
    bool  m_robotStanding;      // true if robot is vertical
    float m_joystickSpeedGain, m_joystickAngleGain, m_joystickRotationGain; // gain applied on joystick x,y to calculate target speed/angle/rotation
    float m_joystickAzimutSpeedGain; // gain applied on joystick x to calculate azimut rotation speed
    float m_leftMotorPWM, m_rightMotorPWM; // calculated PWM to be applied on left/right motors
    float m_joystickX,m_joystickY; // joystick input values on x/y axis. Range is [-1;1]
};
#endif /* REGULATION_H_ */
