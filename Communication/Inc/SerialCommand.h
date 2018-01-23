#ifndef SERIALCOMMAND_H
#define SERIALCOMMAND_H
#include <vector>
#include <string>

class SerialCommand
{
public:
    enum command_e {
        NOP=0,
        badCommand,
        setSpeedRegulMode,
        setAngleRegulMode,
		setAzimutRegulMode,
		unsetAzimutRegulMode,
        calibrateAccelerometer,
        calibrateMotors,
        sendPIDValues,
        sendSettings,
        sendInfo,
        sendKalmanFilterValues,
        setAnglePIDValueP,
        setAnglePIDValueI,
        setAnglePIDValueD,
        setAzimutPIDValueP,
        setAzimutPIDValueI,
        setAzimutPIDValueD,
        setAngleOffset,
        setSpeedPIDValueP,
        setSpeedPIDValueI,
        setSpeedPIDValueD,
        setKalmanFilterValues,
        setControlAngleLimit,
        setControlSpeedLimit,
        setTurningLimit,
        setBackToSpot,
        setJoystickXY,
        setJoystickY,
        setJoystickX,
        setTiltControlXY,
		turnByAngle,
		turnToAngle
 };
    
    SerialCommand(std::string in);
    
    command_e getCommand();
    std::string getStringCommand();
    std::vector<float> getValues();
    
private:
    void parseStringForCommand(std::string in);
    void parseStringForValues(std::string in);
    command_e m_command;
    std::vector<float> m_values;
};

#endif
