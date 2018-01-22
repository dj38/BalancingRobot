#include "SerialCommand.h"
using namespace std;
#include <cstdlib>
//#include "mbed.h"

SerialCommand::SerialCommand(string in)
{
    parseStringForCommand(in);
    parseStringForValues(in);
}

void SerialCommand::parseStringForCommand(string in)
{
    if(in.compare("")==0)       { m_command=NOP; return;}
    if(in.compare(0,2,"SJX")==0) {m_command=setJoystickX; return;}
    if(in.compare(0,3,"SJY")==0) {m_command=setJoystickY; return;}
    if(in.compare(0,3,"SSP")==0) {m_command=setSpeedPIDValueP; return;}
    if(in.compare(0,3,"SSI")==0) {m_command=setSpeedPIDValueI; return;}
    if(in.compare(0,3,"SSD")==0) {m_command=setSpeedPIDValueD; return;}
    if(in.compare(0,3,"SAP")==0) {m_command=setAzimutPIDValueP; return;}
    if(in.compare(0,3,"SAI")==0) {m_command=setAzimutPIDValueI; return;}
    if(in.compare(0,3,"SAD")==0) {m_command=setAzimutPIDValueD; return;}
    if(in.compare(0,3,"SAR")==0) {m_command=setAzimutRegulMode; return;}
    if(in.compare(0,3,"UAR")==0) {m_command=unsetAzimutRegulMode; return;}
    if(in.compare(0,3,"TBA")==0) {m_command=turnByAngle; return;}
    if(in.compare(0,2,"RS")==0) {m_command=setSpeedRegulMode; return;} // "START/STOP" pressed on INFO tab, or enter/leave tab
    if(in.compare(0,2,"RB")==0) {m_command=setAngleRegulMode; return;} // "START/STOP" pressed on INFO tab, or enter/leave tab
    if(in.compare(0,2,"IS")==0) {m_command=NOP; return;} // "START/STOP" pressed on GRAPH tab, or enter/leave tab
    if(in.compare(0,2,"IB")==0) {m_command=NOP; return;} // "START/STOP" pressed on GRAPH tab, or enter/leave tab
//    if(in.compare(0,1,"A"))        {m_command=NOP; return;} // Abort => stopAndReset();
    if(in.compare(0,2,"AC")==0) {m_command=calibrateAccelerometer; return;}
    if(in.compare(0,2,"AM")==0) {m_command=calibrateMotors; return;}
    if(in.compare(0,2,"GP")==0) {m_command=sendPIDValues; return;}
    if(in.compare(0,2,"GS")==0) {m_command=sendSettings; return;}
    if(in.compare(0,2,"GI")==0) {m_command=sendInfo; return;}
    if(in.compare(0,2,"GK")==0) {m_command=sendKalmanFilterValues; return;}
    if(in.compare(0,2,"SP")==0) {m_command=setAnglePIDValueP; return;}
    if(in.compare(0,2,"SI")==0) {m_command=setAnglePIDValueI; return;}
    if(in.compare(0,2,"SD")==0) {m_command=setAnglePIDValueD; return;}
    if(in.compare(0,2,"ST")==0) {m_command=setAngleOffset; return;}
    if(in.compare(0,2,"SK")==0) {m_command=setKalmanFilterValues; return;}
    if(in.compare(0,2,"SA")==0) {m_command=setControlAngleLimit; return;}
    if(in.compare(0,2,"SU")==0) {m_command=setTurningLimit; return;}
    if(in.compare(0,2,"SB")==0) {m_command=setBackToSpot; return;}
    if(in.compare(0,2,"CS")==0) {m_command=setBackToSpot; return;}  //TBC
    if(in.compare(0,2,"CJ")==0) {m_command=setJoystickXY; return;}
    if(in.compare(0,2,"CM")==0) {m_command=setTiltControlXY; return;}
    m_command=badCommand; return; //default

/* list of unmapped commands
    setControlSpeedLimit,
    setMotorsOn,
    setMotorsOff
*/
}

void SerialCommand::parseStringForValues(std::string in)
{
    std::size_t found=in.find_first_of(",");
    while(found!=string::npos)
    {
        if(found!=string::npos)
            in=in.substr(found+1);
        found = in.find_first_of(",");
        string stringVal=in.substr(0,found);
        m_values.push_back(atof(stringVal.c_str()));
    }
}

string SerialCommand::getStringCommand()
{
    switch (m_command) 
    {
        case NOP: return("NOP");
        case badCommand: return("badCommand");
        case setSpeedRegulMode: return("setSpeedRegulMode");
        case setAngleRegulMode: return("setAngleRegulMode");
        case calibrateAccelerometer: return("calibrateAccelerometer");
        case calibrateMotors: return("calibrateMotors");
        case sendPIDValues: return("sendPIDValues");
        case sendSettings: return("sendSettings");
        case sendInfo: return("sendInfo");
        case sendKalmanFilterValues: return("sendKalmanFilterValues");
        case setAnglePIDValueP: return("setAnglePIDValueP");
        case setAnglePIDValueI: return("setAnglePIDValueI");
        case setAnglePIDValueD: return("setAnglePIDValueD");
        case setAzimutPIDValueP: return("setAzimutPIDValueP");
        case setAzimutPIDValueI: return("setAzimutPIDValueI");
        case setAzimutPIDValueD: return("setAzimutPIDValueD");
        case setAngleOffset: return("setAngleOffset");
        case setSpeedPIDValueP: return("setSpeedPIDValueP");
        case setSpeedPIDValueI: return("setSpeedPIDValueI");
        case setSpeedPIDValueD: return("setSpeedPIDValueD");
        case setKalmanFilterValues: return("setKalmanFilterValues");
        case setControlAngleLimit: return("setControlAngleLimit");
        case setControlSpeedLimit: return("setControlSpeedLimit");
        case setTurningLimit: return("setTurningLimit");
        case setBackToSpot: return("setBackToSpot");
        case setJoystickXY: return("setJoystickXY");
        case setJoystickY: return("setJoystickY");
        case setJoystickX: return("setJoystickX");
        case setTiltControlXY: return("setTiltControlXY");
        case turnByAngle: return("turnByAngle");
        case setAzimutRegulMode: return("setAzimutRegulMode");
        case unsetAzimutRegulMode: return("unsetAzimutRegulMode");
    }
    return("undefCommand");
}
 
SerialCommand::command_e SerialCommand::getCommand()
{
    return(m_command);
}

vector<float> SerialCommand::getValues()
{
    return(m_values);
}
