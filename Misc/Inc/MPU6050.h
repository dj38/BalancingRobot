/*
 * MPU6050.h
 *
 *  Created on: 6 avr. 2017
 *      Author: joel
 */

#ifndef MPU6050_H_
#define MPU6050_H_
#include "math.h"
//#include "Beep.h"
#include "StatLogCollection.h"
#include "IOSerialStream.h"
#include "Timer.h"

/** A class that allows to calculate yaw, pitch roll using an MPU6050 device
 *
 *
 * Example:
 * @code
 * // Read from I2C slave at address 0x62
 *
 * #include "mbed.h"
 *
 * I2C i2c(p28, p27);
 *
 * int main() {
 *     int address = 0x62;
 *     char data[2];
 *     i2c.read(address, data, 2);
 * }
 * @endcode
 */
class MPU6050 : public IOStreamList {
public:
    enum initStatus {
        OK = 0,
        noConn,
        failSelfTest
    };

    /** Create a MPU6050 interface, connected to the specified pins
     *
     *  @param sda I2C data line pin
     *  @param scl I2C clock line pin
     *  @param frequency I2C bus frequency
     *  @param beepRef pointer to a beeper object
     */
    MPU6050(I2C_HandleTypeDef *hi2c,GPIO_TypeDef* gpioIntPort, uint16_t gpioIntPin,int frequency=400000); //constructor
    ~MPU6050(); //destructor

    initStatus fullInitMPU6050();
    bool  update(bool updateGyroAccel = true, bool updateTemperature = false);  // when new data are available, do a new acquisition and update Quaternion, raw, pitch, roll and temperature. Returns true when temperature, pitch, roll and yaw have been updated
    float pitch();
    float yaw();
    float roll();
    float temperature(); // Temperature in degrees Centigrade
    StatLogCollection m_StatLogList;
	MPU6050();

private:
    // StatLog * m_StatLogPitch;
    // StatLog * m_StatLogYaw;
    // StatLog * m_StatLogRoll;
    // StatLog * m_StatLogTimer;

    //Beep    *beep;
    I2C_HandleTypeDef	*m_hi2c;
    TimeOut m_timerKalmanStartup;
    Timer   m_timer;

    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    char readByte(uint16_t address, uint16_t subAddress);
    void readBytes(uint16_t address, uint16_t subAddress, uint8_t count, uint8_t * dest);

    void getGres() ;
    void getAres() ;

    void readAccelData();
    void readGyroData();
    void readTempData();

// Configure the motion detection control for low power accelerometer mode
    void LowPowerAccelOnly();

    void resetMPU6050() ;
    void initMPU6050();

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
    void calibrateMPU6050(float * dest1, float * dest2);

// Accelerometer and gyroscope self test; check calibration wrt factory settings
    bool MPU6050SelfTest(); // returns true if test if fail;

// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
    void MadgwickQuaternionUpdate();

// Callback used to updaet Kalman filter coefficients at end of startup (tStartup after the fullInit)
    void cbEndOfStartup();

// Set initial input parameters


    enum Ascale {
        AFS_2G = 0,
        AFS_4G,
        AFS_8G,
        AFS_16G
    };

    enum Gscale {
        GFS_250DPS = 0,
        GFS_500DPS,
        GFS_1000DPS,
        GFS_2000DPS
    };

// Specify sensor full scale
    int m_Gscale;
    int m_Ascale;

    float m_aRes, m_gRes; // scale resolutions per LSB for the sensors

    float m_ax, m_ay, m_az;       // Stores the real accel value in g's
    float m_gx, m_gy, m_gz;       // Stores the real gyro value in degrees per seconds
    float m_gyroBias[3], m_accelBias[3] ; // Bias corrections for gyro and accelerometer
    float m_pitch, m_yaw, m_roll;
    float m_temperature;
    float m_SelfTest[6];

// parameters for 6 DoF sensor fusion calculations
    float m_beta;  // compute beta
    float m_zeta ;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    float m_deltat;                       // integration interval for both filter schemes
    bool  m_firstAcquisition;     // Flag used that indicates that no acquisition have been performed since fullInit
    float m_q[4];                 // vector to hold quaternion

    static const float PI;
    static const float GyroMeasError;    // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
    static const float GyroMeasDrift;    // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    static const float initBeta;
    static const float initZeta;
    static const float steadyStateBeta; // decrease filter gain after stabilized
    static const float steadyStateZeta; // increase bias drift gain after stabilized
    static const float tStartup;        // startup time needed to stabize kalman filter
    static const int   tMinAcqInterval_us; // Defines the min time (in us) interval between 2 acquisitions (related to MPU6050 max freq)

    GPIO_TypeDef* m_gpioIntPort;
    uint16_t m_gpioIntPin;
};

#endif /* MPU6050_H_ */
