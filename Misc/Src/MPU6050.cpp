/*
 * MPU6050.cpp
 *
 *  Created on: 6 avr. 2017
 *      Author: joel
 */

#include "MPU6050.h"

// Define registers per MPU6050, Register Map and Descriptions, Rev 4.2, 08/19/2013 6 DOF Motion sensor fusion device
// Invensense Inc., www.invensense.com
// See also MPU-6050 Register Map and Descriptions, Revision 4.0, RM-MPU-6050A-00, 9/12/2012 for registers not listed in
// above document; the MPU6050 and MPU 9150 are virtually identical but the latter has an on-board magnetic sensor
//
#define XGOFFS_TC        0x00 // Bit 7 PWR_MODE, bits 6:1 XG_OFFS_TC, bit 0 OTP_BNK_VLD
#define YGOFFS_TC        0x01
#define ZGOFFS_TC        0x02
#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B
#define SELF_TEST_X      0x0D
#define SELF_TEST_Y      0x0E
#define SELF_TEST_Z      0x0F
#define SELF_TEST_A      0x10
#define XG_OFFS_USRH     0x13  // User-defined trim values for gyroscope; supported in MPU-6050?
#define XG_OFFS_USRL     0x14
#define YG_OFFS_USRH     0x15
#define YG_OFFS_USRL     0x16
#define ZG_OFFS_USRH     0x17
#define ZG_OFFS_USRL     0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define FF_THR           0x1D  // Free-fall
#define FF_DUR           0x1E  // Free-fall
#define MOT_THR          0x1F  // Motion detection threshold bits [7:0]
#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms
#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL   0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU6050 0x75 // Should return 0x68

// Using the GY-521 breakout board, I set ADO to 0 by grounding through a 4k7 resistor
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
#define ADO 0
#if ADO
#define MPU6050_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU6050_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif

// parameters for 6 DoF sensor fusion calculations
const float MPU6050::PI = 3.14159265358979323846f;
const float MPU6050::GyroMeasError = PI * (60.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 60 deg/s), then reduce after ~10 s to 3
const float MPU6050::GyroMeasDrift = PI * (1.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
const float MPU6050::initBeta = sqrt(3.0f / 4.0f) * GyroMeasError;
const float MPU6050::initZeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;
const float MPU6050::steadyStateBeta = 0.04;  // decrease filter gain after stabilized
const float MPU6050::steadyStateZeta = 0.015; // increase bias drift gain after stabilized
const float MPU6050::tStartup = 10000000;           // startup time (us) needed to stabize kalman filter
const int   MPU6050::tMinAcqInterval_us = 980 ;  // Defines the min time interval (in us) between 2 acquisitions (related to MPU6050 max freq)

//===================================================================================================================
//====== Set of useful function to access acceleratio, gyroscope, and temperature data
//===================================================================================================================

MPU6050::MPU6050(I2C_HandleTypeDef *hi2c,GPIO_TypeDef* gpioIntPort, uint16_t gpioIntPin,int frequency) : IOStreamList(), m_hi2c(hi2c) { // : I2C(sda,scl) //constructor
    //beep=beepRef;
    //m_StatLogPitch = m_StatLogList.appendStatLog("Pitch");
    //m_StatLogYaw   = m_StatLogList.appendStatLog("Yaw");
    //m_StatLogRoll  = m_StatLogList.appendStatLog("Roll");
    //m_StatLogList.setLogCount(true);
    //m_StatLogList.setLogMinMax(false);
    //m_StatLogList.setLogAverage(false);
	Timer m_timer;
	TimeOut m_timerKalmanStartup;
    m_Gscale = GFS_250DPS;
    m_Ascale = AFS_2G;
    getAres();
    getGres();

// Pin definitions
    for (int i=0; i<3; i++) {
        m_gyroBias[i] = 0;// Bias corrections for gyro and accelerometer
        m_accelBias[i] = 0;// Bias corrections for gyro and accelerometer
        m_q[i+1] = 0.0f;  // vector to hold quaternion
    }
    m_q[0] = 1.0f;

// parameters for 6 DoF sensor fusion calculations
    m_beta = initBeta;                           // compute beta
    m_zeta = initZeta;                           // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    m_deltat = 0.0f;                             // integration interval for both filter schemes
    m_firstAcquisition=true;
}

MPU6050::~MPU6050() //destructor
{
    /*if(beep!=0)
        delete beep;*/
    //delete i2c;
}

void MPU6050::writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
{
    uint8_t data_write[2];
    data_write[0] = subAddress;
    data_write[1] = data;
    HAL_I2C_Master_Transmit(m_hi2c,address,data_write,2,100);
}

char MPU6050::readByte(uint16_t address, uint16_t subAddress)
{
	uint8_t data[1]; // `data` will store the register data
    HAL_I2C_Mem_Read(m_hi2c,address,subAddress,1,data,1,100);
    return data[0];
}

void MPU6050::readBytes(uint16_t address, uint16_t subAddress, uint8_t count, uint8_t * dest)
{
	uint8_t data[14];
    HAL_I2C_Mem_Read(m_hi2c,address,subAddress,1,data,count,100);
    for(int ii = 0; ii < count; ii++) {
        dest[ii] = data[ii];
    }
}


void MPU6050::getGres()
{
    switch (m_Gscale) {
            // Possible gyro scales (and their register bit settings) are:
            // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case GFS_250DPS:
            m_gRes = 250.0/32768.0;
            break;
        case GFS_500DPS:
            m_gRes = 500.0/32768.0;
            break;
        case GFS_1000DPS:
            m_gRes = 1000.0/32768.0;
            break;
        case GFS_2000DPS:
            m_gRes = 2000.0/32768.0;
            break;
    }
    m_gRes *= PI/180.0f; // degrees to rad conversion
}

void MPU6050::getAres()
{
    switch (m_Ascale) {
            // Possible accelerometer scales (and their register bit settings) are:
            // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
            // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
        case AFS_2G:
            m_aRes = 2.0/32768.0;
            break;
        case AFS_4G:
            m_aRes = 4.0/32768.0;
            break;
        case AFS_8G:
            m_aRes = 8.0/32768.0;
            break;
        case AFS_16G:
            m_aRes = 16.0/32768.0;
            break;
    }
}


void MPU6050::readAccelData()
{
    int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
    uint8_t rawData[6];  // x/y/z accel register data stored here
    readBytes(MPU6050_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers into data array
    accelCount[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    accelCount[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    accelCount[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    // Now we'll calculate the accleration value into actual g's
    m_ax = (float)accelCount[0]*m_aRes - m_accelBias[0];  // get actual g value, this depends on scale being set
    m_ay = (float)accelCount[1]*m_aRes - m_accelBias[1];
    m_az = (float)accelCount[2]*m_aRes - m_accelBias[2];
}

void MPU6050::readGyroData()
{
    int16_t gyroCount[3];  // Stores the 16-bit signed accelerometer sensor output
    uint8_t rawData[6];  // x/y/z gyro register data stored here
    readBytes(MPU6050_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
    gyroCount[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gyroCount[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gyroCount[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
    // Calculate the gyro value into actual degrees per second
    m_gx = (float)gyroCount[0]*m_gRes; // - m_gyroBias[0];  // get actual gyro value, this depends on scale being set
    m_gy = (float)gyroCount[1]*m_gRes; // - m_gyroBias[1];
    m_gz = (float)gyroCount[2]*m_gRes; // - m_gyroBias[2];
}

void MPU6050::readTempData()
{
    uint8_t rawData[2];  // x/y/z gyro register data stored here
    readBytes(MPU6050_ADDRESS, TEMP_OUT_H, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
    //return (int16_t)(((int16_t)rawData[0]) << 8 | rawData[1]) ;  // Turn the MSB and LSB into a 16-bit value
    int16_t tempCount = ((int16_t)rawData[0] << 8) | rawData[1];
    m_temperature = tempCount / 340. + 36.53; // Temperature in degrees Centigrade
}



// Configure the motion detection control for low power accelerometer mode
void MPU6050::LowPowerAccelOnly()
{

// The sensor has a high-pass filter necessary to invoke to allow the sensor motion detection algorithms work properly
// Motion detection occurs on free-fall (acceleration below a threshold for some time for all axes), motion (acceleration
// above a threshold for some time on at least one axis), and zero-motion toggle (acceleration on each axis less than a
// threshold for some time sets this flag, motion above the threshold turns it off). The high-pass filter takes gravity out
// consideration for these threshold evaluations; otherwise, the flags would be set all the time!

    uint8_t c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x30); // Clear sleep and cycle bits [5:6]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x30); // Set sleep and cycle bits [5:6] to zero to make sure accelerometer is running

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0x38); // Clear standby XA, YA, and ZA bits [3:5]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x00); // Set XA, YA, and ZA bits [3:5] to zero to make sure accelerometer is running

    c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
// Set high-pass filter to 0) reset (disable), 1) 5 Hz, 2) 2.5 Hz, 3) 1.25 Hz, 4) 0.63 Hz, or 7) Hold
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG,  c | 0x00);  // Set ACCEL_HPF to 0; reset mode disbaling high-pass filter

    c = readByte(MPU6050_ADDRESS, CONFIG);
    writeByte(MPU6050_ADDRESS, CONFIG, c & ~0x07); // Clear low-pass filter bits [2:0]
    writeByte(MPU6050_ADDRESS, CONFIG, c |  0x00);  // Set DLPD_CFG to 0; 260 Hz bandwidth, 1 kHz rate

    c = readByte(MPU6050_ADDRESS, INT_ENABLE);
    writeByte(MPU6050_ADDRESS, INT_ENABLE, c & ~0xFF);  // Clear all interrupts
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x40);  // Enable motion threshold (bits 5) interrupt only

// Motion detection interrupt requires the absolute value of any axis to lie above the detection threshold
// for at least the counter duration
    writeByte(MPU6050_ADDRESS, MOT_THR, 0x80); // Set motion detection to 0.256 g; LSB = 2 mg
    writeByte(MPU6050_ADDRESS, MOT_DUR, 0x01); // Set motion detect duration to 1  ms; LSB is 1 ms @ 1 kHz rate

    //wait(0.1);  // Add delay for accumulation of samples
    HAL_Delay(100);  // Add delay for accumulation of samples

    c = readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x07); // Clear high-pass filter bits [2:0]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c |  0x07);  // Set ACCEL_HPF to 7; hold the initial accleration value as a referance

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_2);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c & ~0xC7); // Clear standby XA, YA, and ZA bits [3:5] and LP_WAKE_CTRL bits [6:7]
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, c |  0x47); // Set wakeup frequency to 5 Hz, and disable XG, YG, and ZG gyros (bits [0:2])

    c = readByte(MPU6050_ADDRESS, PWR_MGMT_1);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c & ~0x20); // Clear sleep and cycle bit 5
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, c |  0x20); // Set cycle bit 5 to begin low power accelerometer motion interrupts

}


void MPU6050::resetMPU6050()
{
    // reset device
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);
}


void MPU6050::initMPU6050()
{
// Initialize MPU6050 device
// wake up device
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    HAL_Delay(100); // Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt

// get stable time source
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);  // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

// Configure Gyro and Accelerometer
// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively;
// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
//    writeByte(MPU6050_ADDRESS, CONFIG, 0x03);
    writeByte(MPU6050_ADDRESS, CONFIG, 0x01); // new value set by jd

// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
//    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; the same rate set in CONFIG above
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Use a max rate; modified by jd

// Set gyroscope full scale range
// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
    uint8_t c =  readByte(MPU6050_ADDRESS, GYRO_CONFIG);
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, c | m_Gscale << 3); // Set full scale range for the gyro

// Set accelerometer configuration
    c =  readByte(MPU6050_ADDRESS, ACCEL_CONFIG);
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c & ~0x18); // Clear AFS bits [4:3]
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, c | m_Ascale << 3); // Set full scale range for the accelerometer

    // Configure Interrupts and Bypass Enable
    // Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, enable I2C_BYPASS_EN so additional chips
    // can join the I2C bus and all can be controlled by the Arduino as master
    writeByte(MPU6050_ADDRESS, INT_PIN_CFG, 0x22);
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void MPU6050::calibrateMPU6050(float * dest1, float * dest2)
{
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

// reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    HAL_Delay(100);

// get stable time source
// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
    writeByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
    HAL_Delay(200);

// Configure device for bias calculation
    writeByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    writeByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
    writeByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    HAL_Delay(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
    writeByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
    uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

// Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    HAL_Delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes

// At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
        accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
        gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
        gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
        gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0]  += (int32_t) gyro_temp[0];
        gyro_bias[1]  += (int32_t) gyro_temp[1];
        gyro_bias[2]  += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0]  /= (int32_t) packet_count;
    gyro_bias[1]  /= (int32_t) packet_count;
    gyro_bias[2]  /= (int32_t) packet_count;
    /* Modification by dj
        if(accel_bias[2] > 0L) {
            accel_bias[2] -= (int32_t) accelsensitivity;   // Remove gravity from the z-axis accelerometer bias calculation
        } else {
            accel_bias[2] += (int32_t) accelsensitivity;
        }
    */
    if(accel_bias[0] > 0L) {
        accel_bias[0] -= (int32_t) accelsensitivity;   // Remove gravity from the x-axis accelerometer bias calculation
    } else {
        accel_bias[0] += (int32_t) accelsensitivity;
    }
// End of modification by dj

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
    data[3] = (-gyro_bias[1]/4)       & 0xFF;
    data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
    data[5] = (-gyro_bias[2]/4)       & 0xFF;

// Push gyro biases to hardware registers
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRH, data[0]);
    writeByte(MPU6050_ADDRESS, XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_ADDRESS, YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_ADDRESS, ZG_OFFS_USRL, data[5]);

    dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity; // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
    dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
    readBytes(MPU6050_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t) ((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t) ((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

    for(ii = 0; ii < 3; ii++) {
        if(accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1]/8);
    accel_bias_reg[2] -= (accel_bias[2]/8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0])      & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1])      & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2])      & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_H, data[0]);
//  writeByte(MPU6050_ADDRESS, XA_OFFSET_L_TC, data[1]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_H, data[2]);
//  writeByte(MPU6050_ADDRESS, YA_OFFSET_L_TC, data[3]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_H, data[4]);
//  writeByte(MPU6050_ADDRESS, ZA_OFFSET_L_TC, data[5]);

// Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
    dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
    dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


// Accelerometer and gyroscope self test; check calibration wrt factory settings
bool MPU6050::MPU6050SelfTest() // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4] = {0, 0, 0, 0};
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    writeByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0xF0); // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU6050_ADDRESS, GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    HAL_Delay(250);;  // Delay a while to let the device execute the self-test
    rawData[0] = readByte(MPU6050_ADDRESS, SELF_TEST_X); // X-axis self-test results
    rawData[1] = readByte(MPU6050_ADDRESS, SELF_TEST_Y); // Y-axis self-test results
    rawData[2] = readByte(MPU6050_ADDRESS, SELF_TEST_Z); // Z-axis self-test results
    rawData[3] = readByte(MPU6050_ADDRESS, SELF_TEST_A); // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4 ; // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 4 ; // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03) >> 4 ; // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0]  & 0x1F ; // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1]  & 0x1F ; // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2]  & 0x1F ; // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[0] - 1.0f)/30.0f))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[1] - 1.0f)/30.0f))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0f*0.34f)*(pow( (0.92f/0.34f) , ((selfTest[2] - 1.0f)/30.0f))); // FT[Za] factory trim calculation
    factoryTrim[3] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[3] - 1.0f) ));             // FT[Xg] factory trim calculation
    factoryTrim[4] =  (-25.0f*131.0f)*(pow( 1.046f , (selfTest[4] - 1.0f) ));             // FT[Yg] factory trim calculation
    factoryTrim[5] =  ( 25.0f*131.0f)*(pow( 1.046f , (selfTest[5] - 1.0f) ));             // FT[Zg] factory trim calculation

//  Output self-test results and factory trim calculation if desired
//  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
//  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
//  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
//  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

// Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
// To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        m_SelfTest[i] = 100.0f + 100.0f*(selfTest[i] - factoryTrim[i])/factoryTrim[i]; // Report percent differences
    }
    // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
    return(m_SelfTest[0] > 1.0f || m_SelfTest[1] > 1.0f || m_SelfTest[2] > 1.0f || m_SelfTest[3] > 1.0f || m_SelfTest[4] > 1.0f || m_SelfTest[5] > 1.0f);
}


// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration and rotation rate to produce a quaternion-based estimate of relative
// device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
// but is much less computationally intensive---it can be performed on a 3.3 V Pro Mini operating at 8 MHz!
void MPU6050::MadgwickQuaternionUpdate()
{
    float ax=m_ax; // work on local copies of m_a[xyz] since a[xyz] are normalized
    float ay=m_ay;
    float az=m_az;
    float q1 = m_q[0], q2 = m_q[1], q3 = m_q[2], q4 = m_q[3];         // short name local variable for readability
    float norm;                                               // vector norm
    float f1, f2, f3;                                         // objective funcyion elements
    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33; // objective function Jacobian elements
    float qDot1, qDot2, qDot3, qDot4;
    float hatDot1, hatDot2, hatDot3, hatDot4;
    float gerrx, gerry, gerrz, gbiasx, gbiasy, gbiasz;  // gyro bias error

    // Auxiliary variables to avoid repeated arithmetic
    float _halfq1 = 0.5f * q1;
    float _halfq2 = 0.5f * q2;
    float _halfq3 = 0.5f * q3;
    float _halfq4 = 0.5f * q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
//            float _2q1q3 = 2.0f * q1 * q3;
//            float _2q3q4 = 2.0f * q3 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute the objective function and Jacobian
    f1 = _2q2 * q4 - _2q1 * q3 - ax;
    f2 = _2q1 * q2 + _2q3 * q4 - ay;
    f3 = 1.0f - _2q2 * q2 - _2q3 * q3 - az;
    J_11or24 = _2q3;
    J_12or23 = _2q4;
    J_13or22 = _2q1;
    J_14or21 = _2q2;
    J_32 = 2.0f * J_14or21;
    J_33 = 2.0f * J_11or24;

    // Compute the gradient (matrix multiplication)
    hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    hatDot3 = J_12or23 * f2 - J_33 *f3 - J_13or22 * f1;
    hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize the gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    hatDot1 /= norm;
    hatDot2 /= norm;
    hatDot3 /= norm;
    hatDot4 /= norm;

    // Compute estimated gyroscope biases
    gerrx = _2q1 * hatDot2 - _2q2 * hatDot1 - _2q3 * hatDot4 + _2q4 * hatDot3;
    gerry = _2q1 * hatDot3 + _2q2 * hatDot4 - _2q3 * hatDot1 - _2q4 * hatDot2;
    gerrz = _2q1 * hatDot4 - _2q2 * hatDot3 + _2q3 * hatDot2 - _2q4 * hatDot1;

    // Compute and remove gyroscope biases
    gbiasx += gerrx * m_deltat * m_zeta; // ToDo : check why uninitialized and unused...
    gbiasy += gerry * m_deltat * m_zeta; // ToDo : check why uninitialized and unused...
    gbiasz += gerrz * m_deltat * m_zeta; // ToDo : check why uninitialized and unused...
//           m_gx -= gbiasx;
//           m_gy -= gbiasy;
//           m_gz -= gbiasz;

    // Compute the quaternion derivative
    qDot1 = -_halfq2 * m_gx - _halfq3 * m_gy - _halfq4 * m_gz;
    qDot2 =  _halfq1 * m_gx + _halfq3 * m_gz - _halfq4 * m_gy;
    qDot3 =  _halfq1 * m_gy - _halfq2 * m_gz + _halfq4 * m_gx;
    qDot4 =  _halfq1 * m_gz + _halfq2 * m_gy - _halfq3 * m_gx;

    // Compute then integrate estimated quaternion derivative
    q1 += (qDot1 -(m_beta * hatDot1)) * m_deltat;
    q2 += (qDot2 -(m_beta * hatDot2)) * m_deltat;
    q3 += (qDot3 -(m_beta * hatDot3)) * m_deltat;
    q4 += (qDot4 -(m_beta * hatDot4)) * m_deltat;

    // Normalize the quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    m_q[0] = q1 * norm;
    m_q[1] = q2 * norm;
    m_q[2] = q3 * norm;
    m_q[3] = q4 * norm;

    // Define output variables from updated quaternion---these are Tait-Bryan angles, commonly used in aircraft orientation.
    // In this coordinate system, the positive z-axis is down toward Earth.
    // Yaw is the angle between Sensor x-axis and Earth magnetic North (or true North if corrected for local declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the Earth is positive, up toward the sky is negative.
    // Roll is angle between sensor y-axis and Earth ground plane, y-axis up is positive roll.
    // These arise from the definition of the homogeneous rotation matrix constructed from quaternions.
    // Tait-Bryan angles as well as Euler angles are non-commutative; that is, the get the correct orientation the rotations must be
    // applied in the correct order which for this configuration is yaw, pitch, and then roll.
    // For more see http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles which has additional links.
    m_yaw   = atan2(2.0f * (m_q[1] * m_q[2] + m_q[0] * m_q[3]), m_q[0] * m_q[0] + m_q[1] * m_q[1] - m_q[2] * m_q[2] - m_q[3] * m_q[3]);
    m_pitch = -asin(2.0f * (m_q[1] * m_q[3] - m_q[0] * m_q[2]));
    m_roll  = atan2(2.0f * (m_q[0] * m_q[1] + m_q[2] * m_q[3]), m_q[0] * m_q[0] - m_q[1] * m_q[1] - m_q[2] * m_q[2] + m_q[3] * m_q[3]);
    m_pitch *= 180.0f / PI;
    m_yaw   *= 180.0f / PI;
    m_roll  *= 180.0f / PI;
}

MPU6050::initStatus MPU6050::fullInitMPU6050()
{
    // Read the WHO_AM_I register, this is a good test of communication
    uint8_t whoami_MPU = readByte(MPU6050_ADDRESS, WHO_AM_I_MPU6050);  // Read WHO_AM_I register for MPU-6050
    initStatus status;
    m_beta = initBeta;                           // compute beta
    m_zeta = initZeta;                           // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value
    m_deltat = 0.0f;                             // integration interval for both filter schemes
    if (whoami_MPU != 0x68)
        status=noConn;
    else {
        if(MPU6050SelfTest()) // Start by performing self test and reporting values (1 is fail)
            status=failSelfTest;
        else {
        	HAL_Delay(100);
            resetMPU6050(); // Reset registers to default in preparation for device calibration
            calibrateMPU6050(m_gyroBias, m_accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
            initMPU6050();
            HAL_Delay(100);
            status=OK;
            m_firstAcquisition=true;
            m_timer.start();
        }
    }
    switch (status) {
        case OK : {
        	*this << "MPU initialized for active data mode....\n\r";
            //if(beep!=0) beep->repeatedBeep(440.0,0.01,0.1,2);
            break;
        }
        case failSelfTest : {
        	*this << "ERROR : MPU failed self test\n\r";
            //if(beep!=0) beep->repeatedBeep(880.0,0.01,.2,10);
            break;
        }
        case noConn : {
        	*this << "ERROR : failed to connect to MPU\n\r";
            //if(beep!=0) beep->repeatedBeep(880.0,0.01,0.2,5);
            break;
        }
    }
    return(status);
}

bool MPU6050::update(bool updateGyroAccel, bool updateTemperature)
{
    if((m_timer.read_us() > tMinAcqInterval_us) && (updateGyroAccel || updateTemperature)) { // to avoid an i2c access if it is evidence that no new data is available
        // If data ready bit set, all data registers have new data
        if(readByte(MPU6050_ADDRESS, INT_STATUS) & 0x01) {  // check if data ready interrupt
            if(updateTemperature) {
                readTempData();  // Read the temperature value
            }
            if(updateGyroAccel) {
                m_deltat=m_timer.read();
                m_timer.reset();
                readAccelData();  // Read the x/y/z adc values and updates ax, ay, az
                readGyroData();  // Read the x/y/z adc values and updates gx, gy, gz
                if (m_timerKalmanStartup.timeoutCheck()) {
                	//    if(beep!=0) beep->repeatedBeep(440.0,0.01,0.1,2);
                	    m_beta = steadyStateBeta;  // decrease filter gain after stabilized
                	    m_zeta = steadyStateZeta;  // increase bias drift gain after stabilized
                };
                if (m_firstAcquisition) {
                    m_firstAcquisition=false;
                    m_timerKalmanStartup.setTimeOut(tStartup);
                } else {
                    MadgwickQuaternionUpdate();
                    //m_StatLogPitch->addValue(m_pitch);
                    //m_StatLogYaw->addValue(m_yaw);
                    //m_StatLogRoll->addValue(m_roll);
                }
            }
            return(true);
        }
    }
    return(false);
}

float MPU6050::pitch()
{
    return(m_pitch);
}

float MPU6050::yaw()
{
    return(m_yaw);
}

float MPU6050::roll()
{
    return(m_roll);
}

float MPU6050::temperature()
{
    return(m_temperature);
}
