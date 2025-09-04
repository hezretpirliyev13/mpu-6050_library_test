#ifndef _MPU6050_H_
#define _MPU6050_H_
// #include "/home/hp/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src/Wire.h"

#if defined(ARDUINO_ARCH_AVR)  // Arduino Uno, Nano, etc.
#include <Wire.h>
#include <stdint.h>
#include "Arduino.h"
#elif defined(MCU_STM32F103C8)
// STM32 boards in Arduino IDE
#include <Wire.h>  // STM32duino core also provides a Wire implementation
TwoWire Wire(2, I2C_FAST_MODE);

#else
#error "Unsupported architecture"
#endif

#define FREQUENCY_100kHz 100000
#define FREQUENCY_400kHz 400000

#define MPU6050_WHO_AM_I 0x75

//Addresses
#define MPU6050_ADDRESS_AD0_LOW 0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69
#define MPU6050_DEFAULT_ADDRESS 0x68

#define MPU6050_PWR_MGMT_1 0x6B

#define GYRO_CONFIG 0x1B
//Gyroscope Full Scale
enum MPU6050_GYRO_FS_SCALE : uint8_t { MPU6050_GYRO_FS_250 = 0x00,   // ±250 °/s
                                       MPU6050_GYRO_FS_500 = 0x08,   // ±500 °/s
                                       MPU6050_GYRO_FS_1000 = 0x10,  // ±1000 °/s
                                       MPU6050_GYRO_FS_2000 = 0x18   // ±2000 °/s
};

// Power Management
#define MPU6050_PWR_MGMT_1 0x6B

// DLPF_CONFIG
enum DLPF_Config : uint8_t { MPU6050_DLPF_0_BANDWIDTH_256 = 0x00,  // 8KHz
                             MPU6050_DLPF_1_BANDWIDTH_188,         //1kHz
                             MPU6050_DLPF_2_BANDWIDTH_98,          //1kHz
                             MPU6050_DLPF_3_BANDWIDTH_42,          //1kHz
                             MPU6050_DLPF_4_BANDWIDTH_20,          //1kHz
                             MPU6050_DLPF_5_BANDWIDTH_10,          //1kHz
                             MPU6050_DLPF_6_BANDWIDTH_5 };         //1kHz

// Accelerometer raw data registers
enum MPU6050_RA_ACCEL_REGS : uint8_t { MPU6050_RA_ACCEL_XOUT_H = 0x3B,
                                       MPU6050_RA_ACCEL_XOUT_L = 0x3C,
                                       MPU6050_RA_ACCEL_YOUT_H = 0x3D,
                                       MPU6050_RA_ACCEL_YOUT_L = 0x3E,
                                       MPU6050_RA_ACCEL_ZOUT_H = 0x3F,
                                       MPU6050_RA_ACCEL_ZOUT_L = 0x40 };
// Accelerometer Config
#define ACCEL_CONFIG 0x1C
enum MPU6050_ACC_FS_SCALE : uint8_t { MPU6050_ACCEL_FS_2 = 0x00,     // ±2g
                                      MPU6050_ACCEL_FS_4 = 0x08,     // ±4g
                                      MPU6050_ACCEL_FS_8 = 0x10,     // ±8g
                                      MPU6050_ACCEL_FS_16 = 0x18 };  // ±16g


// Gyroscope raw data registers
#define MPU6050_RA_GYRO_XOUT_H 0x43
#define MPU6050_RA_GYRO_XOUT_L 0x44
#define MPU6050_RA_GYRO_YOUT_H 0x45
#define MPU6050_RA_GYRO_YOUT_L 0x46
#define MPU6050_RA_GYRO_ZOUT_H 0x47
#define MPU6050_RA_GYRO_ZOUT_L 0x48

#define MPU6050_DLPF_CONFIG 0x1A

// #define mpu_6050_dlpf_0 0x00
// #define mpu_6050_dlpf_1 0x01
// #define mpu_6050_dlpf_2 0x02
// #define mpu_6050_dlpf_3 0x03
// #define mpu_6050_dlpf_4 0x04
// #define mpu_6050_dlpf_5 0x05
// #define mpu_6050_dlpf_6 0x06
// #define mpu_6050_dlpf_7 0x07

void ResetDevice(void);
void SetSleepEnabled(bool Status);
void MPU6050Initialize(uint32_t ClockFrequency);
void GetMotion6(float *AccelX, float *AccelY, float *AccelZ, float *GyroX, float *GyroY, float *GyroZ,
                float GyroSensiticvityScaleFactor, float AccelSensitivityScaleFactor);
void GyroscopeAndAccelerometerCalibration(float *AccelOffsetX, float *AccelOffsetY, float *AccelOffsetZ,
                                          float *GyroOffsetX, float *GyroOffsetY, float *GyroOffsetZ,
                                          uint8_t AccelSensitivityRange, uint8_t GyroSensitivityRange, uint16_t LoopCycleNumber,
                                          uint8_t CycleInterval_ms);

uint8_t SetAccelerometerRange(uint8_t Range);
void GyroSensitivity(uint16_t GyroRange);
uint8_t GyroDLPF_Config(uint8_t DLPF_Conf);
uint8_t ReadRegister(uint8_t Register);
void WhoAmI(void);
const char *_text(void);

#endif