#include "mpu6050.h"

// #include <Wire.h>
// #include "/home/hp/.arduino15/packages/arduino/hardware/avr/1.8.6/libraries/Wire/src/Wire.h"

void ResetDevice(void) {
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(0x80);
  Wire.endTransmission();
  delay(100);
}
void WhoAmI(void) {
  uint8_t status;
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_WHO_AM_I);
  status = Wire.endTransmission();

  if (status == 0) {
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 1);
    if (Wire.available()) {
      uint8_t who_am_i = Wire.read();
      Serial.print("WHO_AM_I 0x");
      Serial.println(who_am_i, HEX);
      if (who_am_i == 0x68) {
        Serial.println("MPU6050 reset succesful and verified");
      } else {
        Serial.print("Unexpected WHO_AM_I value after reset: 0x");
        Serial.println(who_am_i, HEX);
      }
    }
  }
  Serial.println("Failed to verify to reset - communication error");
}

void SetSleepEnabled(bool Status) {
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_PWR_MGMT_1);
  Wire.write(Status);
  Wire.endTransmission();
}

void MPU6050Initialize(uint32_t ClockFrequency) {
  uint16_t _frequency;
  switch (ClockFrequency) {
    case FREQUENCY_100kHz:
      _frequency = FREQUENCY_100kHz;
      break;
    case FREQUENCY_400kHz:
      _frequency = FREQUENCY_400kHz;
      break;
    default:
      _frequency = FREQUENCY_100kHz;
      break;
  }
  Wire.setClock(_frequency);
  Wire.begin();
}

void GetMotion6(float *AccelX, float *AccelY, float *AccelZ, float *GyroX, float *GyroY, float *GyroZ,
                float GyroSensiticvityScaleFactor, float AccelSensitivityScaleFactor) {

  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_RA_ACCEL_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 6);

  *AccelX = Wire.read() << 8 | Wire.read();
  *AccelY = Wire.read() << 8 | Wire.read();
  *AccelZ = Wire.read() << 8 | Wire.read();


  *AccelX /= AccelSensitivityScaleFactor;
  *AccelY /= AccelSensitivityScaleFactor;
  *AccelZ /= AccelSensitivityScaleFactor;


  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_RA_GYRO_XOUT_H);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 6);
  *GyroX = Wire.read() << 8 | Wire.read();
  *GyroY = Wire.read() << 8 | Wire.read();
  *GyroZ = Wire.read() << 8 | Wire.read();

  *GyroX /= GyroSensiticvityScaleFactor;
  *GyroY /= GyroSensiticvityScaleFactor;
  *GyroZ /= GyroSensiticvityScaleFactor;
}

void GyroscopeAndAccelerometerCalibration(float *AccelOffsetX, float *AccelOffsetY, float *AccelOffsetZ,
                                          float *GyroOffsetX, float *GyroOffsetY, float *GyroOffsetZ,
                                          uint8_t AccelSensitivityRange, uint8_t GyroSensitivityRange, uint16_t LoopCycleNumber,
                                          uint8_t CycleInterval_ms) {
  float acc_sensitivity_scale_factor = 16384.0;
  float gyro_sensitivity_scale_factor = 131.0;

  float raw_acc_x_sum = 0.0, raw_acc_y_sum = 0.0, raw_acc_z_sum = 0.0;
  float raw_acc_x, raw_acc_y, raw_acc_z;

  float raw_gyro_x_sum = 0.0, raw_gyro_y_sum = 0.0, raw_gyro_z_sum = 0.0;
  float raw_gyro_x, raw_gyro_y, raw_gyro_z;

  for (int i = 0; i < LoopCycleNumber; i++) {
    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(MPU6050_RA_ACCEL_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 6);

    raw_acc_x = Wire.read() << 8 | Wire.read();
    raw_acc_y = Wire.read() << 8 | Wire.read();
    raw_acc_z = Wire.read() << 8 | Wire.read();

    Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
    Wire.write(MPU6050_RA_GYRO_XOUT_H);
    Wire.endTransmission();
    Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 6);

    raw_gyro_x = Wire.read() << 8 | Wire.read();
    raw_gyro_y = Wire.read() << 8 | Wire.read();
    raw_gyro_z = Wire.read() << 8 | Wire.read();

    switch (GyroSensitivityRange) {
      case MPU6050_GYRO_FS_250:
        gyro_sensitivity_scale_factor = 131.0;
        break;
      case MPU6050_GYRO_FS_500:
        gyro_sensitivity_scale_factor = 65.5;
        break;
      case MPU6050_GYRO_FS_1000:
        gyro_sensitivity_scale_factor = 32.8;
        break;
      case MPU6050_GYRO_FS_2000:
        gyro_sensitivity_scale_factor = 16.4;
        break;
      default:
        gyro_sensitivity_scale_factor = 131.0;
        break;
    }

    switch (AccelSensitivityRange) {
      case MPU6050_ACCEL_FS_2:
        acc_sensitivity_scale_factor = 16384.0;
        break;
      case MPU6050_ACCEL_FS_4:
        acc_sensitivity_scale_factor = 8192.0;
        break;
      case MPU6050_ACCEL_FS_8:
        acc_sensitivity_scale_factor = 4096.0;
        break;
      case MPU6050_ACCEL_FS_16:
        acc_sensitivity_scale_factor = 2048.0;
        break;
      default:
        acc_sensitivity_scale_factor = 16384.0;
        break;
    }

    float gyro_x = raw_gyro_x / gyro_sensitivity_scale_factor;
    float gyro_y = raw_gyro_y / gyro_sensitivity_scale_factor;
    float gyro_z = raw_gyro_z / gyro_sensitivity_scale_factor;

    float acc_x = raw_acc_x / acc_sensitivity_scale_factor;
    float acc_y = raw_acc_y / acc_sensitivity_scale_factor;
    float acc_z = raw_acc_z / acc_sensitivity_scale_factor;

    raw_acc_x_sum += acc_x;
    raw_acc_y_sum += acc_y;
    raw_acc_z_sum += acc_z;

    raw_gyro_x_sum += gyro_x;
    raw_gyro_y_sum += gyro_y;
    raw_gyro_z_sum += gyro_z;

    delay(CycleInterval_ms);
  }

  *GyroOffsetX = raw_gyro_x_sum / LoopCycleNumber;
  *GyroOffsetY = raw_gyro_y_sum / LoopCycleNumber;
  *GyroOffsetZ = raw_gyro_z_sum / LoopCycleNumber;

  *AccelOffsetX = raw_acc_x_sum / LoopCycleNumber;
  *AccelOffsetY = raw_acc_y_sum / LoopCycleNumber;
  *AccelOffsetZ = (raw_acc_z_sum / LoopCycleNumber) - 1.0;
}


uint8_t SetAccelerometerRange(uint8_t Range) {
  uint8_t _range = MPU6050_ACCEL_FS_2;
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(ACCEL_CONFIG);
  switch (Range) {
    case MPU6050_ACCEL_FS_2:
      _range = MPU6050_ACCEL_FS_2;
      break;
    case MPU6050_ACCEL_FS_4:
      _range = MPU6050_ACCEL_FS_4;
      break;
    case MPU6050_ACCEL_FS_8:
      _range = MPU6050_ACCEL_FS_8;
      break;
    case MPU6050_ACCEL_FS_16:
      _range = MPU6050_ACCEL_FS_16;
      break;
    default:
      _range = MPU6050_ACCEL_FS_2;
      break;
  }
  Wire.write(_range);
  return (Wire.endTransmission());
}
void GyroSensitivity(uint16_t GyroRange) {
  uint8_t _range = MPU6050_GYRO_FS_250;
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(GYRO_CONFIG);
  switch (GyroRange) {
    case MPU6050_GYRO_FS_250:
      {
        _range = MPU6050_GYRO_FS_250;
        break;
      }
    case MPU6050_GYRO_FS_500:
      {
        _range = MPU6050_GYRO_FS_500;
        break;
      }
    case MPU6050_GYRO_FS_1000:
      {
        _range = MPU6050_GYRO_FS_1000;
        break;
      }
    case MPU6050_GYRO_FS_2000:
      {
        _range = MPU6050_GYRO_FS_2000;
        break;
      }
    default:
      {
        _range = MPU6050_GYRO_FS_250;
        break;
      }
  }
  Wire.write(_range);
  Wire.endTransmission();
}


uint8_t GyroDLPF_Config(uint8_t DLPF_Conf) {
  uint8_t error_code, _bandwidth;
  const char _bandwidth_frequency[][7] = { "256Hz", "188Hz", "98Hz", "42Hz", "20Hz", "10Hz", "5Hz" };
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(MPU6050_DLPF_CONFIG);

  switch (DLPF_Conf) {
    case 0:
      {
        _bandwidth = MPU6050_DLPF_0_BANDWIDTH_256;
        break;
      }
    case 1:
      {
        _bandwidth = MPU6050_DLPF_1_BANDWIDTH_188;
        break;
      }
    case 2:
      {
        _bandwidth = MPU6050_DLPF_2_BANDWIDTH_98;

        break;
      }
    case 3:
      {
        _bandwidth = MPU6050_DLPF_3_BANDWIDTH_42;
        break;
      }
    case 4:
      {
        _bandwidth = MPU6050_DLPF_4_BANDWIDTH_20;
        break;
      }
    case 5:
      {
        _bandwidth = MPU6050_DLPF_5_BANDWIDTH_10;
        break;
      }
    case 6:
      {
        _bandwidth = MPU6050_DLPF_6_BANDWIDTH_5;
        break;
      }
    default:
      Serial.println("Sorry you choose incorrect bandwidth");
      break;
  }
  Wire.write(_bandwidth);
  error_code = Wire.endTransmission();
  if (error_code == 0) {
    // Serial.println("Successfully configured DLPF bandwidth:$_bandwidth_frequency[DLPF_Config]");
    sprintf("Successfully configured DLPF bandwidth:%s",_bandwidth_frequency[DLPF_Conf]);
    // Serial.println(_bandwidth_frequency[DLPF_Conf]);
  } else if (error_code == 1) {
    Serial.println("Data is too long that doesn't fit in the buffer!");
    return 1;
  } else if (error_code == 2) {
    Serial.println("Received NACK on transmit of the address");
    return 2;
  } else if (error_code == 3) {
    Serial.println("Received NACK on transmit of the data");
    return 3;
  } else if (error_code >= 4) {
    Serial.println("Other errors");
  }
}

uint8_t ReadRegister(uint8_t Register) {
  Wire.beginTransmission(MPU6050_DEFAULT_ADDRESS);
  Wire.write(Register);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_DEFAULT_ADDRESS, 1);
  while (Wire.available()) {
    return (Wire.read());
  }
}

const char *_text(void) {
  return "Hello";
}