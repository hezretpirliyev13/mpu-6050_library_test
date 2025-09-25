#include "mpu6050.h"

float accX, accY, accZ, gyroX, gyroY, gyroZ;
float Acc_Offset_X, Acc_Offset_Y, Acc_Offset_Z;
float Gyro_Offset_X, Gyro_Offset_Y, Gyro_Offset_Z;

float Ax, Ay, Az;
MPU6050 mpu6050(0x68);


void setup() {
  Serial.begin(115200);
  // Serial.println(_text());
  while (!Serial) {}
  mpu6050.WhoAmI();
  // ResetDevice();
  // MPU6050Initialize(FREQUENCY_100kHz);
  // SetSleepEnabled(false);
  delay(250);

  mpu6050.SetAccelerometerRange(MPU6050_ACCEL_FS_8);
  mpu6050.GyroDLPF_Config(MPU6050_DLPF_0_BANDWIDTH_256);

  // Serial.print("0x");
  // Serial.println(ReadRegister(0x1C), HEX);

  mpu6050.GyroscopeAndAccelerometerCalibration(&Acc_Offset_X, &Acc_Offset_Y, &Acc_Offset_Z,
                                               &Gyro_Offset_X, &Gyro_Offset_Y, &Gyro_Offset_Z,
                                               2000, 2);

  Serial.print("Acc X:");
  Serial.print(Acc_Offset_X);
  Serial.print(" Y:");
  Serial.print(Acc_Offset_Y);
  Serial.print(" Z:");
  Serial.print(Acc_Offset_Z);
  Serial.print("Gyro X:");
  Serial.print(Gyro_Offset_X);
  Serial.print(" Y:");
  Serial.print(Gyro_Offset_Y);
  Serial.print(" Z:");
  Serial.println(Gyro_Offset_Z);
}


void loop() {
  // GetMotion6(&accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ, 65.5, 4096.0);

  // accX -= Acc_Offset_X;
  // accY -= Acc_Offset_Y;
  // accZ -= Acc_Offset_Z;

  // gyroX -= Gyro_Offset_X;
  // gyroY -= Gyro_Offset_Y;
  // gyroZ -= Gyro_Offset_Z;



  // Serial.print("Acc: X = ");
  // Serial.print(accX);
  // Serial.print(" Y = ");
  // Serial.print(accY);
  // Serial.print(" Z = ");
  // Serial.print(accZ);
  // Serial.print(" Gyro: X = ");
  // Serial.print(gyroX);
  // Serial.print(" Y = ");
  // Serial.print(gyroY);
  // Serial.print(" Z = ");
  // Serial.println(gyroZ);
  // delay(200);
}
