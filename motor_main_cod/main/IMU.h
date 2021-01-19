#ifndef _IMU_H_
#define _IMU_H_
#include "MPU6050.h"
MPU6050 IMU1(MPU6050_ADDRESS_AD0_HIGH);
MPU6050 IMU2(MPU6050_ADDRESS_AD0_LOW);

#define X 0
#define Y 1
#define Z 2

double dt =0;
u32 last_time_IMU;

struct struct_IMU
{
s16 row_gyro[3];
s16 row_acc[3];
float angle_acc[3];
float angle_gyro[3];
float pitch;
float roll;
float yaw;
}s_IMU1,s_IMU2;


s8 IMU_init(void);
s8 IMU_getdata(void);
s8 IMU_consubtion(void);
s8 IMU_print(void);

s8 IMU_init(void)
{
  Serial.println("Initializing I2C devices...");
  IMU1.initialize();
  // verify connection
  Serial.println("Testing IMU1 connections...");
  Serial.println(IMU1.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  IMU1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  IMU1.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);

  IMU2.initialize();
  Serial.println("Testing IMU2 connections...");
  Serial.println(IMU2.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  IMU2.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  IMU2.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
  return 0;
}

s8 IMU_getdata(void)
{
  IMU1.getMotion6(&s_IMU1.row_acc[X], &s_IMU1.row_acc[Y], &s_IMU1.row_acc[Z],&s_IMU1.row_gyro[X],&s_IMU1.row_gyro[Y], &s_IMU1.row_gyro[Z]);
  IMU2.getMotion6(&s_IMU2.row_acc[X], &s_IMU2.row_acc[Y], &s_IMU2.row_acc[Z],&s_IMU2.row_gyro[X],&s_IMU2.row_gyro[Y], &s_IMU2.row_gyro[Z]);

  // s_IMU1.row_acc[X] = s_IMU1.row_acc[X]/2048.0;
  // s_IMU1.row_acc[Y] = s_IMU1.row_acc[Y]/2048.0;
  // s_IMU1.row_acc[Z] = s_IMU1.row_acc[Z]/2048.0;
  // s_IMU1.row_gyro[X] = s_IMU1.row_gyro[X] / 131;
  // s_IMU1.row_gyro[Y] = s_IMU1.row_gyro[Y] / 131;
  // s_IMU1.row_gyro[Z] = s_IMU1.row_gyro[Z] / 131;
  //
  // s_IMU2.row_gyro[X] = s_IMU2.row_gyro[X] / 131;
  // s_IMU2.row_gyro[Y] = s_IMU2.row_gyro[Y] / 131;
  // s_IMU2.row_gyro[Z] = s_IMU2.row_gyro[Z] / 131;
}

s8 IMU_consubtion(void)
{
  float temp_acc_normal_1;
  float temp_acc_normal_2;

  float acc_normal_1[3];
  float acc_normal_2[3];

  float gyro_1[3];
  float gyro_2[3];

  dt = (double)(micros() - last_time_IMU) / 1000000; // Calculate delta time
  last_time_IMU = micros();
//################################# #IMU1 ################################################
  temp_acc_normal_1 = sqrt(pow(s_IMU1.row_acc[X],2)+pow(s_IMU1.row_acc[Y],2)+pow(s_IMU1.row_acc[Z],2));

  acc_normal_1[X] = s_IMU1.row_acc[X]/temp_acc_normal_1;
  acc_normal_1[Y] = s_IMU1.row_acc[Y]/temp_acc_normal_1;
  acc_normal_1[Z] = s_IMU1.row_acc[Z]/temp_acc_normal_1;

  gyro_1[X] = s_IMU1.row_gyro[X] / 131;
  gyro_1[Y] = s_IMU1.row_gyro[Y] / 131;
  gyro_1[Z] = s_IMU1.row_gyro[Z] / 131;

  s_IMU1.angle_gyro[X]+= (gyro_1[X]*dt);
  s_IMU1.angle_gyro[Y]+= (gyro_1[Y]*dt);
  s_IMU1.angle_gyro[Z]+= (gyro_1[Z]*dt);

  // s_IMU1.angle_acc[X] = asin(acc_normal_1[X])*RAD_TO_DEG;
  // s_IMU1.angle_acc[Y] = asin(acc_normal_1[Y])*RAD_TO_DEG;
  s_IMU1.angle_acc[X] = atan2(acc_normal_1[X],sqrt(pow(acc_normal_1[Y],2)+pow(acc_normal_1[Z],2)))*RAD_TO_DEG;
  s_IMU1.angle_acc[Y] = atan2(acc_normal_1[Y],sqrt(pow(acc_normal_1[X],2)+pow(acc_normal_1[Z],2)))*RAD_TO_DEG;

  s_IMU1.roll = 0.5 * (s_IMU1.roll + (gyro_1[X]) * dt) + 0.5 * (s_IMU1.angle_acc[Y]) ;     // Calculate the angle using a Complimentary filter
  s_IMU1.pitch = 0.5 * (s_IMU1.pitch + (gyro_1[Y]) * dt) + 0.5* (-s_IMU1.angle_acc[X]) ;  // Calculate the angle using a Complimentary filter
  s_IMU1.yaw = s_IMU1.angle_gyro[Z];

  //############################################################################################
  //################################# #IMU2 ################################################
  temp_acc_normal_2 = sqrt(pow(s_IMU2.row_acc[X],2)+pow(s_IMU2.row_acc[Y],2)+pow(s_IMU2.row_acc[Z],2));
  acc_normal_2[X] = s_IMU2.row_acc[X]/temp_acc_normal_2;
  acc_normal_2[Y] = s_IMU2.row_acc[Y]/temp_acc_normal_2;
  acc_normal_2[Z] = s_IMU2.row_acc[Z]/temp_acc_normal_2;

  gyro_2[X] = s_IMU2.row_gyro[X] / 131;
  gyro_2[Y] = s_IMU2.row_gyro[Y] / 131;
  gyro_2[Z] = s_IMU2.row_gyro[Z] / 131;

  s_IMU2.angle_gyro[X]+= (gyro_2[X]*dt);
  s_IMU2.angle_gyro[Y]+= (gyro_2[Y]*dt);
  s_IMU2.angle_gyro[Z]+= (gyro_2[Z]*dt);

  s_IMU2.angle_acc[X] = atan2(acc_normal_2[X],sqrt(pow(acc_normal_2[Y],2)+pow(acc_normal_2[Z],2)))*RAD_TO_DEG;
  s_IMU2.angle_acc[Y] = atan2(acc_normal_2[Y],sqrt(pow(acc_normal_2[X],2)+pow(acc_normal_2[Z],2)))*RAD_TO_DEG;

  s_IMU2.roll = 0.5 * (s_IMU2.roll + (gyro_2[X]) * dt) + 0.5 * (s_IMU2.angle_acc[Y]) ;     // Calculate the angle using a Complimentary filter
  s_IMU2.pitch = 0.5 * (s_IMU2.pitch + (gyro_2[Y]) * dt) + 0.5 * (-s_IMU2.angle_acc[X]) ;  // Calculate the angle using a Complimentary filter
  s_IMU2.yaw = s_IMU2.angle_gyro[Z];

}

s8 IMU_print(void)
{
  Serial.print(dt);Serial.print("  ");
  Serial.print("IMU1:");
  // Serial.print(s_IMU1.row_acc[X]);Serial.print("  ");
  // Serial.print(s_IMU1.row_acc[Y]);Serial.print("  ");
  // Serial.print(s_IMU1.row_acc[Z]);Serial.print("  ");
  // Serial.print(s_IMU1.row_gyro[X]);Serial.print("  ");
  // Serial.print(s_IMU1.row_gyro[Y]);Serial.print("  ");
  // Serial.print(s_IMU1.row_gyro[Z]);Serial.print("  ");
  // Serial.print(s_IMU1.angle_acc[X]);Serial.print("  ");
  // Serial.print(s_IMU1.angle_acc[Y]);Serial.print("  ");
  //
  // Serial.print(s_IMU1.angle_gyro[X]);Serial.print("  ");
  // Serial.print(s_IMU1.angle_gyro[Y]);Serial.print("  ");
  // Serial.print(s_IMU1.angle_gyro[Z]);Serial.print("  ");

  Serial.print(s_IMU1.pitch);Serial.print("  ");
  Serial.print(s_IMU1.roll);Serial.print("  ");
  Serial.print(s_IMU1.yaw);Serial.print("  ");

  // Serial.print("\tIMU2:");
  // // Serial.print(s_IMU2.row_acc[X]);Serial.print("  ");
  // // Serial.print(s_IMU2.row_acc[Y]);Serial.print("  ");
  // // Serial.print(s_IMU2.row_acc[Z]);Serial.print("  ");
  // // Serial.print(s_IMU2.row_gyro[X]);Serial.print("  ");
  // // Serial.print(s_IMU2.row_gyro[Y]);Serial.print("  ");
  // // Serial.print(s_IMU2.row_gyro[Z]);Serial.print("  ");
  // // Serial.print(s_IMU2.angle_acc[X]);Serial.print("  ");
  // // Serial.print(s_IMU2.angle_acc[Y]);Serial.print("  ");
  // //
  // // Serial.print(s_IMU2.angle_gyro[X]);Serial.print("  ");
  // // Serial.print(s_IMU2.angle_gyro[Y]);Serial.print("  ");
  // // Serial.print(s_IMU2.angle_gyro[Z]);Serial.print("  ");
  //
  // Serial.print(s_IMU2.pitch);Serial.print("  ");
  // Serial.print(s_IMU2.roll);Serial.print("  ");
  // Serial.print(s_IMU2.yaw);Serial.print("  ");


}
#endif
