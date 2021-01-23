/**************************************************************************
 This is an example for our Monochrome OLEDs based on SSD1306 drivers

 Pick one up today in the adafruit shop!
 ------> http://www.adafruit.com/category/63_98

 This example is for a 128x32 pixel display using I2C to communicate
 3 pins are required to interface (two I2C and one reset).

 Adafruit invests time and resources providing this open
 source code, please support Adafruit and open-source
 hardware by purchasing products from Adafruit!

 Written by Limor Fried/Ladyada for Adafruit Industries,
 with contributions from the open source community.
 BSD license, check license.txt for more information
 All text above, and the splash screen below must be
 included in any redistribution.
 **************************************************************************/
#include "serial_command.h"
#include "my_type.h"
#include <avr/io.h>
#include <avr/wdt.h>
/*************
0 :  none
1 : YPR
2 : acc  XYZ
3 : gyro XYZ
**************/
#define NONE 0
#define YPR 1
#define ACC 2
#define GYRO 3
#define MAG 4
#define ALL 5
u8 serial_out_mode =YPR;
bool flag_acc_calibrarion = 0,flag_gyro_calibrarion=0,flag_mag_calibrarion=0;
 /**************************************************************************/
#include <EEPROM.h>
#define X 0
#define Y 1
#define Z 2
#define PI 3.14
#define TO_DEG 180.0/PI
#define TO_RAD PI/180.0


#include <SPI.h>
#include <Wire.h>
/*************************** ADXL ***************************/
#include "I2Cdev.h"
// #include "ADXL345.h"
// ADXL345 obj_adxl;
// int   adxl_row[3];
// float adxl_acc[3];
// float adxl_normal[3];
// float adxl_angle[2];
// float last_adxl_pitch = 0,last_adxl_roll = 0;
/***********************************************************/
/***************************MPU*****************************/
#include "MPU6050.h"
MPU6050 mpu;
int16_t mpu_acc[3];
int16_t mpu_gyro[3];

float mpu_gyro_rate[3];

float mpu_acc_normal[3];
float mpu_angle_acc[3];
float mpu_angle_gyro[3];

double dt =0;
uint32_t timer=0;
bool flag_start = 0;
bool flag_start_compass = 0;

float pitch=0,roll=0,yaw=0;
/***********************************************************/
/************************* compass**************************/
#include "HMC5883L.h"
HMC5883L compass;
int16_t mag[3];
int16_t mag_row[3];
float Xhorizontal,Yhorizontal;
float heading,heading2;
int16_t offset_mag[3];
#define OFFSET_MAGX 137
#define OFFSET_MAGY 220
#define OFFSET_MAGZ  -22

/***********************************************************/
/*************************** L3G42xx ***********************/
// #include "L3G4200D.h"
// L3G4200D obj_L3G;
// int16_t l3g_rowGyro[3];
// float   l3g_gyro_rate[3];
// float   l3g_angle_gyro[3];
// int32_t pitch=0,roll=0;

/*********************************************************/
float gyro[3];
float angle_gyro[3];
float acc[3];
float angle_acc[3];
float comp_angle_pitch;
float comp_angle_roll;
float comp_angle_yaw;
float mag_gyro_angle_yaw=0;
int16_t offset_acc[3];
int16_t offset_gyro[3];


void acc_manual_offset(int16_t offsetX,int16_t offsetY,int16_t offsetZ);


void setup() {
  Serial.begin(1000000);
  // UCSR0B|=(1<<RXCIE0);
  Serial.println("Serial ok.");


  /********************** whatch dog reset setup*************************/
  wdt_enable(WDTO_2S);
//  WDTCR |= (1<<WDP2)|(1<<WDP1);
//  WDTCR &=~ ((1<<WDP3)|(1<<WDP0));

  MCUSR &= ~(1<<WDRF); //clear interupt flag by set 0 in it
  WDTCSR |= (1<<WDE);   //Watchdog System Reset Enable
  WDTCSR &=~ (1<<WDIE); //desable interupt
  wdt_reset();
  /**********************************************************************/

  pinMode(A0,INPUT_PULLUP);
  pinMode(A1,INPUT_PULLUP);

  Wire.setClock(400000);

  Serial.println("Initializing I2C devices...");

  // obj_adxl.initialize();
  // Serial.println("Testing device connections...");
  // Serial.println(obj_adxl.testConnection() ? "ADXL345 connection successful" : "ADXL345 connection failed");
  // obj_adxl.setRate(0x06);
  // Serial.print("ADXL345_rate:");Serial.print(obj_adxl.getRate());
  // obj_adxl.setLowPowerEnabled(0);
  // accel.setFullResolution(1);
  // accel.setRange(0x03);
  // obj_adxl.setRange(0x00);

  /*************************** MPU *****************************/
  mpu.initialize();
// verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.setRate(0);
  mpu.setFullScaleGyroRange(0);
  mpu.setFullScaleAccelRange(3);
  mpu.setDLPFMode(4);
  Serial.print("DLPF:");Serial.println(mpu.getDLPFMode());
  Serial.print("Rate:");Serial.println(mpu.getRate());
  Serial.print("GyroRate:");Serial.println(mpu.getFullScaleGyroRange());
  Serial.print("AccRate:");Serial.println(mpu.getFullScaleAccelRange());

  mpu.setXGyroOffset(204);
  mpu.setYGyroOffset(-43);
  mpu.setZGyroOffset(38);
  mpu.setXAccelOffset(-2884);
  mpu.setYAccelOffset(-1279);
  mpu.setZAccelOffset(979);
  Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0
  Serial.print(mpu.getXAccelOffset()); Serial.print("\t"); // -76
  Serial.print(mpu.getYAccelOffset()); Serial.print("\t"); // -2359
  Serial.print(mpu.getZAccelOffset()); Serial.print("\t"); // 1688

  compass.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(compass.testConnection() ? "HMC5883L connection successful" : "HMC5883L connection failed");

  /***************************************************
  loade offset from eeprom    adress

  0   1   2   3   4   5       compass
  6   7   8   9   10  11      acc
  12 13   14  15  16  17      gyro
  ****************************************************/
  compass_get_offset();
  acc_get_offset();
  gyro_get_offset();

  // while(1);
}

void loop()
 {
   wdt_reset();
   if(flag_mag_calibrarion == 1)
   {
     compass_calibration();
   }else if(flag_acc_calibrarion ==1)
   {
     acc_calibration();

   }else if(flag_gyro_calibrarion ==1)
   {
     gyro_calibration();
   }

    mpu.getMotion6(&mpu_acc[X], &mpu_acc[Y], &mpu_acc[Z], &mpu_gyro[X], &mpu_gyro[Y], &mpu_gyro[Z]);
    mpu_gyro_rate[X] =  (mpu_gyro[X]-offset_gyro[X])/131.0;
    mpu_gyro_rate[Y] = (mpu_gyro[Y]-offset_gyro[Y])/131.0;
    mpu_gyro_rate[Z] = (mpu_gyro[Z]-offset_gyro[Z])/131.0;

    mpu_acc[X] -= offset_acc[X];
    mpu_acc[Y] -= offset_acc[Y];
    mpu_acc[Z] -= offset_acc[Z];

    float mpu_acc_sum;
    mpu_acc_sum = sqrt(pow(mpu_acc[X],2)+pow(mpu_acc[Y],2)+pow(mpu_acc[Z],2));
    mpu_acc_normal[X] = mpu_acc[X]/mpu_acc_sum;
    mpu_acc_normal[Y] = mpu_acc[Y]/mpu_acc_sum;
    mpu_acc_normal[Z] = mpu_acc[Z]/mpu_acc_sum;

    mpu_angle_acc[X] = asin(mpu_acc_normal[X])*TO_DEG;
    mpu_angle_acc[Y] = asin(mpu_acc_normal[Y])*TO_DEG;

    compass.getHeading(&mag_row[X],&mag_row[Y],&mag_row[Z]);
    float mag_normal[3],sum_mag;
    mag_row[X] = (mag_row[X]-offset_mag[X]);
    mag_row[Y] = (mag_row[Y]-offset_mag[Y]);
    mag_row[Z] = (mag_row[Z]-offset_mag[Z]);
    sum_mag = sqrt(pow(mag_row[X],2)+pow(mag_row[Y],2)+pow(mag_row[Z],2));
    mag_normal[X] = (mag_row[X])/sum_mag;
    mag_normal[Y] = (mag_row[Y])/sum_mag;
    mag_normal[Z] = (mag_row[Z])/sum_mag;

/*####################################################################*/

    acc[X] = mpu_acc_normal[X];
    acc[Y] = mpu_acc_normal[Y];
    acc[Z] = mpu_acc_normal[Z];
    angle_acc[X] = mpu_angle_acc[X];
    angle_acc[Y] = mpu_angle_acc[Y];

    gyro[X] = mpu_gyro_rate[X];
    gyro[Y] = mpu_gyro_rate[Y];
    gyro[Z] = mpu_gyro_rate[Z];

    mag[X] = mag_normal[X];
    mag[Y] = mag_normal[Y];
    mag[Z] = mag_normal[Z];

    if(acc[Z]>0)
    {
      gyro[Y] = -gyro[Y];
      gyro[Z] = -gyro[Z];

    }else
    {
      angle_acc[Y] = -angle_acc[Y];
    }

    // gyro[X] = l3g_gyro_rate[X];
    // gyro[Y] = l3g_gyro_rate[Y];
    // gyro[Z] = l3g_gyro_rate[Z];
/*####################################################################*/

    dt = (double)(micros() - timer)/ 1000000; // Calculate delta time

    if(flag_start == 0) //set the angle_gyro to angle_acc in start
    {
      flag_start = 1;
      angle_gyro[X] = angle_acc[X];
      angle_gyro[Y] = angle_acc[Y];

    }else
    {
      angle_gyro[X] += gyro[X]*dt;
      angle_gyro[Y] += gyro[Y]*dt;
    }

    angle_gyro[Z] += gyro[Z]*dt;

    // Calculate the angle using a Complimentary filter
    comp_angle_pitch = (0.995 * (comp_angle_pitch + gyro[Y]*dt)) + (0.005 * angle_acc[X]);
    comp_angle_roll  = (0.995 * (comp_angle_roll  + gyro[X]*dt)) + (0.005 * angle_acc[Y]);

    comp_angle_yaw   = angle_gyro[Z];

    timer = micros();



    // To calculate heading in degrees. 0 degree indicates North


    if(acc[Z]>0)
    {
      roll   = -comp_angle_roll*TO_RAD;
      pitch  = comp_angle_pitch*TO_RAD;

    }else
    {
      roll   = -comp_angle_roll*TO_RAD;
      pitch  = -comp_angle_pitch*TO_RAD;
    }
    // roll   = -comp_angle_pitch*TO_RAD;
    // pitch  = -comp_angle_roll*TO_RAD;

    Xhorizontal=mag_normal[X]*cos(pitch)+mag_normal[Y]*sin(roll)*sin(pitch)-mag_normal[Z]*cos(roll)*sin(pitch);
    Yhorizontal=mag_normal[Y]*cos(roll)+mag_normal[Z]*sin(roll);


    if(acc[Z]>0)
    {
      roll = -roll;
    }else
    {
      roll = -roll;
      pitch =-pitch;
    }

    heading = atan2(Yhorizontal, Xhorizontal);
    if(heading < 0)
      heading += 2 * M_PI;

    heading2 = atan2(mag_normal[Y], mag_normal[X]);
    heading2 = heading2;//+(15.22*TO_RAD);//Declination Estimated
    if(heading2 < 0)
      heading2 += 2 * M_PI;

    if(flag_start_compass == 0)
    {
      flag_start_compass = 1;
      angle_gyro[Z] = heading * RAD_TO_DEG;
      comp_angle_yaw = angle_gyro[Z];
    }
    // mpu_compute_angle_yaw = mpu_LPF_angle_yaw;

    mag_gyro_angle_yaw = (0.97 * (mag_gyro_angle_yaw + gyro[Z]*dt)) + (0.03 * heading * RAD_TO_DEG);
    if(mag_gyro_angle_yaw>0)mag_gyro_angle_yaw = mag_gyro_angle_yaw;
    else if(mag_gyro_angle_yaw <0)mag_gyro_angle_yaw = 360+mag_gyro_angle_yaw;
    if(mag_gyro_angle_yaw>360 || mag_gyro_angle_yaw< -360)
    {
      mag_gyro_angle_yaw=0;
    }
    yaw = mag_gyro_angle_yaw;

    serial_print();
}

void serial_print(void)
{
  if(serial_out_mode == NONE)
  {
      //none
  }else if(serial_out_mode == YPR)//YPR
  {
    /*############################ PROCESSING ################################*/
    Serial.print("Orientation: ");
    Serial.print(yaw);Serial.print(" ");
    Serial.print(pitch*RAD_TO_DEG);Serial.print(" ");
    Serial.print(roll*RAD_TO_DEG);
    Serial.println(" ");

  }else if(serial_out_mode == ACC)
  {
    Serial.print("ACC: ");
    Serial.print(mpu_acc_normal[X]);Serial.print("\t");
    Serial.print(mpu_acc_normal[Y]);Serial.print("\t");
    Serial.print(mpu_acc_normal[Z]);Serial.print("\t\t");
    Serial.print(angle_acc[X]); Serial.print("\t");
    Serial.print(angle_acc[Y]);
    Serial.println(" ");

  }else if(serial_out_mode == GYRO)
  {
    Serial.print("GYRO: ");
    Serial.print(gyro[X]); Serial.print("\t");
    Serial.print(gyro[Y]); Serial.print("\t");
    Serial.print(gyro[Z]); Serial.print("\t\t");

    Serial.print(angle_gyro[X]); Serial.print("\t");
    Serial.print(angle_gyro[Y]); Serial.print("\t");
    Serial.print(angle_gyro[Z]);
    Serial.println(" ");
  }else if(serial_out_mode == MAG)
  {
    Serial.print("MAG: ");
    Serial.print(mag[X]); Serial.print("\t");
    Serial.print(mag[Y]); Serial.print("\t");
    Serial.print(mag[Z]); Serial.print("\t\t");

    Serial.print(Xhorizontal); Serial.print("\t");
    Serial.print(Yhorizontal); Serial.print("\t\t");

    Serial.print("\theading_com:");
    Serial.print(heading * RAD_TO_DEG);
    Serial.print("\theading2:");
    Serial.print(heading2 * RAD_TO_DEG);
    Serial.println(" ");
  }else if(serial_out_mode == ALL)
  {
    Serial.print("ALL: ");
    Serial.print(angle_acc[X]);Serial.print("\t");
    Serial.print(angle_acc[Y]);Serial.print("\t\t");
    Serial.print(angle_gyro[X]); Serial.print("\t");
    Serial.print(angle_gyro[Y]); Serial.print("\t");
    Serial.print(angle_gyro[Z]);Serial.print("\t\t");
    Serial.print("heading:");
    Serial.print(heading * RAD_TO_DEG);Serial.print("\t");
    Serial.print("heading2:");
    Serial.print(heading2 * RAD_TO_DEG);Serial.print("\t\t");

    Serial.print(yaw);Serial.print("\t");
    Serial.print(pitch*RAD_TO_DEG);Serial.print("\t");
    Serial.print(roll*RAD_TO_DEG);
    Serial.println(" ");

  }

}

// void calibration(void){
//  int32_t a=0;
// //##########################################################################
// //###############################   ACC   ##################################
// //##########################################################################
//    for(a=0;a<10;a++){
//    int32_t i=0,offsetX=0,offsetY=0,offsetZ=0;
//    for(i=0;i<100;i++){
//      accel.getAcceleration(&rowAcc[X], &rowAcc[Y], &rowAcc[Z]);
//      offsetX +=rowAcc[X];
//      offsetY +=rowAcc[Y];
//      offsetZ +=rowAcc[Z];
//      }
//
//    Serial.print(offsetX/100); Serial.print("\t");
//    Serial.print(offsetY/100); Serial.print("\t");
//    Serial.print(offsetZ/100); Serial.println("\t");
//    }
//
void gyro_calibration(void){
  int i=0;
  int32_t totalGyroX=0, totalGyroY=0, totalGyroZ=0;
  int16_t row_gyro[3];
  int16_t sum_gyro[3];
  float gyro[3];
  int16_t row_accel;

  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  my_delay(3000);

  while(1)
  {
    mpu.getMotion6(&row_accel, &row_accel, &row_accel, &row_gyro[X], &row_gyro[Y], &row_gyro[Z]);
    // sum_gyro[X] += row_gyro[X];
    // sum_gyro[Y] += row_gyro[Y];
    // sum_gyro[Z] += row_gyro[Z];
    // Serial.print(row_gyro[X] ); Serial.print("\t"); // 0
    // Serial.print(row_gyro[Y] ); Serial.print("\t"); // 0
    // Serial.print(row_gyro[Z] ); Serial.print("\t"); // 0
    // Serial.print("\n");
    for(i=0;i<5000;i++)
    {
      wdt_reset();
      mpu.getMotion6(&row_accel, &row_accel, &row_accel, &row_gyro[X], &row_gyro[Y], &row_gyro[Z]);
      sum_gyro[X] += row_gyro[X];
      sum_gyro[Y] += row_gyro[Y];
      sum_gyro[Z] += row_gyro[Z];
    }
    // offset_gyro[X] = (sum_gyro[X]/5000)/4;
    // offset_gyro[Y] = (sum_gyro[Y]/5000)/4;
    // offset_gyro[Z] = (sum_gyro[Z]/5000)/4;
    offset_gyro[X] = (sum_gyro[X]/5000);
    offset_gyro[Y] = (sum_gyro[Y]/5000);
    offset_gyro[Z] = (sum_gyro[Z]/5000);

    Serial.print(offset_gyro[X]); Serial.print("\t"); // 0
    Serial.print(offset_gyro[Y]); Serial.print("\t"); // 0
    Serial.print(offset_gyro[Z]); Serial.print("\t"); // 0
    Serial.print("\n");

    gyro_set_offset();
    flag_gyro_calibrarion = 0;
    break;
  }

  // for(i=0;i<2000;i++)
  // {
  //   mpu.getMotion6(&row_accel, &row_accel, &row_accel, &gyro[X], &gyro[Y], &gyro[Z]);
  //   totalGyroX+=gyroX;
  //   totalGyroY+=gyroY;
  //   totalGyroZ+=gyroZ;
  // }
  // mpu.setXGyroOffset(-(totalGyroX/2000)/4);
  // mpu.setYGyroOffset(-(totalGyroY/2000)/4);
  // mpu.setZGyroOffset(-(totalGyroZ/2000)/4);
  // Serial.print(mpu.getXGyroOffset()); Serial.print("\t"); // 0
  // Serial.print(mpu.getYGyroOffset()); Serial.print("\t"); // 0
  // Serial.print(mpu.getZGyroOffset()); Serial.print("\t"); // 0
  // Serial.print("\n");
}
void gyro_get_offset(void)
{
uint8_t offset_gyroh,offset_gyrol;
offset_gyroh = EEPROM.read(12);delay(5);
offset_gyrol = EEPROM.read(13);delay(5);
offset_gyro[X] = ( ((int16_t)offset_gyroh) <<8) | (uint8_t)(offset_gyrol);//H and L

offset_gyroh = EEPROM.read(14);delay(5);
offset_gyrol = EEPROM.read(15);delay(5);
offset_gyro[Y] = ( ((int16_t)offset_gyroh) <<8) | (uint8_t)(offset_gyrol);//H and L

offset_gyroh = EEPROM.read(16);delay(5);
offset_gyrol = EEPROM.read(17);delay(5);
offset_gyro[Z] = ( ((int16_t)offset_gyroh) <<8) | (uint8_t)(offset_gyrol);//H and L

Serial.print("Get gyro offset:");
Serial.print(" offsetX: ");Serial.print(offset_gyro[X]);
Serial.print(" offsetY: ");Serial.print(offset_gyro[Y]);
Serial.print(" offsetZ: ");Serial.println(offset_gyro[Z]);

}

void gyro_set_offset(void)
{
  Serial.print("Set gyro offset:");
  Serial.print(" offsetX: ");Serial.print(offset_gyro[X]);
  Serial.print(" offsetY: ");Serial.print(offset_gyro[Y]);
  Serial.print(" offsetZ: ");Serial.println(offset_gyro[Z]);

  EEPROM.write(12,(uint8_t)(offset_gyro[X]>>8));  delay(5);//H
  EEPROM.write(13,(uint8_t)(offset_gyro[X]&0XFF));delay(5);//L

  EEPROM.write(14,(uint8_t)(offset_gyro[Y]>>8));  delay(5); //H
  EEPROM.write(15,(uint8_t)(offset_gyro[Y]&0XFF));delay(5);//L

  EEPROM.write(16,(uint8_t)(offset_gyro[Z]>>8));  delay(5);//H
  EEPROM.write(17,(uint8_t)(offset_gyro[Z]&0XFF));delay(5);//L
  return ;
}

void acc_calibration(void){
    uint32_t i=0;
    int32_t totalAccelX=0, totalAccelY=0, totalAccelZ=0;
    int16_t row_accel[3];
    float accel[3]={0,0,0};
    int32_t sum_accel[3]={0,0,0};
    int32_t ave_accel[3]={0,0,0};
    int16_t row_gyro;

    offset_acc[X]=0;
    offset_acc[Y]=0;
    offset_acc[Z]=0;

    while(1)
    {
      wdt_reset();
      mpu.getMotion6(&row_accel[X], &row_accel[Y], &row_accel[Z], &row_gyro, &row_gyro, &row_gyro);

      Serial.print(row_accel[X]); Serial.print("\t"); // -76
      Serial.print(row_accel[Y]); Serial.print("\t"); // -2359
      Serial.print(row_accel[Z]); Serial.print("\t"); // 1688

      Serial.print(offset_acc[X]); Serial.print("\t"); // 1688
      Serial.print(offset_acc[Y]); Serial.print("\t"); // 1688
      Serial.print(offset_acc[Z]); Serial.print("\t"); // 1688
      Serial.print("\n");

      if(offset_acc[X] !=0 && offset_acc[Y] !=0 && offset_acc[Z]!=0)
      {
          acc_set_offset();
          flag_acc_calibrarion=0;
          break;
      }

      if(row_accel[X]>2000 && ave_accel[X]== 0)
      {
        Serial.println("#Nose up");
        my_delay(3000);
        for(i=0;i<5000 ; i++)
        {
          wdt_reset();
          mpu.getMotion6(&row_accel[X], &row_accel[Y], &row_accel[Z], &row_gyro, &row_gyro, &row_gyro);
          sum_accel[X] += row_accel[X];
        }
        ave_accel[X] = sum_accel[X]/5000;
        // offset_acc[X] = (2048-ave_accel[X])/8;
        offset_acc[X] = (2048-ave_accel[X])+102;

      }else if(row_accel[Y]>2000 && ave_accel[Y] == 0)
      {
        Serial.println("#Side R");
        my_delay(3000);
        for(i=0;i<5000 ; i++)
        {
          wdt_reset();
          mpu.getMotion6(&row_accel[X], &row_accel[Y], &row_accel[Z], &row_gyro, &row_gyro, &row_gyro);
          sum_accel[Y] += row_accel[Y];
        }
        ave_accel[Y] = sum_accel[Y]/5000;
        // offset_acc[Y] = (2048-ave_accel[Y])/8;
        offset_acc[Y] = (2048-ave_accel[Y])-81;

      }else if(row_accel[Z]>2000 && ave_accel[Z] == 0)
      {
        Serial.println("#level");
        my_delay(3000);
        for(i=0;i<5000 ; i++)
        {
          wdt_reset();
          mpu.getMotion6(&row_accel[X], &row_accel[Y], &row_accel[Z], &row_gyro, &row_gyro, &row_gyro);
          sum_accel[Z] += row_accel[Z];
        }
        ave_accel[Z] = sum_accel[Z]/5000;
        // offset_acc[Z] = (2048-ave_accel[Z])/8;
        offset_acc[Z] = (2048-ave_accel[Z]);
      }


      if(Serial.available())
        {
          char c;
          c = Serial.read();
          if(c=='#')
          {
            flag_acc_calibrarion=0;
            break;
          }
        }
    }
}
void acc_get_offset(void)
{
uint8_t offset_acch,offset_accl;
offset_acch = EEPROM.read(6);delay(5);
offset_accl = EEPROM.read(7);delay(5);
offset_acc[X] = ( ((int16_t)offset_acch) <<8) | (uint8_t)(offset_accl);//H and L

offset_acch = EEPROM.read(8);delay(5);
offset_accl = EEPROM.read(9);delay(5);
offset_acc[Y] = ( ((int16_t)offset_acch) <<8) | (uint8_t)(offset_accl);//H and L

offset_acch = EEPROM.read(10);delay(5);
offset_accl = EEPROM.read(11);delay(5);
offset_acc[Z] = ( ((int16_t)offset_acch) <<8) | (uint8_t)(offset_accl);//H and L

Serial.print("Get acc offset:");
Serial.print(" offsetX: ");Serial.print(offset_acc[X]);
Serial.print(" offsetY: ");Serial.print(offset_acc[Y]);
Serial.print(" offsetZ: ");Serial.println(offset_acc[Z]);

}

void acc_set_offset(void)
{
  Serial.print("Set acc offset:");
  Serial.print(" offsetX: ");Serial.print(offset_acc[X]);
  Serial.print(" offsetY: ");Serial.print(offset_acc[Y]);
  Serial.print(" offsetZ: ");Serial.println(offset_acc[Z]);

  EEPROM.write(6,(uint8_t)(offset_acc[X]>>8));  delay(5);//H
  EEPROM.write(7,(uint8_t)(offset_acc[X]&0XFF));delay(5);//L

  EEPROM.write(8,(uint8_t)(offset_acc[Y]>>8));  delay(5); //H
  EEPROM.write(9,(uint8_t)(offset_acc[Y]&0XFF));delay(5);//L

  EEPROM.write(10,(uint8_t)(offset_acc[Z]>>8));  delay(5);//H
  EEPROM.write(11,(uint8_t)(offset_acc[Z]&0XFF));delay(5);//L
  return ;
}

void acc_manual_offset(int16_t offsetX,int16_t offsetY,int16_t offsetZ)
{
  acc_get_offset();
  offset_acc[X]-=offsetX;
  offset_acc[Y]-=offsetY;
  offset_acc[Z]-=offsetZ;
  acc_set_offset();
}
void compass_calibration(void)
{
  // ##########################################################################
  // ###############################   #COMPASS  ##############################
  // ##########################################################################
  int16_t maxX=0,minX=0,aveX=0;
  int16_t maxY=0,minY=0,aveY=0;
  int16_t maxZ=0,minZ=0,aveZ=0;
       //calibration
  while(1){
   wdt_reset();
   compass.getHeading(&mag[X], &mag[Y], &mag[Z]);
   if(mag[X]>maxX)maxX=mag[X];
   if(mag[X]<minX)minX=mag[X];

   if(mag[Y]>maxY)maxY=mag[Y];
   if(mag[Y]<minY)minY=mag[Y];

   if(mag[Z]>maxZ)maxZ=mag[Z];
   if(mag[Z]<minZ)minZ=mag[Z];

   Serial.print(mag[X]); Serial.print("\t");
   Serial.print(mag[Y]); Serial.print("\t");
   Serial.print(mag[Z]); Serial.print("\t\t");
   Serial.print(maxX);Serial.print("\t");
   Serial.print(minX);Serial.print("\t");
   aveX = (maxX+minX)/2;
   Serial.print(aveX);Serial.print("\t\t");

   Serial.print(maxY);Serial.print("\t");
   Serial.print(minY);Serial.print("\t");
   aveY = (maxY+minY)/2;
   Serial.print(aveY);Serial.print("\t\t");

   Serial.print(maxZ);Serial.print("\t");
   Serial.print(minZ);Serial.print("\t");
   aveZ = (maxZ+minZ)/2;
   Serial.print(aveZ);Serial.println("\t\t");

   // if(flag_mag_calibrarion == 0) break;
   if(Serial.available())
     {
       char c;
       c = Serial.read();
       if(c=='#')
       {
        flag_mag_calibrarion = 0;
        break;
       }
     }
   }

   offset_mag[X] = aveX;
   offset_mag[Y] = aveY;
   offset_mag[Z] = aveZ;

   compass_set_offset();
   flag_mag_calibrarion = 0;

  // ##########################################################################
  // ##########################################################################

}

void compass_get_offset(void)
{
uint8_t offset_magh,offset_magl;
offset_magh = EEPROM.read(0);delay(5);
offset_magl = EEPROM.read(1);delay(5);
offset_mag[X] = ( ((int16_t)offset_magh) <<8) | (uint8_t)(offset_magl);//H and L

offset_magh = EEPROM.read(2);delay(5);
offset_magl = EEPROM.read(3);delay(5);
offset_mag[Y] = ( ((int16_t)offset_magh) <<8) | (uint8_t)(offset_magl);//H and L

offset_magh = EEPROM.read(4);delay(5);
offset_magl = EEPROM.read(5);delay(5);
offset_mag[Z] = ( ((int16_t)offset_magh) <<8) | (uint8_t)(offset_magl);//H and L

Serial.print("Get compass offset:");
Serial.print(" offsetX: ");Serial.print(offset_mag[X]);
Serial.print(" offsetY: ");Serial.print(offset_mag[Y]);
Serial.print(" offsetZ: ");Serial.println(offset_mag[Z]);

}

void compass_set_offset(void)
{

  Serial.print("Set compass offset:");
  Serial.print(" offsetX: ");Serial.print(offset_mag[X]);
  Serial.print(" offsetY: ");Serial.print(offset_mag[Y]);
  Serial.print(" offsetZ: ");Serial.println(offset_mag[Z]);

  EEPROM.write(0,(uint8_t)(offset_mag[X]>>8));  delay(5);//H
  EEPROM.write(1,(uint8_t)(offset_mag[X]&0XFF));delay(5);//L

  EEPROM.write(2,(uint8_t)(offset_mag[Y]>>8));  delay(5); //H
  EEPROM.write(3,(uint8_t)(offset_mag[Y]&0XFF));delay(5);//L

  EEPROM.write(4,(uint8_t)(offset_mag[Z]>>8));  delay(5);//H
  EEPROM.write(5,(uint8_t)(offset_mag[Z]&0XFF));delay(5);//L
  return ;
}


void my_delay(uint16_t time)
{
  uint16_t i = 0;
  time = time/100;
  for(i=0 ; i<time ; i++)
  {
    wdt_reset();
    delay(100);
  }
  return;
}
