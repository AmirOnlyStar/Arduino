#include "my_SSD1309.h"
// #include <Arduino.h>
// #include<SPI.h>

/***************************MPU*****************************/
#include <Wire.h>
#include "MPU6050.h"

MPU6050 mpu;
int16_t mpu_acc[3];
int16_t mpu_gyro[3];

float mpu_gyro_rate[3];

float mpu_acc_normal[3];
float mpu_angle_acc[3];
float mpu_angle_gyro[3];

float mpu_compute_angle_pitch;
float mpu_compute_angle_roll;
float mpu_compute_angle_yaw;
double dt =0;
uint32_t timer=0;
bool flag_start = 0;

#define X 0
#define Y 1
#define Z 2
#define PI 3.14
#define TO_DEG 180.0/PI
#define TO_RAD PI/180.0
/***********************************************************/


// extern unsigned char SSD1309_buffer[buffer_size];

void setup()
// int main()
{
  init();
  Serial.begin(1000000);
  Serial.println("Serial Ready...");
  setup_GPIOs();
  OLED_init();
  pinMode(A1,INPUT_PULLUP);
  pinMode(A2,INPUT_PULLUP);

  digitalWrite(A4,HIGH);
  digitalWrite(A5,HIGH);

  Wire.setClock(400000);

  mpu.initialize();
  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  Serial.print("Rate:");Serial.print(mpu.getRate());
  Serial.print("GyroRate:");Serial.print(mpu.getFullScaleGyroRange());
  Serial.print("AccRate:");Serial.print(mpu.getFullScaleAccelRange());
  mpu.setXGyroOffset(204);
  mpu.setYGyroOffset(-43);
  mpu.setZGyroOffset(38);
  mpu.setXAccelOffset(-2884);
  mpu.setYAccelOffset(-1279);
  mpu.setZAccelOffset(979);

}

void loop()
// while(1)
{

  // int8_t pitch=0,roll=0;

  // static uint8_t counter = 0;


  // if(digitalRead(A2) == 0)
  // {
  //   counter++;
  //   OLED_clear_buffer();
  //   // delay(500);
  // }
  // if(digitalRead(A1) == 0)
  // {
  //   counter--;
  //   OLED_clear_buffer();
  //   // delay(500);
  // }
  //
  // testdrawcircle(counter,counter);
  // OLED_referesh();
  // delay(500);

  mpu.getMotion6(&mpu_acc[X], &mpu_acc[Y], &mpu_acc[Z], &mpu_gyro[X], &mpu_gyro[Y], &mpu_gyro[Z]);
      mpu_gyro_rate[X] = -mpu_gyro[X]/131.0;
      mpu_gyro_rate[Y] = -mpu_gyro[Y]/131.0;
      mpu_gyro_rate[Z] = mpu_gyro[Z]/131.0;

      float mpu_acc_sum;
      mpu_acc_sum = sqrt(pow(mpu_acc[X],2)+pow(mpu_acc[Y],2)+pow(mpu_acc[Z],2));
      mpu_acc_normal[X] = mpu_acc[X]/mpu_acc_sum;
      mpu_acc_normal[Y] = mpu_acc[Y]/mpu_acc_sum;
      mpu_acc_normal[Z] = mpu_acc[Z]/mpu_acc_sum;

      if(mpu_acc_normal[Z]>=0 )
      {
        mpu_angle_acc[X] =  asin(mpu_acc_normal[X])*(180.0/3.14);
      }else if(mpu_acc_normal[Z]<=0 && mpu_acc_normal[X]>=0)
      {
        mpu_angle_acc[X] =  180-asin(mpu_acc_normal[X])*(180.0/3.14);
      }else if(mpu_acc_normal[Z]<=0 && mpu_acc_normal[X]<=0)
      {
        mpu_angle_acc[X] =  -180-asin(mpu_acc_normal[X])*(180.0/3.14);
      }

      mpu_angle_acc[Y] =  -asin(mpu_acc_normal[Y])*(180.0/3.14);


      dt = (double)(micros() - timer) / 1000000; // Calculate delta time

      if(flag_start == 0)
      {
        flag_start = 1;
        mpu_angle_gyro[X] = mpu_angle_acc[X];
        mpu_angle_gyro[Y] = mpu_angle_acc[Y];
      }else
      {
        mpu_angle_gyro[X] += mpu_gyro_rate[X]*dt;
        mpu_angle_gyro[Y] += mpu_gyro_rate[Y]*dt;
      }

      mpu_angle_gyro[Z] += mpu_gyro_rate[Z]*dt;

      mpu_compute_angle_pitch = (0.99 * (mpu_compute_angle_pitch + mpu_gyro_rate[Y]*dt)) + (0.01 * mpu_angle_acc[X]);
      mpu_compute_angle_roll  = (0.99 * (mpu_compute_angle_roll  + mpu_gyro_rate[X]*dt)) + (0.01 * mpu_angle_acc[Y]); // Calculate the angle using a Complimentary filter
      // compute_angle_yaw   = 0.98 * (compute_angle_yaw   + angle_gyro[Z]) + 0.02 * heading;
      mpu_compute_angle_yaw   = ( 0.2 * mpu_angle_gyro[Z] )+ (0.8 * mpu_compute_angle_yaw ); //LOW pass filter
      timer = micros();

      Serial.print(mpu_acc_normal[X]); Serial.print("\t");
      Serial.print(mpu_acc_normal[Y]); Serial.print("\t");
      Serial.print(mpu_acc_normal[Z]); Serial.print("\t\t");

      Serial.print(mpu_angle_acc[X]); Serial.print("\t");
      Serial.print(mpu_angle_acc[Y]); Serial.print("\t\t");


      // Serial.print(mpu_angle_gyro[X]); Serial.print("\t");
      // Serial.print(mpu_angle_gyro[Y]); Serial.print("\t");
      // Serial.print(mpu_angle_gyro[Z]); Serial.print("\t\t");

      Serial.print(mpu_compute_angle_pitch);Serial.print(" ");
      Serial.print(mpu_compute_angle_roll);Serial.print(" ");
      Serial.print(mpu_compute_angle_yaw);Serial.print(" ");
      Serial.println(" ");
      testdrawcircle(mpu_compute_angle_pitch,-mpu_compute_angle_roll);
      OLED_referesh();
      OLED_clear_buffer();

    }//end loop
// }//end main


void testdrawcircle(int8_t pitch , int8_t roll) {
  uint8_t centerH = DISP_HIGHT /2 ;
  uint8_t centerW = DISP_WITH / 2 ;

  // int32_t pitch1,roll1;
  //
  // pitch1 = pitch*1;
  // roll1  = roll*1;
  //
  // static int32_t pitch2 = pitch1 ,roll2 = roll1;
  //
  // if(pitch1>pitch2+1)pitch2++;
  // else if(pitch1<pitch2-1)pitch2--;
  //
  // if(roll1>roll2+1)roll2++;
  // else if(roll1<roll2-1)roll2--;

  // OLED_clear_buffer();
  // OLED_clear_screen();


  OLED_print_char(0,10,92+32);//^
  OLED_print_char(0,30,93+32);//>
  OLED_print_int(8,10,(int)abs(pitch));
  OLED_print_int(8,30,(int)abs(roll));



  if(pitch > 20) pitch=20;
  else if(pitch < -20) pitch= -20;

  if(roll > 50) roll=50;
  else if(roll < -50) roll= -50;

  Draw_V_Line(centerW-29,centerH,centerH+pitch,1);
  Draw_V_Line(centerW-30,centerH,centerH+pitch,1);
  Draw_V_Line(centerW+29,centerH,centerH+pitch,1);
  Draw_V_Line(centerW+30,centerH,centerH+pitch,1);


  Draw_H_Line(centerW,centerW+roll,centerH+29,1);
  Draw_H_Line(centerW,centerW+roll,centerH+30,1);
  Draw_H_Line(centerW,centerW+roll,centerH-29,1);
  Draw_H_Line(centerW,centerW+roll,centerH-30,1);

  Draw_Circle(centerW + roll, centerH + pitch, 10, YES, ON);

  Draw_Circle(centerW , centerH , 15, NO, ON);


  Draw_V_Line(centerW,centerH-28,centerH+28,ON);
  Draw_H_Line(centerW-28,centerW+28,centerH,ON);

  Draw_Circle(centerW , centerH , 28, NO, ON);





  // delay(2000);
}
