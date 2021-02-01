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
/***********************************************************/
int32_t pitch=0,roll=0;


void setup()
{
  Serial.begin(1000000);
  Serial.println("Serial Ready...");
  setup_GPIOs();
  OLED_init();

  // Wire.setClock(400000);
  //
  // mpu.initialize();
  // // verify connection
  // Serial.println("Testing device connections...");
  // Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  // Serial.print("Rate:");Serial.print(mpu.getRate());
  // Serial.print("GyroRate:");Serial.print(mpu.getFullScaleGyroRange());
  // Serial.print("AccRate:");Serial.print(mpu.getFullScaleAccelRange());
  // mpu.setXGyroOffset(204);
  // mpu.setYGyroOffset(-43);
  // mpu.setZGyroOffset(38);
  // mpu.setXAccelOffset(-2884);
  // mpu.setYAccelOffset(-1279);
  // mpu.setZAccelOffset(979);

}
void loop()
{
testdrawcircle(10,15);
  // OLED_clear_buffer();
  //
  // Draw_Circle(63, 31, 10, NO, ON);
  // Draw_Circle(63, 31, 30, NO, ON);
  // Draw_Line(63, 0, 63, 63, ON);
  // Draw_Line(33, 31, 93, 31, ON);
  // delay(500);


    //      OLED_fill(0x00);
    //      OLED_print_string(36, 1, "MicroArena");
    //      OLED_print_string(12, 2, "fb.com/MicroArena");
    //      for(p = 0; p < 250; p++)
    //      {
    //          f += 0.1;
    //          i += 1;
    //
    //          OLED_print_float(52, 5, f, 1);
    //          OLED_print_int(52, 6, i);
    //          OLED_print_chr(52, 7, p);

    // mpu.getMotion6(&mpu_acc[X], &mpu_acc[Y], &mpu_acc[Z], &mpu_gyro[X], &mpu_gyro[Y], &mpu_gyro[Z]);
    // mpu_gyro_rate[X] = -mpu_gyro[X]/131.0;
    // mpu_gyro_rate[Y] = -mpu_gyro[Y]/131.0;
    // mpu_gyro_rate[Z] = mpu_gyro[Z]/131.0;
    //
    // float mpu_acc_sum;
    // mpu_acc_sum = sqrt(pow(mpu_acc[X],2)+pow(mpu_acc[Y],2)+pow(mpu_acc[Z],2));
    // mpu_acc_normal[X] = mpu_acc[X]/mpu_acc_sum;
    // mpu_acc_normal[Y] = mpu_acc[Y]/mpu_acc_sum;
    // mpu_acc_normal[Z] = mpu_acc[Z]/mpu_acc_sum;
    //
    // if(mpu_acc_normal[Z]>=0 )
    // {
    //   mpu_angle_acc[X] =  asin(mpu_acc_normal[X])*(180.0/3.14);
    // }else if(mpu_acc_normal[Z]<=0 && mpu_acc_normal[X]>=0)
    // {
    //   mpu_angle_acc[X] =  180-asin(mpu_acc_normal[X])*(180.0/3.14);
    // }else if(mpu_acc_normal[Z]<=0 && mpu_acc_normal[X]<=0)
    // {
    //   mpu_angle_acc[X] =  -180-asin(mpu_acc_normal[X])*(180.0/3.14);
    // }
    //
    // mpu_angle_acc[Y] =  -asin(mpu_acc_normal[Y])*(180.0/3.14);
    //
    // dt = (double)(micros() - timer) / 1000000; // Calculate delta time
    //
    // if(flag_start == 0)
    // {
    //   flag_start = 1;
    //   mpu_angle_gyro[X] = mpu_angle_acc[X];
    //   mpu_angle_gyro[Y] = mpu_angle_acc[Y];
    // }else
    // {
    //   mpu_angle_gyro[X] += mpu_gyro_rate[X]*dt;
    //   mpu_angle_gyro[Y] += mpu_gyro_rate[Y]*dt;
    // }
    //
    // mpu_angle_gyro[Z] += mpu_gyro_rate[Z]*dt;
    //
    // mpu_compute_angle_pitch = (0.9 * (mpu_compute_angle_pitch + mpu_gyro_rate[Y]*dt)) + (0.1 * mpu_angle_acc[X]);
    // mpu_compute_angle_roll  = (0.9 * (mpu_compute_angle_roll  + mpu_gyro_rate[X]*dt)) + (0.1 * mpu_angle_acc[Y]); // Calculate the angle using a Complimentary filter
    // // compute_angle_yaw   = 0.98 * (compute_angle_yaw   + angle_gyro[Z]) + 0.02 * heading;
    // mpu_compute_angle_yaw   = ( 0.2 * mpu_angle_gyro[Z] )+ (0.8 * mpu_compute_angle_yaw ); //LOW pass filter
    // timer = micros();
    //
    //
    // Serial.print(mpu_acc_normal[X]); Serial.print("\t");
    // Serial.print(mpu_acc_normal[Y]); Serial.print("\t");
    // Serial.print(mpu_acc_normal[Z]); Serial.print("\t\t");
    //
    // Serial.print(mpu_angle_acc[X]); Serial.print("\t");
    // Serial.print(mpu_angle_acc[Y]); Serial.print("\t\t");
    // Serial.print(mpu_acc_normal[Z]); Serial.println("\t\t");
    //
    // // Serial.print(mpu_angle_gyro[X]); Serial.print("\t");
    // // Serial.print(mpu_angle_gyro[Y]); Serial.print("\t");
    // // Serial.print(mpu_angle_gyro[Z]); Serial.print("\t\t");
    // //
    // // Serial.print(mpu_compute_angle_pitch);Serial.print(" ");
    // // Serial.print(mpu_compute_angle_roll);Serial.print(" ");
    // // Serial.print(mpu_compute_angle_yaw);Serial.println(" ");
    //
    // testdrawcircle(mpu_compute_angle_pitch,-mpu_compute_angle_roll);


}//end loop
// }


void testdrawcircle(float pitch , float roll) {
  int32_t centerH = DISP_HIGHT /2 ;
  int32_t centerW = DISP_WITH / 2 ;

  int32_t pitch1,roll1;

  pitch1 = pitch*1;
  roll1  = roll*1;

  static int32_t pitch2 = pitch1 ,roll2 = roll1;

  if(pitch1>pitch2+1)pitch2++;
  else if(pitch1<pitch2-1)pitch2--;

  if(roll1>roll2+1)roll2++;
  else if(roll1<roll2-1)roll2--;

  OLED_clear_buffer();
  OLED_clear_screen();


  // display.fillCircle(centerW + roll1 , centerH + pitch1 , 10, SSD1306_WHITE);
  Draw_Circle(centerW + roll1, centerH + pitch1, 10, NO, ON);

  // display.drawCircle(centerW, centerH, 15, SSD1306_WHITE);
  Draw_Circle(centerW , centerH , 15, NO, ON);

  // display.drawLine(centerW-31, centerH, centerW+31, centerH, SSD1306_WHITE);
  // display.drawLine(centerW, centerH-31, centerW, centerH+31, SSD1306_WHITE);
  Draw_Line(centerW-31, centerH, centerW+31, centerH, ON);
  Draw_Line(centerW, centerH-31, centerW, centerH+31, ON);


  // display.drawCircle(centerW, centerH, 30, SSD1306_WHITE);
  Draw_Circle(centerW , centerH , 30, NO, ON);


  // display.setTextSize(2);      // Normal 1:1 pixel scale
  // display.setTextColor(SSD1306_WHITE); // Draw white text
  // char str[10]="\0";


  // display.setCursor(5, 5);
  // display.print((int)abs(pitch));
  // display.drawLine(0, centerH, 0, centerH+pitch, SSD1306_WHITE);
  // display.drawLine(1, centerH, 1, centerH+pitch, SSD1306_WHITE);
  //
  // display.setCursor(97, 43);
  // display.print((int)abs(roll));
  // display.drawLine(centerW, 62, centerW+roll, 62, SSD1306_WHITE);
  // display.drawLine(centerW, 63, centerW+roll, 63, SSD1306_WHITE);


  // delay(2000);
}
