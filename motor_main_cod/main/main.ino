#include "my_type.h"
#include "my_step.h"
#include "GPIO.h"
#include "RC.h"
#include "IMU.h"
#include "RPM.h"
#include "ESC.h"
#include "control.h"
#include "RPM_control.h"


my_PID PID1;

#include "serial_command.h"


u32 last_time ;
u32 dt_main=0 ;
u32 last_time_main ;
void test_pid(void);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(250000);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("Serial ready>>>");

  GPIO_init();
  RC_init();
  IMU_init();
  step_init();
  RPM_init();
  ESC_init();
  RPM_control_init();

  PID1.PID_set_param(0.1,0,0);



  // step_enable();
}

void loop()
{

  dt_main = (double)(micros() - last_time_main) ; // Calculate delta time
  last_time_main= micros();
  // step_callback();
  IMU_getdata();
  IMU_consubtion();
  RPM_callback();
  RC_callback();
  mot_callback();

  // float err,out;
  // err = sp-s_IMU1.pitch;
  // out = PID1.PID_calculate(err);
  // step_run2(out);

  // Serial.print(s_IMU1.pitch);Serial.print("  SP:");
  // Serial.print(sp);Serial.print("  ERR:");
  // Serial.print(err);Serial.print("  OUT:");
  // Serial.print(out);Serial.println("  ");
  // // out = PID1.PID_calculate(err);


  if(millis()-last_time > 100)
  {
    last_time = millis();
    // Serial.print(dt_main);
    // Serial.print("\r\n");

    // Serial.println(PID1.PID_calculate(1));
    // IMU_print();

    step_print();
    RC_print();
    ESC_print();
    mot_print();
    RPM_print();
    // Serial.print(dt_main);
    // Serial.print("\r\n");
    Serial.print("\r\n");
  }



  // put your main code here, to run repeatedly
  // RC_print();

}



//
