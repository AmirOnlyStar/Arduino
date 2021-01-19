#ifndef _RPMCONTROL_H_
#define _RPMCONTROL_H_

#define RPM_MAX  8000

my_PID mot1_PID;

void RPM_control_init(void);
void mot_callback(void);
void mot1_setSpeed(float speed);
void mot_print(void);

float mot1_SP;
float temp1=0;
float temp2=0;
float temp3=0;
float RPM_percent=0;

void RPM_control_init(void)
{
mot1_PID.PID_set_param(0.1,0.1,0);
}

void mot1_setSpeed(float speed) //RPM
{
 mot1_SP = (speed*100)/RPM_MAX; //set point
}
void mot_callback(void)
{
//direct motor drive
// ESC1_write(RC1);
// ESC2_write(RC2);

  mot1_SP = RC1;

  if(RC1==0)mot1_PID.PID_reset();

  RPM_percent = RPM1*100/RPM_MAX;

  // temp1 = (mot1_SP*0.5)+(RPM_percent*0.5);
  temp1 = mot1_SP-RPM_percent;
  temp2 = mot1_PID.PID_calculate(temp1);
  // if(temp2>100) temp2 = 100;
  // if(temp2<0) temp2 = 0;
  ESC1_write(temp2);
  temp3 = (RC2-50)*2;
  if(abs(temp3)>5)
  {
    step_run2(temp3);
  }else
  {
    step_stop();
  }

  // ESC1_write(RC1);
}

void mot_print(void)
{
  Serial.print("  MOT1_er:");
  Serial.print(temp1);
  Serial.print("  MOT1_PID_OUT:");
  Serial.print(temp2);
  Serial.print("  RPM1_per:");
  Serial.print(RPM_percent);
}

#endif
