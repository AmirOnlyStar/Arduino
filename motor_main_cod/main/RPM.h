#ifndef _RCMETER_H_
  #define _RCMETER_H_

float RPM1,RPM2;
float temp_RPM1,temp_RPM2;
bool flag_RPM1,flag_RPM2;
u32 last_time_RPM1,last_time_RPM2;

s8 RPM_init(void);
s8 RPM_callback(void);
void RPM_print(void);
void ISR_RPM1(void);
void ISR_RPM2(void);




s8 RPM_init(void)
{
  attachInterrupt(digitalPinToInterrupt(2), ISR_RPM1, CHANGE );
  attachInterrupt(digitalPinToInterrupt(3), ISR_RPM2, CHANGE );
}
s8 RPM_callback(void)
{
  if(RPM1<100) RPM1=0;
  if(RPM2<100) RPM2=0;
}
void ISR_RPM1(void)
{
  // Serial.println("OK111");
  if(digitalRead(2) == 0 && flag_RPM1 == 0)
  {
    temp_RPM1 = (1.0/(micros()-last_time_RPM1))*1000000*60;
    if(temp_RPM1 != 0 && temp_RPM1 <10000) //avoid fast changes
    {
      RPM1 = temp_RPM1;
    }

    flag_RPM1 = 1;
    last_time_RPM1 = micros();
  }else if(digitalRead(2) == 1 && flag_RPM1 == 1)
  {
    flag_RPM1 = 0;
  }

}
void ISR_RPM2(void)
{
  // Serial.println("OK222");
  if(digitalRead(3) == 0 && flag_RPM2 == 0)
  {
    temp_RPM2 = (1.0/(micros()-last_time_RPM2))*1000000*60;
    if(temp_RPM2 != 0 && temp_RPM2 <10000)//avoid fast changes
    {
      RPM2 = temp_RPM2;
    }
    flag_RPM2 = 1;
    last_time_RPM2 = micros();
  }else if(digitalRead(3) == 1 && flag_RPM2 == 1)
  {
    flag_RPM2 = 0;
  }

}

void RPM_print(void)
{
  Serial.print("  RPM1: ");
  Serial.print(RPM1);
  Serial.print("  RPM2: ");
  Serial.print(RPM2);
}

#endif
