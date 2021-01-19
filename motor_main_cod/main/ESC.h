#ifndef _ESC_H_
#define _ESC_H_
#include <ServoTimer2.h>

#define ESC_MAX 2000 //us
#define ESC_MIN 1000 //us

ServoTimer2 ESC1;
ServoTimer2 ESC2;

u32 temp_ESC1_with = 0;
u32 temp_ESC2_with = 0;

s8 ESC_init(void);
s8 ESC1_write(int arg);
s8 ESC2_write(int arg);
s8 ESC_print(void);

s8 ESC_init(void)
{
  ESC1.attach(5);
  ESC2.attach(6);
}

s8 ESC1_write(int arg) //percent
{
  if(arg<0) arg = 0;
  else if(arg>100) arg = 100;

  temp_ESC1_with = ( ((float)arg/100.0)*(ESC_MAX-ESC_MIN) )+ESC_MIN;

  ESC1.write(temp_ESC1_with);
}

s8 ESC2_write(int arg)
{
  if(arg<0) arg = 0;
  else if(arg>100) arg = 100;

  temp_ESC2_with = ( ((float)arg/100.0)*(ESC_MAX-ESC_MIN) )+ESC_MIN;
  ESC2.write(temp_ESC2_with);
}

s8 ESC_print(void)
{
  Serial.print("  ESC1:");
  Serial.print(temp_ESC1_with);
  Serial.print("  ESC2:");
  Serial.print(temp_ESC2_with);
}

#endif
