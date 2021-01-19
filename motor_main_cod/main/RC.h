#ifndef _RC_H_
#define _RC_H_
#include <avr/interrupt.h>

#define RC_MAX 1900 //us
#define RC_MIN 1100 //us

s8 RC_init(void);
s8 RC_callback(void);
void RC_print(void);

bool flag_rc1 = 0;
bool flag_rc2 = 0;
u32 last_time_RC1;
u32 last_time_RC2;
u32 channel_1_width;
u32 channel_2_width;
u8 RC1,RC2;

//######################################## RC_INIT #############################
s8 RC_init(void)
{
  // PCICR |= 0b00000001;    // turn on port b
  PCICR |= 0b00000010;    // turn on port c
  // PCICR |= 0b00000100;    // turn on port d
  // PCICR |= 0b00000111;    // turn on all ports

  // PCMSK0 |= 0b00000001;    // turn on pin PB0, which is PCINT0, physical pin 14
  PCMSK1 |= 0b00001100;    // turn on pin PC4, which is PCINT12, physical pin 27
  // PCMSK2 |= 0b10000001;    // turn on pins PD0 & PD7, PCINT16 & PCINT23
}

//######################################## RC_CALLBACK #########################
s8 RC_callback(void)
{
  if((micros()-last_time_RC1)>100000) channel_1_width=0;
  if((micros()-last_time_RC2)>100000) channel_2_width=0;
}
//######################################## RC_PRINT ############################
void RC_print(void)
{
  Serial.print("  CH1:");
  // Serial.print(channel_1_width);
  // Serial.print("  ");
  Serial.print(RC1);
  Serial.print("  CH2:");
  // Serial.print(channel_2_width);
  // Serial.print("  ");
  Serial.print(RC2);
}
//######################################## RC_ISR #############################
// ISR(PCINT0_vect){}    // Port B, PCINT0 - PCINT7
ISR(PCINT1_vect)
{
  if(digitalRead(PIN_RC1) == 1 && flag_rc1 == 0)
  {
    flag_rc1 = 1;
    last_time_RC1 = micros();
  }else if(digitalRead(PIN_RC1) == 0 && flag_rc1 == 1)
  {
    flag_rc1 =0;
    channel_1_width = micros()-last_time_RC1;
    RC1 = float(channel_1_width-RC_MIN)/(RC_MAX-RC_MIN)*100;
    if(RC1<0) RC1=0;
    if(RC1>100) RC1=100;
  }

  if(digitalRead(PIN_RC2) == 1 && flag_rc2 == 0)
  {
    flag_rc2 = 1;
    last_time_RC2 = micros();
  }else if(digitalRead(PIN_RC2) == 0 && flag_rc2 == 1)
  {
    flag_rc2 =0;
    channel_2_width = micros()-last_time_RC2;
    RC2 = float(channel_2_width-RC_MIN)/(RC_MAX-RC_MIN)*100;
    if(RC2<0) RC2=0;
    if(RC2>100) RC2=100;
  }
}
 // Port C, PCINT8 - PCINT14
// ISR(PCINT2_vect){}    // Port D, PCINT16 - PCINT23
#endif
