#ifndef _STEPMOTOR_H_
  #define _STEPMOTOR_H_

#include "GPIO.h"
#include "my_type.h"

#define STEP_FPWM_MAX 5000//Hz
#define STEP_REV 1600 //pulse per rout

static s32 step_SP=0;
static s32 step_pass = 0;

const u32 CPU_Fclk = 16000000;
u32 PWM_Fpwm ;
u32 PWM_top ;
u32 PWM_top_half ;

u32 last_time_pulse;
bool flag_pulse = 0;
u32 dt_step;
u32 last_time_step;
s8 step_init(void);
s8 step_enable(void);
s8 step_disable(void);
s8 step_forward(void);
s8 step_backward(void);
s8 step_run(s32 deg,float speed); //DEG    //RPS
s8 step_run2(s32 speed_percent);   //RPS
s8 step_callback(void);
s8 step_stop(void);
s8 step_print(void);


s8 step_init(void)
{
  // cli();
  // TCCR1A= _BV(WGM11) | _BV(WGM10);
  // TCCR1B= _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  // TIMSK1|=_BV(OCIE1A) ; //enable intrupt
  //TIMSK1 &= (~(_BV(OCIE1A)));//disable intrupt

  TCCR1A=(1<<COM1A1) | (0<<COM1A0) | (0<<COM1B1) | (0<<COM1B0) | (1<<WGM11) | (0<<WGM10);
  TCCR1B=(0<<ICNC1) | (0<<ICES1) | (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (0<<CS11) | (1<<CS10);
  // TIMSK1=(0<<ICIE1) | (0<<OCIE1B) | (1<<OCIE1A) | (0<<TOIE1);
  ICR1H=0x18;
  ICR1L=0xFF;
  OCR1AH=0x0C;
  OCR1AL=0x80;

  sei();
}
s8 step_enable(void)
{
  digitalWrite(PIN_STEP_MOTOR_EN,LOW);
  TIMSK1|=_BV(OCIE1A) ;//enable intrupt
}

s8 step_disable(void)
{
  digitalWrite(PIN_STEP_MOTOR_EN,HIGH);
  TIMSK1 &= (~(_BV(OCIE1A)));//disable intrupt
}

s8 step_forward(void){
  digitalWrite(PIN_STEP_MOTOR_DIR_N,HIGH);
  digitalWrite(PIN_STEP_MOTOR_DIR_P,LOW);
}

s8 step_backward(void)
{
  digitalWrite(PIN_STEP_MOTOR_DIR_N,LOW);
  digitalWrite(PIN_STEP_MOTOR_DIR_P,HIGH);
}

s8 step_run(s32 deg,float speed)//DEG    //RPS
{

  if(deg>0) step_forward();
  else if(deg<0)step_backward();

  step_pass = 0;
  step_SP = (STEP_REV*deg)/360;

  //speed calculator for fast pwm pinMode
  //Fpwm = Fclk/(1+TOP)
  //TOP = (Fclk/Fpwm)-1
  PWM_Fpwm =  speed*STEP_REV;

  if(PWM_Fpwm>STEP_FPWM_MAX)PWM_Fpwm = STEP_FPWM_MAX;

  PWM_top = (CPU_Fclk/PWM_Fpwm)-1;
  PWM_top_half = PWM_top/2;

  ICR1H=(PWM_top>>8);
  ICR1L=PWM_top;
  OCR1AH=(PWM_top_half>>8); //duty cycle
  OCR1AL=PWM_top_half;

  step_enable();
}

s8 step_run2(s32 speed_percent)
{
  const u32 max_rotation = 360;
  const float max_speed = 3.1;
  float speed;
  if(speed_percent>0) step_forward();
  else if(speed_percent<0)step_backward();
  speed_percent = abs(speed_percent);

  speed = (max_speed*speed_percent)/100;

  if(speed != 0)
  {
    step_pass = 0;
    step_SP = (STEP_REV*max_rotation)/360;

    //speed calculator for fast pwm pinMode
    //Fpwm = Fclk/(1+TOP)
    //TOP = (Fclk/Fpwm)-1
    PWM_Fpwm =  speed*STEP_REV;

    if(PWM_Fpwm>STEP_FPWM_MAX)PWM_Fpwm = STEP_FPWM_MAX;

    PWM_top = (CPU_Fclk/PWM_Fpwm)-1;
    PWM_top_half = PWM_top/2;

    ICR1H=(PWM_top>>8);
    ICR1L=PWM_top;
    OCR1AH=(PWM_top_half>>8); //duty cycle
    OCR1AL=PWM_top_half;

    step_enable();

  }else
  {
    step_disable();
  }

}

// s8 step_callback(void)
// {
//   if(step_pass<step_SP)
//   {
//     if(flag_pulse == 0 && (micros()-last_time_pulse)>step_delay)
//     {
//       flag_pulse = 1;
//       digitalWrite(PIN_STEP_MOTOR_PULSE,LOW);
//       last_time_pulse = micros();
//     }else if(flag_pulse == 1 && (micros()-last_time_pulse)>step_delay)
//     {
//       digitalWrite(PIN_STEP_MOTOR_PULSE,HIGH);
//       flag_pulse = 0;
//       step_pass++;
//       last_time_pulse = micros();
//     }
//   //############################################
// }else if(step_pass>step_SP)
//   {
//     if(flag_pulse == 0 && (micros()-last_time_pulse)>step_delay)
//     {
//       flag_pulse = 1;
//       digitalWrite(PIN_STEP_MOTOR_PULSE,LOW);
//       last_time_pulse = micros();
//     }else if(flag_pulse == 1 && (micros()-last_time_pulse)>step_delay)
//     {
//       digitalWrite(PIN_STEP_MOTOR_PULSE,HIGH);
//       flag_pulse = 0;
//       step_pass--;
//       last_time_pulse = micros();
//     }
//
//   }else
//   {
//     step_disable();
//   }
// }


ISR(TIMER1_COMPA_vect)
{
  dt_step = (double)(micros() - last_time_step) ; // Calculate delta time
  last_time_step= micros();

  if(step_pass<step_SP)
  {
    step_pass++;
  //############################################
  }else if(step_pass>step_SP)
  {
    step_pass--;
  }else
  {
    step_disable();
  }

  // whatever
}

// ISR(TIMER1_CAPT_vect)
// {
//   // dt_step = (double)(micros() - last_time_step) ; // Calculate delta time
//   // last_time_step= micros();
//   // Serial.println("OK2");
//
// }
s8 step_stop(void)
{
  step_pass = 0;
  step_SP = 0;

}
s8 step_print(void)
{
  // Serial.print("  step_dt:");
  // Serial.print(dt_step);
  Serial.print("  step_SP:");
  Serial.print(step_SP);
  Serial.print("  step_pass:");
  Serial.print(step_pass);
  // Serial.print("  WM_Fpwm:");
  // Serial.print(PWM_Fpwm);
  // Serial.print("  top:");
  // Serial.print(PWM_top,HEX);
  // Serial.print("  top_half:");
  // Serial.print(PWM_top_half,HEX);
  // Serial.print("\tstep_delay:");
  // Serial.print(step_delay*2);

}


#endif
