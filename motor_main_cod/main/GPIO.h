
#ifndef _GPIO_H_
#define _GPIO_H_
#include "my_type.h"
//########################### #GPIO_PIN ############################
#define PIN_SDA 18//A4
#define PIN_SCL 19//A5
//########################### #LOADCELL_PIN ########################
#define PIN_LOADCELL_SCK 15 //A1
#define PIN_LOADCELL_SCK 14 //A0
//########################### #RPMMETER_PIN ########################
#define PIN_RPM_METER1 2
#define PIN_RPM_METER2 3
//########################### #STEPMOTOR_PIN #######################
#define PIN_STEP_MOTOR_EN     4
#define PIN_STEP_MOTOR_DIR_P  7
#define PIN_STEP_MOTOR_DIR_N  8
#define PIN_STEP_MOTOR_PULSE  9

//########################### #DCMOTOR_PIN #########################
#define PIN_DC_MOTOR_A1 10
#define PIN_DC_MOTOR_A2 12
#define PIN_DC_MOTOR_B1 11
#define PIN_DC_MOTOR_B2 13
//########################### #ESC_PIN ########################
#define PIN_ESC1 5
#define PIN_ESC2 6
//########################### #RC_PIN ########################
#define PIN_RC1 17 //A3 PCINT11
#define PIN_RC2 16 //A2 PCINT10

s8 GPIO_init(void);

s8 GPIO_init(void)
{
  pinMode(PIN_RPM_METER1,INPUT_PULLUP);
  pinMode(PIN_RPM_METER2,INPUT_PULLUP);


  pinMode(PIN_STEP_MOTOR_EN,OUTPUT);
    digitalWrite(PIN_STEP_MOTOR_EN,HIGH); //disable step motor

  pinMode(PIN_STEP_MOTOR_DIR_P,OUTPUT);
    digitalWrite(PIN_STEP_MOTOR_DIR_P,HIGH);

  pinMode(PIN_STEP_MOTOR_DIR_N,OUTPUT);
    digitalWrite(PIN_STEP_MOTOR_DIR_N,HIGH);

  pinMode(PIN_STEP_MOTOR_PULSE,OUTPUT);
    digitalWrite(PIN_STEP_MOTOR_PULSE,HIGH);


  pinMode(PIN_DC_MOTOR_A1,OUTPUT);
    digitalWrite(PIN_DC_MOTOR_A1,HIGH);
  pinMode(PIN_DC_MOTOR_A2,OUTPUT);
    digitalWrite(PIN_DC_MOTOR_A2,HIGH);
  pinMode(PIN_DC_MOTOR_B1,OUTPUT);
    digitalWrite(PIN_DC_MOTOR_B1,HIGH);
  pinMode(PIN_DC_MOTOR_B2,OUTPUT);
      digitalWrite(PIN_DC_MOTOR_B2,HIGH);


  // pinMode(PIN_ESC1,OUTPUT);
  //     digitalWrite(PIN_ESC1,LOW);
  // pinMode(PIN_ESC2,OUTPUT);
  //     digitalWrite(PIN_ESC2,LOW);
  return 0;
}


#endif
