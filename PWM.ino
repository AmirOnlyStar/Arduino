
#include"util/delay.h"

#define TILT_MOTOR_EN   (1<<5) /*PD5*/
#define TILT_MOTOR_STEP (1<<6) /*PD6*/
#define TILT_MOTOR_DIR  (1<<7) /*PD7*/

#define PAN_MOTOR_EN   (1<<2)  /*PB2*/
#define PAN_MOTOR_STEP (1<<3)  /*PB3*/
#define PAN_MOTOR_DIR  (1<<4)  /*PB4*/

unsigned int pulse_count_tilt ;
unsigned int pulse_counter_tilt ;
unsigned int counter_soft_stop_tilt = 0;
unsigned int counter_soft_start_tilt = 0;

unsigned int pulse_count_pan;
unsigned int pulse_counter_pan ;
unsigned int counter_soft_stop_pan = 0;
unsigned int counter_soft_start_pan= 0;

#define SW1 A0
#define SW2 A1
#define SW3 A2
#define SW4 A3

// #define MAX_OCN 0XFE  //min speed
// #define MIN_OCN 0X50  //max speed
// #define MAX_OCN 0X9E  //min speed
// #define MIN_OCN 0X50  //max speed
#define MAX_OCN 0XAE  //min speed
#define MIN_OCN 0X70  //max speed

unsigned int travel_soft_tilt = 1000;
unsigned int point_soft_tilt = 0;

unsigned int travel_soft_pan = 1000;
unsigned int point_soft_pan = 0;
unsigned int ch1,ch2,ch3,ch4;
volatile  int flag_end = 0;
volatile  int flag_move = 0;
volatile  int flag_key_perss_pan = 0;
volatile  int flag_key_perss_tilt = 0;
volatile  int flag_cc_pan = 0;
volatile  int flag_ccw_pan = 0;
volatile  int flag_cc_tilt = 0;
volatile  int flag_ccw_tilt = 0;

 // #define MODE1
#define MODE2

void setup()
{

  DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (1<<DDB4) | (1<<DDB3) | (1<<DDB2) | (1<<DDB1) | (0<<DDB0);
  PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

  DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
  PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (1<<PORTC3) | (1<<PORTC2) | (1<<PORTC1) | (1<<PORTC0);

  DDRD=(1<<DDD7) | (1<<DDD6) | (1<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
  PORTD=(0<<PORTD7) | (0<<PORTD6) | (0<<PORTD5) | (0<<PORTD4) | (0<<PORTD3) | (0<<PORTD2) | (0<<PORTD1) | (0<<PORTD0);

  // Timer/Counter 0 initialization
  // Clock source: System Clock
  // Clock value: 250.000 kHz
  // Mode: CTC top=OCR0A
  // OC0A output: Toggle on compare match
  // OC0B output: Disconnected
  // Timer Period: 0.504 ms
  // Output Pulse(s):
  // OC0A Period: 1.008 ms Width: 0.504 ms
  TCCR0A=(0<<COM0A1) | (1<<COM0A0) | (0<<COM0B1) | (0<<COM0B0) | (1<<WGM01) | (0<<WGM00);
  // TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
  TCNT0=0x00;
  OCR0A=0x0A;
  OCR0B=0x00;

  TIMSK0 &=~ (1<<OCIE0A); //stop interupt
  TCCR0B=0x00;            //stop clock

  // Timer/Counter 2 initialization
  // Clock source: System Clock
  // Clock value: 250.000 kHz
  // Mode: CTC top=OCR2A
  // OC2A output: Toggle on compare match
  // OC2B output: Disconnected
  // Timer Period: 0.504 ms
  // Output Pulse(s):
  // OC2A Period: 1.008 ms Width: 0.504 ms
  ASSR=(0<<EXCLK) | (0<<AS2);
  TCCR2A=(0<<COM2A1) | (1<<COM2A0) | (0<<COM2B1) | (0<<COM2B0) | (1<<WGM21) | (0<<WGM20);
  // TCCR2B=(0<<WGM22) | (1<<CS22) | (0<<CS21) | (0<<CS20);
  TCNT2=0x00;
  OCR2A=0x0A;
  OCR2B=0x00;

  TIMSK2 &=~ (1<<OCIE2A); //stop interupt
  TCCR2B=0x00;            //stop clock

  /****************
  fout = (Fclk / 2*N*(1+OCRnx))
  N=64
  ****************/
  // Timer/Counter 0 Interrupt(s) initialization //compare match interupt
  // TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);
  // Timer/Counter 2 Interrupt(s) initialization
  // TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);
  sei();
  Serial.begin(1000000);
}

void loop()
{
// PORTB ^=(1<<5);
  // PORTB |=(1<<5);
  // delay(5);
  // PORTB &=~(1<<5);
  // delay(1);
  while(1)
  {
    #ifdef MODE1
    move_tilt(-90);
    Serial.println(flag_end);
    while(flag_end == 0)
    {
    }
    _delay_ms(500);
    Serial.println("#End-");
    move_tilt(90);
    while(flag_end == 0)
    {
    }
    Serial.println("#End+");
    _delay_ms(500);

    // if(Serial.available())
    // {
    //   char ch =0;
    //   char str[50]="\0";
    //   char temp_str[50]="\0";
    //   int counter = 0;
    //   int32_t number;
    //
    //   while(Serial.available())
    //   {
    //     ch = Serial.read();
    //     str[counter++] = ch;
    //   }
    //   Serial.print("String:");Serial.println(str);
    //
    //   if(str[0] == 'A')
    //   {
    //     sscanf(str,"%*[^:]:%s",temp_str);
    //     number = atoi(temp_str);
    //     Serial.print("number");Serial.println(number);
    //     Serial.println("Start");
    //     move_tilt(number);
    //     // move_pan(number);
    //   }else if(str[0] == 'B')
    //   {
    //     sscanf(str,"%*[^:]:%s",temp_str);
    //     number = atoi(temp_str);
    //     Serial.print("number");Serial.println(number);
    //     Serial.println("Start-");
    //     number = number*(-1);
    //     move_tilt(number);
    //     // move_pan(number);
    //   }
    // }

    #endif

    #ifdef MODE2
    ch1 = digitalRead(SW1);
    ch2 = digitalRead(SW2);
    ch3 = digitalRead(SW3);
    ch4 = digitalRead(SW4);

    Serial.print(ch1);Serial.print(" ");
    Serial.print(ch2);Serial.print(" ");
    Serial.print(ch3);Serial.print(" ");
    Serial.print(ch4);Serial.print(" ");
    Serial.println(flag_key_perss_pan);
    Serial.println(flag_key_perss_tilt);
    /*move pan*/
    if(ch1==0 && ch2==1 && flag_ccw_pan == 0)
    {
      flag_cc_pan = 1;
      move_pan(1);
    }else if(ch1==1 && ch2==0 && flag_cc_pan == 0)
    {
      flag_ccw_pan = 1;
      move_pan(-1);
    }else
    {
      flag_key_perss_pan = 0;
    }
    /*move tilt*/
    if(ch3==0 && ch4==1 && flag_ccw_tilt == 0)
    {
      flag_cc_tilt = 1;
       move_tilt(1);
    }else if(ch3==1 && ch4==0 && flag_cc_tilt == 0)
    {
      flag_ccw_tilt = 1;
      move_tilt(-1);
    }else
    {
      flag_key_perss_tilt = 0;
    }
    #endif
  }
}
/*tilt motor*/
ISR(TIMER0_COMPA_vect)
{
  pulse_counter_tilt ++;
  #ifdef MODE1 /* move degree */

  if(pulse_counter_tilt >=  pulse_count_tilt)
  {
    motor_tilt_stop();
    // motor_disable();
    pulse_counter_tilt = 0;
    Serial.println("End");
    flag_end = 1;

  }else if(pulse_counter_tilt >= (pulse_count_tilt - travel_soft_tilt))//soft stop
  {
    counter_soft_stop_tilt++;
    if(counter_soft_stop_tilt == point_soft_tilt)
    {
      OCR0A += 1;
      if(OCR0A>=MAX_OCN) OCR0A = MAX_OCN; //min speed
      counter_soft_stop_tilt = 0;
    }

  }else if(pulse_counter_tilt <= travel_soft_tilt) //soft start
  {
    counter_soft_start_tilt++;
    if(counter_soft_start_tilt == point_soft_tilt)
    {
      OCR0A -= 1;
      if(OCR0A<=MIN_OCN) OCR0A = MIN_OCN; //max speed
      counter_soft_start_tilt = 0;
    }
  }
  #endif

  #ifdef MODE2 /* move step */

  if(flag_key_perss_tilt == 0)//soft stop
  {
    counter_soft_stop_tilt++;
    if(counter_soft_stop_tilt >= point_soft_tilt)
    {
      OCR0A += 1;
      if(OCR0A>=MAX_OCN)
      {
        motor_tilt_stop();
        Serial.println("Stop key press");
        flag_end = 1;
        flag_ccw_tilt=0;
        flag_cc_tilt=0;
      }
      counter_soft_stop_tilt = 0;
    }

  }else if((pulse_counter_tilt <= travel_soft_tilt) && (flag_key_perss_tilt == 1)) //soft start
  {
    counter_soft_start_tilt++;
    if(counter_soft_start_tilt == point_soft_tilt)
    {
      OCR0A -= 1;
      if(OCR0A<=MIN_OCN) OCR0A = MIN_OCN; //max speed
      counter_soft_start_tilt = 0;
    }
  }
  #endif
}
/*pan motor*/
ISR(TIMER2_COMPA_vect)
{
  pulse_counter_pan ++;
  #ifdef MODE1 /* move degree */
  if(pulse_counter_pan >=  pulse_count_pan)
  {
    motor_pan_stop();
    // motor_disable();
    pulse_counter_pan = 0;
    Serial.println("End");
    flag_end = 1;

  }else if(pulse_counter_pan >= (pulse_count_pan - travel_soft_pan))//soft stop
  {
    counter_soft_stop_pan++;
    if(counter_soft_stop_pan == point_soft_pan)
    {
      OCR2A += 1;
      if(OCR2A>=MAX_OCN) OCR2A = MAX_OCN; //min speed
      counter_soft_stop_pan = 0;
    }

  }else if(pulse_counter_pan <= travel_soft_pan) //soft start
  {
    counter_soft_start_pan++;
    if(counter_soft_start_pan == point_soft_pan)
    {
      OCR2A -= 1;
      if(OCR2A<=MIN_OCN) OCR2A = MIN_OCN; //max speed
      counter_soft_start_pan = 0;
    }
  }
  #endif

  #ifdef MODE2 /* move step */

  if(flag_key_perss_pan == 0)//soft stop
  {
    counter_soft_stop_pan++;
    if(counter_soft_stop_pan >= point_soft_pan)
    {
      OCR2A += 1;
      if(OCR2A>=MAX_OCN)
      {
        motor_pan_stop();
        Serial.println("Stop key press");
        flag_end = 1;
        flag_ccw_pan=0;
        flag_cc_pan=0;
      }
      counter_soft_stop_pan = 0;
    }

  }else if((pulse_counter_pan <= travel_soft_pan) && (flag_key_perss_pan == 1)) //soft start
  {
    counter_soft_start_pan++;
    if(counter_soft_start_pan == point_soft_pan)
    {
      OCR2A -= 1;
      if(OCR2A<=MIN_OCN) OCR2A = MIN_OCN; //max speed
      counter_soft_start_pan = 0;
    }
  }
  #endif

}

void move_tilt(int32_t degree)//deg//s
{
  //1/32
  //12800 pulse each route
  //0.028125 each pusle degree
  //35.55555 PULSE FOR 1 DEGREE

   //1/16
   //6400 pulse each route
   //0.05625 each pusle degree
   //17.7778 1 PULSE FOR 1 DEGREE
  if(degree>0)
  {
    PORTD |=  (TILT_MOTOR_DIR);
    motor_tilt_enable();
  }else
  {
    PORTD &= ~(TILT_MOTOR_DIR);
    motor_tilt_enable();
  }

  #ifdef MODE1
    uint16_t temp_pulse;
    float temp_Fout;
    temp_pulse = abs(degree) * 35.55555 * 6;
    // temp_pulse = abs(degree) * 17.7778 * 6; //6 gear adaptor
    pulse_count_tilt=temp_pulse;

    point_soft_tilt = (travel_soft_tilt)/(MAX_OCN-MIN_OCN);
    Serial.print("travelpoint:");Serial.println(point_soft_tilt);

    // OCR0A=temp_speed;
    OCR0A = MAX_OCN; //minimum speed
    PORTD &= ~(TILT_MOTOR_STEP);

    TIMSK0 |=(1<<OCIE0A);
    TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
    pulse_counter_tilt = 0;
    flag_end = 0;
  #endif

  #ifdef MODE2
    if(flag_key_perss_tilt == 0)
    {
      point_soft_tilt = (travel_soft_tilt)/(MAX_OCN-MIN_OCN);
      Serial.print("travelpoint:");Serial.println(point_soft_tilt);
      flag_key_perss_tilt = 1;
      OCR0A = MAX_OCN; //minimum speed
      PORTD &= ~(TILT_MOTOR_STEP);
      TIMSK0 |=(1<<OCIE0A);
      TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
      pulse_counter_tilt = 0;
    }
  #endif
}

void motor_tilt_disable(void)
{
  PORTD |= (TILT_MOTOR_EN);     /*active low*/
  // PORTD &= ~(TILT_MOTOR_EN); /*active high*/
}
void motor_tilt_enable(void)
{
  // PORTD |= (TILT_MOTOR_EN); /*active low*/
  PORTD &= ~(TILT_MOTOR_EN);   /*active high*/
}

void motor_tilt_stop(void)
{
  TIMSK0 &=~ (1<<OCIE0A); //stop interupt
  TCCR0B=0x00;            //stop clock
}

void move_pan(int32_t degree)//deg//s
{
  //1/32
  //12800 pulse each route
  //0.028125 each pusle degree
  //35.55555 PULSE FOR 1 DEGREE

   //1/16
   //6400 pulse each route
   //0.05625 each pusle degree
   //17.7778 1 PULSE FOR 1 DEGREE
  if(degree>0)
  {
    PORTB |=  (PAN_MOTOR_DIR);
    motor_pan_enable();
  }else
  {
    PORTB &= ~(PAN_MOTOR_DIR);
    motor_pan_enable();
  }

  #ifdef MODE1
    uint16_t temp_pulse;
    float temp_Fout;
    temp_pulse = abs(degree) * 35.55555 * 6;
    // temp_pulse = abs(degree) * 17.7778 * 6; //6 gear adaptor
    pulse_count_tilt=temp_pulse;

    point_soft_tilt = (travel_soft_tilt)/(MAX_OCN-MIN_OCN);
    Serial.print("travelpoint:");Serial.println(point_soft_tilt);

    // OCR0A=temp_speed;
    OCR2A = MAX_OCN; //minimum speed
    PORTD &= ~(TILT_MOTOR_STEP);

    TIMSK2 |=(1<<OCIE2A);
    TCCR2B=(0<<WGM22) | (0<<CS22) | (1<<CS21) | (1<<CS20);
    pulse_counter_tilt = 0;
    flag_end = 0;
  #endif

  #ifdef MODE2
    if(flag_key_perss_pan == 0)
    {
      point_soft_pan = (travel_soft_pan)/(MAX_OCN-MIN_OCN);
      Serial.print("travelpoint:");Serial.println(point_soft_pan);
      flag_key_perss_pan = 1;
      OCR2A = MAX_OCN; //minimum speed
      PORTD &= ~(PAN_MOTOR_STEP);
      TIMSK2 |=(1<<OCIE2A);
      TCCR2B=(0<<WGM02) | (0<<CS22) | (1<<CS21) | (1<<CS20);
      pulse_counter_pan = 0;
    }
  #endif
}

void motor_pan_disable(void)
{
  PORTB |= (PAN_MOTOR_EN);     /*active low*/
  // PORTD &= ~(TILT_MOTOR_EN); /*active high*/
}
void motor_pan_enable(void)
{
  // PORTD |= (TILT_MOTOR_EN); /*active low*/
  PORTB &= ~(PAN_MOTOR_EN);   /*active high*/
}

void motor_pan_stop(void)
{
  TIMSK2 &=~ (1<<OCIE2A); //stop interupt
  TCCR2B=0x00;            //stop clock
}
