long unsigned int timer = 0;
long unsigned int timer_out = 0;
long unsigned int last_time_pulse = 0;
unsigned int pulse_counter = 0;
bool flag_sleep = 0;
bool flag_pulse_start = 0;
// long unsigned int last_time = 0;

void int_rutin(void);
void sleep_en(void);
void sleep_di(void);

#define RST         4    //pin3
#define TINY13_IN     0    //pin5   past TINY13_OUT
#define WTD         1    //pin6
#define TEST        3    //pin2

void setup() {
  // put your setup code here, to run once:

pinMode(RST,OUTPUT);
  digitalWrite(RST,LOW);
pinMode(TINY13_IN,INPUT);
  digitalWrite(TINY13_IN,LOW);
  
digitalWrite(WTD,HIGH);
  
pinMode(TEST,OUTPUT);
  digitalWrite(TEST,LOW);
// last_time = millis();
timer=0;
//attachInterrupt(digitalPinToInterrupt(WTD), int_rutin, CHANGE);
MCUCR |=  (1<<ISC00); // INT0 enable any change 
MCUCR &= ~(1<<ISC01); 
GIMSK  |= (1<<INT0); //enable INT0

sei();
//sleep_en();
//sleep_di();

}

void loop() {
//  sleep_en();
//  digitalWrite(RST,HIGH);
//  delay(20);
//  digitalWrite(RST,LOW);
//  delay(20);
//  digitalWrite(RST,HIGH);
//  delay(100);
//  digitalWrite(RST,LOW);
//  delay(100);
//  // put your main code here, to run repeatedly:
//  timer += 5;
//  if(flag_sleep == 0)
//  {
//    
//    if(timer>20000 )//ms 20s
//    {
//      digitalWrite(RST,HIGH);
//      delay(500);
//      digitalWrite(RST,LOW);
//      timer = 0;
//    }
// 
//  }else
//  {
//    sleep_en(); 
//    timer = 0;
//  }
//
//  if(flag_pulse_start == 0)
//  {
//    last_time_pulse = millis();
//  }
//  if(millis()- last_time_pulse>300) //500ms
//  {
//    if(pulse_counter>30)
//    {
//      flag_sleep = 1;
//      digitalWrite(TEST,HIGH);
//    }else if(pulse_counter>10 && pulse_counter<30)
//    {
//      flag_sleep = 0;
//      sleep_di();
//      digitalWrite(TEST,LOW);
//    }
//    pulse_counter = 0;
//    flag_pulse_start = 0;
//  }
//  delay(5);
}

//void int_rutin(void)
//{
//  digitalWrite(RST,HIGH);
//  delay(100);
//  digitalWrite(RST,LOW);
//  delay(100);
//}
ISR (PCINT0_vect)
{
  flag_pulse_start = 1;
  pulse_counter++;
}
ISR (INT0_vect)
{
  timer = 0;
  digitalWrite(RST,HIGH);
  delay(100);
  digitalWrite(RST,LOW);
  delay(100);

}
void sleep_en(void)
{
  GIMSK  |= (1<<PCIE); //enable PCIN
  PCMSK |= (1<<PCINT0);
  
  ADCSRA &= ~(1<<ADEN); //disable ADC
  MCUCR |= (1<<SM1)|(0<<SM0); //set power down sleep
  MCUCR |= (0<<ISC01)|(1<<ISC00); // INT0 enable any change 
  MCUCR |= (1<<SE);
  
  sei();
  asm("sleep");
  cli();
  MCUCR &= ~(1<<SE);
  ADCSRA |= (1<<ADEN); //Enable ADC
  sei();
  

}
void sleep_di(void)
{
  MCUCR &= ~(1<<SE);
//  DDRB  = (0 << PB5) | (1 << PB4) | (1 << PB3) | (0 << PB2) | (0 << PB1) | (0 << PB0);
//  PORTB = (1 << PB5) | (0 << PB4) | (0 << PB3) | (1 << PB2) | (1 << PB1) | (1 << PB0);
}
