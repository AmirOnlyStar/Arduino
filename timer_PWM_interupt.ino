void setup()
{


  DDRB=(0<<DDB7) | (0<<DDB6) | (1<<DDB5) | (0<<DDB4) | (1<<DDB3) | (0<<DDB2) | (1<<DDB1) | (0<<DDB0);
  PORTB=(0<<PORTB7) | (0<<PORTB6) | (0<<PORTB5) | (0<<PORTB4) | (0<<PORTB3) | (0<<PORTB2) | (0<<PORTB1) | (0<<PORTB0);

  DDRC=(0<<DDC6) | (0<<DDC5) | (0<<DDC4) | (0<<DDC3) | (0<<DDC2) | (0<<DDC1) | (0<<DDC0);
  PORTC=(0<<PORTC6) | (0<<PORTC5) | (0<<PORTC4) | (0<<PORTC3) | (0<<PORTC2) | (0<<PORTC1) | (0<<PORTC0);

  DDRD=(0<<DDD7) | (1<<DDD6) | (0<<DDD5) | (0<<DDD4) | (0<<DDD3) | (0<<DDD2) | (0<<DDD1) | (0<<DDD0);
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
  TCCR0B=(0<<WGM02) | (0<<CS02) | (1<<CS01) | (1<<CS00);
  TCNT0=0x00;
  OCR0A=0xFF;
  OCR0B=0x00;

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
  TCCR2B=(0<<WGM22) | (1<<CS22) | (0<<CS21) | (0<<CS20);
  TCNT2=0x00;
  OCR2A=0x55;
  OCR2B=0x00;

  /****************
  fout = (Fclk / (1+OCRnx))
  ****************/

  // Timer/Counter 0 Interrupt(s) initialization //compare match interupt
  TIMSK0=(0<<OCIE0B) | (1<<OCIE0A) | (0<<TOIE0);
  // Timer/Counter 2 Interrupt(s) initialization
  TIMSK2=(0<<OCIE2B) | (1<<OCIE2A) | (0<<TOIE2);
  // sei();

}

void loop()
{
// PORTB ^=(1<<5);
  // PORTB |=(1<<5);
  // delay(5);
  // PORTB &=~(1<<5);
  // delay(1);
}

ISR(TIMER0_COMPA_vect)
{
PORTB ^=(1<<PORTB5);
}

ISR(TIMER2_COMPA_vect)
{
// PORTB ^=(1<<PORTB5);
}
