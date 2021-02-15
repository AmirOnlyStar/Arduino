 #include "EEPROM.h"
 float a = -311.31;
 float c = -355.55;
 uint32_t a1;
 float b;
 uint8_t b1,b2,b3,b4;
 void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  flaot_parser(a);
 
}
void loop() {

}


void flaot_parser(float the_float)
{
//  uint32_t * fake_float;
//  uint8_t bytes[4];
////  float the_float;
//  
//  fake_float = (*uint32_t) &the_float;
//  bytes = (*uint8_t)fake_float;
//  
//  Serial.print("bytes[0]:");Serial.println(bytes[0]);
//  Serial.print("bytes[1]:");Serial.println(bytes[1]);
//  Serial.print("bytes[2]:");Serial.println(bytes[2]);
//  Serial.print("bytes[3]:");Serial.println(bytes[3]);
  float data;
   float data2;
  Serial.print("the_float:");Serial.println(the_float);
  EEPROM.put(0,the_float);delay(5);
  EEPROM.put(4,c);delay(5);
  EEPROM.get(0,data);delay(5);
   EEPROM.get(4,data2);delay(5);
  Serial.print("data:");Serial.println(data);
  Serial.print("data2:");Serial.println(data2);
  
}
