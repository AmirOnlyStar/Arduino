#include<SPI.h>
SPIClass my_spi;
void setup() {
  // put your setup code here, to run once:
my_spi.begin();
my_spi.setClockDivider(SPI_CLOCK_DIV128);
my_spi.setDataMode(SPI_MODE3);
//SPI.setMOde();
//SPI.beginTransaction(SPISettings(250000, MSBFIRST, SPI_MODE3));
}

void loop() {
  // put your main code here, to run repeatedly:
  my_spi.transfer(0X55);

}
