#include "mpu9250.h"


void mpu9250::initMPU9250(void)
{
  write_byte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);  // Clear sleep mode bit (6), enable all sensors
  write_byte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else


}

void mpu9250::initAK8963(void)
{


}


static void mpu9250::read_bytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest)
{
  _wire->beginTransmission(address);         // Initialize the Tx buffer
  _wire->write(subAddress);                  // Put slave register address in Tx buffer
  _wire->endTransmission(false);  // Send the Tx buffer, but send a restart to keep connection alive
  // if (i2c_err_) print_i2c_error();
  uint8_t i = 0;
  _wire->requestFrom(address, count);  // Read bytes from slave register address
  while (_wire->available())
  {
    dest[i++] = _wire->read();
  }  // Put read results in the Rx buffer
}

static uint8_t mpu9250::read_byte(uint8_t address, uint8_t subAddress)
{
  uint8_t temp_read;
  read_bytes(address,subAddress,1,&temp_read);
  return temp_read;
}

static void mpu9250::write_byte(uint8_t address, uint8_t subAddress, uint8_t data)
{
  _wire->beginTransmission(address);    // Initialize the Tx buffer
  _wire->write(subAddress);             // Put slave register address in Tx buffer
  _wire->write(data);                   // Put data in Tx buffer
  _wire->endTransmission();  // Send the Tx buffer
}

uint8_t mpu9250::testconnection(void)
{
  uint8_t temp_read=0;
  temp_read = read_byte(MPU9250_ADDRESS,WHO_AM_I_MPU9250);
  // return temp_read;
  if(temp_read == 0x71)return 1 ;
  else return 0;

}
