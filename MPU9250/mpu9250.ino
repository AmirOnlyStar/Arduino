#include "mpu9250.h"
#include <Wire.h>

mpu9250 imu(Wire);

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  Serial.println("Serial RDY ...");

  Serial.println(imu.testconnection() ? "MPU9250 connection Successful." : "MPU9250 connection Failed!");
}

void loop()
{


}
