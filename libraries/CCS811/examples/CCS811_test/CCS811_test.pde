/***************************************************************************
  This is a library for the CCS811 air 

  This sketch reads the sensor

  Designed specifically to work with the Adafruit CCS811 breakout
  ----> http://www.adafruit.com/products/3566

  These sensors use I2C to communicate. The device's I2C address is 0x5A

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <I2C.h>
I2C i2cx = I2C(400000);

#include "CCS811.h"
CCS811 ccs = CCS811((I2C*) &i2cx);

uint32_t counter=0;

void setup()
{
  Serial.begin(115200);
  
  Serial.println("CCS811 test");

  while(!ccs.begin())
  {
    Serial.println("Failed to start sensor! Please check your wiring.");
    delay (500);
  }

  //calibrate temperature sensor
  while(!ccs.dataReady());
  ccs.offset_temp_C = ccs.ntc_temp_C - 25.0f;
}

void loop()
{
  // Serial.print("Try ");
  // Serial.println(++counter,DEC);
  if(ccs.dataReady())
  {
    Serial.print("CO2: ");
    Serial.print(ccs.eCO2);
    Serial.print("ppm, TVOC: ");
    Serial.print(ccs.TVOC);
    Serial.print("ppb   Temp:");
    Serial.println(ccs.ntc_temp_C - ccs.offset_temp_C);
  }
}