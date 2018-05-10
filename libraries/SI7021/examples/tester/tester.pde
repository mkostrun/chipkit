/**************************************************************************/
/*!
    @file     tester.pde
    @author   M. Kostrun
    @license  BSD (see license.txt)

    Example for the SI7021 temperature and relative humidity sensor
    for Digilent UNO32 development board

    @section  HISTORY
    v1.0 - First release
 */
/**************************************************************************/

// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
#include <I2C.h>
I2C i2cx = I2C(400000);

#include <SI7021.h>
SI7021 trh_probe = SI7021((I2C*) &i2cx, 1000);

void print_all(void)
{
  Serial.print(trh_probe.temperature_c,1);
  Serial.print(",");
  Serial.print(trh_probe.rel_humidity_pc,1);
  Serial.print("\n");
}

void setup()
{
  Serial.begin(115200);
  delay (1000);
  Serial.print("starting\n");
}

void loop()
{
  if (trh_probe.measure())
  {
    print_all();
  }
}
