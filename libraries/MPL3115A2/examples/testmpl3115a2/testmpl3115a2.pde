/**************************************************************************/
/*!
    @file     testmpl3115a2.pde
    @author   M. Kostrun
    @license  BSD (see license.txt)

    Example for the MPL3115A2 barometric pressure sensor
    for Digilent UNO32 development board

    This is a library for the Adafruit MPL3115A2 breakout
    ----> https://www.adafruit.com/products/1893

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!

    @section  HISTORY
    v1.0 - First release
*/
/**************************************************************************/
// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
#include <I2C.h>
I2C i2cx = I2C(400000);

#include <MPL3115A2.h>
MPL3115A2 barometer = MPL3115A2((I2C*) &i2cx);

uint16_t 
    n_avg=10, counter_avg=0;

int32_t 
    avg_p_pa=0, last_p_pa=0;

uint8_t 
    print_notcompleted=0, cont_print=1;

float
    avg_temp_c, last_temp_c;


void print_all(void)
{
  Serial.print(last_temp_c,1);
  Serial.print(",");
  Serial.print((int32_t) last_p_pa);
  Serial.print("\n");
}

void setup()
{
  Serial.begin(115200);

  // barometer
  while (!barometer.begin())
  {
    Serial.print("Pressure sensor not connected or failed to initialize\n");
    delay(500);
  }
#ifdef DEBUG
  Serial.print("Pressure sensor initialized!\n");
#endif
}

void loop()
{
  if (barometer.data_ready(MPL3115A2_MODE_P))
  {
    counter_avg = (counter_avg + 1) % n_avg;
    if (!counter_avg)
    {
      last_p_pa = avg_p_pa / n_avg;
      last_temp_c = avg_temp_c / n_avg;
      avg_p_pa = 0;
      avg_temp_c = 0;
      print_notcompleted = 1;
    }
    avg_p_pa += ((int32_t) barometer.pressure_Pa - STANDARD_PRESSURE_PA);
    avg_temp_c += barometer.temperature_C;
  }

  if (cont_print)
  {
    if (print_notcompleted)
    {
      print_all();
      print_notcompleted = 0;
    }
  }
}
