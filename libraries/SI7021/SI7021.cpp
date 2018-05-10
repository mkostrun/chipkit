/*!
 * @file SI7021.cpp
 *
 * @mainpage SI7021 temperature and humidity sensor
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for SI7021 driver for the
 * chipkit/digilent development platform.
 *
    This is a library for the Adafruit Si7021 breakout board
    ----> https://www.adafruit.com/products/3251

    Adafruit invests time and resources providing this open source code,
    please support Adafruit and open-source hardware by purchasing
    products from Adafruit!
 *
 *
 * @section dependencies Dependencies
 *
 * @section author Author
 *
 * Written by Marijan Kostrun. 
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "SI7021.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new SI7021 class
*/
/**************************************************************************/
SI7021::SI7021(I2C * i2cx, uint32_t dt_ms = 1000)
{
  i2c = (I2C *) i2cx;
  dt_between_calls_ms = dt_ms;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param twoWire Optional TwoWire I2C object
    @return true on successful startup, false otherwise
*/
/**************************************************************************/
void SI7021::reset(void)
{
  i2c->write(SI7021_ADDRESS<<1, SI7021_RESET, 0, NULL);
  state = 0;
  return;
}

float SI7021::read_t_prev_meas_rh( void )
{
  uint8_t  d[3];
  i2c->read(SI7021_ADDRESS, SI7021_READ_T_PREV_RH, 3, (uint8_t *)d, 1);

  temperature_c = (float) (d[1] | d[0]<<8);
  temperature_c *= 175.72f;
  temperature_c /= 65536.0f;
  temperature_c -= 46.85f;
  return temperature_c;
}

uint8_t SI7021::measure( void )
{
  uint8_t d[3];
  if (state == 0)
  {
    if ( (millis() - time_last_call_ms < dt_between_calls_ms) )
      return 0;

    time_last_call_ms = millis();

    i2c->write(SI7021_ADDRESS, SI7021_MEAS_RH_NOHOLD, 0, NULL);
    state = 1;

    return 0;
  }

  // Si7021 NACK means measurement hasnt been done yet
  if (i2c->read(SI7021_ADDRESS, 3, (uint8_t *)d, 1))
    return 0;

  rel_humidity_pc = (float) (d[1] | d[0]<<8);
  rel_humidity_pc *= 125.0f;
  rel_humidity_pc /= 65536.0f;
  rel_humidity_pc -= 6.0f;

  // get temperature
  read_t_prev_meas_rh();

  state = 0;

  return 1;
}









