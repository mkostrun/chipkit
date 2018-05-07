/*!
 * @file MPL3115A2.cpp
 *
 * @mainpage MPL3115A2 alitmeter
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for MPL3115A2 driver for the Digilent
 * UNO32 platform.  It is designed specifically to work with the
 * Adafruit MPL3115A2 breakout board: https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 *
 * @section dependencies Dependencies
 *
 * @section author Author
 *
 * Written by Marijan Kostrun. 
 * Inspired by the Adafruit_MPL3115A2 library written
 * by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "MPL3115A2.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MPL3115A2 class
*/
/**************************************************************************/
MPL3115A2::MPL3115A2(I2C * i2cx)
{
  i2c = (I2C *) i2cx;
}

/**************************************************************************/
/*!
    @brief  Setups the HW (reads coefficients values, etc.)
    @param twoWire Optional TwoWire I2C object
    @return true on successful startup, false otherwise
*/
/**************************************************************************/
boolean MPL3115A2::begin(void)
{
  /*Initialize I2C Peripheral*/

  if (i2c->read_byte(MPL3115A2_ADDRESS,MPL3115A2_WHOAMI,1) != 0xC4)
    return 0;

  i2c->write_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1, MPL3115A2_CTRL_REG1_RST);
  delay(10);

  while( i2c->read_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1,1) & MPL3115A2_CTRL_REG1_RST )
  {
    delay(10);
  }

  _ctrl_reg1.reg = MPL3115A2_CTRL_REG1_OS128 | MPL3115A2_CTRL_REG1_ALT;

  i2c->write_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);

  i2c->write_byte(MPL3115A2_ADDRESS,
             MPL3115A2_PT_DATA_CFG,
             MPL3115A2_PT_DATA_CFG_TDEFE | MPL3115A2_PT_DATA_CFG_PDEFE | MPL3115A2_PT_DATA_CFG_DREM
                 );

  return 1;
}

/**************************************************************************/
/*!
    @brief  Checks if the pressure (mode=0) or altitude (mode=1) data is ready
    @return 1 if so, 0 otherwise 
 */
/**************************************************************************/
boolean MPL3115A2::data_ready(uint8_t mode)
{
  static uint8_t pdr_state=0;
  uint8_t status;
  static uint32_t t_last_call_ms;
  uint32_t rval;
  uint8_t * d = (uint8_t *) &rval;

  switch(pdr_state)
  {
    case 0:
      if (millis() - t_last_call_ms < 10)
        break;
      t_last_call_ms=millis();
      if (!(i2c->read_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1, 1) & MPL3115A2_CTRL_REG1_OST))
        pdr_state = 1;
      break;

    case 1:
      if (mode)
        _ctrl_reg1.bit.ALT = 1;
      else
        _ctrl_reg1.bit.ALT = 0;
      i2c->write_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);
      _ctrl_reg1.bit.OST = 1;
      i2c->write_byte(MPL3115A2_ADDRESS,MPL3115A2_CTRL_REG1, _ctrl_reg1.reg);
      t_last_call_ms=millis();
      pdr_state = 2;
      break;

    case 2:
      if (millis() - t_last_call_ms < 10)
        return 0;
      t_last_call_ms=millis();
      status = i2c->read_byte(MPL3115A2_ADDRESS,MPL3115A2_REGISTER_STATUS, 1);
      if (! (status & MPL3115A2_REGISTER_STATUS_PDR) && ! (status & MPL3115A2_REGISTER_STATUS_TDR))
        return 0;
      pdr_state = 3;
      break;

    case 3:
      // p or alt
      rval=0;
      i2c->read(MPL3115A2_ADDRESS,MPL3115A2_REGISTER_PRESSURE_MSB, 3, (uint8_t *)d, 1);
      // swap endianess
      d[3] = d[2];
      d[2] = d[0];
      d[0] = d[3];
      d[3] = 0x00;
      if (mode)
        altitude_m = rval;
      else
        pressure_Pa = rval >> 6;
      // temp
      rval=0;
      i2c->read(MPL3115A2_ADDRESS,MPL3115A2_REGISTER_TEMP_MSB, 2, (uint8_t *)d, 1);
      // swap endianess
      d[2] = d[1];
      d[1] = d[0];
      d[0] = d[2];
      d[2] = 0x00;
      rval >>= 4;
      if (rval & 0x800)
      {
        rval |= 0xF000;
      }
      temperature_C = (float) rval / 16.0f;
      pdr_state = 0;
      return 1;
  }

  return 0;
}

/**************************************************************************/
/*!
    @brief  Set the local sea level barometric pressure
    @param pascal the pressure to use as the baseline
*/
/**************************************************************************/
void MPL3115A2::setSeaPressure(uint32_t pascal)
{
  uint16_t b=pascal>>1;
  // swap endianess
  uint8_t  d[2];
  d[0] = b>>8;
  d[1] = b;
  i2c->write(MPL3115A2_ADDRESS,MPL3115A2_BAR_IN_MSB, 2, d);
}

