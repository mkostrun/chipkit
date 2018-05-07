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

#include "I2C.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MPL3115A2 class
*/
/**************************************************************************/
I2C::I2C(uint32_t freq)
{
  if (_init)
    return;

  /*Initialize I2C Peripheral*/
#if defined(__32MX695F512L__)
  // WF32: has pull-up resistors for SDA2,SCL2 lines on board
  pinMode(62,OUTPUT);
  pinMode(63,OUTPUT);
  digitalWrite(62,HIGH);
  digitalWrite(63,HIGH);
#endif

  // Configure Various I2C Options
  I2CConfigure(I2Cx, I2C_ENABLE_HIGH_SPEED);

  // Set the I2C baudrate
  I2CSetFrequency(I2Cx, PBCLK, freq);

  // Start it up
  I2CEnable(I2Cx, (BOOL) 1);

  _init = 1;
}


/**************************************************************************/
/*!
    @brief  read 1 byte of data at the specified address
    @param reg the address to read
    @return the read data byte
*/
/**************************************************************************/
uint8_t I2C::read_byte(uint8_t dev, uint8_t reg, uint8_t rs)
{
  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 0;

  MasterWriteI2C(reg);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 0;

  if (rs)
  {
    //initiate repeated start and read data
    RestartI2C();
    IdleI2C();
  }

  // Read from device
  MasterWriteI2C(dev<<1 | 1);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 0;

  uint8_t rval = MasterReadI2C();

  // Send NACK:
  I2CxCONbits.ACKDT = 1; //Set for NotACk
  I2CxCONbits.ACKEN = 1;
  while (I2CxCONbits.ACKEN); //wait for ACK to complete
  I2CxCONbits.ACKDT = 0; //Set for NotACk

  StopI2C();
  IdleI2C();

  return rval;
}

uint8_t I2C::read(uint8_t dev, uint8_t reg, uint16_t size, uint8_t * data, uint8_t rs)
{
  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 1;

  MasterWriteI2C(reg);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 1;

  //initiate repeated start and read data
  if (rs)
  {
    RestartI2C();
    IdleI2C();
  }

  // Read from device
  MasterWriteI2C(dev<<1 | 1);
  IdleI2C();
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 1;

  uint16_t i=0;
  while (i<size)
  {
    data[i] = MasterReadI2C();
    i++;
    if (i<size)
    {
      IdleI2C();
      // Send ACK:
      I2CxCONbits.ACKDT = 0; //Set for ACk
      I2CxCONbits.ACKEN = 1;
      while (I2CxCONbits.ACKEN);
    }
  }
  // Send NACK:
  I2CxCONbits.ACKDT = 1; //Set for NotACk
  I2CxCONbits.ACKEN = 1;
  while (I2CxCONbits.ACKEN); //wait for ACK to complete
  I2CxCONbits.ACKDT = 0; //Set for NotACk

  StopI2C();
  IdleI2C();

  return 0;
}

/**************************************************************************/
/*!
    @brief  write a byte of data to the specified address
    @param reg the address to write to
    @param data the byte to write
*/
/**************************************************************************/
void I2C::write_byte(uint8_t dev, uint8_t reg, uint8_t data)
{
  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK ) // slave should respond with ACK
    return;

  MasterWriteI2C(reg);
  IdleI2C();
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK ) // slave should respond with ACK
    return;

  MasterWriteI2C(data); // slave should respond with NACK (send no more data). do we care?
  IdleI2C();

  StopI2C();
  IdleI2C();
}

/**************************************************************************/
/*!
    @brief write number of bytes to specific register
    @param reg the address to write to
    @param size number of bytes to write
    @param data the bytes of data to write
 */
/**************************************************************************/
void I2C::write(uint8_t dev, uint8_t reg, uint16_t size, uint8_t * data)
{
  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  MasterWriteI2C(reg);
  IdleI2C();
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  uint16_t i = 0;
  while (i<size)
  {
    MasterWriteI2C(data[i]);
    i++;
    if (i<size)
    {
      IdleI2C();
      if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
        return;
    }
  }

  StopI2C();
  IdleI2C();
}


