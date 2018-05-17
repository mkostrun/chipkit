/*!
 * @file I2C.cpp
 *
 * @mainpage I2C class for chipkit/digilent uno32, max32 and wf32 (PIC32 MX) development boards
 *
 * @section intro_sec Introduction
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

#include "I2C.h"

/**************************************************************************/
/*!
    @brief Re-sets the I2C bus frequency
    @param freq frequency of I2C bus
 */
/**************************************************************************/
void I2C::SetFrequency(uint32_t freq)
{
  // Set the I2C baudrate
  if (f != freq)
  {
    f = freq;
    I2CSetFrequency(I2Cx, PBCLK, f);
    IdleI2C();
  }
}

/**************************************************************************/
/*!
    @brief Instantiates I2C class
    @param freq frequency of I2C bus
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
  //I2CConfigure(I2Cx, I2C_ENABLE_HIGH_SPEED);
  I2CConfigure(I2Cx, I2C_ENABLE_SLAVE_CLOCK_STRETCHING);


  // Set the I2C baudrate
  SetFrequency(freq);

  // Start it up
//   OpenI2C1(I2C_ON | I2C_7BIT_ADD | I2C_STR_EN, BRG_VAL);
  I2CEnable(I2Cx, (BOOL) 1);

  _init = 1;
}


/**************************************************************************/
/*!
    @brief read a byte of data at the specified address
    @param dev the device to write to
    @param reg the register as single byte on the device to write to
    @param rs  do repeated start when switching from write to read atomic i2c operations
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

uint8_t I2C::read_byte_no_nack(uint8_t dev, uint8_t reg, uint8_t rs)
{
  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 0xff;

  MasterWriteI2C(reg);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 0xfe;

  StopI2C();
  IdleI2C();

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
    return 0xfd;

  uint8_t rval = MasterReadI2C();
  IdleI2C();

  StopI2C();
  IdleI2C();

  return rval;
}

/**************************************************************************/
/*!
    @brief read bytes of data at the specified address
    @param dev the device to write to
    @param size number of data bytes requested from the slave
    @param data array of bytes for storage
    @param s send start condition 
    @return 0 if successful, 1 if failure
 */
/**************************************************************************/
uint8_t I2C::read(uint8_t dev, uint16_t size, uint8_t * data, uint8_t s)
{
  if (s)
  {
    StartI2C();
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
    IdleI2C();
    i++;
    if (i<size)
    {
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
    @brief read bytes of data at the specified address without NACK at the end, just stop
    @param dev the device to write to
    @param size number of data bytes requested from the slave
    @param data array of bytes for storage
    @param s send start condition 
    @return 0 if successful, 1 if failure
 */
uint8_t I2C::read_no_nack(uint8_t dev, uint16_t size, uint8_t * data, uint8_t s)
{
  if (s)
  {
    StartI2C();
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

  StopI2C();
  IdleI2C();

  return 0;
}

/**************************************************************************/
/*!
    @brief read bytes of data at the specified address
    @param dev the device to write to
    @param reg the register as single byte on the device to write to
    @param size number of data bytes requested from the slave
    @param data array of bytes for storage
    @param rs do repeated start when switching from write to read atomic i2c operations
    @return 0 if successful, 1 if failure
 */
/**************************************************************************/
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
    @brief read half-words of data at the specified address
    @param dev the device to write to
    @param reg the register as half-word on the device to write to - slave requruires HO,LO bytes
    @param size number of data bytes requested from the slave
    @param data array of half-words for storage - slave responds with big-endian data which is converted to little-endian
    @param rs do repeated start when switching from write to read atomic i2c operations
    @return 0 if successful, 1 if failure
 */
/**************************************************************************/
uint8_t I2C::read(uint8_t dev, uint16_t reg, uint16_t size, uint16_t * data, uint8_t rs)
{
  uint8_t * _r = (uint8_t *) & reg;

  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 1;

  MasterWriteI2C(_r[1]);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return 1;

  MasterWriteI2C(_r[0]);
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
    // set pointer
    _r = (uint8_t *) &data[i];

    // get
    _r[1] = MasterReadI2C();
    IdleI2C();
    //  Send ACK:
    I2CxCONbits.ACKDT = 0; //Set for ACk
    I2CxCONbits.ACKEN = 1;
    while (I2CxCONbits.ACKEN);

    // get LO
    _r[0] = MasterReadI2C();

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
    @brief  write register of a specified device
    @param dev the device to write to
    @param reg the register as single byte on the device to write to
    @param data the byte to write
 */
/**************************************************************************/
void I2C::write_reg(uint8_t dev, uint8_t reg)
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

  StopI2C();
  IdleI2C();
}

/**************************************************************************/
/*!
    @brief  write a byte of data to a register of a specified device
    @param dev the device to write to
    @param reg the register as single byte on the device to write to
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
    @brief  write a half-word of data to a register of a specified device
    @param dev the device to write to
    @param reg the register as half-word on the device to write to
    @param data the half-word to write to 
 */
/**************************************************************************/
void I2C::write_word(uint8_t dev, uint16_t reg, uint16_t data)
{
  uint8_t * _r = (uint8_t *) & reg;

  StartI2C();
  IdleI2C();

  MasterWriteI2C(dev<<1); // start transmission to device
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  MasterWriteI2C(_r[1]);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  MasterWriteI2C(_r[0]);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  _r = (uint8_t *) & data;

  MasterWriteI2C(_r[1]);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  MasterWriteI2C(_r[0]);
  IdleI2C();     //Wait to complete
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  StopI2C();
  IdleI2C();
}

/**************************************************************************/
/*!
    @brief write number of bytes to specific register
    @param dev the device to write to
    @param reg the register as single byte on the device to write to
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

  // wait for slave to acknowledge
  if ( I2CxSTATbits.ACKSTAT != I2C_ACK )
    return;

  // send bytes
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

  // done with transmissions
  StopI2C();
  IdleI2C();
}


