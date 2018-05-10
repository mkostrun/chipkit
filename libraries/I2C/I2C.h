/*!
 * @file I2C.h
 *
 * This is I2C driver for the Digilent UNO32 that supports repeated start on MPIDE
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Written by Marijan Kostrun.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#ifndef _CHIPKIT_I2C_H_
#define _CHIPKIT_I2C_H_

#include "WProgram.h"
#include <plib.h>

#if defined(__32MX320F128H__) || defined(__32MX795F512L__)
  // UNO32 or MAX32
  #define StartI2C        StartI2C1
  #define RestartI2C      RestartI2C1
  #define StopI2C         StopI2C1
  #define IdleI2C         IdleI2C1
  #define MasterWriteI2C  MasterWriteI2C1
  #define MasterReadI2C   MasterReadI2C1
  #define I2Cx            I2C1
  #define I2CxSTATbits    I2C1STATbits
  #define I2CxCONbits     I2C1CONbits
  #define I2CxBRG         I2C1BRG
#elif defined(__32MX695F512L__)   // WF32
  #define StartI2C        StartI2C2
  #define RestartI2C      RestartI2C2
  #define StopI2C         StopI2C2
  #define IdleI2C         IdleI2C2
  #define MasterWriteI2C  MasterWriteI2C2
  #define MasterReadI2C   MasterReadI2C2
  #define I2Cx            I2C2
  #define I2CxSTATbits    I2C2STATbits
  #define I2CxCONbits     I2C2CONbits
  #define I2CxBRG         I2C2BRG
#endif

#ifndef SYSCLK
#define SYSCLK (80000000L)
#endif

#ifndef PBCLK
#define PBCLK (80000000L)
#endif

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for I2C
*/
/**************************************************************************/
class I2C
{
  public:
    I2C(uint32_t freq = 100000);
    void SetFrequency(uint32_t f);
    uint8_t read (uint8_t dev, uint16_t size, uint8_t * data, uint8_t s);
    uint8_t read_no_nack(uint8_t dev, uint16_t size, uint8_t * data, uint8_t s);
    uint8_t read (uint8_t dev, uint8_t reg, uint16_t size, uint8_t * data, uint8_t rs);
    uint8_t read (uint8_t dev, uint16_t reg, uint16_t size, uint16_t * data, uint8_t rs);
    uint8_t read_byte (uint8_t dev, uint8_t reg, uint8_t rs);
    uint8_t read_byte_no_nack(uint8_t dev, uint8_t reg, uint8_t rs);
    void write_reg(uint8_t dev, uint8_t reg);
    void write_byte (uint8_t dev, uint8_t reg, uint8_t data);
    void write_word(uint8_t dev, uint16_t reg, uint16_t data);
    void write (uint8_t dev, uint8_t reg, uint16_t size, uint8_t * data);

  private:
    uint8_t _init;
    uint32_t f;
};


#endif

