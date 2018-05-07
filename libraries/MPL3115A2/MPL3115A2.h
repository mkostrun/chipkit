/*!
 * @file MPL3115A2.h
 *
 * This is part of MPL3115A2 driver for the Digilent UNO32.  It is
 * designed specifically to work with the Adafruit MPL3115A2 breakout:
 * https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Written by Marijan Kostrun.
 * Based/Inspired by the
 *      Adafruit_MPL3115A2.h
 * written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WProgram.h"
#include <plib.h>

#include <I2C.h>
#define MPL3115A2_ADDRESS                       (0x60)    ///< default I2C address 1100000

/**************************************************************************/
/*!
    @brief  MPL3115A2 registers
*/
/**************************************************************************/
#define MPL3115A2_REGISTER_STATUS       (0x00)
#define MPL3115A2_REGISTER_PRESSURE_MSB (0x01)
#define MPL3115A2_REGISTER_PRESSURE_CSB (0x02)
#define MPL3115A2_REGISTER_PRESSURE_LSB (0x03)
#define MPL3115A2_REGISTER_TEMP_MSB     (0x04)
#define MPL3115A2_REGISTER_TEMP_LSB     (0x05)
#define MPL3115A2_REGISTER_DR_STATUS    (0x06)
#define MPL3115A2_OUT_P_DELTA_MSB       (0x07)
#define MPL3115A2_OUT_P_DELTA_CSB       (0x08)
#define MPL3115A2_OUT_P_DELTA_LSB       (0x09)
#define MPL3115A2_OUT_T_DELTA_MSB       (0x0A)
#define MPL3115A2_OUT_T_DELTA_LSB       (0x0B)
#define MPL3115A2_WHOAMI                (0x0C)
#define MPL3115A2_BAR_IN_MSB            (0x14)
#define MPL3115A2_BAR_IN_LSB            (0x15)


/**************************************************************************/
/*!
    @brief  MPL3115A2 status register bits
*/
/**************************************************************************/
#define MPL3115A2_REGISTER_STATUS_TDR    0x02
#define MPL3115A2_REGISTER_STATUS_PDR    0x04
#define MPL3115A2_REGISTER_STATUS_PTDR   0x08


/**************************************************************************/
/*!
    @brief  MPL3115A2 PT DATA register bits
*/
/**************************************************************************/
#define MPL3115A2_PT_DATA_CFG       0x13
#define MPL3115A2_PT_DATA_CFG_TDEFE 0x01
#define MPL3115A2_PT_DATA_CFG_PDEFE 0x02
#define MPL3115A2_PT_DATA_CFG_DREM  0x04


/**************************************************************************/
/*!
    @brief  MPL3115A2 control registers
*/
/**************************************************************************/
#define MPL3115A2_CTRL_REG1  (0x26)
#define MPL3115A2_CTRL_REG2  (0x27)
#define MPL3115A2_CTRL_REG3  (0x28)
#define MPL3115A2_CTRL_REG4  (0x29)
#define MPL3115A2_CTRL_REG5  (0x2A)


/**************************************************************************/
/*!
    @brief  MPL3115A2 control register bits
*/
/**************************************************************************/
#define MPL3115A2_CTRL_REG1_SBYB 0x01
#define MPL3115A2_CTRL_REG1_OST  0x02
#define MPL3115A2_CTRL_REG1_RST  0x04
#define MPL3115A2_CTRL_REG1_RAW  0x40
#define MPL3115A2_CTRL_REG1_ALT  0x80
#define MPL3115A2_CTRL_REG1_BAR  0x00


/**************************************************************************/
/*!
    @brief  MPL3115A2 oversample values
*/
/**************************************************************************/
#define MPL3115A2_CTRL_REG1_OS1    0x00
#define MPL3115A2_CTRL_REG1_OS2    0x08
#define MPL3115A2_CTRL_REG1_OS4    0x10
#define MPL3115A2_CTRL_REG1_OS8    0x18
#define MPL3115A2_CTRL_REG1_OS16   0x20
#define MPL3115A2_CTRL_REG1_OS32   0x28
#define MPL3115A2_CTRL_REG1_OS64   0x30
#define MPL3115A2_CTRL_REG1_OS128  0x38


#define MPL3115A2_REGISTER_STARTCONVERSION (0x12) ///< start conversion
/*=========================================================================*/

#define MPL3115A2_MODE_ALT  (0x01)
#define MPL3115A2_MODE_P    (0x00)
#define STANDARD_PRESSURE_PA ((int32_t) 101325L)

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with MPL3115A2 altimeter
*/
/**************************************************************************/
class MPL3115A2
{
  public:
    MPL3115A2(I2C * i2cx);
    boolean begin(void);
    boolean data_ready( uint8_t alt );
    void setSeaPressure(uint32_t pascal);
    uint32_t pressure_Pa;
    uint32_t altitude_m;
    float    temperature_C;

  private:
    I2C * i2c;

  typedef union
  {
    struct
    {
        uint8_t SBYB:1;
        uint8_t OST:1;
        uint8_t RST:1;
        uint8_t OS:3;
        uint8_t RAW:1;
        uint8_t ALT:1;
    } bit;
    uint8_t reg;
  } ctrl_reg1;

  ctrl_reg1 _ctrl_reg1;
};


