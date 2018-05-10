/*!
 * @file SI7021.h
 *
 * This is part of SI7021 driver for the Digilent UNO32.  It is
 * designed specifically to work with the Adafruit SI7021 breakout:
 * https://www.adafruit.com/products/1893
 *
 * These sensors use I2C to communicate, 2 pins (SCL+SDA) are required
 * to interface with the breakout.
 *
 * Written by Marijan Kostrun.
 * Based/Inspired by the
 *      Adafruit_SI7021.h
 * written by Kevin "KTOWN" Townsend for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */

#include "WProgram.h"
#include <plib.h>

#include <I2C.h>

//
// I2C stuff
//
#define SI7021_ADDRESS 0x40

//
// COMMANDS
//
// address
#define SI7021_MEAS_RH_HOLD                           0XE5
#define SI7021_MEAS_RH_NOHOLD                         0XF5
#define SI7021_MEAS_T_HOLD                            0XE3
#define SI7021_MEAS_T_NOHOLD                          0XF3
#define SI7021_READ_T_PREV_RH                         0XE0
#define SI7021_RESET                                  0XFE
#define SI7021_WRITE_RHT_USER_REG1                    0XE6
#define SI7021_READ_RHT_USER_REG1                     0XE7

#define __swap16(x) {uint8_t c = d[0]; d[0] = d[1]; d[1] = c;}


/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with SI7021 altimeter
*/
/**************************************************************************/
class SI7021
{
  public:
    SI7021(I2C * i2cx, uint32_t dt);
    void reset(void);
    uint8_t measure( void );
    float temperature_c, rel_humidity_pc;

  private:
    I2C * i2c;
    uint8_t state;
    uint32_t time_last_call_ms;
    uint32_t dt_between_calls_ms;
    float read_t_prev_meas_rh( void );
};


