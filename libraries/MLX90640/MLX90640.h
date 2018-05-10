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

/**************************************************************************/
/*!
    @brief  MLX90640 registers
*/
/**************************************************************************/
#define MLX90640_REGISTER_STATUS       (0x00)
#define MLX90640_REGISTER_PRESSURE_MSB (0x01)
#define MLX90640_REGISTER_PRESSURE_CSB (0x02)
#define MLX90640_REGISTER_PRESSURE_LSB (0x03)
#define MLX90640_REGISTER_TEMP_MSB     (0x04)
#define MLX90640_REGISTER_TEMP_LSB     (0x05)
#define MLX90640_REGISTER_DR_STATUS    (0x06)
#define MLX90640_OUT_P_DELTA_MSB       (0x07)
#define MLX90640_OUT_P_DELTA_CSB       (0x08)
#define MLX90640_OUT_P_DELTA_LSB       (0x09)
#define MLX90640_OUT_T_DELTA_MSB       (0x0A)
#define MLX90640_OUT_T_DELTA_LSB       (0x0B)
#define MLX90640_WHOAMI                (0x0C)
#define MLX90640_BAR_IN_MSB            (0x14)
#define MLX90640_BAR_IN_LSB            (0x15)


/**************************************************************************/
/*!
    @brief  MLX90640 status register bits
*/
/**************************************************************************/
#define MLX90640_REGISTER_STATUS_TDR    0x02
#define MLX90640_REGISTER_STATUS_PDR    0x04
#define MLX90640_REGISTER_STATUS_PTDR   0x08


/**************************************************************************/
/*!
    @brief  MLX90640 PT DATA register bits
*/
/**************************************************************************/
#define MLX90640_PT_DATA_CFG       0x13
#define MLX90640_PT_DATA_CFG_TDEFE 0x01
#define MLX90640_PT_DATA_CFG_PDEFE 0x02
#define MLX90640_PT_DATA_CFG_DREM  0x04


/**************************************************************************/
/*!
    @brief  MLX90640 control registers
*/
/**************************************************************************/
#define MLX90640_CTRL_REG1  (0x26)
#define MLX90640_CTRL_REG2  (0x27)
#define MLX90640_CTRL_REG3  (0x28)
#define MLX90640_CTRL_REG4  (0x29)
#define MLX90640_CTRL_REG5  (0x2A)


/**************************************************************************/
/*!
    @brief  MLX90640 control register bits
*/
/**************************************************************************/
#define MLX90640_CTRL_REG1_SBYB 0x01
#define MLX90640_CTRL_REG1_OST  0x02
#define MLX90640_CTRL_REG1_RST  0x04
#define MLX90640_CTRL_REG1_RAW  0x40
#define MLX90640_CTRL_REG1_ALT  0x80
#define MLX90640_CTRL_REG1_BAR  0x00


/**************************************************************************/
/*!
    @brief  MLX90640 oversample values
*/
/**************************************************************************/
#define MLX90640_CTRL_REG1_OS1    0x00
#define MLX90640_CTRL_REG1_OS2    0x08
#define MLX90640_CTRL_REG1_OS4    0x10
#define MLX90640_CTRL_REG1_OS8    0x18
#define MLX90640_CTRL_REG1_OS16   0x20
#define MLX90640_CTRL_REG1_OS32   0x28
#define MLX90640_CTRL_REG1_OS64   0x30
#define MLX90640_CTRL_REG1_OS128  0x38


#define MLX90640_REGISTER_STARTCONVERSION (0x12) ///< start conversion
/*=========================================================================*/

#define MLX90640_MODE_ALT  (0x01)
#define MLX90640_MODE_P    (0x00)
#define STANDARD_PRESSURE_PA ((int32_t) 101325L)

/**************************************************************************/
/*! 
    @brief  Class that stores state and functions for interacting with MLX90640 IR Array
*/
/**************************************************************************/
typedef struct
{
  int16_t kVdd;
  int16_t vdd25;
  float KvPTAT;
  float KtPTAT;
  uint16_t vPTAT25;
  uint16_t alphaPTAT;
  int16_t gainEE;
  float tgc;
  float cpKv;
  float cpKta;
  uint8_t resolutionEE;
  uint8_t calibrationModeEE;
  float KsTa;
  float ksTo[4];
  int16_t ct[4];
  float alpha[768];
  int16_t offset[768];
  float kta[768];
  float kv[768];
  float cpAlpha[2];
  int16_t cpOffset[2];
  float ilChessC[3];
  uint16_t brokenPixels[5];
  uint16_t outlierPixels[5];
} paramsMLX90640;

class MLX90640
{
  public:
    MLX90640(I2C * i2cx, uint8_t addr);
    uint8_t SetRefreshRate(uint8_t refreshRate);
    uint8_t SetInterleavedMode(void);
    uint8_t SetChessMode(void);
    void    DumpEE(uint16_t * eeData);
    void    ExtractParameters(uint16_t * eeData);
    uint8_t   GetFrameData(uint16_t *frameData);
    uint8_t   GetFrameData_Blocking(uint16_t *frameData);
    float     GetTa(uint16_t *frameData);
    float     GetVdd(uint16_t *frameData);
    void      CalculateTo(uint16_t *frameData, float emissivity, float tr, float *result);

  private:
    I2C * i2c;
    uint8_t address;
    paramsMLX90640 mlx90640;
    void ExtractVDDParameters(uint16_t * eeData);
    void ExtractPTATParameters(uint16_t * eeData);
    void ExtractGainParameters(uint16_t * eeData);
    void ExtractTgcParameters(uint16_t * eeData);
    void ExtractResolutionParameters(uint16_t * eeData);
    void ExtractKsTaParameters(uint16_t * eeData);
    void ExtractKsToParameters(uint16_t * eeData);
    void ExtractAlphaParameters(uint16_t * eeData);
    void ExtractOffsetParameters(uint16_t * eeData);
    void ExtractKtaPixelParameters(uint16_t * eeData);
    void ExtractKvPixelParameters(uint16_t * eeData);
    void ExtractCPParameters(uint16_t * eeData);
    void ExtractCILCParameters(uint16_t * eeData);
    int8_t CheckAdjacentPixels(uint16_t pix1, uint16_t pix2);
    int8_t ExtractDeviatingPixels(uint16_t * eeData);

    uint8_t state;
    uint32_t counter_ms;
};


