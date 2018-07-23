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

#define  DELTA(u32_x,u32_y)  ( (u32_x) >= (u32_y) ? (u32_x)-(u32_y) : ((0xffffffff - (u32_y)) + (u32_x)) )

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

    uint8_t  state;
    uint32_t refresh_period_ms;
    uint32_t counter_ms;
};


