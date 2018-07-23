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
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 *
 */
#include "MLX90640.h"

/**************************************************************************/
/*!
    @brief  Instantiates a new MLX90640 class
 */
/**************************************************************************/
MLX90640::MLX90640(I2C * i2cx, uint8_t addr)
{
  i2c = (I2C *) i2cx;
  address = addr;
  state = 0;
  counter_ms = 0;
}

uint8_t MLX90640::SetRefreshRate(uint8_t refreshRate)
{
  uint16_t controlRegister1, data_check;
  uint16_t value = (refreshRate & 0x07)<<7;

  i2c->read(address, (uint16_t)0x800D, (uint16_t)1, (uint16_t*) &controlRegister1, 1);

  value = (controlRegister1 & 0xFC7F) | value;

  i2c->write_word(address, 0x800D, value);

  i2c->read (address, (uint16_t) 0x800D, (uint16_t)1, (uint16_t *)&data_check, 1);

  if (value == data_check)
  {
    switch (refreshRate)
    {
      case 0:
        // 0.5 Hz
        refresh_period_ms = 2000;
        break;

      case 1:
        // 1 Hz
        refresh_period_ms = 1000;
        break;

      case 2:
        // 2 Hz
        refresh_period_ms = 500;
        break;

      case 3:
        // 4 Hz
        refresh_period_ms = 250;
        break;

      case 4:
        // 8 Hz
        refresh_period_ms = 125;
        break;

      case 5:
        // 16 Hz
        refresh_period_ms = 62;
        break;

      case 6:
        // 32 Hz
        refresh_period_ms = 31;
        break;

      case 7:
        // 64 Hz
        refresh_period_ms = 15;
        break;
    }

    // refresh period is 90% of actual
    refresh_period_ms *= 80;
    refresh_period_ms /= 100;
    return 0;
  }

  return 1;
}

uint8_t MLX90640::SetInterleavedMode()
{
  uint16_t controlRegister1, data_check;

  i2c->read(address, 0x800D, 1, (uint16_t *) &controlRegister1, 1);

  uint16_t value = (controlRegister1 & 0xEFFF);

  i2c->write_word(address, 0x800D, value);

  i2c->read (address, (uint16_t) 0x800D, (uint16_t)1, (uint16_t *)&data_check, 1);

  if (value == data_check)
    return 0;

  return 1;
}

uint8_t MLX90640::SetChessMode()
{
  uint16_t controlRegister1, data_check;

  i2c->read(address, 0x800D, 1, (uint16_t *) &controlRegister1, 1);

  uint16_t value = (controlRegister1 | 0x1000);

  i2c->write_word(address, 0x800D, value);

  i2c->read (address, (uint16_t) 0x800D, (uint16_t)1, (uint16_t *)&data_check, 1);

  if (value == data_check)
    return 0;

  return 1;
}

void MLX90640::DumpEE(uint16_t *eeData)
{
  i2c->read(address, 0x2400, 832, (uint16_t *) eeData, 1);
}

void MLX90640::ExtractVDDParameters(uint16_t * eeData)
{
  int16_t kVdd;
  int16_t vdd25;

  kVdd = eeData[0x33];

  kVdd = (eeData[0x33] & 0xFF00) >> 8;
  if(kVdd > 127)
  {
    kVdd = kVdd - 256;
  }
  kVdd = 32 * kVdd;
  vdd25 = eeData[0x33] & 0x00FF;
  vdd25 = ((vdd25 - 256) << 5) - 8192;

  mlx90640.kVdd = kVdd;
  mlx90640.vdd25 = vdd25;
}

void MLX90640::ExtractPTATParameters(uint16_t * eeData)
{
  float KvPTAT;
  float KtPTAT;
  int16_t vPTAT25;
  int16_t alphaPTAT;

  KvPTAT = (eeData[0x32] & 0xFC00) >> 10;
  if(KvPTAT > 31)
  {
    KvPTAT = KvPTAT - 64;
  }
  KvPTAT = KvPTAT/4096;

  KtPTAT = eeData[0x32] & 0x03FF;
  if(KtPTAT > 511)
  {
    KtPTAT = KtPTAT - 1024;
  }
  KtPTAT = KtPTAT/8;

  vPTAT25 = eeData[0x31];

  alphaPTAT = ((eeData[16] & 0xF000) >> 14) + 8;

  mlx90640.KvPTAT = KvPTAT;
  mlx90640.KtPTAT = KtPTAT;
  mlx90640.vPTAT25 = vPTAT25;
  mlx90640.alphaPTAT = alphaPTAT;
}

void MLX90640::ExtractGainParameters(uint16_t * eeData)
{
  int16_t gainEE;
  gainEE = eeData[0x30];
  if(gainEE > 32767)
  {
    gainEE = gainEE - 65536;
  }

  mlx90640.gainEE = gainEE;
}

void MLX90640::ExtractTgcParameters(uint16_t * eeData)
{
  float tgc;
  tgc = eeData[0x3c] & 0x00FF;
  if(tgc > 127.0f)
  {
    tgc = tgc - 256.0f;
  }
  tgc = tgc / 32.0f;

  mlx90640.tgc = tgc;
}

void MLX90640::ExtractResolutionParameters(uint16_t * eeData)
{
  uint8_t resolutionEE;
  resolutionEE = (eeData[56] & 0x3000) >> 12;

  mlx90640.resolutionEE = resolutionEE;
}

//------------------------------------------------------------------------------

void MLX90640::ExtractKsTaParameters(uint16_t * eeData)
{
  float KsTa;
  KsTa = (eeData[0x3c] & 0xFF00) >> 8;
  if(KsTa > 127)
  {
    KsTa = KsTa -256;
  }
  KsTa = KsTa / 8192.0f;

  mlx90640.KsTa = KsTa;
}

//------------------------------------------------------------------------------

void MLX90640::ExtractKsToParameters(uint16_t * eeData)
{
  int32_t KsToScale;
  int8_t  step;

  step = ((eeData[0x3f] & 0x3000) >> 12) * 10;

  mlx90640.ct[0] = -40;
  mlx90640.ct[1] = 0;
  mlx90640.ct[2] = (eeData[0x3f] & 0x00F0) >> 4;
  mlx90640.ct[3] = (eeData[0x3f] & 0x0F00) >> 8;

  mlx90640.ct[2] = mlx90640.ct[2]*step;
  mlx90640.ct[3] = mlx90640.ct[2] + mlx90640.ct[3]*step;

  KsToScale = (eeData[0x3f] & 0x000F) + 8;
  KsToScale = 1 << KsToScale;

  mlx90640.ksTo[0] = eeData[0x3d] & 0x00FF;
  mlx90640.ksTo[1] = (eeData[0x3d] & 0xFF00) >> 8;
  mlx90640.ksTo[2] = eeData[0x3e] & 0x00FF;
  mlx90640.ksTo[3] = (eeData[0x3e] & 0xFF00) >> 8;


  for(int32_t i = 0; i < 4; i++)
  {
    if(mlx90640.ksTo[i] > 127.0f)
    {
      mlx90640.ksTo[i] = mlx90640.ksTo[i] - 256.0f;
    }
    mlx90640.ksTo[i] = mlx90640.ksTo[i] / (float) KsToScale;
  }
}

//------------------------------------------------------------------------------

void MLX90640::ExtractAlphaParameters(uint16_t * eeData)
{
  int32_t accRow[24];
  int32_t accColumn[32];
  int32_t p = 0;
  int32_t alphaRef;
  uint8_t alphaScale;
  uint8_t accRowScale;
  uint8_t accColumnScale;
  uint8_t accRemScale;


  accRemScale    =   eeData[32] & 0x000F;
  accColumnScale = ( eeData[32] & 0x00F0) >> 4;
  accRowScale    = ( eeData[32] & 0x0F00) >> 8;
  alphaScale     = ((eeData[32] & 0xF000) >> 12) + 30;
  alphaRef       = eeData[33];

  for(int32_t i = 0; i < 6; i++)
  {
    p = i * 4;
    accRow[p + 0] = (eeData[34 + i] & 0x000F);
    accRow[p + 1] = (eeData[34 + i] & 0x00F0) >> 4;
    accRow[p + 2] = (eeData[34 + i] & 0x0F00) >> 8;
    accRow[p + 3] = (eeData[34 + i] & 0xF000) >> 12;
  }

  for(int32_t i = 0; i < 24; i++)
  {
    if (accRow[i] > 7)
    {
      accRow[i] = accRow[i] - 16;
    }
  }

  for(int32_t i = 0; i < 8; i++)
  {
    p = i * 4;
    accColumn[p + 0] = (eeData[40 + i] & 0x000F);
    accColumn[p + 1] = (eeData[40 + i] & 0x00F0) >> 4;
    accColumn[p + 2] = (eeData[40 + i] & 0x0F00) >> 8;
    accColumn[p + 3] = (eeData[40 + i] & 0xF000) >> 12;
  }

  for(int32_t i = 0; i < 32; i ++)
  {
    if (accColumn[i] > 7)
    {
      accColumn[i] = accColumn[i] - 16;
    }
  }

  for(int32_t i = 0; i < 24; i++)
  {
    for(int32_t j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      mlx90640.alpha[p] = (eeData[64 + p] & 0x03F0) >> 4;
      if (mlx90640.alpha[p] > 31)
      {
        mlx90640.alpha[p] = mlx90640.alpha[p] - 64;
      }
      mlx90640.alpha[p] = mlx90640.alpha[p] * (float)(1<<accRemScale);
      mlx90640.alpha[p] = (alphaRef + accRow[i] * (float)(1<<accRowScale)
          + accColumn[j] * (float)(1<< accColumnScale) + mlx90640.alpha[p]);
      mlx90640.alpha[p] = mlx90640.alpha[p] / powf(2.0f,(float)alphaScale);
    }
  }
}

//------------------------------------------------------------------------------

void MLX90640::ExtractOffsetParameters(uint16_t * eeData)
{
  int32_t occRow[24];
  int32_t occColumn[32];
  int32_t p = 0;
  int16_t offsetRef;
  uint8_t occRowScale;
  uint8_t occColumnScale;
  uint8_t occRemScale;


  occRemScale = (eeData[16] & 0x000F);
  occColumnScale = (eeData[16] & 0x00F0) >> 4;
  occRowScale = (eeData[16] & 0x0F00) >> 8;
  offsetRef = eeData[17];
  if (offsetRef > 32767)
  {
    offsetRef = offsetRef - 65536;
  }

  for(int32_t i = 0; i < 6; i++)
  {
    p = i * 4;
    occRow[p + 0] = (eeData[18 + i] & 0x000F);
    occRow[p + 1] = (eeData[18 + i] & 0x00F0) >> 4;
    occRow[p + 2] = (eeData[18 + i] & 0x0F00) >> 8;
    occRow[p + 3] = (eeData[18 + i] & 0xF000) >> 12;
  }

  for(int32_t i = 0; i < 24; i++)
  {
    if (occRow[i] > 7)
    {
      occRow[i] = occRow[i] - 16;
    }
  }

  for(int32_t i = 0; i < 8; i++)
  {
    p = i * 4;
    occColumn[p + 0] = (eeData[24 + i] & 0x000F);
    occColumn[p + 1] = (eeData[24 + i] & 0x00F0) >> 4;
    occColumn[p + 2] = (eeData[24 + i] & 0x0F00) >> 8;
    occColumn[p + 3] = (eeData[24 + i] & 0xF000) >> 12;
  }

  for(int32_t i = 0; i < 32; i ++)
  {
    if (occColumn[i] > 7)
    {
      occColumn[i] = occColumn[i] - 16;
    }
  }

  for(int32_t i = 0; i < 24; i++)
  {
    for(int32_t j = 0; j < 32; j ++)
    {
      p = 32 * i +j;
//       mlx90640.offset[p] = 0;
      mlx90640.offset[p] = (eeData[64 + p] & 0xFC00) >> 10;
      if (mlx90640.offset[p] > 31)
      {
        mlx90640.offset[p] = mlx90640.offset[p] - 64;
      }
      mlx90640.offset[p] = mlx90640.offset[p] * (float)(1<<occRemScale);
      mlx90640.offset[p] = (offsetRef + occRow[i] * (float) (1<< occRowScale)
          + occColumn[j] * (float) (1<< occColumnScale) + mlx90640.offset[p]);
    }
  }
}

//------------------------------------------------------------------------------

void MLX90640::ExtractKtaPixelParameters(uint16_t * eeData)
{
  int p = 0;
  int8_t KtaRC[4];
  int8_t KtaRoCo;
  int8_t KtaRoCe;
  int8_t KtaReCo;
  int8_t KtaReCe;
  uint8_t ktaScale1;
  uint8_t ktaScale2;
  uint8_t split;

  // row = i+1 is odd [i is even]
  // col = j+1 is odd [j is even]
  KtaRoCo = (eeData[54] & 0xFF00) >> 8;
  if (KtaRoCo > 127)
  {
    KtaRoCo = KtaRoCo - 256;
  }
  KtaRC[0] = KtaRoCo;

  // row = i+1 is even [i is odd]
  // col = j+1 is odd  [j is even]
  KtaReCo = (eeData[54] & 0x00FF);
  if (KtaReCo > 127)
  {
    KtaReCo = KtaReCo - 256;
  }
  KtaRC[2] = KtaReCo;

  // row = i+1 is odd  [i is even]
  // col = j+1 is even [j is odd]
  KtaRoCe = (eeData[55] & 0xFF00) >> 8;
  if (KtaRoCe > 127)
  {
    KtaRoCe = KtaRoCe - 256;
  }
  KtaRC[1] = KtaRoCe;

  // row = i+1 is even [i is odd]
  // col = j+1 is even [j is odd]
  KtaReCe = (eeData[55] & 0x00FF);
  if (KtaReCe > 127)
  {
    KtaReCe = KtaReCe - 256;
  }
  KtaRC[3] = KtaReCe;

  ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
  ktaScale2 = (eeData[56] & 0x000F);

  for(int32_t i = 0; i < 24; i++)
  {
    for(int32_t j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      split = 2*(i & 0x01) + (j & 0x01);
//       mlx90640.kta[p] = 0;
      mlx90640.kta[p] = (eeData[64 + p] & 0x000E) >> 1;
      if (mlx90640.kta[p] > 3)
      {
        mlx90640.kta[p] = mlx90640.kta[p] - 8;
      }
      mlx90640.kta[p] = mlx90640.kta[p] * (float) (1<<ktaScale2);
      mlx90640.kta[p] = KtaRC[split] + mlx90640.kta[p];
      mlx90640.kta[p] = mlx90640.kta[p] / (float) (1<<ktaScale1);
    }
  }
}

//------------------------------------------------------------------------------

void MLX90640::ExtractKvPixelParameters(uint16_t * eeData)
{
  int p = 0;
  int8_t KvT[4];
  int8_t KvRoCo;
  int8_t KvRoCe;
  int8_t KvReCo;
  int8_t KvReCe;
  uint8_t kvScale;
  uint8_t split;

  // row = i+1 is odd [i is even]
  // col = j+1 is odd [j is even]
  KvRoCo = (eeData[52] & 0xF000) >> 12;
  if (KvRoCo > 7)
  {
    KvRoCo = KvRoCo - 16;
  }
  KvT[0] = KvRoCo;

  // row = i+1 is even [i is odd]
  // col = j+1 is odd  [j is even]
  KvReCo = (eeData[52] & 0x0F00) >> 8;
  if (KvReCo > 7)
  {
    KvReCo = KvReCo - 16;
  }
  KvT[2] = KvReCo;

  // row = i+1 is odd  [i is even]
  // col = j+1 is even [j is odd]
  KvRoCe = (eeData[52] & 0x00F0) >> 4;
  if (KvRoCe > 7)
  {
    KvRoCe = KvRoCe - 16;
  }
  KvT[1] = KvRoCe;

  // row = i+1 is even [i is odd]
  // col = j+1 is even [j is odd]
  KvReCe = (eeData[52] & 0x000F);
  if (KvReCe > 7)
  {
    KvReCe = KvReCe - 16;
  }
  KvT[3] = KvReCe;

  kvScale = (eeData[56] & 0x0F00) >> 8;


  for(int32_t i = 0; i < 24; i++)
  {
    for(int32_t j = 0; j < 32; j ++)
    {
      p = 32 * i + j;
      split = 2*(i & 0x01) + (j & 0x01);
//       mlx90640.kv[p] = 0;
      mlx90640.kv[p] = KvT[split];
      mlx90640.kv[p] = mlx90640.kv[p] / (float) (1<<kvScale);
    }
  }
}

//------------------------------------------------------------------------------

void MLX90640::ExtractCPParameters(uint16_t * eeData)
{
  float alphaSP[2];
  int16_t offsetSP[2];
  float cpKv;
  float cpKta;
  uint8_t alphaScale;
  uint8_t ktaScale1;
  uint8_t kvScale;

  alphaScale = ((eeData[32] & 0xF000) >> 12) + 27;

  offsetSP[0] = (eeData[58] & 0x03FF);
  if (offsetSP[0] > 511)
  {
    offsetSP[0] = offsetSP[0] - 1024;
  }

  offsetSP[1] = (eeData[58] & 0xFC00) >> 10;
  if (offsetSP[1] > 31)
  {
    offsetSP[1] = offsetSP[1] - 64;
  }
  offsetSP[1] = offsetSP[1] + offsetSP[0];

  alphaSP[0] = (eeData[57] & 0x03FF);
  if (alphaSP[0] > 511.0f)
  {
    alphaSP[0] = alphaSP[0] - 1024.0f;
  }
  alphaSP[0] = alphaSP[0] /  powf(2.0f,(float)alphaScale);

  alphaSP[1] = (eeData[57] & 0xFC00) >> 10;
  if (alphaSP[1] > 31.0f)
  {
    alphaSP[1] = alphaSP[1] - 64.0f;
  }
  alphaSP[1] = (1.0f + alphaSP[1]/128.0f) * alphaSP[0];

  cpKta = (eeData[59] & 0x00FF);
  if (cpKta > 127.0f)
  {
    cpKta = cpKta - 256.0f;
  }
  ktaScale1 = ((eeData[56] & 0x00F0) >> 4) + 8;
  mlx90640.cpKta = cpKta / powf(2.0f,(float)ktaScale1);

  cpKv = (eeData[59] & 0xFF00) >> 8;
  if (cpKv > 127.0f)
  {
    cpKv = cpKv - 256.0f;
  }
  kvScale = (eeData[56] & 0x0F00) >> 8;
  mlx90640.cpKv = cpKv / powf(2.0f,(float)kvScale);

  mlx90640.cpAlpha[0] = alphaSP[0];
  mlx90640.cpAlpha[1] = alphaSP[1];
  mlx90640.cpOffset[0] = offsetSP[0];
  mlx90640.cpOffset[1] = offsetSP[1];
}

//------------------------------------------------------------------------------

void MLX90640::ExtractCILCParameters(uint16_t * eeData)
{
  float ilChessC[3];
  uint8_t calibrationModeEE;

/*  while (1)
  {
    Serial.print(eeData[12],HEX);
    Serial.print("\n");
    delay(500);
  }*/
//   calibrationModeEE = (eeData[10] & 0x0800) >> 4;
//   calibrationModeEE = calibrationModeEE ^ 0x80;
  calibrationModeEE = ((eeData[12] & 0x1000) >> 12);

  ilChessC[0] = (eeData[53] & 0x003F);
  if (ilChessC[0] > 31)
  {
    ilChessC[0] = ilChessC[0] - 64;
  }
  ilChessC[0] = ilChessC[0] / 16.0f;

  ilChessC[1] = (eeData[53] & 0x07C0) >> 6;
  if (ilChessC[1] > 15)
  {
    ilChessC[1] = ilChessC[1] - 32;
  }
  ilChessC[1] = ilChessC[1] / 2.0f;

  ilChessC[2] = (eeData[53] & 0xF800) >> 11;
  if (ilChessC[2] > 15)
  {
    ilChessC[2] = ilChessC[2] - 32;
  }
  ilChessC[2] = ilChessC[2] / 8.0f;

  mlx90640.calibrationModeEE = calibrationModeEE;
  mlx90640.ilChessC[0] = ilChessC[0];
  mlx90640.ilChessC[1] = ilChessC[1];
  mlx90640.ilChessC[2] = ilChessC[2];
}

int8_t MLX90640::CheckAdjacentPixels(uint16_t pix1, uint16_t pix2)
{
  int32_t pixPosDif;

  pixPosDif = pix1 - pix2;
  if(pixPosDif > -34 && pixPosDif < -30)
  {
    return -6;
  }
  if(pixPosDif > -2 && pixPosDif < 2)
  {
    return -6;
  }
  if(pixPosDif > 30 && pixPosDif < 34)
  {
    return -6;
  }
  return 0;
}

int8_t MLX90640::ExtractDeviatingPixels(uint16_t * eeData)
{
  uint16_t pixCnt = 0;
  uint16_t brokenPixCnt = 0;
  uint16_t outlierPixCnt = 0;
  int warn = 0;
  int i;

  for(pixCnt = 0; pixCnt<5; pixCnt++)
  {
    mlx90640.brokenPixels[pixCnt] = 0xFFFF;
    mlx90640.outlierPixels[pixCnt] = 0xFFFF;
  }

  pixCnt = 0;
  while (pixCnt < 768 && brokenPixCnt < 5 && outlierPixCnt < 5)
  {
    if(eeData[pixCnt+64] == 0)
    {
      mlx90640.brokenPixels[brokenPixCnt] = pixCnt;
      brokenPixCnt = brokenPixCnt + 1;
    }
    else if((eeData[pixCnt+64] & 0x0001) != 0)
    {
      mlx90640.outlierPixels[outlierPixCnt] = pixCnt;
      outlierPixCnt = outlierPixCnt + 1;
    }

    pixCnt = pixCnt + 1;

  }

  if(brokenPixCnt > 4)
  {
    warn = -3;
  }
  else if(outlierPixCnt > 4)
  {
    warn = -4;
  }
  else if((brokenPixCnt + outlierPixCnt) > 4)
  {
    warn = -5;
  }
  else
  {
    for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
    {
      for(i=pixCnt+1; i<brokenPixCnt; i++)
      {
        warn = CheckAdjacentPixels(mlx90640.brokenPixels[pixCnt],mlx90640.brokenPixels[i]);
        if(warn != 0)
        {
          return warn;
        }
      }
    }

    for(pixCnt=0; pixCnt<outlierPixCnt; pixCnt++)
    {
      for(i=pixCnt+1; i<outlierPixCnt; i++)
      {
        warn = CheckAdjacentPixels(mlx90640.outlierPixels[pixCnt],mlx90640.outlierPixels[i]);
        if(warn != 0)
        {
          return warn;
        }
      }
    }

    for(pixCnt=0; pixCnt<brokenPixCnt; pixCnt++)
    {
      for(i=0; i<outlierPixCnt; i++)
      {
        warn = CheckAdjacentPixels(mlx90640.brokenPixels[pixCnt],mlx90640.outlierPixels[i]);
        if(warn != 0)
        {
          return warn;
        }
      }
    }

  }


  return warn;

}

void MLX90640::ExtractParameters( uint16_t * eeData )
{
  ExtractVDDParameters(eeData);
  ExtractPTATParameters(eeData);
  ExtractGainParameters(eeData);
  ExtractTgcParameters(eeData);
  ExtractResolutionParameters(eeData);
  ExtractKsTaParameters(eeData);
  ExtractKsToParameters(eeData);
  ExtractAlphaParameters(eeData);
  ExtractOffsetParameters(eeData);
  ExtractKtaPixelParameters(eeData);
  ExtractKvPixelParameters(eeData);
  ExtractCPParameters(eeData);
  ExtractCILCParameters(eeData);
  ExtractDeviatingPixels(eeData);
}


uint8_t MLX90640::GetFrameData(uint16_t *frameData)
{
  uint16_t dataReady=1;
  uint16_t controlRegister1;
  uint16_t statusRegister;
  uint32_t time_now=millis();
  uint32_t dt = DELTA(time_now,counter_ms);

  if (state == 0)
  {
    state = 1;
    counter_ms = time_now;
    return 0;
  }

  if (state == 1)
  {
    if ( dt >= refresh_period_ms )
    {
      state = 2;
      counter_ms = time_now;
    }
    return 0;
  }

  // once a millisecond querry the sensor if it has completed
  if ( dt < 1 )
    return 0;
  counter_ms = time_now;
  i2c->read(address, 0x8000, 1, (uint16_t *) &statusRegister, 1);
  dataReady = statusRegister & 0x0008;
  if (!dataReady)
    return 0;

  // request new data
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->write_word(address, 0x8000, 0x0030);
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->read(address, 0x0400, 832, (uint16_t *) frameData, 1);
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->read(address, 0x8000, 1, (uint16_t *) &statusRegister, 1);
  dataReady = statusRegister & 0x0008;
  if (dataReady)
    return 0;

  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->delay_1us();
  i2c->read(address, 0x800D, 1, (uint16_t *) &controlRegister1, 1);
  frameData[832] = controlRegister1;
  frameData[833] = statusRegister & 0x0007;
  state = 0;
  return 1;

}

uint8_t MLX90640::GetFrameData_Blocking(uint16_t *frameData)
{
/*  uint16_t dataReady = 1;
  uint16_t controlRegister1;
  uint16_t statusRegister;
  uint8_t  cnt = 0;

  if (state == 0)
  {
    counter_ms = millis();
    state = 1;
    return 0;
  }

  if ( millis() - counter_ms < 1 )
    return 0;
  i2c->read(address, 0x8000, 1, (uint16_t *) &statusRegister, 1);
  counter_ms = millis();
  dataReady = statusRegister & 0x0008;
  if (!dataReady)
    return 0;

  for (cnt=0; cnt<4; cnt++)
  {
    i2c->write_word(address, 0x8000, 0x0030);
  }
  i2c->read(address, 0x0400, 832, frameData, 1);
  i2c->read(address, 0x8000, 1, &statusRegister, 1);
  dataReady = statusRegister & 0x0008;
  i2c->read(address, 0x800D, 1, &controlRegister1, 1);
  frameData[832] = controlRegister1;
  frameData[833] = statusRegister & 0x0007;
  state = 0;
  return 1;*/
}

float MLX90640::GetVdd(uint16_t *frameData)
{
  float vdd;
  float resolutionCorrection;
  int32_t resolutionRAM;

  vdd = frameData[810];
  if(vdd > 32767.0f)
  {
    vdd = vdd - 65536.0f;
  }
  resolutionRAM = (frameData[832] & 0x0C00) >> 10;
  resolutionCorrection = powf(2.0f, (float)mlx90640.resolutionEE) / powf(2.0f, (float)resolutionRAM);
  vdd = (resolutionCorrection * vdd - mlx90640.vdd25) / mlx90640.kVdd + 3.3f;
  return vdd;
}

float MLX90640::GetTa(uint16_t *frameData)
{
  float ptat;
  float ptatArt;
  float vdd;
  float ta;

  vdd = GetVdd(frameData);

  ptat = frameData[800];
  if(ptat > 32767.0f)
  {
    ptat = ptat - 65536.0f;
  }

  ptatArt = frameData[768];
  if(ptatArt > 32767.0f)
  {
    ptatArt = ptatArt - 65536.0f;
  }
  ptatArt = (ptat / (ptat * mlx90640.alphaPTAT + ptatArt)) * 262144.0f;

  ta = (ptatArt / (1.0f + mlx90640.KvPTAT * (vdd - 3.3f)) - mlx90640.vPTAT25);
  ta = ta / mlx90640.KtPTAT + 25.0f;

  return ta;
}

void MLX90640::CalculateTo(uint16_t *frameData, float emissivity, float tr, float *result)
{
  float vdd;
  float ta;
  float ta4;
  float tr4;
  float taTr;
  float gain;
  float irDataCP[2];
  float irData;
  float alphaCompensated;
  uint8_t mode;
  int8_t ilPattern;
  int8_t chessPattern;
  int8_t pattern;
  int8_t conversionPattern;
  float Sx;
  float To;
  float alphaCorrR[4];
  int8_t range;


  vdd = GetVdd(frameData);
  ta = GetTa(frameData);

  //math: ta4 = powf((ta + 273.15f), 4.0f);
  ta4  = ta + 273.15f;
  ta4 *= ta4;
  ta4 *= ta4;

  //math: tr4 = powf((tr + 273.15f), 4.0f);
  tr4  = tr + 273.15f;
  tr4 *= tr4;
  tr4 *= tr4;

  taTr = tr4 - (tr4-ta4)/emissivity;

  alphaCorrR[0] = 1.0f / (1.0f + mlx90640.ksTo[0] * 40.0f);
  alphaCorrR[1] = 1.0f ;
  alphaCorrR[2] = (1.0f + mlx90640.ksTo[2] * mlx90640.ct[2]);
  alphaCorrR[3] = alphaCorrR[2] * (1 + mlx90640.ksTo[3] * (mlx90640.ct[3] - mlx90640.ct[2]));

//------------------------- Gain calculation -----------------------------------
  gain = frameData[778];
  if(gain > 32767)
  {
    gain = gain - 65536.0f;
  }

  gain = mlx90640.gainEE / gain;

//------------------------- To calculation -------------------------------------
  mode = (frameData[832] & 0x1000) >> 12;

  irDataCP[0] = frameData[776];
  irDataCP[1] = frameData[808];
  for( int i = 0; i < 2; i++)
  {
    if(irDataCP[i] > 32767)
    {
      irDataCP[i] = irDataCP[i] - 65536;
    }
    irDataCP[i] = irDataCP[i] * gain;
  }
  irDataCP[0] = irDataCP[0] - mlx90640.cpOffset[0] * (1.0f + mlx90640.cpKta * (ta - 25.0f))
      * (1.0f + mlx90640.cpKv * (vdd - 3.3f));

  if( mode ==  mlx90640.calibrationModeEE)
  {
    irDataCP[1] = irDataCP[1] - mlx90640.cpOffset[1]
        * (1.0f + mlx90640.cpKta * (ta - 25.0f))
        * (1.0f + mlx90640.cpKv * (vdd - 3.3f));
  }
  else
  {
    irDataCP[1] = irDataCP[1] - (mlx90640.cpOffset[1] + mlx90640.ilChessC[0])
        * (1.0f + mlx90640.cpKta * (ta - 25.0f))
        * (1.0f + mlx90640.cpKv * (vdd - 3.3f));
  }

  for(int32_t pixelNumber = 0; pixelNumber < 768; pixelNumber++)
  {
//     ilPattern = int((pixelNumber) / 32) - int(int((pixelNumber) / 32) / 2) * 2;
//     chessPattern = ilPattern ^ (pixelNumber - int(pixelNumber/2)*2);
    ilPattern = (pixelNumber & 0x20) >> 5;
    chessPattern = ilPattern ^ (pixelNumber & 0x01);

//     conversionPattern = (int((pixelNumber - 2) / 4) - int((pixelNumber - 1) / 4)
//     + int((pixelNumber + 1) / 4) - int((pixelNumber) / 4)) * (1 - 2 * ilPattern);
    conversionPattern = ( ((pixelNumber + 2)>>2) - ((pixelNumber + 3)>>2)
        + ((pixelNumber + 1)>>2) - (pixelNumber>>2) ) * (1 - ilPattern<<1);

    if(mode == 0)
    {
      pattern = ilPattern;
    }
    else
    {
      pattern = chessPattern;
    }

    if(pattern == frameData[833])
    {
      irData = frameData[pixelNumber];
      if(irData > 32767)
      {
        irData = irData - 65536;
      }
      irData = irData * gain;

      irData = irData - mlx90640.offset[pixelNumber]*(1 + mlx90640.kta[pixelNumber]*(ta - 25.0f))
          *(1.0f + mlx90640.kv[pixelNumber]*(vdd - 3.3f));

      if(mode !=  mlx90640.calibrationModeEE)
      {
        irData = irData + mlx90640.ilChessC[2] * (2 * ilPattern - 1) - mlx90640.ilChessC[1] * conversionPattern;
      }

      irData = irData / emissivity;

      irData = irData - mlx90640.tgc * ((1-pattern) * irDataCP[0] + pattern * irDataCP[1]);

      alphaCompensated = ( mlx90640.alpha[pixelNumber]
          - mlx90640.tgc * ((1-pattern) * mlx90640.cpAlpha[0] + pattern * mlx90640.cpAlpha[1]))
          * (1 + mlx90640.KsTa * (ta - 25) );

      Sx = powf((float)alphaCompensated, 3.0f) * (irData + alphaCompensated * taTr);
      Sx = sqrtf(sqrtf(Sx)) * mlx90640.ksTo[1];

      To = sqrtf(sqrtf(irData/(alphaCompensated * (1.0f - mlx90640.ksTo[1] * 273.15f) + Sx) + taTr)) - 273.15f;

      if(To < mlx90640.ct[1])
      {
        range = 0;
      }
      else if(To < mlx90640.ct[2])
      {
        range = 1;
      }
      else if(To < mlx90640.ct[3])
      {
        range = 2;
      }
      else
      {
        range = 3;
      }

      To = sqrtf(sqrtf(irData / (alphaCompensated * alphaCorrR[range] * (1 + mlx90640.ksTo[range] * (To - mlx90640.ct[range]))) + taTr));
      result[pixelNumber] = To;
    }
  }
}


