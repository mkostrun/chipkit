#include "CCS811.h"

/**************************************************************************/
/*! 
    @brief  trigger a software reset of the device
 */
/**************************************************************************/
void CCS811::SWReset()
{
  //reset sequence from the datasheet
  uint8_t seq[] = {0x11, 0xE5, 0x72, 0x8A};
  i2c->write(_i2caddr, CCS811_SW_RESET, 4, seq);
}

/**************************************************************************/
/*! 
    @brief  Setups the I2C interface and hardware and checks for communication.
    @param  addr Optional I2C address the sensor can be found on. Default is 0x5A
    @returns True if device is set up, false on any failure
*/
/**************************************************************************/
bool CCS811::begin(void)
{
  uint16_t rval=0x00;

  // Serial.print("Reset");
  SWReset();
  delay(100);

  //check that the HW id is correct
  i2c->read(_i2caddr,CCS811_HW_ID,0x0002, (uint8_t *) &rval, 0x01);
  rval &= 0xff;  
  if(rval != CCS811_HW_ID_CODE)
  {
    return false;
  }

  //try to start the app
  i2c->write(_i2caddr,CCS811_BOOTLOADER_APP_START,0,NULL);
  delay(100);
  
  //make sure there are no errors and we have entered application mode
  if(checkError())
    return false;

  disableInterrupt();

  if(!_status.FW_MODE)
    return false;

  //default to read every second
  setDriveMode(CCS811_DRIVE_MODE_1SEC);

  return true;
}

/**************************************************************************/
/*! 
    @brief  sample rate of the sensor.
    @param  mode one of CCS811_DRIVE_MODE_IDLE, CCS811_DRIVE_MODE_1SEC, CCS811_DRIVE_MODE_10SEC, CCS811_DRIVE_MODE_60SEC, CCS811_DRIVE_MODE_250MS.
*/
void CCS811::setDriveMode(uint8_t mode)
{
	_meas_mode.DRIVE_MODE = mode;
  i2c->write_byte(_i2caddr,CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  enable the data ready interrupt pin on the device.
*/
/**************************************************************************/
void CCS811::enableInterrupt()
{
	_meas_mode.INT_DATARDY = 1;
  i2c->write_byte(_i2caddr,CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  disable the data ready interrupt pin on the device
*/
/**************************************************************************/
void CCS811::disableInterrupt()
{
	_meas_mode.INT_DATARDY = 0;
  i2c->write_byte(_i2caddr, CCS811_MEAS_MODE, _meas_mode.get());
}

/**************************************************************************/
/*! 
    @brief  checks if data is available to be read.
    @returns True if data is ready, false otherwise.
*/
/**************************************************************************/
bool CCS811::dataReady()
{
  if (millis()-time_call_ms<6)
    return false;
  
  time_call_ms = millis();

  uint32_t rval=0;
  uint8_t *data = (uint8_t *) & rval;

  i2c->write_reg(_i2caddr,CCS811_STATUS);
  i2c->read(_i2caddr,0x0004, (uint8_t *) &rval, 0x01);

  // i2c->read(_i2caddr,CCS811_STATUS,0x0002, (uint8_t *) &rval, 0x01);
  _status.set( *data );
  
  if(!_status.ERROR)
  {
    if(_status.DATA_READY)
    {      
      readNTC();
      readData();
      return true;  
    }    
  }
    
  return false;  
}

/**************************************************************************/
/*! 
    @brief  calculate the temperature using the onboard NTC resistor.
    @returns temperature as a double.
*/
/**************************************************************************/
void CCS811::readNTC()
{
  uint8_t buf[5];
  uint32_t vref=0, vntc=0;
  uint8_t * r = (uint8_t *) &vref;
  uint8_t * n = (uint8_t *) &vntc;
  
  i2c->read(_i2caddr,CCS811_NTC, 5, buf, 1);
  r[0] = buf[1];
  r[1] = buf[0];
  n[0] = buf[3];
  n[1] = buf[2];
  
  //from ams ccs811 app note
  uint32_t rntc = vntc * CCS811_REF_RESISTOR / vref;  
  
  ntc_temp_C  = logf((float) rntc / (float) CCS811_REF_RESISTOR); // 1
  ntc_temp_C /= 3380.0f; // 2
  ntc_temp_C += 1.0f / (25.0f + 273.15f); // 3
  ntc_temp_C  = 1.0f / ntc_temp_C; // 4
  ntc_temp_C -= 273.15f; // 5
  // return (ntc_temp_C - _tempOffset);
  return;
}

/**************************************************************************/
/*! 
    @brief  read and store the sensor data. This data can be accessed with getTVOC() and geteCO2()
    @returns 0 if no error, error code otherwise.
*/
/**************************************************************************/
void CCS811::readData()
{
  uint8_t buf[6];
  uint8_t * e = (uint8_t *) &eCO2;
  uint8_t * t = (uint8_t *) &TVOC;
  i2c->read(_i2caddr,CCS811_ALG_RESULT_DATA, 6, buf, 1);
  e[0] = buf[1];
  e[1] = buf[0];
  t[0] = buf[3];
  t[1] = buf[2];
  
  _status.set( buf[4] );  
  
  return;
}

/**************************************************************************/
/*! 
    @brief  set the humidity and temperature compensation for the sensor.
    @param humidity the humidity data as a percentage. For 55% humidity, pass in integer 55.
    @param temperature the temperature in degrees C as a decimal number. For 25.5 degrees C, 
    pass in 25.5
*/
/**************************************************************************/
void CCS811::setEnvironmentalData(float humidity, float temp)
{
  float d;
  uint16_t h = 0x0000;
  uint8_t  *h_ptr = (uint8_t  *) &h;
  uint16_t t = 0x0000;
  uint8_t  *t_ptr = (uint8_t  *) &t;

  
  /* Humidity is stored as an unsigned 16 bits in 1/512%RH. The
  default value is 50% = 0x64, 0x00. As an example 48.5%
  humidity would be 0x61, 0x00.*/
  d = 512.f * humidity;
  h = (uint16_t) d;

  /* Temperature is stored as an unsigned 16 bits integer in 1/512
  degrees; there is an offset: 0 maps to -25°C. The default value is
  25°C = 0x64, 0x00. As an example 23.5% temperature would be
  0x61, 0x00.
  The internal algorithm uses these values (or default values if
  not set by the application) to compensate for changes in
  relative humidity and ambient temperature.*/
  d = 512.0f * (temp - 25.0f);
  t = (uint16_t) d;
  
  uint8_t buf[] = { h_ptr[1], h_ptr[0], t_ptr[1], t_ptr[0]};
  i2c->write(_i2caddr, CCS811_ENV_DATA, 4, buf);
}

/**************************************************************************/
/*! 
    @brief  set interrupt thresholds
    @param low_med the level below which an interrupt will be triggered.
    @param med_high the level above which the interrupt will ge triggered.
    @param hysteresis optional histeresis level. Defaults to 50
*/
/**************************************************************************/
void CCS811::setThresholds(uint16_t low_med, uint16_t med_high, uint8_t hysteresis)
{
  uint8_t *lo_ptr = (uint8_t  *) &low_med;
  uint8_t *hi_ptr = (uint8_t  *) &med_high;
	uint8_t buf[] = 
  {
    lo_ptr[1],
    lo_ptr[0],
    hi_ptr[1],
    hi_ptr[0],
    hysteresis
  };

  i2c->write(_i2caddr, CCS811_THRESHOLDS, 5, buf);
}


/**************************************************************************/
/*! 
    @brief   read the status register and store any errors.
    @returns the error bits from the status register of the device.
*/
/**************************************************************************/
bool CCS811::checkError()
{
  uint8_t r[2];

  i2c->read(_i2caddr,CCS811_STATUS,0x0002, (uint8_t *) r, 0x01);
  _status.set( *r );
	return _status.ERROR;
}

