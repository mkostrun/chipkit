/**************************************************************************/
/*!
    @file     IR_Wireless_Camera_System/ver_1_0
    @author   M. Kostrun
    @license  BSD (see license.txt)

    Example for the MLX90640 IR sensor
    for Digilent/Chipkit development boards

    @section  HISTORY
    v1.0 - First release
 */
/**************************************************************************/

#undef WORK
#undef HOME
#undef DEBUG
#undef MLX_USE_OCTAKELVINS
#undef MLX_USE_DEKAKELVINS
#undef MLX_USE_FAHRENHEITS

// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
#include <I2C.h>
I2C i2cx = I2C(400000);

#include <MLX90640.h>
#define TA_SHIFT 8
MLX90640 camera1= MLX90640((I2C*) &i2cx, 0x33);

#define MLX_USE_FAHRENHEITS
#define FIRMWARE "eye_r_eye-mlx90640_32x24-v_1_0"
char buffer[3076];
uint8_t print_2_serial=1;
uint16_t * eeData = (uint16_t *) buffer;
uint16_t * pFrame = (uint16_t *) buffer;
float mlx90640To[768];
float eTa;                  //Ta for emissivity compensation
float emissivity = 1.0f;

#define DEBUG

void setup(void)
{
  Serial.begin(230400);
  delay(500);

  while (camera1.SetRefreshRate(1))
  {
    Serial.print("Failed to configure 'camera1' refresh rate\n");
    delay(500);
  }
#ifdef DEBUG
  Serial.print("'camera1' refresh rate successfully configured\n");
#endif

  while (camera1.SetInterleavedMode())
//   while (camera1.SetChessMode())
  {
    Serial.print("Failed to configure 'camera1' interleaved mode\n");
    delay(500);
  }
#ifdef DEBUG
  Serial.print("'camera1' successfully configured in interleaved mode\n");
#endif
  camera1.DumpEE(eeData);
  camera1.ExtractParameters(eeData);
}

void loop(void)
{

begin_loop:
  
  if (camera1.GetFrameData(pFrame))
  {
    eTa = camera1.GetTa(pFrame) - TA_SHIFT;
    camera1.CalculateTo(pFrame, emissivity, eTa, mlx90640To);

    if (print_2_serial)
    {
      PrintToBuffer(mlx90640To,0x01);
      Serial.print(buffer);
    }
  }

  
  //
  // parser processing input from serial port
  //
  process_serial_input();

} // loop()

