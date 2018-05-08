// Power by connecting Vin to 3-5V, GND to GND
// Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
#include <I2C.h>
I2C i2cx = I2C(1000000);

#include <MLX90640.h>
#define TA_SHIFT 8
MLX90640 camera1= MLX90640((I2C*) &i2cx, 0x33);

#define FIRMWARE "ir_camera-mlx90640-ver_1_0"
char buffer[3076];
uint8_t printme=1;
uint16_t * eeData = (uint16_t *) buffer;
uint16_t * pFrame = (uint16_t *) buffer;
float mlx90640To[768];
float eTa;                  //Ta for emissivity compensation
float emissivity = 1.0f;

#define DEBUG

void PrintToU(float *p)
{
  uint16_t idx=1,i;
  uint16_t d;
  buffer[0]='S';
  for(i=0; i<768; i++)
  {
    d = 10 * *p++;
    sprintf(buffer+idx,"%04x",d);
    idx+=4;
  }
  buffer[idx++] = 'E';
  buffer[idx++] = '\n';
  buffer[idx++] = 0;
  Serial.print(buffer);
}

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
  
  if (Serial.available())
  {
    char c = Serial.read();
    if (printme)
    {
      switch (c)
      {
        case 'q':
          printme=0;
          break;
      }
      goto begin_loop;
    }

    switch (c)
    {
      case '0':
      case '1':
      case '3':
      case '4':
      case '5':
      case '6':
      case '7':
        camera1.SetRefreshRate(c - '0');
        break;

      case 'c':
        camera1.SetChessMode();
        break;

      case 'i':
        camera1.SetInterleavedMode();
        break;

      case '?':
        Serial.print(FIRMWARE "\n");
        break;

      case 'p':
        printme=1;
        break;

      default:
        Serial.print("?\n");
        break;

    } // switch

  } // Serial.available()

  if (camera1.GetFrameData(pFrame))
  {
    eTa = camera1.GetTa(pFrame) - TA_SHIFT;
    camera1.CalculateTo(pFrame, emissivity, eTa, mlx90640To);

    if (printme)
    {
      PrintToU(mlx90640To);
    }
  }


} // loop()

