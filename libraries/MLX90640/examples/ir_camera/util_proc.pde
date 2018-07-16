void PrintToBuffer(float *p, uint8_t nl)
{
  uint16_t idx=1,i;
  uint16_t d;
  buffer[0]='S';
  for(i=0; i<768; i++)
  {
#if defined(MLX_USE_OCTAKELVINS)
    d = 8 * *p++;
    sprintf(buffer+idx,"%03x",d);
    idx+=3;
#elif defined(MLX_USE_DEKAKELVINS)
    d = 10 * *p++;
    sprintf(buffer+idx,"%04x",d);
    idx+=4;
#elif defined(MLX_USE_FAHRENHEITS)
    /** F = 9/5 * C + 32
      *   = 9/5 * K - (9/5*273.15 - 32) = 9/5 * K - 459.67
      */
    float dummy = 1.8f * *(p++) - 459.67f;
    if (dummy > 255.0f)
      dummy = 255.0f;
    else if (dummy < 0.0f)
      dummy = 0.0f;
    uint8_t d = (uint8_t) roundf(dummy);
    sprintf(buffer+idx,"%02x",d);
    idx+=2;
#else
    error
#endif
  }
  buffer[idx++] = 'E';
  if (nl)
  {
    buffer[idx++] = '\n';
  }
  buffer[idx++] = 0;
}

uint8_t process_camera_cmd( char c )
{
  uint8_t rval=1;

  if ((c < 0x30)&&(c > 0xf0))
    return 0;

  switch (c)
  {
    case 'r':
      // reboot the camera using the watchdog timer
      WDTCONSET = 0x8000;
      delay(9999);
      break;

    case '0': /* 0.5 Hz frame rate */
    case '1': /*   1 Hz frame rate */
    case '2': /*   2 Hz frame rate */
    case '3': /*   4 Hz frame rate */
    case '4': /*   8 Hz frame rate */
    case '5': /*  16 Hz frame rate */
    case '6': /*  32 Hz frame rate */
    case '7': /*  64 Hz frame rate */
      camera1.SetRefreshRate(c - '0');
      break;

    case 'h':
      camera1.SetChessMode();
      break;

    case 'i':
      camera1.SetInterleavedMode();
      break;

    default:
      rval = 0;
      break;

  } // switch

  return rval;
}


void process_serial_input( void )
{
  if (!Serial.available())
    return;

  uint8_t c = Serial.read();

  if (print_2_serial)
  {
    if (c == 'q')
    {
      print_2_serial = 0;
    }
    return;
  }

  if (process_camera_cmd(c))
    return;

  switch (c)
  {
    case '?':
      Serial.print(FIRMWARE "\n");
     break;

    case 'p':
      print_2_serial = 1;
      break;

    default:
      Serial.print("?\n");
      break;
  }

  return;
}
