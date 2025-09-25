#include <HX711.h>
#include "HSU_config.h"
#include "HSU_modbus.h"
#include "HSU_calc.h"
#include "HSU_init.h"

HX711 scale1;
HX711 scale2;
ArduinoLEDMatrix matrix;
ModbusMaster Lnode;
ModbusMaster Rnode;
STATE Device = {0, 0, 0, 0, STOP, STRAIGHT};
int state = 0;

void setup()
{
  init();
  Serial.println("Setup complete");
}

void loop()
{
  Device.Lweight = -scale1.get_units() * 1000;
  Device.Rweight = scale2.get_units() * 1000;
  calculateSpeed(&Device);
  calculateVelocity(&Device);
  DirectionImage(&Device, matrix);
  print_log();
      
  switch (state)
  {
    case 0:
      setControlEnable(Lnode);
      state = 1;
      break;
    case 1:
      if (getControl(Lnode) == CTRL_WORD_ENABLE_SERVO)
      state = 2;
      else {
        Serial.println("Error: Servo 4 ON failed");
        state = 0;
      }
      break;

    case 2:
      setControlEnable(Rnode);
      state = 3;
      break;

    case 3:
      if (getControl(Rnode) == CTRL_WORD_ENABLE_SERVO)
        state = 4;
      else {
        Serial.println("Error: Servo 5 ON failed");
        state = 2;
      }
      break;

    case 4:
      setMode(Lnode, OPERATING_MODE_VELOCITY);
      state = 5;
      break;

    case 5:
      if (getMode(Lnode) == OPERATING_MODE_VELOCITY)
        state = 6;
      else {
        Serial.println("Error: Mode set for Servo 4 failed");
        state = 4;
      }
      break;

    case 6:
      setMode(Rnode, OPERATING_MODE_VELOCITY);
      state = 7;
      break;

    case 7:
      if (getMode(Rnode) == OPERATING_MODE_VELOCITY)
        state = 8;
      else {
        Serial.println("Error: Mode set for Servo 5 failed");
        state = 6;
      }
      break;

    case 8:
      setVelocity(Lnode, -Device.Lvelocity);
      setVelocity(Rnode, Device.Rvelocity);
      break;
  }
}