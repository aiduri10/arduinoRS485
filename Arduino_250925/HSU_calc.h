#ifndef HSU_CALC_H
#define HSU_CALC_H
#include "HSU_config.h"
#include "Arduino_LED_Matrix.h"
#include "ohturtle.h"

typedef enum SPEED {
  BACK = 0,
  STOP = 1,
  SLOW = 2,
  MEDIUM = 3,
  FAST = 4
};

typedef enum DIRECTION {
  STRAIGHT = 0,
  LEFT = 1,
  RIGHT = 2
};

static const char* SPEED_STRINGS[] = {
  "후진", "정지", "저속", "중속", "고속"
};

static const char* DIRECTION_STRINGS[] = {
  "직진", "좌회전", "우회전"
};

typedef struct{
  int Lweight;
  int Rweight;
  int16_t Lvelocity;
  int16_t Rvelocity;
  SPEED speed;
  DIRECTION direction;
}STATE;

DIRECTION calculateDirection(int Lweight, int Rweight);
void calculateSpeed(STATE* state);
void calculateVelocity(STATE* state);
void DirectionImage(STATE* state, ArduinoLEDMatrix& matrix);

#endif