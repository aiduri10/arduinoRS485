#include "HSU_calc.h"


DIRECTION calculateDirection(int Lweight, int Rweight)
{
  int diff = Rweight - Lweight;
  if (diff > 500) { return RIGHT; }
  else if (diff < -500) { return LEFT; }
  else { return STRAIGHT; }
}

void calculateSpeed(STATE* state)
{
  int Lweight = state->Lweight;
  int Rweight = state->Rweight;

  if ((Lweight > -800 && Lweight < 500) || (Rweight > -800 && Rweight < 500)) {
    state->speed = STOP;
    state->direction = STRAIGHT;
    return;
  }

  if (Lweight <= -800 && Rweight <= -800) {
    state->speed = BACK;
    state->direction = STRAIGHT;
    return;
  }

  int avgWeight = (Lweight + Rweight) / 2;
  state->direction = calculateDirection(Lweight, Rweight);

  if (avgWeight >= 500 && avgWeight < 2000) { state->speed = SLOW; return; }
  if (avgWeight >= 2000 && avgWeight < 4000) { state->speed = MEDIUM; return; }
  if (avgWeight >= 4000) { state->speed = FAST; return; }

  state->speed = STOP;
}

void calculateVelocity(STATE* state)
{
  if (state->speed == BACK) {
    state->Rvelocity = BACK_VELOCITY;
    state->Lvelocity = BACK_VELOCITY;
  }
  else if (state->speed == STOP) {
    state->Rvelocity = 0;
    state->Lvelocity = 0;
  }
  else if (state->speed == SLOW) {
    if (state->direction == STRAIGHT) {
      state->Rvelocity = SLOW_VELOCITY;
      state->Lvelocity = SLOW_VELOCITY;
    } else if (state->direction == LEFT) {
      state->Rvelocity = SLOW_VELOCITY;
      state->Lvelocity = SLOW_VELOCITY - TURN_VELOCITY;
    } else if (state->direction == RIGHT) {
      state->Rvelocity = SLOW_VELOCITY - TURN_VELOCITY;
      state->Lvelocity = SLOW_VELOCITY;
    }
  }
  else if (state->speed == MEDIUM) {
    if (state->direction == STRAIGHT) {
      state->Rvelocity = MEDIUM_VELOCITY;
      state->Lvelocity = MEDIUM_VELOCITY;
    } else if (state->direction == LEFT) {
      state->Rvelocity = MEDIUM_VELOCITY;
      state->Lvelocity = MEDIUM_VELOCITY - TURN_VELOCITY;
    } else if (state->direction == RIGHT) {
      state->Rvelocity = MEDIUM_VELOCITY - TURN_VELOCITY;
      state->Lvelocity = MEDIUM_VELOCITY;
    }
  }
  else if (state->speed == FAST) {
    if (state->direction == STRAIGHT) {
      state->Rvelocity = FAST_VELOCITY;
      state->Lvelocity = FAST_VELOCITY;
    } else if (state->direction == LEFT) {
      state->Rvelocity = FAST_VELOCITY;
      state->Lvelocity = FAST_VELOCITY - TURN_VELOCITY;
    } else if (state->direction == RIGHT) {
      state->Rvelocity = FAST_VELOCITY - TURN_VELOCITY;
      state->Lvelocity = FAST_VELOCITY;
    }
  }
}

void DirectionImage(STATE* state, ArduinoLEDMatrix& matrix)
{
  if (state->speed == STOP) { matrix.loadFrame(num5); return; }
  else if (state->speed == BACK) { matrix.loadFrame(num2); return; }
  else if (state->direction == STRAIGHT) { matrix.loadFrame(num1); return; }
  else if (state->direction == LEFT) { matrix.loadFrame(num3); return; }
  else if (state->direction == RIGHT) { matrix.loadFrame(num4); return; }
}