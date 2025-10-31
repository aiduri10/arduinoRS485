#ifndef HSU_CONFIG_H
#define HSU_CONFIG_H
#include <stdint.h>
#include <HX711.h>
#include "HSU_modbus.h"
#include "HSU_calc.h"
const int Control = 4;

const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;
const int LOADCELL_DOUT_PIN2 = 8;
const int LOADCELL_SCK_PIN2 = 9;

const float calibration_factor1 = 420000; 
const float calibration_factor2 = 420000;


const int16_t BACK_VELOCITY = -30;
const int16_t SLOW_VELOCITY = 50;
const int16_t MEDIUM_VELOCITY = 70;
const int16_t FAST_VELOCITY = 90;
const int16_t TURN_VELOCITY = 15;
const int16_t FAST_TURN_VELOCITY = 30;


#endif
