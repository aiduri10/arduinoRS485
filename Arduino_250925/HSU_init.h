#ifndef HSU_INIT_H
#define HSU_INIT_H

#include <Arduino.h>
#include <HX711.h>
#include <Arduino_LED_Matrix.h>
#include <ModbusMaster.h>
#include "HSU_config.h"
#include "HSU_modbus.h"
#include "HSU_calc.h"

void init();
void motorInit();
void loadcellInit();
void print_log();

extern HX711 scale1;
extern HX711 scale2;
extern ArduinoLEDMatrix matrix;
extern ModbusMaster Lnode;
extern ModbusMaster Rnode;
extern STATE Device;
#endif