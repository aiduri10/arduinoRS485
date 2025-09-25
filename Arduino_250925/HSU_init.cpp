#include "HSU_init.h"

void motorInit()
{
  pinMode(Control,OUTPUT);
  digitalWrite(Control,LOW);

  Serial1.begin(115200);
  
  Lnode.begin(5, Serial1);
  Rnode.begin(4, Serial1);
  Lnode.preTransmission(preTransmission);
  Lnode.postTransmission(postTransmission);
  Rnode.preTransmission(preTransmission);
  Rnode.postTransmission(postTransmission);
  setDecelerationTime(Lnode, 100);
  setDecelerationTime(Rnode, 100);
}

void loadcellInit()
{
  scale1.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  scale1.tare();
  scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);
  scale2.tare();
  scale1.set_scale(calibration_factor1);
  scale2.set_scale(calibration_factor2);
}

void init()
{
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup1");
  matrix.begin();
  Serial.println("Setup2");
  motorInit();
  Serial.println("Setup3");
  loadcellInit();
}

void print_log()
{
  Serial.print("왼쪽 감지 무게: ");
  Serial.print(Device.Lweight);
  Serial.print(" g | ");

  Serial.print("오른쪽 감지 무게: ");
  Serial.print(Device.Rweight);
  Serial.print(" g | ");

  Serial.print("속도: ");
  Serial.print(SPEED_STRINGS[Device.speed]); 
  Serial.print(" | 방향: ");
  Serial.println(DIRECTION_STRINGS[Device.direction]); 
}