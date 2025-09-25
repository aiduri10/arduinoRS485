#include "HSU_modbus.h"

void setControlEnable(ModbusMaster& node)
{
  // 모터를 활성화하기 위해 제어 워드 레지스터에 enable 값(0x08)을 씁니다.
  node.writeSingleRegister(REG_ADDR_CONTROL_WORD, CTRL_WORD_ENABLE_SERVO);
}
void setMode(ModbusMaster& node, uint16_t operatingMode)
{
  // 지정된 동작 모드 값을 동작 모드 레지스터에 씁니다.
  node.writeSingleRegister(REG_ADDR_OPERATING_MODE, operatingMode);
}
void setVelocity(ModbusMaster& node, int16_t velocity)
{
  // 목표 속도 값을 해당 레지스터에 씁니다. 이 값은 모터가 도달하고자 하는 속도를 나타냅니다.
  node.writeSingleRegister(REG_ADDR_TARGET_SPEED, velocity);
}
void setAccelerationTime(ModbusMaster& node, uint16_t accelTime)
{
  // 가속 시간 값을 해당 레지스터에 씁니다. 이 값은 모터가 최고 속도까지 가속하는 데 걸리는 시간을 결정합니다.
  node.writeSingleRegister(REG_ADDR_ACCEL_TIME, accelTime);
}
void setDecelerationTime(ModbusMaster& node, uint16_t decelTime)
{
  // 감속 시간 값을 해당 레지스터에 씁니다. 이 값은 모터가 정지하는 데 걸리는 시간을 결정합니다.
  node.writeSingleRegister(REG_ADDR_DECEL_TIME, decelTime);
}
int16_t getActualVelocity(ModbusMaster& node)
{
  // 실제 속도 레지스터에서 모터의 현재 속도 값을 읽어옵니다.
  uint8_t result = node.readHoldingRegisters(REG_ADDR_ACTUAL_SPEED, 1);
  
  // 읽기 작업이 성공했는지 확인한 후 값을 반환합니다.
  if (result == node.ku8MBSuccess) {
    int16_t value = (int16_t)node.getResponseBuffer(0);
    return value;
  }
  
  // 실패 시 오류를 나타내기 위해 0을 반환합니다.
  return 0;
}

uint16_t getControl(ModbusMaster& node)
{
  // 현재 제어 워드 값을 레지스터에서 읽어옵니다.
  uint8_t result = node.readHoldingRegisters(REG_ADDR_CONTROL_WORD, 1);
  
  // 읽기 작업이 성공하면 값을 반환합니다.
  if (result == node.ku8MBSuccess) {
    int16_t value = node.getResponseBuffer(0);
    return value;
  }
  
  // 실패 시 0을 반환합니다.
  return 0;
}

uint16_t getMode(ModbusMaster& node)
{
  // 현재 동작 모드 값을 레지스터에서 읽어옵니다.
  uint8_t result = node.readHoldingRegisters(REG_ADDR_OPERATING_MODE, 1);
  
  // 읽기 작업이 성공하면 값을 반환합니다.
  if (result == node.ku8MBSuccess) {
    int16_t value = node.getResponseBuffer(0);
    return value;
  }
  
  // 실패 시 0을 반환합니다.
  return 0;
}

void preTransmission()
{
  digitalWrite(Control, 1);
}
void postTransmission()
{
  digitalWrite(Control, 0);
  delay(10);
}