#ifndef HSU_MODBUS_H
#define HSU_MODBUS_H

#include <ModbusMaster.h>
#include "HSU_config.h"

// 모드버스 레지스터 주소 정의
static const uint16_t REG_ADDR_CONTROL_WORD      = 0x2031;
static const uint16_t REG_ADDR_OPERATING_MODE    = 0x2032;
static const uint16_t REG_ADDR_TARGET_SPEED      = 0x203A;
static const uint16_t REG_ADDR_ACCEL_TIME        = 0x2037;
static const uint16_t REG_ADDR_DECEL_TIME        = 0x2038;
static const uint16_t REG_ADDR_ACTUAL_SPEED      = 0x202C;

// 제어 단어 값 정의
static const uint16_t CTRL_WORD_ENABLE_SERVO     = 0x08;
static const uint16_t CTRL_WORD_STOP_SERVO       = 0x07;

// 동작 모드 값 정의
static const uint16_t OPERATING_MODE_POSITION    = 0x01;
static const uint16_t OPERATING_MODE_VELOCITY    = 0x03;
static const uint16_t OPERATING_MODE_TORQUE      = 0x04;



// 모터 제어 함수 정의
/**
 * @brief 서보 모터를 활성화합니다.
 * @param node 모드버스 마스터 객체
 * @note 드라이브의 제어 워드 레지스터에 ENABLE 값을 씁니다.
 */
void setControlEnable(ModbusMaster& node);

/**
 * @brief 서보 드라이브의 동작 모드를 설정합니다.
 * @param node 모드버스 마스터 객체
 * @param operatingMode 설정하려는 동작 모드 (예: 0x01: 위치, 0x03: 속도, 0x04: 토크)
 */
void setMode(ModbusMaster& node, uint16_t operatingMode);

/**
 * @brief 서보 모터의 목표 속도를 설정합니다.
 * @param node 모드버스 마스터 객체
 * @param velocity 설정하려는 목표 속도 (단위: rpm)
 */
void setVelocity(ModbusMaster& node, int16_t velocity);

/**
 * @brief 서보 모터의 가속 시간을 설정합니다.
 * @param node 모드버스 마스터 객체
 * @param accelTime 설정하려는 가속 시간 (단위: ms)
 */
void setAccelerationTime(ModbusMaster& node, uint16_t accelTime);

/**
 * @brief 서보 모터의 감속 시간을 설정합니다.
 * @param node 모드버스 마스터 객체
 * @param decelTime 설정하려는 감속 시간 (단위: ms)
 */
void setDecelerationTime(ModbusMaster& node, uint16_t decelTime);

/**
 * @brief 서보 모터의 현재 실제 속도를 읽습니다.
 * @param node 모드버스 마스터 객체
 * @return 현재 실제 속도 값 (단위: rpm)
 */
int16_t getActualVelocity(ModbusMaster& node);

/**
 * @brief 서보 드라이브의 제어 워드 값을 읽습니다.
 * @param node 모드버스 마스터 객체
 * @return 현재 제어 워드 레지스터 값
 */
uint16_t getControl(ModbusMaster& node);

/**
 * @brief 서보 드라이브의 현재 동작 모드 값을 읽습니다.
 * @param node 모드버스 마스터 객체
 * @return 현재 동작 모드 값
 */
uint16_t getMode(ModbusMaster& node);



void preTransmission();
void postTransmission();

#endif