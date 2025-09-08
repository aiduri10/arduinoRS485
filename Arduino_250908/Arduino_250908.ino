enum SPEED {
    BACK = 0,
    STOP,
    SLOW,
    MEDIUM,
    FAST
};

enum DIRECTION {
    STRAIGHT = 0,
    LEFT,
    RIGHT
};

typedef struct{
    int Lweight;
    int Rweight;
    int16_t LVelocity;
    int16_t RVelocity;
    enum SPEED speed;
    enum DIRECTION direction;
}STATE;

enum DIRECTION FuncDirection(int Lweight, int Rweight);
void FuncSpeedState(STATE* state);
void ControlMotors(STATE* state);
void DirectionImage(STATE* state);

#include <SoftwareSerial.h>
#include <ModbusMaster.h>
#include "HX711.h"
#include "Arduino_LED_Matrix.h"
#include "ohturtle.h"


#define Control 4

// HX711 pins
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3
#define LOADCELL_DOUT_PIN2 8
#define LOADCELL_SCK_PIN2 9

HX711 scale1;
HX711 scale2;
ArduinoLEDMatrix matrix;

ModbusMaster Lnode;
ModbusMaster Rnode;

int calibration_factor1 = 420000;
int calibration_factor2 = 420000;




void Servo_On(ModbusMaster& node)
{
    node.writeSingleRegister(0x2031, 0x08);
}
void Mode_Set(ModbusMaster& node)
{
    node.writeSingleRegister(0x2032, 0x03);
}
void velocity_Set(ModbusMaster& node, int16_t velocity)
{
    node.writeSingleRegister(0x203A, velocity);
}
void ACC_Time_Set(ModbusMaster& node, uint16_t acc_time)
{
    node.writeSingleRegister(0x2037, acc_time);
}
void DEC_Time_Set(ModbusMaster& node, uint16_t dec_time)
{
    node.writeSingleRegister(0x2038, dec_time);
}
int16_t Read_Speed(ModbusMaster& node)
{
    uint8_t result = node.readHoldingRegisters(0x202C, 1);
    if (result == node.ku8MBSuccess) {
        int16_t value = (int16_t)node.getResponseBuffer(0);
        return value;
    }
    return 0;
}

uint16_t Read_Servo_On(ModbusMaster& node)
{
    uint8_t result = node.readHoldingRegisters(0x2031, 1);
    if (result == node.ku8MBSuccess) {
        int16_t value = node.getResponseBuffer(0);
        return value;
    }
    return 0;
}

uint16_t Read_Mode_Set(ModbusMaster& node)
{
    uint8_t result = node.readHoldingRegisters(0x2032, 1);
    if (result == node.ku8MBSuccess) {
        int16_t value = node.getResponseBuffer(0);
        return value;
    }
    return 0;
}

void preTransmission()
{
    digitalWrite(Control, 1);
}
void postTransmission()
{
    digitalWrite(Control, 0);
}

// -------------------------------- Calc State --------------------------------

enum DIRECTION FuncDirection(int Lweight, int Rweight)
{
    int diff = Rweight - Lweight;
    if (diff > 500) { return RIGHT; }
    else if (diff < -500) { return LEFT; }
    else { return STRAIGHT; }
}

void FuncSpeedState(STATE* state)
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
    state->direction = FuncDirection(Lweight, Rweight);

    if (avgWeight >= 500 && avgWeight < 2000) { state->speed = SLOW; return; }
    if (avgWeight >= 2000 && avgWeight < 4000) { state->speed = MEDIUM; return; }
    if (avgWeight >= 4000) { state->speed = FAST; return; }

    state->speed = STOP;
}

void ControlMotors(STATE* state)
{
    int16_t BACK_velocity = -30;
    int16_t SLOW_velocity = 50;
    int16_t MEDIUM_velocity = 70;
    int16_t FAST_velocity = 90;
    int16_t TURN_velocity = 15;

    if (state->speed == BACK){
        state->RVelocity = BACK_velocity;
        state->LVelocity = BACK_velocity;
    }
    else if (state->speed == STOP){
        state->RVelocity = 0;
        state->LVelocity = 0;
    }
    else if (state->speed == SLOW){
        if (state->direction == STRAIGHT){
            state->RVelocity = SLOW_velocity;
            state->LVelocity = SLOW_velocity;
        } else if (state->direction == LEFT){
            state->RVelocity = SLOW_velocity;
            state->LVelocity = SLOW_velocity - TURN_velocity;
        } else if (state->direction == RIGHT){
            state->RVelocity = SLOW_velocity - TURN_velocity;
            state->LVelocity = SLOW_velocity;
        }
    }
    else if (state->speed == MEDIUM){
        if (state->direction == STRAIGHT){
            state->RVelocity = MEDIUM_velocity;
            state->LVelocity = MEDIUM_velocity;
        } else if (state->direction == LEFT){
            state->RVelocity = MEDIUM_velocity;
            state->LVelocity = MEDIUM_velocity - TURN_velocity;
        } else if (state->direction == RIGHT){
            state->RVelocity = MEDIUM_velocity - TURN_velocity;
            state->LVelocity = MEDIUM_velocity;
        }
    }
    else if (state->speed == FAST){
        if (state->direction == STRAIGHT){
            state->RVelocity = FAST_velocity;
            state->LVelocity = FAST_velocity;
        } else if (state->direction == LEFT){
            state->RVelocity = FAST_velocity;
            state->LVelocity = FAST_velocity - TURN_velocity;
        } else if (state->direction == RIGHT){
            state->RVelocity = FAST_velocity - TURN_velocity;
            state->LVelocity = FAST_velocity;
        }
    }
}

void DirectionImage(STATE* state)
{
    if (state->speed == STOP) { matrix.loadFrame(num5); return; }
    else if (state->speed == BACK) { matrix.loadFrame(num2); return; }
    else if (state->direction == STRAIGHT) { matrix.loadFrame(num1); return; }
    else if (state->direction == LEFT) { matrix.loadFrame(num3); return; }
    else if (state->direction == RIGHT) { matrix.loadFrame(num4); return; }
}

// -------------------------------- Setup / Loop --------------------------------

void setup()
{
    pinMode(Control,OUTPUT);
    digitalWrite(Control,LOW);
    Serial1.begin(115200);
    Serial.begin(115200);
    delay(1000);
    Lnode.begin(5, Serial1);
    Rnode.begin(4, Serial1);
    matrix.begin();

    scale1.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale1.set_scale();
    scale1.tare();

    scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);
    scale2.set_scale();
    scale2.tare();
 
    DEC_Time_Set(Lnode, 100);
    delay(10);
    DEC_Time_Set(Rnode, 100);
    delay(10);

    Serial.println("Setup complete");
}

int state = 0;
STATE Device = {0,0,0,0,STOP,STRAIGHT};

void loop()
{
    scale1.set_scale(calibration_factor1);
    scale2.set_scale(calibration_factor2);

    Device.Lweight = -scale1.get_units() * 1000;
    Device.Rweight = scale2.get_units() * 1000;
  
    FuncSpeedState(&Device);

    Serial.print("왼쪽 감지 무게: ");
    Serial.print(Device.Lweight);
    Serial.print(" g | ");

    Serial.print("오른쪽 감지 무게: ");
    Serial.print(Device.Rweight);
    Serial.print(" g | ");

    Serial.print("속도: ");
    Serial.print(Device.speed);
    Serial.print(" | 방향: ");
    Serial.println(Device.direction);

    ControlMotors(&Device);
    DirectionImage(&Device);

    switch (state)
    {
        case 0:
            Servo_On(Lnode);
            state=1;
            delay(10);
            break;

        case 1:
            if ( Read_Servo_On(Lnode) == 0x08)
                state=2;
            else {
                Serial.println("Error: Servo 4 ON failed");
                state = 0;
            }
            delay(10);
            break;

        case 2:
            Servo_On(Rnode);
            state=3;
            delay(10);
            break;

        case 3:
            if (Read_Servo_On(Rnode) == 0x08)
                state=4;
            else {
                Serial.println("Error: Servo 5 ON failed");
                state = 2;
            }
            delay(10);
            break;

        case 4:
            Mode_Set(Lnode);
            state=5;
            delay(10);
            break;

        case 5:
            if (Read_Mode_Set(Lnode) == 0x03)
                state=6;
            else {
                Serial.println("Error: Mode set for Servo 4 failed");
                state = 4;
                delay(10);
            }
            break;

        case 6:
            Mode_Set(Rnode);
            state=7;
            delay(10);
            break;

        case 7:
            if (Read_Mode_Set(Rnode) == 0x03)
                state=8;
            else {
                Serial.println("Error: Mode set for Servo 5 failed");
                state = 6;
                delay(10);
            }
            break;

        case 8:
            velocity_Set(Lnode, -Device.LVelocity);
            delay(10);
            velocity_Set(Rnode, Device.RVelocity);
            state=8;
            break;
    }

    delay(1);
}
