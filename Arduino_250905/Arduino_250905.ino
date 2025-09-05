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
#include "HX711.h"
#include "Arduino_LED_Matrix.h"
#include "ohturtle.h"

SoftwareSerial RS485(0,1);
#define Control 4

// HX711 pins
#define LOADCELL_DOUT_PIN 2
#define LOADCELL_SCK_PIN 3
#define LOADCELL_DOUT_PIN2 8
#define LOADCELL_SCK_PIN2 9

HX711 scale1;
HX711 scale2;
ArduinoLEDMatrix matrix;

int calibration_factor1 = 420000;
int calibration_factor2 = 420000;

// Function codes
#define FC_Read_Holding_Registers 0x03
#define FC_Write_Holding_Register 0x06
#define FC_Write_Multiple_Holding_Register 0x10

#define BUFFER_SIZE 128
#define DATA_SIZE 128

// Control words
#define Enable 0x08
#define Stop 0x07
#define Emergency_Stop 0x05

// Register addresses
#define Control_Word 0x2031
#define Control_Mode 0x2032
#define ACC_Time 0x2037
#define DEC_Time 0x2038
#define Target_Velocity 0x203A
#define Current_Speed 0x202C

unsigned int crc16;
unsigned char frame[BUFFER_SIZE];
uint16_t Multiple_Data[DATA_SIZE];

void Servo_On(uint8_t Driver_ID)
{
    WriteHoldingRegister(Driver_ID,Control_Word,Enable);
}

// Mode : 0x01~0x02 - Position / 0x03 - Velocity / 0x04 - Torque
void Mode_Set(uint8_t Driver_ID,uint16_t Mode)
{
    WriteHoldingRegister(Driver_ID,Control_Mode,Mode);
}

void velocity_Set(uint8_t Driver_ID,int16_t Velocity)
{
    WriteHoldingRegister(Driver_ID,Target_Velocity,Velocity);
}

void ACC_Time_Set(uint8_t Driver_ID,uint16_t Target_ACC_time)
{
    WriteHoldingRegister(Driver_ID,ACC_Time,Target_ACC_time);
}

void DEC_Time_Set(uint8_t Driver_ID,uint16_t Target_DEC_time)
{
    WriteHoldingRegister(Driver_ID,DEC_Time,Target_DEC_time);
}

uint16_t Read_Speed(uint8_t Driver_ID)
{
    return ReadHoldingRegister(Driver_ID, Current_Speed, 1);
}

uint16_t Read_Servo_On(uint8_t Driver_ID)
{
    return ReadHoldingRegister(Driver_ID,Control_Word,1);
}

uint16_t Read_Mode_Set(uint8_t Driver_ID)
{
    return ReadHoldingRegister(Driver_ID,Control_Mode, 1);
}

// FC 03: Read holding register(s)
uint16_t ReadHoldingRegister(uint8_t Driver_ID,uint16_t ADDR, uint16_t Num)
{
    unsigned char frameSize = 0;
    frame[frameSize++] = Driver_ID;
    frame[frameSize++] = FC_Read_Holding_Registers;
    frame[frameSize++] = ADDR >> 8;
    frame[frameSize++] = ADDR & 0xFF;
    frame[frameSize++] = Num >> 8;
    frame[frameSize++] = Num & 0xFF;

    crc16 = calculateCRC(frameSize);
    frame[frameSize++] = crc16 >> 8;
    frame[frameSize++] = crc16 & 0xFF;
  
    while (RS485.available()) { RS485.read(); }
    delay(1);
    sendPacket(frameSize);
    delay(1);

    int responseSize = 5 + Num * 2;
    uint8_t response[BUFFER_SIZE];
    int i = 0;
    while ((i < responseSize)) {
        if (RS485.available()) {
            response[i++] = RS485.read();
        }
    }

    if (i >= 5) {
        uint16_t res_value = (response[3] << 8) | response[4];
        return res_value;
    }
}

// FC 06: Write single holding register
void WriteHoldingRegister(uint8_t Driver_ID,uint16_t ADDR, uint16_t SetData)
{
    unsigned char frameSize = 0;
    frame[frameSize++] = Driver_ID;
    frame[frameSize++] = FC_Write_Holding_Register;
    frame[frameSize++] = ADDR >> 8;
    frame[frameSize++] = ADDR & 0xFF;
    frame[frameSize++] = SetData >> 8;
    frame[frameSize++] = SetData & 0xff;

    crc16 = calculateCRC(frameSize);
    frame[frameSize++] = crc16 >> 8;
    frame[frameSize++] = crc16 & 0xFF;
    sendPacket(frameSize);
}

// FC 16: Write multiple holding registers
void WriteMultipleHoldingRegister(uint8_t Driver_ID,uint16_t ADDR,uint16_t Num,uint16_t *SetData)
{
    unsigned char frameSize = 0;
    unsigned char N;
    frame[frameSize++] = Driver_ID;
    frame[frameSize++] = FC_Write_Multiple_Holding_Register;
    frame[frameSize++] = ADDR >> 8;
    frame[frameSize++] = ADDR & 0xFF;
    frame[frameSize++] = Num >> 8;
    frame[frameSize++] = Num & 0xFF;
    for(N = 0; N < Num; N++){
        frame[frameSize++] = SetData[N] >> 8;
        frame[frameSize++] = SetData[N] & 0xFF;
    }
    crc16 = calculateCRC(frameSize);
    frame[frameSize++] = crc16 >> 8;
    frame[frameSize++] = crc16 & 0xFF;
    sendPacket(frameSize);
}

// CRC
unsigned int calculateCRC(unsigned char bufferSize)
{
    unsigned int temp, temp2, flag;
    temp = 0xFFFF;
    for (unsigned char i = 0; i < bufferSize; i++){
        temp = temp ^ frame[i];
        for (unsigned char j = 1; j <= 8; j++){
            flag = temp & 0x0001;
            temp >>= 1;
            if (flag)
                temp ^= 0xA001;
        }
    }
    temp2 = temp >> 8;
    temp = (temp << 8) | temp2;
    temp &= 0xFFFF;
    return temp;
}

// Send packet
void sendPacket(unsigned char bufferSize)
{
    digitalWrite(Control,HIGH);
    for (unsigned char i = 0; i < bufferSize; i++)
        RS485.write(frame[i]);
    RS485.flush();
    digitalWrite(Control,LOW);
}

uint16_t Read_Velocity(uint8_t Driver_ID)
{
    return ReadHoldingRegister(Driver_ID, Target_Velocity, 1);
}

// -------------------------------- Calc State --------------------------------

enum DIRECTION FuncDirection(int Lweight, int Rweight)
{
    int diff = Rweight - Lweight;
    if (diff > 1000) { return RIGHT; }
    else if (diff < -1000) { return LEFT; }
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
    RS485.begin(115200);
    Serial.begin(115200);
    delay(1000);

    matrix.begin();

    scale1.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
    scale1.set_scale();
    scale1.tare();

    scale2.begin(LOADCELL_DOUT_PIN2, LOADCELL_SCK_PIN2);
    scale2.set_scale();
    scale2.tare();
 
    DEC_Time_Set(4,100);
    delay(10);
    DEC_Time_Set(5,100);
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
            Servo_On(4);
            state=1;
            delay(10);
            break;

        case 1:
            if (Read_Servo_On(4) == 0x08)
                state=2;
            else {
                Serial.println("Error: Servo 4 ON failed");
                state = 0;
            }
            delay(10);
            break;

        case 2:
            Servo_On(5);
            state=3;
            delay(10);
            break;

        case 3:
            if (Read_Servo_On(5) == 0x08)
                state=4;
            else {
                Serial.println("Error: Servo 5 ON failed");
                state = 2;
            }
            delay(10);
            break;

        case 4:
            Mode_Set(4, 3);
            state=5;
            delay(10);
            break;

        case 5:
            if (Read_Mode_Set(4) == 0x03)
                state=6;
            else {
                Serial.println("Error: Mode set for Servo 4 failed");
                state = 4;
                delay(10);
            }
            break;

        case 6:
            Mode_Set(5, 3);
            state=7;
            delay(10);
            break;

        case 7:
            if (Read_Mode_Set(5) == 0x03)
                state=8;
            else {
                Serial.println("Error: Mode set for Servo 5 failed");
                state = 6;
                delay(10);
            }
            break;

        case 8:
            velocity_Set(4, Device.LVelocity);
            delay(10);
            velocity_Set(5, -Device.RVelocity);
            state=8;
            break;
    }

    delay(1);
}
