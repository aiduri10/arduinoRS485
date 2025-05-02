//이 예제는 Arduino mega2560 및 MAX485를 사용하여 테스트가 진행된 예제입니다.
//이 예제어서는 Modbus 프로토콜을 다루고 있으며 FC 0x06 0x10을 사용하고있습니다.

#include <SoftwareSerial.h>
SoftwareSerial RS485(0,1); //RO(RX) -> 2 , DI(TX) -> 3
#define Control 4 // DE&RE -> 4
//기능 코드 function code
#define FC_Read_Holding_Registers 0x03
#define FC_Write_Holding_Register 0x06
#define FC_Write_Multiple_Holding_Register 0x10

#define BUFFER_SIZE 128
#define DATA_SIZE 128
//제어: 가능, 정지, 긴급 정지
#define Enable 0x08
#define Stop 0x07
#define Emergency_Stop 0x05
//제어 주소
#define Control_Word 0x2031
//모드 주소
#define Control_Mode 0x2032
//가속 시간, 감속 시간 주소
#define ACC_Time 0x2037
#define DEC_Time 0x2038
//목표 속도
#define Target_Velocity 0x203A
//최고 속도
#define Current_Speed 0x202C

//uint8_t Driver_ID = 0x01;
unsigned int crc16;
unsigned char frame[BUFFER_SIZE];
uint16_t Multiple_Data[DATA_SIZE];

void setup() 
{
  pinMode(Control,OUTPUT);
  digitalWrite(Control,LOW); // 송신모드 사용 LOW시 수신모드
  RS485.begin(115200); // baudrate 115200으로 지정, Parity = None, Stop Bit = 1, Data bit = 8
  Serial.begin(115200);
}
int state = 0;
void loop() 
{
  uint16_t Velocity = 0;

  switch (state)
  {
    case 0:
      // 서보온 - 4번
      Servo_On(4);
      state++;
      delay(10);
      break;

    case 1:
      // 서보온 체크 - 4번
      if (Read_Servo_On(4) == 0x08)
        state++;
      else {
        Serial.println("Error: Servo 4 ON failed");
        state = 0;
      }
      delay(10);
      break;

    case 2:
      // 서보온 - 5번
      Servo_On(5);
      state++;
      delay(10);
      break;

    case 3:
      // 서보온 체크 - 5번
      if (Read_Servo_On(5) == 0x08)
        state++;
      else {
        Serial.println("Error: Servo 5 ON failed");
        state = 2;
      }
      delay(10);
      break;

    case 4:
      // 모드셋 - 4번
      Mode_Set(4, 3);
      state++;
      delay(10);
      break;

    case 5:
      // 모드셋 체크 - 4번
      if (Read_Mode_Set(4) == 0x03)
        state++;
      else {
        Serial.println("Error: Mode set for Servo 4 failed");
        state = 4;
      }
      delay(10);
      break;

    case 6:
      // 모드셋 - 5번
      Mode_Set(5, 3);
      state++;
      delay(10);
      break;

    case 7:
      // 모드셋 체크 - 5번
      if (Read_Mode_Set(5) == 0x03)
        state++;
      else {
        Serial.println("Error: Mode set for Servo 5 failed");
        state = 6;
      }
      delay(10);
      break;

    case 8:
      // 속도 설정 - 4번
      velocity_Set(4, Velocity);
      state++;
      delay(10);
      break;

    case 9:
      // 속도 설정 - 5번
      velocity_Set(5, Velocity);
      state++;
      delay(10);
      break;

    case 10:
      // 속도 체크 - 4번
      if (abs(Read_Speed(4) - (Velocity * 10)) < 50)
        state++;
      else {
        Serial.println("Error: Speed check for Servo 4 failed");
        state = 8;
      }
      delay(10);
      break;

    case 11:
      // 속도 체크 - 5번
      if (abs(Read_Speed(5) - (Velocity * 10)) >= 50)
        Serial.println("Error: Speed check for Servo 5 failed");
      state = 8;
      delay(10);
      break;
  }
}





void Servo_On(uint8_t Driver_ID)
{
  WriteHoldingRegister(Driver_ID,Control_Word,Enable);
}

// Mode : 0x01~0x02 - Position / 0x03 - Velocity / 0x04 - Torque
void Mode_Set(uint8_t Driver_ID,uint16_t Mode)
{
  WriteHoldingRegister(Driver_ID,Control_Mode,Mode);
}
// 속도 지령
void velocity_Set(uint8_t Driver_ID,uint16_t Velocity)
{
  WriteHoldingRegister(Driver_ID,Target_Velocity,Velocity);
}

// 가속시간 지령
void ACC_Time_Set(uint8_t Driver_ID,uint16_t Target_ACC_time)
{
  WriteHoldingRegister(Driver_ID,ACC_Time,Target_ACC_time);
}
uint16_t Read_Velocity(uint8_t Driver_ID)
{
  ReadHoldingRegister(Driver_ID, Target_Velocity, 1);
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

//FC 03 단일, 다중 읽기 레지스터
//ID 드라이버 값 , ADDR 읽을 레지스터 ,읽을 Word 수량 
/**********************************************************************************************
* ex)  ADDR 0x6200 읽기 : WriteHoldingRegister(0x01,0x6200,0x01);
**********************************************************************************************/
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
  
  //버퍼 비우기
  while (RS485.available()) 
  {
    RS485.read();
  }
  delay(10);
  sendPacket(frameSize);      // 명령 전송
  delay(10);                  // 응답 대기
  
  int responseSize = 5 + Num * 2;  // 예상 응답 길이 (주소 + 기능 + 바이트 수 + 데이터 + CRC)
  uint8_t response[BUFFER_SIZE];
  int i = 0;
  while ((i < responseSize)) 
  {
    if (RS485.available()) 
    {
      response[i++] = RS485.read();
    }
  }

  if (i >= 5)
  {
    uint16_t res_value = (response[3] << 8) | response[4];
    return res_value;
  }
}



//FC 06 단일 데이터 쓰기
//ID 드라이버 값 , ADDR 쓸 레지스터 ,SetData 입력할 데이터
/**********************************************************************************************
* ex)  ADDR 0x6200 쓰기 : WriteHoldingRegister(0x01,0x6200,0x01);
**********************************************************************************************/
uint16_t WriteHoldingRegister(uint8_t Driver_ID,uint16_t ADDR, uint16_t SetData)
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

//FC 10 다중 데이터 쓰기
//ID 드라이버 값 , ADDR = 시작어드레스,Num 입력할 레지스터 수, SetData 입력할 데이터
/**********************************************************************************************
* Multiple_Data = {0x0001,0x0000,0x1388,0x0064,0x0064,0x0064,0x0000,0x0010,0,};
* ex) ADDR 0x6200~6207 쓰기 : WriteMultipleHoldingRegister(0x01,0x6200,0x08,Multiple_Data);
**********************************************************************************************/
uint16_t WriteMultipleHoldingRegister(uint8_t Driver_ID,uint16_t ADDR,uint16_t Num,uint16_t *SetData)
{
 unsigned char frameSize = 0;
 unsigned char N;
 frame[frameSize++] = Driver_ID;
 frame[frameSize++] = FC_Write_Multiple_Holding_Register;
 frame[frameSize++] = ADDR >> 8;
 frame[frameSize++] = ADDR & 0xFF;
 frame[frameSize++] = Num >> 8;
 frame[frameSize++] = Num & 0xFF;
 for(N = 0; N < Num; N++)
 {
  frame[frameSize++] = SetData[N] >> 8;
  frame[frameSize++] = SetData[N] & 0xFF;
 }
  crc16 = calculateCRC(frameSize);
  frame[frameSize++] = crc16 >> 8;
  frame[frameSize++] = crc16 & 0xFF;
  sendPacket(frameSize);
}

//CRC 계산기
unsigned int calculateCRC(unsigned char bufferSize) 
{
  unsigned int temp, temp2, flag;
  temp = 0xFFFF;
  for (unsigned char i = 0; i < bufferSize; i++)
  {
    temp = temp ^ frame[i];
    for (unsigned char j = 1; j <= 8; j++)
    {
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
//Send Data
void sendPacket(unsigned char bufferSize)
{
  digitalWrite(Control,HIGH);
  for (unsigned char i = 0; i < bufferSize; i++)
    RS485.write(frame[i]);
    
  RS485.flush();
  digitalWrite(Control,LOW);
}
