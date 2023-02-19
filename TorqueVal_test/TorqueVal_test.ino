//ポテションメータのトルク値変換&送信
#include <mcp_can.h>
#include <SPI.h>
MCP_CAN CAN(10);
const int INPUT_PIN = 0;  //アナログ入力ピン
int Val = 0;  //アナログ入力の変数

void setup() {
  Serial.begin(9600);
  Val = analogRead(INPUT_PIN);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte buf[] = { B00000001, 0, 0, 0, 0, 0, 0, 0 };  //0byte目はMG-ECU:on, Co放電要求:off 固定

  
  float AnaMin = 1023.0 * 0.1;          //出力関数：0°＝10%
  float AnaMax = 1023.0 * 0.9;          //出力関数：360°＝90%
  float a = 120.0 / (AnaMax - AnaMin);  //傾き,120:最大トルク60の2倍
  float b = -a * AnaMin;

  int y = a * Val + b;     //トルク範囲に置き換える0～120
  float Torque = 0.5 * y;  //トルク値, 0～60
  buf[1] = Torque;

  //CAN.sendMsgBuf(0x301, 0, 8, buf);

  unsigned char sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf);  //ID:0x02, 標準フレーム:0, データ長:8
  if (sndStat == CAN_OK) {
    Serial.print("CANsend:");
    Serial.print(Torque);
    Serial.print(" [Nm]");
  } else {
    Serial.print("Error Sending Message...");
  }

  Serial.print("  ||  ");  //これ以降は参照値，コメントアウトしてOK
  Serial.print("Val: ");
  Serial.print(Val);
  Serial.print("   y:");
  Serial.print(y);
  Serial.print("   Torque:");
  Serial.print(Torque);
  Serial.println(" [Nm]");

  //delay(100);
}
