//Cpu1とインバータのtest code
//CAN送信はID:785のみ，受信はID:769
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);
const int IGNSW_PIN = 2;  //IGNSWのデジタル出力ピン

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte MG_ECU;
  static byte dispat = 0;
  static byte presen = 0;
  static byte status;
  static byte disoder;
  unsigned char sndStat;
  static byte torque;

  int IGNSWsta = digitalRead(IGNSW_PIN);  //IGNSWの状態，HIGH or LOW

  static byte buf_s[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN通信送信バッファ

  unsigned long id;  //ID
  byte len;          //フレームの長さ
  byte buf_r[8];     //CAN通信受信バッファ
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf_r);
    //ID:769
    if (id == 0x301) {
      MG_ECU = bitRead(buf_r[0], 0);
      disoder = bitRead(buf_r[0], 1);
      byte torque = buf_r[1];
    }
  }
  status = B111;  //rapid discharge
  if (IGNSWsta == LOW) {
    disoder = bitRead(buf_r[0], 1);
    if (dispat == 0) {
      if (presen != 1) {
        byte buf_s[] = { 0xA, 0xB0, 0x36, 0x4, 0, 0, 0, 0 };
        for (int i = 0; i < 3; i++) {
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Successfully Sent");
          } else {
            Serial.println("Error Sending Message...");
          }
        }
        presen = 1;
      }
      byte buf_s[] = { 0x3B, 0, 0, 0, 0, 0, 0, 0 };
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Successfully Sent");
      } else {
        Serial.println("Error Sending Message...");
      }
    } else if (dispat == 1) {
      byte buf_s[] = { 0x3B, 0, 0, 0, B10010000, B00000001, 0, 0 };  //400V
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Successfully Sent, 400V");
      } else {
        Serial.println("Error Sending Message...");
      }
      if (disoder == B10) {
        Serial.println("Discharging.....");
        delay(2500);
        Serial.println("Now Standby");
        byte buf_s[] = { 0x13, 0xFF, 0xFF, 0, 0, 0, 0, 0xA0 };
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("Successfully Sent, standby");
        } else {
          Serial.println("Error Sending Message...");
        }
        dispat = 0;
      }
    }
  } else if (IGNSWsta == HIGH) {
    MG_ECU = bitRead(buf_r[0], 0);
    status = B001;  //precharge
    if (status == B001) {
      byte buf_s[] = { 0xA, 0, 0, 0, 0, 0, 0, 0 };
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Successfully Sent, Precharge");
      } else {
        Serial.println("Error Sending Message...");
      }
      if (MG_ECU == 1) {
        status = B011;                                                    //torque control
        byte buf_s[] = { B011100, 0, 0, 0, B10010000, B00000001, 0, 0 };  //400V
        if (sndStat == CAN_OK) {
          Serial.println("Successfully Sent, Torque control");
        } else {
          Serial.println("Error Sending Message...");
        }
      }
    } else if (status == B011) {
      if (MG_ECU == 1) {
        byte buf_s[] = { B011100, 0, 0, 0, B10010000, B00000001, 0, 0 };
        CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        Serial.print("torque:");
        Serial.print(torque, DEC);
        Serial.println("[Nm]");
      } else if (MG_ECU == 0) {
        status = B111;
        dispat = 1;
      }
    }
  }
}
