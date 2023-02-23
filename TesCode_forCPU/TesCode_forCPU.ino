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
  static int MG_ECU;      //Mg-ECU実行要求，ON：1，OFF：0
  static int dispat = 0;  //dischargeのパターン，0：初期化後，1：高電圧印加後（通常時）
  static int presen = 0;
  static int status;   //制御状態
  static int disoder;  //Co放電要求の状態変数，Active：1，Inactive：0
  unsigned char sndStat;
  static int torque;

  int IGNSWsta = digitalRead(IGNSW_PIN);  //IGNSWの状態，HIGH or LOW

  static byte buf_s[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN通信送信バッファ

  unsigned long id;  //ID
  byte len;          //フレームの長さ
  byte buf_r[8];     //CAN通信受信バッファ
                     // if (CAN.checkReceive() == CAN_MSGAVAIL) {
  CAN.readMsgBuf(&id, &len, buf_r);
  //ID:769
  if (id == 0x301) {
    MG_ECU = bitRead(buf_r[0], 0);
    disoder = bitRead(buf_r[0], 1);
    torque = buf_r[1];
  }


  if (IGNSWsta == LOW) {
    //status = B111;  //rapid discharge
    disoder = bitRead(buf_r[0], 1);
    if (dispat == 0) {
      if (presen != 1) {
        byte buf_s[] = { 0xA, 0xB0, 0x36, 0x4, 0, 0, 0, 0 };
        for (int i = 0; i < 3; i++) {
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Successfully Sent");
          } else {
            Serial.println("Error...");
          }
        }
        presen = 1;
      }
      byte buf_s[] = { 0x3B, 0, 0, 0, 0, 0, 0, 0 };  //discharge状態を送信
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Successfully Sent, discharge");
      } else {
        Serial.println("Error...");
      }
    } else if (dispat == 1) {
      byte buf_s[] = { 0x3B, 0, 0, 0, B10010000, B00000001, 0, 0 };  //discharge状態を送信, 400V
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("rapid discharge, 400V");
      } else {
        Serial.println("Error...");
      }
      if (disoder == B10) {  //CO放電要求 Active
        Serial.println("Discharging.....");
        delay(2500);
        Serial.println("Now Standby");
        byte buf_s[] = { 0x13, 0xFF, 0xFF, 0, 0, 0, 0, 0xA0 };  //standby 状態を送信
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("Successfully Sent, standby");
        } else {
          Serial.println("Error...");
        }
        dispat = 0;
      }
    }
  } else if (IGNSWsta == HIGH) {
    MG_ECU = bitRead(buf_r[0], 0);
    //status = B001;  //precharge
    if (status != B011) {  //torque状態じゃないなら以下を実行する
      byte buf_s[] = { 0xA, 0, 0, 0, 0, 0, 0, 0 };
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Precharging....");
      } else {
        Serial.println("Error...");
      }
      if (MG_ECU == 1) {
        status = B011;                                                    //torque control 状態にする
        byte buf_s[] = { B011100, 0, 0, 0, B10010000, B00000001, 0, 0 };  //400V
        if (sndStat == CAN_OK) {
          Serial.println("MG-ECU : ON | Torque control");
        } else {
          Serial.println("Error...");
        }
      }
    } else if (status == B011) {  //torque control 状態
      if (MG_ECU == 1) {
        byte buf_s[] = { B011100, 0, 0, 0, B10010000, B00000001, 0, 0 };
        CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        Serial.print("torque:");
        Serial.print(torque, DEC);
        Serial.println("[Nm]");
      } else if (MG_ECU == 0) {
        status = B111;  //rapid discharge
        dispat = 1;
      }
    }
  }
}
