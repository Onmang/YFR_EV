//Cpu1とインバータのtest code
//CAN送信はID:785のみ，受信はID:769
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);

const int toCPU1 = 2;  //IGNSWのデジタル出力ピン
const int presta = 8;  //precharge 制御ピンもどき

void setup() {
  Serial.begin(9600);
  pinMode(toCPU1, OUTPUT);
  pinMode(presta, OUTPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte ignsw;
  static byte MG_ECU;      //Mg-ECU実行要求，ON：1，OFF：0
  static byte dispat = 0;  //dischargeのパターン，0：初期化後，1：高電圧印加後（通常時）
  static byte presen = 0;
  static byte status;   //制御状態
  static byte disoder;  //Co放電要求の状態変数，Active：1，Inactive：0
  unsigned char sndStat;

  static byte buf_s[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN通信送信バッファ

  unsigned long id;      //ID
  byte len;              //フレームの長さ
  static byte buf_r[8];  //CAN通信受信バッファ

  char val = Serial.read();

  if (val == '0') {
    ignsw = 0;
  } else if (val == '1') {
    ignsw = 1;
  }
  if (ignsw == 0) {
    digitalWrite(presta, LOW);
    digitalWrite(toCPU1, LOW);
    Serial.println("TS : now OFF");

    CAN.readMsgBuf(&id, &len, buf_r);
    MG_ECU = bitRead(buf_r[0], 0);
    if (MG_ECU == 0) {  //MG-ECU : off?
      if (dispat == 1) {
        status == B111;                                                //rapid discharge状態
        byte buf_s[] = { 0x3B, 0, 0, 0, B10010000, B00000001, 0, 0 };  //discharge状態を送信, 400V
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("rapid discharge, 400V");
        } else {
          Serial.println("Error...");
        }
        CAN.readMsgBuf(&id, &len, buf_r);
        disoder = bitRead(buf_r[0], 1);
        if (disoder == B10) {  //CO放電要求 Active
          Serial.println("Discharging.....");
          delay(2500);
          Serial.println("Now Standby");
          CAN.readMsgBuf(&id, &len, buf_r);
          disoder = bitRead(buf_r[0], 1);
          byte buf_s[] = { 0x13, 0xFF, 0xFF, 0, 0, 0, 0, 0xA0 };  //standby 状態を送信
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("standby");
          } else {
            Serial.println("Error...");
          }
          dispat = 0;
        }
      } else if (dispat == 0) {
        if (presen != 1) {
          byte buf_s[] = { 0xA, 0xB0, 0x36, 0x4, 0, 0, 0, 0 };
          for (int i = 0; i < 3; i++) {
            sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
            if (sndStat == CAN_OK) {
              Serial.println("Successfully Sent");
              //Serial.println(status, BIN);
            } else {
              Serial.println("Error...");
            }
          }
          presen = 1;
        }
        status = B111;                                 //rapid discharge状態
        byte buf_s[] = { 0x3B, 0, 0, 0, 0, 0, 0, 0 };  //discharge状態を送信
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("normaly discharge");
          //Serial.println(status, BIN);
        } else {
          Serial.println("Error...");
        }
      }
    }
  } else if (ignsw == 1) {
    digitalWrite(toCPU1, HIGH);
    //Serial.println("TS : ON");
    if (status != B011) {  //torque状態じゃないなら以下を実行する
      status = B001;
      byte buf_s[] = { 0xA, 0, 0, 0, 0, 0, 0, 0 };
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Precharging....");
        //Serial.println(status, BIN);
        delay(3000);
        Serial.println("to torque control");  ////
        digitalWrite(presta, HIGH);
        //Serial.println(status,BIN);  ////
      } else {
        Serial.println("Error...");
      }

      CAN.readMsgBuf(&id, &len, buf_r);
      MG_ECU = bitRead(buf_r[0], 0);
      if (MG_ECU == 1) {
        status = B011;                                                      //torque control 状態にする
        byte buf_s[] = { B00011100, 0, 0, 0, B10010000, B00000001, 0, 0 };  //400V
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("MG-ECU : ON || Torque control");
          //Serial.println(status,BIN);
        } else {
          Serial.println("Error...");
        }
        dispat = 1;
      } else if (MG_ECU == 0) {
        status = B001;  //precharge 状態
      }
    } else if (status == B011) {  //torque control 状態
      if (MG_ECU == 1) {
        byte buf_s[] = { B00011100, 0, 0, 0, B10010000, B00000001, 0, 0 };
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.print("Torque control ||  ");
        } else {
          Serial.println("Error...");
        }
        CAN.readMsgBuf(&id, &len, buf_r);
        int torque = buf_r[1];
        //Serial.print("torque:");
        Serial.print(torque, DEC);
        Serial.println("[Nm]");
      }
    }
  }
}  //<==loop終了
