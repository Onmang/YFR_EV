//Cpu1とインバータのtest code
//CAN送信はID:785のみ，受信はID:769
//Error 状態は無視
//プログラム起動orリセット後 == GLV:ON 状態
#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);

const int IGNSW_PIN = 2;  //IGNSWの電圧信号
const int DCmotor_PIN = 4;

//void TS_rise(void);  //割り込み関数,LOW→HIGH
void TS_fall(void);  //割り込み関数,HIGH→LOW
int TSsta = 0;
int status = 0;  //制御状態
int OpeSta = 0;
int Va;

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);
  pinMode(DCmotor_PIN, OUTPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
  //attachInterrupt(0, TS_rise, RISING);
  attachInterrupt(0, TS_fall, FALLING);
}

void loop() {
  static byte MG_ECU;  //Mg-ECU実行要求，ON：1，OFF：0


  static byte disoder;  //Co放電要求の状態変数，Active：1，Inactive：0
  unsigned char sndStat;

  static byte buf_s[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN通信送信バッファ

  unsigned long id;      //ID
  byte len;              //フレームの長さ
  static byte buf_r[8];  //CAN通信受信バッファ

  TSsta = digitalRead(IGNSW_PIN);
  if (TSsta == HIGH) {
    OpeSta = 1;
    //status = B001;  //TS on ⇒ precharge状態
    Va = 1;
    //Serial.println("TS : ON");
  } else if (TSsta == LOW) {
    OpeSta = 0;
    //status = B111;  //TS off ⇒ discharge状態
    //Serial.println("TS : OFF");
  }

  if (OpeSta == 0) {  //LOW
    CAN.readMsgBuf(&id, &len, buf_r);
    MG_ECU = bitRead(buf_r[0], 0);
    if (MG_ECU == 0) {                                                   //MG-ECU : off?
      if (status != B010) {                                              //Not standby?
        if (Va == 1) {                                                   //残留電圧＝400v?
          byte buf_s[] = { 0x3B, 0, 0, 0, B01000000, B00000110, 0, 0 };  //discharge状態を送信, 400V
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("rapid discharge, 400V");
          } else {
            Serial.println("Error...");
          }
        } else if (Va == 0) {
          byte buf_s[] = { 0x3B, 0, 0, 0, B11101100, B00000000, 0, 0 };  //discharge状態を送信, 59V
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("rapid discharge, <60V");
          } else {
            Serial.println("Error...");
          }
        }
        CAN.readMsgBuf(&id, &len, buf_r);
        disoder = bitRead(buf_r[0], 1);
        if (disoder == 1) {  //CO放電要求 Active
          Serial.println("Discharging.....");
          delay(2500);                                   //discharge時間
          status = B010;                                 //standby 状態
          byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby 状態を送信,
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Transision to standby");
          } else {
            Serial.println("Error...");
          }
        } else if (disoder == 0) {
          status = B111;  //discharge状態にする
        }
      } else if (status == B010) {                     //standby状態?
        byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby 状態を送信
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("standby");
        } else {
          Serial.println("Error...");
        }
        CAN.readMsgBuf(&id, &len, buf_r);
        disoder = bitRead(buf_r[0], 1);
        if (disoder != 0) { Serial.println("Please send [INACTIVE]"); }
      }
    }
  } else if (OpeSta == 1) {  //HIGH
    if (status != B011) {    //not torque?
      byte buf_s[] = { 0xA, 0, 0, 0, 0, 0, 0, 0 };
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("Precharging....");
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
          Serial.println("MG-ECU : ON  || Transision Torque control");
        } else {
          Serial.println("Error...");
        }
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
        Serial.println(torque, DEC);
        int y = map(torque, 0, 120, 0, 255);
        analogWrite(DCmotor_PIN, y);
      }
    }
  }
}  //<==loop終了

void TS_fall(void) {  //HIGH→LOW
  analogWrite(DCmotor_PIN, 0);
  
  Serial.println("TS : OFF");
}
// void TS_rise(void) {  //LOW→HIGH
//   TSsta = 1;
//   status = B001;  //TS on ⇒ precharge状態
//   Va = 1;
//   Serial.println("TS : ON");
// }
