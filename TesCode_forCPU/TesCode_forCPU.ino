//Cpu1とインバータのtest code
//CAN送信はID:785のみ，受信はID:769
//Error 状態は無視
//プログラム起動orリセット後 == GLV:ON 状態
#include <mcp_can.h>
#include <SPI.h>
#include <math.h>

MCP_CAN CAN(10);

const int IGNSW_PIN = 2;  //IGNSWの電圧信号
const int DCmotor_PIN = 4;
float d = -1 / (1500 * 0.0006);  //電圧算出用

//void TS_fall(void);  //割り込み関数,HIGH→LOW
float T[51] = { [0] = 0.0 };

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);
  pinMode(DCmotor_PIN, OUTPUT);
  int k = 0;
  while (T[k] < 5) {
    T[k + 1] = T[k] + 0.1;
    k++;
  }

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
  //attachInterrupt(0, TS_fall, FALLING);
}

void loop() {
  static byte MG_ECU;    //Mg-ECU実行要求，ON：1，OFF：0
  static int TSsta;      //TSの状態
  static int status;     //制御状態
  static int error_sta;  //以上制御
  static byte disoder;   //Co放電要求の状態変数，Active：1，Inactive：0
  static int i, j;
  static int16_t voltage;  //算出用電圧

  static byte buf_s[] = { 0, 0, 0, 0, 0, 0, 0, 0 };  //CAN通信送信バッファ
  unsigned char sndStat;
  unsigned long id;      //ID
  byte len;              //フレームの長さ
  static byte buf_r[8];  //CAN通信受信バッファ


  TSsta = digitalRead(IGNSW_PIN);

  if (TSsta == LOW) {  //LOW
    j = 0;
    CAN.readMsgBuf(&id, &len, buf_r);
    if (bitRead(buf_r[0], 0) == 0) { MG_ECU = 0; }
    if (MG_ECU == 0) {       //MG-ECU : off?
      if (status != B010) {  //Not standby?
        status = B111;       //discharge状態にする
        voltage = 400 * exp(d * T[i]);
        i++;
        byte buf_s[] = { 0x3B, 0, 0, 0, 0, 0, 0, 0 };  //discharge状態を送信, 400V
        buf_s[4] |= (byte)((voltage & 0x3F) << 2);
        buf_s[5] |= (byte)((voltage >> 6) & 0xF);
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.print("rapid discharge, now ");
          Serial.print(voltage);
          Serial.println("[v]");
        } else {
          Serial.println("Error...");
        }
        CAN.readMsgBuf(&id, &len, buf_r);
        if (bitRead(buf_r[0], 1) == 1) { disoder = 1; }
        if (disoder == 1) {  //CO放電要求 Active
          Serial.println("Discharging.....");
          if (voltage < 60) {
            status = B010;                                 //standby状態
            byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby状態を送信
            buf_s[4] |= (byte)((voltage & 0x3F) << 2);
            buf_s[5] |= (byte)((voltage >> 6) & 0xF);
            sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
            if (sndStat == CAN_OK) {
              Serial.println("Move to standby");
            } else {
              Serial.println("Error...");
            }
          }
        } else if (disoder == 0) {
          //Serial.println("Please send [ACTIVE]");
          if (voltage < 60) {
            error_sta = B101;                              //異常状態をcritical errorに更新
            byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby状態を送信
            buf_s[4] |= (byte)((voltage & 0x3F) << 2);
            buf_s[5] |= (byte)((voltage >> 6) & 0xF);
            buf_s[7] = (buf_s[7] | B00000101) << 5;
            sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);  //standby & critical error
            if (sndStat == CAN_OK) {
              Serial.println("Standby, Critical Error!!!  Please send [ACTIVE]");
            } else {
              Serial.println("Error...");
            }
          }
        }

      } else if (status == B010) {  //standby状態
        if (error_sta != B101) {
          byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby状態を送信
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.print("standby, now ");
            Serial.print(voltage);
            Serial.println("[v]");
          } else {
            Serial.println("Error...");
          }
          CAN.readMsgBuf(&id, &len, buf_r);
          if (bitRead(buf_r[0], 1) == 0) { disoder = 0; }
          if (disoder != 0) Serial.println("Please send [INACTIVE]");
        } else if (error_sta == B101) {
          byte buf_s[] = { 0x13, 0, 0, 0, 0, 0, 0, 0 };  //standby
          buf_s[4] |= (byte)((voltage & 0x3F) << 2);
          buf_s[5] |= (byte)((voltage >> 6) & 0xF);
          buf_s[7] = (buf_s[7] | B00000101) << 5;
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);  //standby & critical error
          if (sndStat == CAN_OK) {
            Serial.println("Critical Error!!!  ");
          } else {
            Serial.println("Error...");
          }
        }
      }
    } else if (MG_ECU == 1) {
      Serial.println("Please OFF MG_ECU");
    }
  } else if (TSsta == HIGH) {  //HIGH
    i = 0;
    if (status != B011) {                   //not torque?
      status = B001;                        //precharge状態
      voltage = 400 * (1 - exp(d * T[j]));  //precharge電圧算出
      j++;
      byte buf_s[] = { 0xA, 0, 0, 0, 0, 0, 0, 0 };
      buf_s[4] |= (byte)((voltage & 0x3F) << 2);
      buf_s[5] |= (byte)((voltage >> 6) & 0xF);
      sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);  //precharge状態を送信
      if (sndStat == CAN_OK) {
        Serial.print("Precharging....   ");
        Serial.println(voltage, DEC);
      } else {
        Serial.println("Error...");
      }
      if (voltage > 340) {  //precharge判定条件
        CAN.readMsgBuf(&id, &len, buf_r);
        if (bitRead(buf_r[0], 0) == 1) MG_ECU = 1;
        if (MG_ECU == 1) {  //MG_ECU on?
          status = B011;    //torque control状態にする
          //voltage = 400;
          byte buf_s[] = { B00011100, 0, 0, 0, 0, 0, 0, 0 };
          buf_s[4] |= (byte)((voltage & 0x3F) << 2);
          buf_s[5] |= (byte)((voltage >> 6) & 0xF);
          sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);  //torque control状態を送信
          if (sndStat == CAN_OK) {
            Serial.println("MG-ECU : ON >>>>>>  Move to torque control");
          } else {
            Serial.println("Error...");
          }
        }
      }
    } else if (status == B011) {  //torque control状態
      if (MG_ECU == 1) {
        byte buf_s[] = { B00011100, 0, 0, 0, 0, 0, 0, 0 };
        buf_s[4] |= (byte)((voltage & 0x3F) << 2);
        buf_s[5] |= (byte)((voltage >> 6) & 0xF);
        sndStat = CAN.sendMsgBuf(0x311, 0, 8, buf_s);  //torque control状態を送信
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

// void TS_fall(void) {  //HIGH→LOW
//   analogWrite(DCmotor_PIN, 0);

//   Serial.println("TS : OFF");
// }
