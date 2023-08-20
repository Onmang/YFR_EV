/*メイン制御コード/Cpu1_Ver_A
このプログラム概要：
                  インバータへの指令送信は基本1回行う，送信失敗➡成功するまで送信する．
                  懸念点：こちらの送信のタイミングとINVの受信タイミングが合わない場合があるかも(現時点で問題は確認されない)
トルク値リミットorフィードバックを実装
*/

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);  //CAN通信ポート

const int IGNSW_PIN = 5;      //IGNSWのデジタル入力ピン
const int Precharge_PIN = 4;  //Precharge制御マイコンのデジタル入力ピン
const int PreLED_PIN = 2;     //Precharge完了LED出力ピン

const int RtoD_PIN = 9;     //Ready to Drive 入力ピン
const int RtoDLED_PIN = 3;  //Ready to Drive LED出力ピン

//####APPS_1
const int APPS_PIN_1 = A4;  //APPS,アナログ入力ピン
int APPS_val_1 = 0;         //APPS,アナログ入力の変数

/*
//####APPS_2
const int APPS_PIN_2 = A5;  //APPS,アナログ入力ピン
int APPS_val_2 = 0;         //APPS,アナログ入力の変数
*/
//####BSE
const int Brake_PIN = A6;    //Brake, アナログ入力ピン
int Brake_val = 0;           //Brake,アナログ入力の変数
int Breke_RtoD_Thold = 500;  // Brake, Ready to Drive用の閾値



const float min_val = 1024 * 0.1;  //APPS,PST360-G2 出力関数：0°＝10%
const float max_val = 1024 * 0.9;  //APPS,PST360-G2 出力関数：360°＝90%
const int deg_0 = 20;              //APPS,作動開始角度
const int deg_m = 30;              //APPS,作動限界角度
const int TorMin = 2000;           //APPS,入力値下限, DEC:2000～2120, HEX:0x7d0～0x848, INV:0～60[Nm]
const int TorMax = 2060;           //APPs,入力値上限

//回転数リミッターの変数
//int T_delta = TorMin;
//const int N_lim = 9000;  //回転数limit[rpm]

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);
  pinMode(Precharge_PIN, INPUT);
  pinMode(PreLED_PIN, OUTPUT);
  pinMode(APPS_PIN_1, INPUT);
  //pinMode(APPS_PIN_2, INPUT);
  pinMode(RtoD_PIN, INPUT);
  pinMode(RtoDLED_PIN, OUTPUT);
  pinMode(Brake_PIN, INPUT);


  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //CAN通信送信バッファ
  unsigned long id;                                                          //ID
  byte len;                                                                  //フレームの長さ
  byte buf_r[8];                                                             //CAN通信受信バッファ

  unsigned short Motor_cur, Voltage, Anomaly_sig;
  short Motor_rev;
  unsigned short Gate_sta;
  static unsigned short Mg_ecu, Op_status;
  unsigned char sndStat;

  int IGNSWsta = digitalRead(IGNSW_PIN);  //IGNSWの状態，HIGH or LOW
  static int ECUsta = 0;                  //MG-ECUの状態変数，ON：1，OFF：0
  static int Dissta_on = 0;               //Co放電要求 Active状態 1:送信済み or 0：未送信
  static int Dissta_off = 0;              //Co放電要求 Inactive状態 1：送信済み or 0：未送信
  //CAN処理
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf_r);
    //ID:785
    if (id == 0x311) {
      if (1) {
        Serial.print("ID:");
        Serial.print(id);
        Serial.print(" => ");
      }
      //MG-ECUシャットダウン許可
      Mg_ecu = bitRead(buf_r[0], 0);
      if (0) {
        Serial.print("ECUsd:");
        if (Mg_ecu == B0) {
          Serial.print("Not Enable;  ");
        } else if (Mg_ecu == B1) {
          Serial.print("Enable;  ");
        }
      }

      //制御状態
      Op_status = buf_r[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;
      //モニター使用時はif(1), 不使用はif(0)
      if (1) {
        Serial.print("Status:");
        if (Op_status == B000) {
          Serial.print("init;  ");  //init
        } else if (Op_status == B001) {
          Serial.print("Precharge;  ");  //Precharge
        } else if (Op_status == B010) {
          Serial.print("Standby;  ");  //Standby
        } else if (Op_status == B011) {
          Serial.print("Troque Contorol;  ");  //Torque control
        } else if (Op_status == B111) {
          Serial.print("Rapid Discharge;  ");  //Rapid discharge
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }
      //ゲート駆動状態
      Gate_sta = buf_r[0];
      for (int i = 3; i < 8; i++) {
        bitClear(Gate_sta, i);  //LSBから3-7ビット目を「0」ビットにする
      }
      Gate_sta = Gate_sta >> 1;
      if (0) {
        Serial.print("gate:");
        if (Gate_sta == B00) {
          Serial.print("Short;  ");  //Short circuit
        } else if (Gate_sta == B01) {
          Serial.print("Free;  ");  //freeWheel
        } else if (Gate_sta == B10) {
          Serial.print("Run;  ");  //PWM run
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }

      //モータ回転数[rpm]
      Motor_rev = (buf_r[2] << 8) | buf_r[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000
      if (1) {
        Serial.print("Rev:");
        Serial.print(Motor_rev);
        Serial.print(" [rpm];  ");
      }

      //モータ相電流
      Motor_cur = (buf_r[4] << 8) | buf_r[3];  //モータ相電流3byteと4byteを結合
      for (int m = 10; m < 16; m++) {
        bitClear(Motor_cur, m);  //LSBから10-15ビット目を「0」ビットにする
      }
      Motor_cur = Motor_cur * 0.5;  //モータ相電流[Arms]
      if (0) {
        Serial.print("Cur:");
        Serial.print(Motor_cur);
        Serial.print(" [Arms];  ");
      }

      //モータ電圧
      Voltage = (buf_r[5] << 8) | buf_r[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]
      if (1) {
        Serial.print("Vol:");
        Serial.print(Voltage);
        Serial.print(" [V];  ");
      }

      //異常状態 信号
      Anomaly_sig = buf_r[7] >> 5;  //異常状態 信号
      if (1) {
        Serial.print("Error: ");
        if (Anomaly_sig == B000) {
          Serial.print("No Error;  ");
        } else if (Anomaly_sig == B001) {
          Serial.print("power limit;  ");  //derating, モータ出力制限
        } else if (Anomaly_sig == B010) {
          Serial.print("Warning;  ");
        } else if (Anomaly_sig == B100) {
          Serial.print("Error;  ");
        } else if (Anomaly_sig == B101) {
          Serial.print("Critical Error;  ");
        } else {
          Serial.print("--;  ");  //--:Reserved
        }
      }
      Serial.println();

    }
    //ID:801
    else if (id == 0x321) {
      if (1) {
        Serial.print("ID:");
        Serial.print(id);
        Serial.print(" => ");
      }
      // 温度
      short INV_deg = buf_r[0] - 40;  //インバータ温度[℃],オフセット-40
      short Mo_deg = buf_r[4] - 40;   //モータ温度[℃],オフセット-40
      if (1) {
        Serial.print("INV:");
        Serial.print(INV_deg, DEC);
        Serial.print(" [℃];  ");
      }
      if (1) {
        Serial.print("Motor:");
        Serial.print(Mo_deg, DEC);
        Serial.print(" [℃];  ");
      }
      Serial.println();
      //"トルク制限値"は必要ないので処理しない
    }
  }
  //IGNSWの状態分岐，HIGH or LOW
  if (IGNSWsta == LOW) {
    digitalWrite(PreLED_PIN, LOW);                                        //PrechargeLED : OFF (-)
    digitalWrite(RtoDLED_PIN, LOW);                                       //RtoDLED : OFF (-)
    if (ECUsta != 0) {                                                    //MG-ECU"OFF"ではないなら以下を実行
      byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //"MG-ECU"OFF"送信"
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("MG-ECU : OFF");
        ECUsta = 0;
      } else {
        Serial.println("MG-ECU (ON) : Error Sending Message...");
        ECUsta = 1;
      }
    }
    if (Op_status == B111) {  //rapid discharge状態
      if (Dissta_on != 1) {   //Co放電要求Active済みじゃないなら以下を実行
        if (Voltage >= 60) {
          byte buf_s[] = { 0x02, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //"Co放電要求 Active送信"
          sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Discharge Command : ON");
            Dissta_on = 1;
          } else {
            Serial.println("Dis command (ON): Error Sending Message...");
            Dissta_on = 0;
          }
        }
      }
    } else if (Op_status == B010) {  //standby状態
      //"Co放電要求 Inactive送信"
      if (Dissta_off != 1) {
        byte buf_s[] = { 0x00, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("Discharge Command : OFF");
          Dissta_off = 1;
        } else {
          Serial.println("Dis command (OFF): Error Sending Message...");
          Dissta_off = 0;
        }
      }
    }
  } else if (IGNSWsta == HIGH) {
    if (0) {
      Serial.print("ID:301 => ");
    }
    Dissta_on = 0;  //dischargeの指令を初期化
    Dissta_off = 0;
    if (Op_status == B011) {  //torque control状態
      //"トルク値CAN送信プログラム"
      // トルク送信値の範囲⇒DEC:2000～2120, HEX:0x7d0～0x848
      digitalWrite(RtoDLED_PIN, HIGH);                                    //RtoDLED : OFF (グリーン)
      digitalWrite(PreLED_PIN, LOW);                                      //PrechargeLED : OFF (-) ※念のため
      byte buf_s[] = { 0x01, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };  //0byte目はMG-ECU:on, Co放電要求:off 固定

      APPS_val_1 = analogRead(APPS_PIN_1);
      //APPS_val_2 = analogRead(APPS_PIN_2);

      int deg = map(APPS_val_1, min_val, max_val, 0, 359);     //アナログ入力値を角度に置き換え
      int deg_in = constrain(deg, deg_0, deg_m);             //角度の範囲を制限
      int T_in = map(deg_in, deg_0, deg_m, TorMin, TorMax);  //角度をトルク値に変換

      /*
      ///////回転数リミッター
      if (T_delta > T_in) {
        T_delta = T_in;
      } else if (Motor_rev > N_lim) {
        if (T_in > T_delta) {
          T_delta++;
        } else if (T_delta > 2000) {
          T_delta--;
        }
      }
      int T_out = T_in - T_delta;
      ///////回転数リミッター
      */

      int T_out = T_in;  //回転数リミッターなし

      buf_s[1] = byte(T_out & 0xFF);
      buf_s[2] = byte((T_out >> 8) & 0xF);
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //トルク値CAN送信

      if (sndStat == CAN_OK) {
        if (0) {
          Serial.print("Tor:");
          Serial.print(T_out * 0.5, DEC);
          Serial.println(" [Nm]  ");
        }
      } else {
        Serial.println("Torque : Error Sending Message...");
      }
    } else if (Op_status == B001) {  //Precharge状態
      int Presta = digitalRead(Precharge_PIN);
      if (Presta == HIGH) {              //Cpu5:precharge制御は完了か？
        digitalWrite(PreLED_PIN, HIGH);  //PrechargeLED : ON (オレンジ)
        if (ECUsta != 1) {               //MG-ECU"OFF"なら以下を実行
          Brake_val = analogRead(Brake_PIN);
          int RtoD = digitalRead(RtoD_PIN);
          // Ready to Drive の操作
          if (Brake_val > Breke_RtoD_Thold && RtoD == HIGH) {
            //"MG-ECU"ON"送信"
            byte buf_s[] = { 0x01, 0xD0, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
            sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //ID, 標準フレーム:0, データ長:8
            if (sndStat == CAN_OK) {
              Serial.println("MG-ECU : ON");
              ECUsta = 1;
              digitalWrite(PreLED_PIN, LOW);  //PrechargeLED : OFF (-)
            } else {
              Serial.println("MG-ECU (ON) : Error Sending Message...");
              ECUsta = 0;
            }
          }
        }
      }
    }
  }
}  //loop終