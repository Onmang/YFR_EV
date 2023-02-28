//メイン制御コード/Cpu1_Ver_A
//このプログラム概要：
/////////////////////インバータへの指令送信は基本1回行う，送信失敗➡成功するまで送信する．
/////////////////////懸念点：こちらの送信のタイミングとINVの受信タイミングが合わない場合があるかも
//残る課題
//1.トルク値リミットorフィードバックを実装
//2.トルク値の入力範囲を確認，小数点にならない
//3.情報モニター機能

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);              //CAN通信ポート
const int IGNSW_PIN = 2;      //IGNSWのデジタル入力ピン
const int Precharge_PIN = 4;  //Precharge制御マイコンのデジタル入力ピン

const int APPS_PIN = 0;  //APPS,アナログ入力ピン
int val = 0;             //APPS,アナログ入力の変数
float min = 1024 * 0.1;  //APPS,PST360-G2 出力関数：0°＝10%
float max = 1024 * 0.9;  //APPS,PST360-G2 出力関数：360°＝90%
int deg_0 = 20;          //APPS,作動開始角度
int deg_m = 40;          //APPS,作動限界角度
int TorMin = 0;          //APPS,入力値下限
int TorMax = 120;        //APPs,入力値上限

int N_lim = 9000;  //回転数limit[rpm]

void setup() {
  Serial.begin(9600);
  pinMode(IGNSW_PIN, INPUT);
  pinMode(Precharge_PIN, INPUT);

  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
    CAN.setMode(MCP_NORMAL);
  } else {
    Serial.println("Can init fail");
  }
}

void loop() {
  static byte buf_s[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //CAN通信送信バッファ

  unsigned long id;  //ID
  byte len;          //フレームの長さ
  byte buf_r[8];     //CAN通信受信バッファ

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

      //MG-ECUシャットダウン許可
      Mg_ecu = bitRead(buf_r[0], 0);

      //制御状態
      Op_status = buf_r[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;

      //ゲート駆動状態
      Gate_sta = buf_r[0];
      for (int i = 3; i < 8; i++) {
        bitClear(Gate_sta, i);  //LSBから3-7ビット目を「0」ビットにする
      }
      Gate_sta = Gate_sta >> 1;

      //モータ回転数[rpm]
      Motor_rev = (buf_r[2] << 8) | buf_r[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000

      //モータ相電流
      Motor_cur = (buf_r[4] << 8) | buf_r[3];  //モータ相電流3byteと4byteを結合
      for (int m = 10; m < 16; m++) {
        bitClear(Motor_cur, m);  //LSBから10-15ビット目を「0」ビットにする
      }
      Motor_cur = Motor_cur;  //モータ相電流[Arms]

      //モータ電圧
      Voltage = (buf_r[5] << 8) | buf_r[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]

      //異常状態 信号
      Anomaly_sig = buf_r[7] >> 5;  //異常状態 信号
    }
    //ID:801
    else if (id == 0x321) {
      // 温度
      short INV_deg = buf_r[0] - 40;  //インバータ温度[℃],オフセット-40
      short Mo_deg = buf_r[4] - 40;   //モータ温度[℃],オフセット-40
      //トルク制限値は必要ないので処理しない
    }
  }
  //IGNSWの状態分岐，HIGH or LOW
  if (IGNSWsta == LOW) {
    if (ECUsta != 0) {  //MG-ECU"OFF"ではないなら以下を実行
      //"MG-ECU"OFF"送信"
      byte buf_s[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
      if (sndStat == CAN_OK) {
        Serial.println("MG-ECU : OFF");
        ECUsta = 0;
      } else {
        Serial.println("Error Sending Message...");
        ECUsta = 1;
      }
    }
    if (Op_status == B111) {  //rapid discharge状態か？
      if (Dissta_on != 1) {   //Co放電要求Active済みじゃないなら以下を実行
        if (Voltage >= 60) {
          //"Co放電要求 Active送信"
          byte buf_s[] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
          sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
          if (sndStat == CAN_OK) {
            Serial.println("Discharge Command : ON");
            Dissta_on = 1;
          } else {
            Serial.println("Error Sending Message...");
            Dissta_on = 0;
          }
        }
      }
    } else if (Op_status == B010) {  //standby状態か？
      //"Co放電要求 Inactive送信"
      if (Dissta_off != 1) {
        byte buf_s[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
        sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);
        if (sndStat == CAN_OK) {
          Serial.println("Discharge Command : OFF");
          Dissta_off = 1;
        } else {
          Serial.println("Error Sending Message...");
          Dissta_off = 0;
        }
      }
    }
  } else if (IGNSWsta == HIGH) {
    Dissta_on = 0;  //dischargeの指令を初期化
    Dissta_off = 0;
    if (Op_status == B011) {  //torque control状態か？
      //"トルク値CAN送信プログラム"
      byte buf_s[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };  //0byte目はMG-ECU:on, Co放電要求:off 固定
      val = analogRead(APPS_PIN);

      int deg = map(val, min, max, 0, 359);                    //アナログ入力値を角度に置き換え
      int deg_in = constrain(deg, deg_0, deg_m);               //角度の範囲を制限
      int Torque = map(deg_in, deg_0, deg_m, TorMin, TorMax);  //角度をトルク値に変換

      buf_s[1] = Torque;
      sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //トルク値CAN送信
      if (sndStat == CAN_OK) {
        Serial.print("torque : ");
        Serial.println(Torque);
      } else {
        Serial.println("Error Sending Message...");
      }
    } else if (Op_status == B001) {  //Precharge状態か？
      int Presta = digitalRead(Precharge_PIN);
      if (Presta == HIGH) {  //Cpu5:precharge制御は完了か？
        if (ECUsta != 1) {   //MG-ECU"ON"ではないなら以下を実行
          //"MG-ECU"ON"送信"
          byte buf_s[] = { 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
          sndStat = CAN.sendMsgBuf(0x301, 0, 8, buf_s);  //ID, 標準フレーム:0, データ長:8
          if (sndStat == CAN_OK) {
            Serial.println("MG-ECU : ON");
            ECUsta = 1;
          } else {
            Serial.println("Error Sending Message...");
            ECUsta = 0;
          }
        }
      }
    }
  }
}  //loop終了