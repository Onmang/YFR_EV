//メイン制御コード/Cpu1

#include <mcp_can.h>
#include <SPI.h>

MCP_CAN CAN(10);          //CAN通信ポート
const int IGNSW_PIN = 2;  //IGNSWのデジタル入力ピン

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
  unsigned long id;  //ID
  byte len;          //フレームの長さ
  byte buf[8];       //CANdata配列:8byte

  unsigned short Motor_cur, Voltage, Anomaly_sig;
  short Motor_rev;
  unsigned short Mg_ecu, Op_status, Gate_sta;

  int IGNSWsta = digitalRead(IGNSW_PIN);  //IGNSWの状態，HIGH or LOW
  static int ECUsta = 0;                  //MG-ECUの状態変数，ON：1，OFF：0
  static int Dissta = 0;                  //Co放電要求の状態変数，Active：1，Inactive：0

  //CAN処理
  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf);
    //ID:785
    if (id == 0x311) {
      
      //MG-ECUシャットダウン許可
      Mg_ecu = bitRead(buf[0], 0);

      //制御状態
      Op_status = buf[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;

      //ゲート駆動状態
      Gate_sta = buf[0];
      for (int i = 3; i < 8; i++) {
        bitClear(Gate_sta, i);  //LSBから3-7ビット目を「0」ビットにする
      }
      Gate_sta = Gate_sta >> 1;

      //モータ回転数[rpm]
      Motor_rev = (buf[2] << 8) | buf[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000

      //モータ相電流
      Motor_cur = (buf[4] << 8) | buf[3];  //モータ相電流3byteと4byteを結合
      for (int m = 10; m < 16; m++) {
        bitClear(Motor_cur, m);  //LSBから10-15ビット目を「0」ビットにする
      }
      Motor_cur = Motor_cur;  //モータ相電流[Arms]

      //モータ電圧
      Voltage = (buf[5] << 8) | buf[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]

      //異常状態 信号
      Anomaly_sig = buf[7] >> 5;  //異常状態 信号
    }
    //ID:801
    else if (id == 0x321) {
      // 温度
      short INV_deg = buf[0] - 40;  //インバータ温度[℃],オフセット-40
      short Mo_deg = buf[4] - 40;   //モータ温度[℃],オフセット-40
      //トルク制限値は必要ないので処理しない
    }
  }
  //IGNSWの状態分岐，HIGH or LOW
  if (IGNSWsta == LOW) {
    //MG-ECUの状態をチェック
    if (ECUsta != 0) {  //MG-ECU"OFF"ではないなら以下を実行
      //"MG-ECU"OFF"送信"
    }
    if (Op_status == B111) {  //rapid discharge状態か？
      if (Dissta != 1) {      //Co放電要求Active済みじゃないなら以下を実行
        if (Voltage >= 60) {
          //"Co放電要求 Active送信"
        }
      }
      if (Op_status == B010) {  //standby状態か？
        //"Co放電要求 Inactive送信"
      }
    }
  } else if (IGNSWsta == HIGH) {
    //torque control状態か？
    if (Op_status == B011) {
      //"トルク値CAN送信プログラム"
    } else if (Op_status == B001) {  //Precharge状態か？
      //if:Cpu5 precharge完了なら以下を実行//
      if (ECUsta != 1) {
        //"MG-ECU"ON"送信"
      }
    }
  }
}  //loop終了
