//Cpu5＿無負荷試験モニター用
//すべてのIDを受信
#include <mcp_can.h>
#include <SPI.h>
#include <LiquidCrystal.h>
// LCD ←→ Arduinoのピンの割り当て
// rs      →   D7
// rw      →   GND
// enable  →   D8
// d4      →   D9
// d5      →   D10
// d6      →   D11
// d7      →   D12

// [構文]LiquidCrystal(rs, rw,  enable, d0, d1, d2, d3, d4, d5, d6, d7)

LiquidCrystal lcd(7, 8, 9, 10, 11, 12);
MCP_CAN CAN(10);

void setup() {
  Serial.begin(9600);
  if (CAN.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    CAN.setMode(MCP_NORMAL);
    //CAN.begin(IDの種類, canの通信速度, モジュールとの通信レート（ex:水晶発振子の周波数）)
  } else {
    Serial.println("Can init fail");
  }
  lcd.begin(16, 2);  // LCDの桁数と行数を指定する(16桁2行)
  lcd.clear();       // LCD画面をクリア
}

void loop() {
  unsigned long id;
  byte len;
  byte buf[8];
  unsigned short Voltage, Op_status;
  short Motor_rev;
  float torque;

  if (CAN.checkReceive() == CAN_MSGAVAIL) {
    CAN.readMsgBuf(&id, &len, buf);
    //ID:785
    if (id == 0x311) {
      Op_status = buf[0];
      for (int n = 6; n < 8; n++) {
        bitClear(Op_status, n);  //LSBから6-7ビット目を「0」ビットにする
      }
      Op_status = Op_status >> 3;

      Motor_rev = (buf[2] << 8) | buf[1];
      Motor_rev = Motor_rev - 14000;  //モータ回転数[rpm],オフセット-14000

      Voltage = (buf[5] << 8) | buf[4];  //モータ電圧4byteと5byteを結合
      for (int j = 12; j < 16; j++) {
        bitClear(Voltage, j);  //LSBから12-15ビット目を「0」ビットにする
      }
      Voltage = Voltage >> 2;  //モータ電圧[V]

      //制御状態
      lcd.setCursor(0, 0);
      if (Op_status == B000) {
        lcd.print("INT");  //init
      } else if (Op_status == B001) {
        lcd.print("PRE");  //Precharge
      } else if (Op_status == B010) {
        lcd.print("STB");  //Standby
      } else if (Op_status == B011) {
        lcd.print("TOR");  //Torque control
        lcd.setCursor(0, 1);
        lcd.print(Motor_rev, DEC);
        lcd.print(" rpm");
      } else if (Op_status == B111) {
        lcd.print("DIS");  //Rapid discharge
      }
      //モータ状態
      lcd.setCursor(6, 0);
      lcd.print("Volt:");
      lcd.print(Voltage, DEC);
      lcd.print(" V");
    }
    //ID:769
    else if (id == 0x301) {
      torque = 0.5 * buf[1];
      int torque = round(torque);
      lcd.setCursor(11, 1);
      lcd.print(torque, DEC);
      lcd.print(" Nm");
    }
  }

}  //<<loop終了
