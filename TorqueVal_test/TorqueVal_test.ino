//ポテションメータのトルク値変換

const int INPUT_PIN = 0;  //アナログ入力ピン
int Val = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  Val = analogRead(INPUT_PIN);
  float AnaMin = 1023.0 * 0.1;          //出力関数：0°＝10%
  float AnaMax = 1023.0 * 0.9;          //出力関数：360°＝90%
  float a = 120.0 / (AnaMax - AnaMin);  //傾き,120:最大トルク60の2倍
  float b = -a * AnaMin;

  int y = a * Val + b;     //トルク範囲に置き換える0～120
  float Torque = 0.5 * y;  //トルク値, 0～60

  Serial.print("Val: ");
  Serial.print(Val);
  Serial.print("   y:");
  Serial.print(y);
  Serial.print("   Torque:");
  Serial.print(Torque);
  Serial.println(" [Nm]");

  //delay(100);
}
