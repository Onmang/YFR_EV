//ポテションメータのトルク値変換

const int INPUT_PIN = 0;
int val = 0;
float Torque;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(INPUT_PIN);

  float y = val * 120/1023;  //0～120に置き換える
  int z = round(y);              //yを丸める
  Torque = 0.5 * z;              //トルク値, 0～60

  Serial.print("val: ");
  Serial.print(val);

  Serial.print("   y:");
  Serial.print(y);
  Serial.print("   z:");
  Serial.print(z);
  Serial.print("   Torque:");
  Serial.print(Torque);
  Serial.println(" [Nm]");

  //delay(100);
}
