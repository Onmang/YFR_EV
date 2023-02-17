//ポテションメータのテストコード

const int INPUT_PIN = 0;
int val = 0;
float voltage;

void setup() {
  Serial.begin(9600);
}

void loop() {
  val = analogRead(INPUT_PIN);
  voltage = val * 5.0 / 1024.0;

  Serial.print("val: ");
  Serial.print(val);
  Serial.print("    Volt");
  Serial.print(voltage);
  Serial.println("[V]");

    delay(100);
}
