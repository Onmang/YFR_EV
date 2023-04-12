//ポテションメータのテストコード

const int INPUT_PIN = 0;
int val = 0;


const float min = 1024 * 0.1;
const float max = 1024 * 0.9;
const int deg_0 = 20;
const int deg_m = 40;
const int Tormin = 0;
const int Tormax = 60;

void setup() {
  Serial.begin(9600);
  pinMode(INPUT_PIN, INPUT);
}

void loop() {
  val = analogRead(INPUT_PIN);
  int deg = map(val, min, max, 0, 359);
  int deg_in = constrain(deg, deg_0, deg_m);
  int Torque = map(deg_in, deg_0, deg_m, Tormin, Tormax);
  float Voltage = val * 5.0 / 1024.0;

  Serial.print("val:");
  Serial.print(val);
  Serial.print("  deg:");
  Serial.print(deg);
  //Serial.print("  deg_in:");
  //Serial.print(deg_in);
  Serial.print("  Voltage:");
  Serial.print(Voltage);
  Serial.print("  Torque:");
  Serial.print(Torque);

  Serial.println();


  //delay(100);
}
