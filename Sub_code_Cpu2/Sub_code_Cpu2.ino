//CPU2,precharge制御プログラム
//このプログラム概要：
/////////////////////GLV:onでマイコン起動，TS:onでprecharge制御開始
/////////////////////制御時間2.5s, 制御完了後0.5秒delayしてCPU１にdigitalwriteで知らせる

const int IGNSW_PIN = 2;     //IGNSWのデジタル入力ピン,割り込み
const int AIR_PIN = 7;       //AIR+の制御ピン
const int PRErelay_PIN = 8;  //precharge relayの制御ピン
const int PREsend_PIN = 4;   //CPU1につなげるピン
int Presta = 0;  //precharge実行したかを判断する, 1:実行済み，0：未実行
int TS = 0;

void PreStop(void);  //割り込み関数

void setup() {
  pinMode(IGNSW_PIN, INPUT);
  pinMode(AIR_PIN, OUTPUT);
  pinMode(PRErelay_PIN, OUTPUT);
  pinMode(PREsend_PIN, OUTPUT);

  attachInterrupt(0, PreStop, FALLING);

  digitalWrite(PRErelay_PIN, LOW);
  digitalWrite(AIR_PIN, LOW);
  delay(500);  //プログラムが動いてから急にリレーが閉じないようにする
}

void loop() {
  TS = digitalRead(IGNSW_PIN);

  if (TS == HIGH) {
    if (Presta != 1) {
      digitalWrite(PRErelay_PIN, HIGH);
      digitalWrite(AIR_PIN, LOW);
      delay(3000);
      digitalWrite(PRErelay_PIN, LOW);
      digitalWrite(AIR_PIN, HIGH);

      digitalWrite(PREsend_PIN, HIGH);  //Cpu1に送信
      Presta = 1;
    }
  } else if (TS == LOW) {
    digitalWrite(PRErelay_PIN, LOW);
    digitalWrite(AIR_PIN, LOW);
    digitalWrite(PREsend_PIN, LOW);
    Presta = 0;
  }
}  //loop終了

void PreStop(void) {
  digitalWrite(PRErelay_PIN, LOW);
  digitalWrite(AIR_PIN, LOW);
  digitalWrite(PREsend_PIN, LOW);
}