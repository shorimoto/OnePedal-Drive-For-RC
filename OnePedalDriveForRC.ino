#include <EEPROM.h>
#include <Servo.h>
#define DEBUG 0
#define MULTIPLE 5
#define BREAKLED 7  //ブレーキ表示用LED
#define RPMSETTINGPIN 8 //最大rpm設定用(PULLUP抵抗有り)
#define RECVSETTINGPIN 9 //受信機側設定用押しボタン入力(PULLUP抵抗有り)
#define RECVPIN 11  //受信機信号入力
#define ESCPIN 12 //ESC出力(Servoライブラリ経由)
#define LEDPIN 13 //ボード上のLED


Servo ESC;
int txValue = 0;  //受信機入力値
int RecvNutralValue = 0;  //受信機入力値：ニュートラル
int RecvForwardValue = 0; //受信機入力値：前進最大値
int RecvBackValue = 0;  //受信機入力値：後進最大値
int intForwardValue = 0;  //受信機入力前進幅
int intBackValue = 0; //受信機入力後進幅
int maxRPM = 0; //ブレーキ量算出用最大RPM
unsigned long beforeTimer;  //前回計算実行時間(μ)
int currentRPM = 0;  //回転数
int RPMTable[50]; //操作量に応じた回転数
volatile int CurrentRPMTable[MULTIPLE + 1]; //回転数取得用テーブル
unsigned long TimerTable[MULTIPLE + 1];  //回転数取得用テーブルに対応した算出時刻テーブル
int RPMTableCounter;  //回転数テーブル読み込み順番
int BreakDelayRPM; //ブレーキ回転数差分
boolean isBreak;  //ブレーキ介入有無  true:有り false:なし


/**
 * 受信機入力を+100～-100で返す
 * pulseIn使用に注意
 */
float InputValue(){
  txValue = pulseIn(RECVPIN , HIGH);
  int a = 0;
  if (txValue > RecvNutralValue){
    a = ((long)(txValue - RecvNutralValue) * 100) / (RecvForwardValue - RecvNutralValue);
  }else{
    a = ((long)(RecvNutralValue - txValue) * 100) / (RecvNutralValue - RecvBackValue);
    a = -1 * a ;
  }
  return a;
}

/**
 * 設定書き込み
 */
int WriteEEPROM(int address , int arg){
  EEPROM.write(address,highByte(arg));
  EEPROM.write(address + 1,lowByte(arg));
}
/**
 * 設定読み込み
 */
int ReadEEPROM(int address){
  byte high = EEPROM.read(address);
  byte low = EEPROM.read(address + 1);
  int ret=word(high,low);
  return ret;
}

/**
 * RPM更新
 */
void updateRPM(){
  unsigned long currentTimer = millis();
  //50ミリ秒経過後にrpmを計算する
  if ((currentTimer - beforeTimer) > 50) {
    currentRPM = CurrentRPMTable[RPMTableCounter] * 60000 / (currentTimer - TimerTable[RPMTableCounter]);
    //算出に使用した変数を初期化
    CurrentRPMTable[RPMTableCounter] = 0;
    TimerTable[RPMTableCounter] = currentTimer;
    if (RPMTableCounter >= MULTIPLE){
      RPMTableCounter = 0;
    }else{
      RPMTableCounter++;
    }
    beforeTimer = currentTimer;
  }
}

/**
 * 割り込み処理
 */
void rpm()
{
  //rCount++;
  for (int i = 0 ; i <= MULTIPLE ; i++){
    CurrentRPMTable[i]++;
  }
}

/**
 * ボタンが開放されるまで待機
 * 手抜きチャタリング対策兼同期PG対策
 */
void BlinkBreakLED(int pinNo){
  while(digitalRead(pinNo) == LOW){
    digitalWrite(BREAKLED , HIGH);
    delay(50);
    digitalWrite(BREAKLED , LOW);
    delay(50);
  }
  delay(50);
}

void setup() {
  pinMode(RECVPIN , INPUT);
  pinMode(LEDPIN , OUTPUT);
  pinMode(BREAKLED , OUTPUT);
  pinMode(RECVSETTINGPIN , INPUT_PULLUP);
  pinMode(RPMSETTINGPIN , INPUT_PULLUP);
  
  ESC.attach(ESCPIN);
  Serial.begin(9600) ;
  
  //========
  //変数初期化
  //========
  beforeTimer = millis();
  //回転数保持用テーブル初期化
  for( int i = 0 ; i <= MULTIPLE ; i++){
    RPMTable[i] = 0;
    TimerTable[i] = 0;
  }
  isBreak = false;
  //割込処理定義
  attachInterrupt(0, rpm, RISING);
  
  //===============================
  //ショートした状態で起動した際は設定変更
  //===============================
  //受信機信号設定
  //設定ボタン押下時、ブレーキ表示用LED点滅→離すと点灯
  if (digitalRead(RECVSETTINGPIN) == LOW){
    BlinkBreakLED(RECVSETTINGPIN);  //ボタンが離されるまで点滅待機
    
    digitalWrite(BREAKLED , HIGH);
    Serial.println ("setting mode");
    for (int i = 0 ; i < 3 ; i++) {
      #if DEBUG
        //モード
        if (i == 0) Serial.println("Nutral");
        else if (i == 1) Serial.println("Forward");
        else if (i == 2) Serial.println("Back");
      #endif
      while (true){
        //モードをLED表示 1回:ニュートラル 2回:前進最大 3回:後進最大
        for (int j = 0 ; j <= i ; j++){
          digitalWrite(LEDPIN , HIGH);
          delay(200);
          digitalWrite(LEDPIN , LOW);
          delay(200);
        }
        int v = pulseIn(RECVPIN , HIGH);
        Serial.println(v);
        //セットボタンが押された場合は設定値を書き込んで次へ
        if (digitalRead(RECVSETTINGPIN) == LOW){
          BlinkBreakLED(RECVSETTINGPIN);
          WriteEEPROM(i * 2 , v);
          break;
        }
        delay(200);
        int j = 0;
      }
    }
    digitalWrite(BREAKLED , LOW);
  }
  
  //最大RPM設定
  //操作量2/90単位で回転数記憶
  //注:0→90と90→0だとかなり回転数異なる
  if (digitalRead(RPMSETTINGPIN) == LOW){
    BlinkBreakLED(RPMSETTINGPIN);
    Serial.println("RPMSetting");
    delay(3000);
    ESC.write(180);
    for (int i = 0 ; i <= MULTIPLE ; i++){
      delay (2000);  
      updateRPM();
      Serial.println(currentRPM);
    }
   
    for (int k = 44 ; k >= 0 ; k--){
      digitalWrite(LEDPIN , HIGH);
      delay(200);
      digitalWrite(LEDPIN , LOW);
      delay(200);
      ESC.write(90 + (k * 2));
      for (int i = 0 ; i <= MULTIPLE ; i++){
        delay(200);
        updateRPM();
        Serial.println(currentRPM);
      }
      delay(200);
      updateRPM();
      Serial.print(k * 2);
      Serial.print(" : ");
      Serial.println(currentRPM);
      WriteEEPROM((4 + k) * 2 , currentRPM);
    }
    ESC.write(90);
  }
  delay(500);
  digitalWrite(BREAKLED , LOW);
  //===================
  //EEPROMから設定読み出し
  //===================
  RecvNutralValue = ReadEEPROM(0 * 2);  //受信機ニュートラル
  RecvForwardValue = ReadEEPROM(1 * 2); //受信機ForwordMAX
  RecvBackValue = ReadEEPROM(2 * 2);  //受信機BackMax
  intForwardValue = RecvForwardValue - RecvNutralValue; //前進最大幅
  intBackValue = RecvNutralValue - RecvBackValue; //後進最大幅
  BreakDelayRPM = ReadEEPROM(3 * 2);  //ブレーキ差分
  for (int i = 0 ; i < 45 ; i++){
    int v = ReadEEPROM((4 + i) * 2);
    RPMTable[i] = v;
    #if DEBUG
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(RPMTable[i]);
    #endif
  }
  #if DEBUG
    Serial.print(RecvBackValue);
    Serial.print(" / ");
    Serial.print(RecvNutralValue);
    Serial.print(" / ");
    Serial.println(RecvForwardValue);
    Serial.println ("start");
  #endif
  //自動ブレーキ開始調整値表示
  for (int i = 0 ; i < BreakDelayRPM ; i++){
    digitalWrite(BREAKLED , HIGH);
    delay(150);
    digitalWrite(BREAKLED , LOW);
    delay(150);
  }
  #if DEBUG
    Serial.print("BreakDelayRPM:");
    Serial.println(BreakDelayRPM);
  #endif
}




void loop() {

  //自動ブレーキ開始調整値設定
  //受信設定押下でモード入り
  //RPM押下で＋調整、10越すと1に戻る
  //受信設定押下でモード出る
  if (digitalRead(RECVSETTINGPIN) == LOW){
    BlinkBreakLED(RECVSETTINGPIN);
    digitalWrite(LEDPIN , HIGH);
    digitalWrite(BREAKLED, LOW);
    delay(50);
    while(true){
      for (int i = 0 ; i < BreakDelayRPM ; i++)
      {
        digitalWrite(BREAKLED , HIGH);
        delay(150);
        digitalWrite(BREAKLED , LOW);
        delay(150);
      }
      delay(500);
      if (digitalRead(RPMSETTINGPIN) == LOW){
        BlinkBreakLED(RPMSETTINGPIN);
        BreakDelayRPM++;
        if (BreakDelayRPM > 10) BreakDelayRPM = 1;
      }else if(digitalRead(RECVSETTINGPIN) == LOW){
        BlinkBreakLED(RECVSETTINGPIN);
        WriteEEPROM(3 * 2 , BreakDelayRPM);
        break;
      }
    }
    digitalWrite(LEDPIN , LOW);
    digitalWrite(BREAKLED, LOW);
  }
  
  int in = InputValue();

  //RPMを更新する
  updateRPM();

  //操作量　-100～100をサーボlib用に-90～90に変更
  int escOutput;  
  escOutput = 90 + (in * 0.9);
  
  //ブレーキ基準となる回転数を取得
  //操作量が前進側5%以上、かつ基準値回転数より現在回転数が調整値以上の場合、ブレーキ量を算出しESC操作量を上書き
  int sp = in * 0.9 / 2;
  if (sp >= 45) sp = 44;
  int RPMStd = RPMTable[sp];
  if (in > 5 && (currentRPM - RPMStd) > (BreakDelayRPM * 200) ){
    //ブレーキ操作% ＝ 100 - (回転数の比率)
    int BreakWidth = -1 * (100 - (((long)RPMStd * 100) / currentRPM));
    escOutput = 90 + (BreakWidth * 0.9);
    #if DEBUG
      Serial.print("!Brake! ");
      Serial.println(BreakWidth);
    #endif
    isBreak = true;
  }else{
    isBreak = false;
  }
  digitalWrite(BREAKLED , isBreak);
  #if DEBUG 
    Serial.print("      ");
    Serial.print(sp);
    Serial.print(" B-RPM:");
    Serial.print(RPMStd);
    Serial.print(" C-RPM:");
    Serial.print(currentRPM);
    Serial.print(" outputESC:");
    Serial.println(escOutput);
    delay(50);
  #endif
  
  //操作信号出力
  ESC.write(escOutput);
}
