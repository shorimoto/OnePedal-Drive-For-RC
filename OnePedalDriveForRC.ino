#include <EEPROM.h>
#include <Servo.h>
#define DEBUG 0
#define MULTIPLE 5  //RPM取得多重度
#define RPMSTEP 15  //回転数記憶時の差分(μs)
#define RPMTABLESIZE 120 //RPM記憶領域配列要素数

#define BTNPIN1 16 //受信機側設定用押しボタン入力(PULLUP抵抗有り)
#define BTNPIN2 8 //最大rpm設定用(PULLUP抵抗有り)
#define LEDPIN1 15  //ブレーキ表示用LED
#define LEDPIN2 6  //LED1
#define LEDPINB 13 //ボード上のLED

#define RECVPIN 10  //受信機信号入力
#define ESCPIN 9 //ESC出力(Servoライブラリ経由)


Servo ESC;
unsigned long RecvNutralValue = 0;  //受信機入力値：ニュートラル
unsigned long RecvForwardValue = 0; //受信機入力値：前進最大値
unsigned long RecvBackValue = 0;  //受信機入力値：後進最大値

unsigned long beforeTimer;  //前回計算実行時間(μ)
int currentRPM = 0;  //回転数
int RPMTable[RPMTABLESIZE]; //操作量に応じた回転数//ざっくり50も有れば足りる
volatile int CurrentRPMTable[MULTIPLE + 1]; //回転数取得用テーブル
unsigned long TimerTable[MULTIPLE + 1];  //回転数取得用テーブルに対応した算出時刻テーブル
int RPMTableCounter;  //回転数テーブル読み込み順番
int BreakForce; //ブレーキ回転数差分
int BreakDepth; //自動ブレーキ最大量
boolean isBreak;  //ブレーキ介入有無  true:有り false:なし
int AttachedBtnFlg = 0; //割り込みボタン入力

/**
 * 設定書き込み
 */
void WriteEEPROM_Int(int address , int arg){
  EEPROM.write(address,highByte(arg));
  EEPROM.write(address + 1,lowByte(arg));
}
/**
 * 設定書き込み
 */
void WriteEEPROM_Long(int address , long arg){
  for (byte i = 0 ; i < 4 ; i++)
  {
    EEPROM[address + i] = (arg >> (i * 8)) & B11111111;
  }
}

/**
 * 設定読み込み
 */
int ReadEEPROM_Int(int address){
  byte high = EEPROM.read(address);
  byte low = EEPROM.read(address + 1);
  int ret=word(high,low);
  return ret;
}
/**
 * 設定読み込み
 */
long ReadEEPROM_Long(int address){
  long ret = 0;
  for (byte i = 0 ; i < 4 ; i++)
  {
    ret += EEPROM[address + i] << (i * 8);
  }
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
 * 割り込み処理　回転数
 */
void rpm()
{
  //rCount++;
  for (int i = 0 ; i <= MULTIPLE ; i++){
    CurrentRPMTable[i]++;
  }
}
/**
 * 割り込み処理　ボタン
 */
void BtnPushed()
{
  if (digitalRead(BTNPIN1) == 0) AttachedBtnFlg |= 1;
  if (digitalRead(BTNPIN2) == 0) AttachedBtnFlg |= 2;
}

/**
 * ボタンが開放されるまで待機
 * 手抜きチャタリング対策兼同期PG対策
 */
void BlinkLED(int pinNo , int ledPinNo){
  while(digitalRead(pinNo) == LOW){
    digitalWrite(ledPinNo , HIGH);
    delay(50);
    digitalWrite(ledPinNo , LOW);
    delay(50);
  }
  delay(50);
}

/**
 * ステータス表示を繰り返す
 * ボタン押下で戻る
 */
void SteteLED(int counter1 , int counter2)
{
  int c = counter1;
  if (c < counter2) c = counter2;
  
  while(true){
    for(int i = 0 ; i < c ; i++)
    {
      Serial.print("SteteLED  ");
      Serial.print(counter1);
      Serial.print(":");
      Serial.print(counter1);
      Serial.print(":");
      Serial.print(i);
      Serial.print(":");
      Serial.println(c);
      if (AttachedBtnFlg != 0) break;
      if (i < counter1) digitalWrite(LEDPIN1 , HIGH);
      if (i < counter2) digitalWrite(LEDPIN2 , HIGH);
      delay(200);
      if (AttachedBtnFlg != 0) break;
      if (i < counter1) digitalWrite(LEDPIN1 , LOW);
      if (i < counter2) digitalWrite(LEDPIN2 , LOW);
      delay(200);
    }
    if (AttachedBtnFlg != 0) break;
    delay(200);
    if (AttachedBtnFlg != 0) break;
    delay(200);
  }
}

void setup() {
  delay(1000);
  #if DEBUG
    while (!Serial)
  #endif

  pinMode(RECVPIN , INPUT);
  pinMode(LEDPINB , OUTPUT);
  pinMode(LEDPIN1 , OUTPUT);
  pinMode(LEDPIN2 , OUTPUT);
  pinMode(BTNPIN1 , INPUT_PULLUP);
  pinMode(BTNPIN2 , INPUT_PULLUP);
  pinMode(2 , INPUT_PULLUP);  //割り込み
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
  BreakDepth = 10;
  BreakForce = 1;
  //割込処理定義
  attachInterrupt(0, rpm, RISING);  //3
  attachInterrupt(1, BtnPushed, FALLING);  //2
  
  //===============================
  //ショートした状態で起動した際は設定変更
  //===============================
  //受信機信号設定
  //設定ボタン押下時、ブレーキ表示用LED点滅→離すと点灯
  if (digitalRead(BTNPIN1) == LOW){
    BlinkLED(BTNPIN1 , LEDPIN2);  //ボタンが離されるまで点滅待機
    
    digitalWrite(LEDPIN1 , HIGH);
    Serial.println ("setting mode");
    for (int i = 0 ; i < 3 ; i++) {
      //モード
      if (i == 0) Serial.println("Nutral");
      else if (i == 1) Serial.println("Forward");
      else if (i == 2) Serial.println("Back");
      while (true){
        AttachedBtnFlg = 0; //割り込みリセット
        //モードをLED表示 1回:ニュートラル 2回:前進最大 3回:後進最大
        SteteLED(i , 0);
        if (AttachedBtnFlg > 0) {
          int v = pulseIn(RECVPIN , HIGH);
          Serial.println(v);
          WriteEEPROM_Int(i * 2 , v);
          break;
        }
      }
    }
    digitalWrite(LEDPIN1 , LOW);
  }
  //設定値読み込み
  RecvNutralValue = ReadEEPROM_Int(0 * 2);  //受信機ニュートラル
  RecvForwardValue = ReadEEPROM_Int(1 * 2); //受信機ForwordMAX
  RecvBackValue = ReadEEPROM_Int(2 * 2);  //受信機BackMax
  ESC.writeMicroseconds(RecvNutralValue);
  Serial.print(RecvBackValue);
  Serial.print(" / ");
  Serial.print(RecvNutralValue);
  Serial.print(" / ");
  Serial.println(RecvForwardValue);
  Serial.println ("start");
  //最大RPM設定
  //操作量2/90単位で回転数記憶
  //注:0→90と90→0だとかなり回転数異なる
  if (digitalRead(BTNPIN2) == LOW){
    BlinkLED(BTNPIN2 , LEDPIN1);
    Serial.println("RPMSetting");
    delay(3000);
    ESC.writeMicroseconds(RecvForwardValue);
    //RPM初期化
    for (int i = 0 ; i <= MULTIPLE ; i++){
      delay (2000);  
      updateRPM();
      Serial.println(currentRPM);
    }
   
    unsigned long current = RecvForwardValue;
    int k = 0;

    while(RecvNutralValue < current){
      
      digitalWrite(LEDPIN2 , HIGH);
      delay(200);
      digitalWrite(LEDPIN2 , LOW);
      delay(100);
      ESC.writeMicroseconds(current);
      for (int i = 0 ; i <= MULTIPLE ; i++){
        delay(200);
        updateRPM();
        Serial.println(currentRPM);
      }
      delay(200);
      updateRPM();
      Serial.print(k);
      Serial.print(" : ");
      Serial.print(current);
      Serial.print(" = ");
      Serial.println(currentRPM);
      WriteEEPROM_Long(10 + (k++ * 4), currentRPM);  //回転数
      current -= RPMSTEP;
    }
    ESC.writeMicroseconds(RecvNutralValue);
    //残りをすべて0埋め
    for(; k < RPMTABLESIZE ;) WriteEEPROM_Long(10 + (k++ * 4), 0);  //回転数
    
  }
  delay(500);
  digitalWrite(LEDPIN1 , LOW);
  //===================
  //EEPROMから設定読み出し
  //===================
  
  BreakForce = ReadEEPROM_Int(3 * 2);  //ブレーキ差分
  BreakDepth = ReadEEPROM_Int(4 * 2);
  if (BreakForce < 0) BreakForce = 1;
  if (BreakDepth < 0) BreakDepth = 10;
  Serial.print("BreakForce:");
  Serial.println(BreakForce);
  Serial.print("BreakDepth:");
  Serial.println(BreakDepth);
  for (int i = 0 ; i < RPMTABLESIZE ; i++){
    int v = ReadEEPROM_Long(10 + (i * 4));
    RPMTable[i] = v;
    Serial.print(i);
    Serial.print(" : ");
    Serial.println(RPMTable[i]);
  }

  //準備完了合図
  digitalWrite(LEDPIN1 , HIGH);
  digitalWrite(LEDPIN2 , HIGH);
  delay(2000);
  digitalWrite(LEDPIN1 , LOW);
  digitalWrite(LEDPIN2 , LOW);
  delay(150);
  AttachedBtnFlg = 0;
}

int UpdateValue(int state , int value , int minValue, int maxValue){
  AttachedBtnFlg = 0;
  int ret = value;
  while(true){
    SteteLED(state , ret);
    
    #if DEBUG
      Serial.print("UpdateValue = ");
      Serial.print(state);
      Serial.print(" : ");
      Serial.print(ret);
      Serial.print(" = ");
      Serial.print(AttachedBtnFlg & 1);
      Serial.print(" / ");
      Serial.println(AttachedBtnFlg & 2);
    #endif
    delay(200);
    if ((AttachedBtnFlg & 1) > 0){
      AttachedBtnFlg = 0;
      break;
    }else if ((AttachedBtnFlg & 2) > 0){
      AttachedBtnFlg = 0;
      ret++;
      if (ret > maxValue) ret = minValue;
      #if DEBUG
        Serial.println(ret);
      #endif
    }
  }
  return ret;
}


void loop() {

  if ((AttachedBtnFlg & 1) > 0 ){
    if (currentRPM <= 0 ){
      delay(200);
      //1:BreakForce 1～10 ブレーキ操作量調整。
      //大きいほど強めにブレーキが効く（＝基準回転数が設定値×200だけ小さくなる）
      #if DEBUG
        Serial.println("BreakForce Setting");
      #endif
      BreakForce = UpdateValue(1 , BreakForce , 1 , 10);
      WriteEEPROM_Int(3 * 2 , BreakForce);
      delay(200);
      //2:BreakDepth 1～10 自動ブレーキ強さ *10%
      #if DEBUG
        Serial.println("BreakDepth Setting");
      #endif
      BreakDepth = UpdateValue(2 , BreakDepth , 1 , 10);
      WriteEEPROM_Int(4 * 2 , BreakDepth);  

      //準備完了合図
      digitalWrite(LEDPIN1 , HIGH);
      digitalWrite(LEDPIN2 , HIGH);
      delay(2000);
      digitalWrite(LEDPIN1 , LOW);
      digitalWrite(LEDPIN2 , LOW);
    }
    AttachedBtnFlg = 0;
  }
  
  unsigned long recvValue = pulseIn(RECVPIN , HIGH);

  //RPMを更新する
  updateRPM();
  #if DEBUG
    Serial.print("      ");
    Serial.print(recvValue);
    Serial.print("  RPM:");
    Serial.print(currentRPM);
  #endif

  //プロポ電源OFF(パルス幅が500以下)の場合、ニュートラル固定。
  //500以上の時に制御を行う。
  if (recvValue < 500) {
    recvValue = RecvNutralValue;
    isBreak = true;
  }else{
    //前進側の場合のみ。
    isBreak = false;
    if (recvValue > (RecvNutralValue + 20)){
      if (recvValue > RecvForwardValue ) recvValue = RecvForwardValue;
      long RPMStd = RPMTable[(int)((RecvForwardValue - recvValue) / RPMSTEP)];  //基準回転数
      
      #if DEBUG
        Serial.print(" RPMStd:");
        Serial.print(RPMStd);
      #endif
      //基準回転数以上の場合にブレーキ。
      if (currentRPM - RPMStd > 200){ //(BreakForce * 200)){
        #if DEBUG
          Serial.print(" -- ");
          Serial.print(((long)RPMStd * 100)/ currentRPM);
        #endif
        //ブレーキ操作量を強くする為に基準回転数を設定値＊200だけ下げる
        long RPMStd2 = RPMStd - (BreakForce - 1) * 200;
        if (RPMStd2 <= 0 ) RPMStd2 = 0;
        /*
         * ブレーキ量(%) = 100 - (基準回転数:RPMStd2 / 現在の回転数:currentRPM * 100)
         * ブレーキ側パルス幅 = ニュートラルパルス幅:RecvNutralValue - ブレーキ側パルス幅:RecvBackValue
         * ブレーキ量パルス幅換算値 = ブレーキ側パルス幅 * ブレーキ量(%) * (自動ブレーキ最大強さ(設定値):BreakDepth / 10)
         * 出力パルス幅 = ニュートラルパルス幅 - ブレーキ量パルス幅換算値
        */
        recvValue = RecvNutralValue - ((RecvNutralValue - RecvBackValue) * BreakDepth * ( 100 - (RPMStd2 * 100 / currentRPM))) / 1000;
        isBreak = true;
        #if DEBUG
          Serial.println("");
          Serial.print("!Brake! ");
          Serial.print(100 - ((RPMStd2 * 100) / currentRPM));
          Serial.print("% → ");
          Serial.print(recvValue);
        #endif
      }
    }
  }
  #if DEBUG
    Serial.println("");
    delay(50);
  #endif
  digitalWrite(LEDPIN2 , isBreak);
  
  //操作信号出力
  ESC.writeMicroseconds(recvValue);
}
