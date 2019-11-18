/*
 * Copyright 2019 shigeyuki horimoto
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
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
#define RPMSTEP 15  //回転数記憶時の差分(μs)
#define RPMTABLESIZE 120 //RPM記憶領域配列要素数

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
int BreakDelayRPM; //ブレーキ回転数差分
boolean isBreak;  //ブレーキ介入有無  true:有り false:なし


/**
 * 設定書き込み
 */
void WriteEEPROM_Int(int address , int arg){
  EEPROM.write(address,highByte(arg));
  EEPROM.write(address + 1,lowByte(arg));
}
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
          WriteEEPROM_Int(i * 2 , v);
          break;
        }
        delay(200);
      }
    }
    digitalWrite(BREAKLED , LOW);
  }
  //設定値読み込み
  RecvNutralValue = ReadEEPROM_Int(0 * 2);  //受信機ニュートラル
  RecvForwardValue = ReadEEPROM_Int(1 * 2); //受信機ForwordMAX
  RecvBackValue = ReadEEPROM_Int(2 * 2);  //受信機BackMax
  ESC.writeMicroseconds(RecvNutralValue);
  #if DEBUG
    Serial.print(RecvBackValue);
    Serial.print(" / ");
    Serial.print(RecvNutralValue);
    Serial.print(" / ");
    Serial.println(RecvForwardValue);
    Serial.println ("start");
  #endif
  //最大RPM設定
  //操作量2/90単位で回転数記憶
  //注:0→90と90→0だとかなり回転数異なる
  if (digitalRead(RPMSETTINGPIN) == LOW){
    BlinkBreakLED(RPMSETTINGPIN);
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
      
      digitalWrite(LEDPIN , HIGH);
      delay(200);
      digitalWrite(LEDPIN , LOW);
      delay(100);
      ESC.writeMicroseconds(current);
      for (int i = 0 ; i <= MULTIPLE ; i++){
        delay(200);
        updateRPM();
        Serial.println(currentRPM);
      }
      delay(200);
      updateRPM();
      #if DEBUG
        Serial.print(k);
        Serial.print(" : ");
        Serial.print(current);
        Serial.print(" = ");
        Serial.println(currentRPM);
      #endif
      WriteEEPROM_Long(10 + (k++ * 4), currentRPM);  //回転数
      current -= RPMSTEP;
    }
    ESC.writeMicroseconds(RecvNutralValue);
    //残りをすべて0埋め
    for(; k < RPMTABLESIZE ;) WriteEEPROM_Long(10 + (k++ * 4), 0);  //回転数
    
  }
  delay(500);
  digitalWrite(BREAKLED , LOW);
  //===================
  //EEPROMから設定読み出し
  //===================

  BreakDelayRPM = ReadEEPROM_Int(3 * 2);  //ブレーキ差分
  for (int i = 0 ; i < RPMTABLESIZE ; i++){
    int v = ReadEEPROM_Long(10 + (i * 4));
    RPMTable[i] = v;
    #if DEBUG
      Serial.print(i);
      Serial.print(" : ");
      Serial.println(RPMTable[i]);
    #endif
  }

  //準備完了合図
  digitalWrite(BREAKLED , HIGH);
  delay(2000);
  digitalWrite(BREAKLED , LOW);
  delay(150);
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
        WriteEEPROM_Int(3 * 2 , BreakDelayRPM);
        break;
      }
    }
    digitalWrite(LEDPIN , LOW);
    digitalWrite(BREAKLED, LOW);
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
    //設置状態でのRPMと空転状態のRPM比較なのでマイナスでも問題無い
    currentRPM -= 200;
    if (currentRPM < 0 ) currentRPM = 0;
    if (currentRPM - RPMStd > 0){ //(BreakDelayRPM * 200)){
      #if DEBUG
        Serial.print(" -- ");
        Serial.print(((long)RPMStd * 100)/ currentRPM);
      #endif
      long RPMStd2 = RPMStd - (BreakDelayRPM - 1) * 200;
      if (RPMStd2 <= 0 ) RPMStd2 = 0;
      recvValue = RecvNutralValue - ((RecvNutralValue - RecvBackValue) * ( 100 - (RPMStd2 * 100 / currentRPM))) / 100;
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
  #if DEBUG
    Serial.println("");
    delay(50);
  #endif
  digitalWrite(BREAKLED , isBreak);
  
  //操作出力
  ESC.writeMicroseconds(recvValue);
}
