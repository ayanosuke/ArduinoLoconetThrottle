/*

LocoNet Handregler (Throttle)

by Philipp Gahtow, 
http://pgahtow.de/wiki/index.php?title=Arduino_Loconet_Throttle

Revision History:
v0.5: 17.06.12
      Einstellbereich Adresse erweitert auf 2024
      Standart Adresse auf von 0 auf 1 gesetzt.
v0.4: 16.06.12
      Adresszählerinit angepasst, +1 entfernt!
      Bei PowerOff Sleep mit GoldCap aktiviert, variablen bleiben erhalten!
      Schaltplan V0 LCD mit 1k nach GND, LCD mit Dioden um Spannungsabfluss AVR->GND zu verhindern
v0.3: 15.06.12
      Anpassung des Speichervorhangs für Adresse und Slot bei powerdown.
v0.2: 29.05.12 
      Änderung und Anpassung für die neue Bibliothek
v0.1: Erste lauffähige Version


*/

//Bibliotheken:
#include <LocoNet.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <avr/sleep.h>

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

//Adresse des Handregler als ID:
#define ID 0x20

#define EEPROMOn true    //soll der EEPROM genutzt werden?

lnMsg        *LnPacket;
LnBuf        LnTxBuffer;

//7seg LED Table http://deviceplus.jp/hobby/entry008/  http://keikato.cocolog-nifty.com/blog/2012/09/auto-19.html
boolean Num_Array[36][7]={
  {1,1,1,1,1,1,0}, //0 H68/TR文字
  {0,1,1,0,0,0,0}, //1
  {1,1,0,1,1,0,1}, //2
  {1,1,1,1,0,0,1}, //3
  {0,1,1,0,0,1,1}, //4
  {1,0,1,1,0,1,1}, //5
  {1,0,1,1,1,1,1}, //6
  {1,1,1,0,0,1,0}, //7
  {1,1,1,1,1,1,1}, //8
  {1,1,1,1,0,1,1}, //9
  {1,1,1,0,1,1,1}, //A 10
  {0,0,1,1,1,1,1}, //B
  {1,0,0,1,1,1,0}, //C
  {0,1,1,1,1,0,1}, //D
  {1,0,0,1,1,1,1}, //E
  {1,0,0,0,1,1,1}, //F 15
  {1,0,1,1,1,1,0}, //G
  {0,1,1,0,1,1,1}, //H
  {1,0,0,1,1,0,0}, //I
  {0,1,1,1,1,0,0}, //J 
  {0,1,0,1,1,1,1}, //K 20
  {0,0,0,1,1,1,0}, //L
  {1,0,1,0,1,0,1}, //M
  {0,0,1,0,1,0,1}, //N
  {1,0,1,1,1,1,0}, //O               
  {1,1,0,0,1,1,1}, //P 25
  {1,1,1,0,0,1,1}, //Q
  {0,0,0,0,1,1,1}, //R
  {0,0,1,1,0,1,1}, //S
  {1,0,0,0,1,1,0}, //T 
  {0,1,1,1,1,1,0}, //U 30
  {0,1,1,1,1,1,1}, //V
  {1,0,1,1,1,0,0}, //W
  {0,1,0,1,0,1,1}, //X
  {0,1,1,1,0,1,1}, //Y 
  {1,0,0,1,0,0,1}  //Z 35   
                
};

#define DP 10
#define ssegD1 11
#define ssegD2 12
#define LoconetRxd 12
#define LoconetTxd 13

#define analogkey A0
#define ShiftPin A1    //Shift Button
#define DIR A2
#define Shift2Pin 13   //Shift2 Button
#define DirPin 19      //Button Fahrtrichtung
#define EnAPin A3    //Incremental A
#define EnBPin A4    //Incremental B

#define PowerPin 2    //Power down Sensor Pin

#define SPEED_DISP 1
#define FUNC_DISP 2
#define ADRS_DISP 3
#define DIR_DISP 4
#define EMG_DISP 5

#define sizebut 10      //Anzahl der Taster（ボタンの数）
int butState[sizebut] = {0, 0, 0, 0, 0, 0, 1, 0, 0, 0};    //last Button State

#define F0 0
#define F1 1
#define F2 2
#define F3 3
#define F4 4
#define F5 5
#define F6 6
#define F7 7
#define F8 8
#define F9 9
#define F10 10
#define F11 11
#define F12 12
#define F13 13
#define F14 14
#define F15 15
#define F16 16
#define F17 17
#define F18 18
#define F19 19

#define C1 1
#define C2 2
#define C3 3
#define C4 4
#define C5 5

//#define DIR 5
#define EnA 6
#define EnB 7
#define SHIFT 8
#define SHIFT2 9

#define THROWN 0
#define CLOSED 32

int ledMode;
char dp=0;
char lgf=0;
char dirf=0;
char funcf=0;
char dispSpeed;
char FuncKey=0;

word ledfunktion = 0;     //Funktionszustände für LED's über Taster F0-F4 (16 Bit = bis F16, F0 direkt!)
byte TickCount = 0;        //flash counter wait on F0 LED

#define AdrMax 2024      //größe einstellbare Adresse サイズ調節可能なアドレス
#define SpeedMax 127     //höchste Geschwindigkeit. (vielleicht auch hier Stufen nutzen -> FredI!)

LocoNetThrottleClass Throttle;
LocoNetClass Loconet;
        
uint32_t              LastThrottleTimerTick;
uint32_t              LastShiftTimerTick;
uint32_t              LastFunctionTimerTick;
uint32_t              LastDiTimerTick;
uint32_t              LastAddressTimerTick;
uint32_t              LastDirTimerTick;
uint32_t              LastTimerTick;
uint32_t              LastAdrTimerTick;

word Adresse = 1;    //default Adresse 16-bit
byte Slot = 0xFF;           //akt. Slot zur Adr. (kein)
byte AdrH = 0;          //high Anteil Adresse
byte AdrL = 0;          //low Anteil Adresse
byte PointDir = 0;

byte PowerOff = 0;    //Versorgungsspannung liegt an.

//EEPROM Speicherbereiche:
#define EEPROMAdrH 13    //Adresse High
#define EEPROMAdrL 14    //Adresse Low
#define EEPROMSlot 16    //Slot

//Arduino Pin定義
//https://www.flickr.com/photos/arms22/8457793740/
// 0:DI:Rxd
// 1:DO:Txd
// 2:DO:7seg a
// 3:DO:7seg b
// 4:DO:7seg c
// 5:DO:7seg d
// 6:DO:7seg e
// 7:DO:7seg f
// 8:DI:LocoNet Rxd
// 9:DO:7seg g
//10:DO:7seg DP
//11:DO:7seg D2
//12:DO:7seg D1
//13:DO:LocoNet Txd
//A0:AI:analog key
//A1:DI:shift key  :Pull UP設定
//A2:DI:EC11 SW    :Pull UP設定
//A3:DI:EC11 A     :Pull UP設定
//A4:DI:EC11 B     :Pull UP設定
//A5:

//--------------------------------------------------------------------------------------------------
// 初期化
//--------------------------------------------------------------------------------------------------
void setup()
{
  // 7seg led pinMode 初期化
  for (int i=2; i<=12; i++){ 
    pinMode(i,OUTPUT);
  }
     
  //key board pinMode 初期化
  pinMode(A0,INPUT);
  pinMode(A1,INPUT_PULLUP); 
  pinMode(A2,INPUT_PULLUP); 
  pinMode(A3,INPUT_PULLUP); 
  pinMode(A4,INPUT_PULLUP); 

  // CPU Sleep Modes 
  // SM2 SM1 SM0 Sleep Mode
  // 0    0  0 Idle
  // 0    0  1 ADC Noise Reduction
  // 0    1  0 Power-down
  // 0    1  1 Power-save
  // 1    0  0 Reserved
  // 1    0  1 Reserved
  // 1    1  0 Standby(1)

  cbi( SMCR,SE );      // sleep enable, power down mode
  cbi( SMCR,SM0 );     // power down mode
  sbi( SMCR,SM1 );     // power down mode
  cbi( SMCR,SM2 );     // power down mode
  
  LocoNet.init(LoconetTxd);   // First initialize the LocoNet interface
                              // RxPinは,PB0固定 
  Throttle.init(0, 0, ID);      //erzeuge Handregler

//  Serial.end();
  Serial.begin(57600);  // Configure the serial port for 57600 baud
  Serial.println("Smile Loconet Throttle");
  
  getOldSlot();    //Hole letzte Adr. und Slot aus EEPROM falls verfügbar!
  Adresse = 1;
  ledMode = ADRS_DISP;
}

#if 0
void debugstate(){
     if (Serial.available()) {
        // Process serial input for commands from the host.
        int ch = Serial.read();
        Serial.println(ch);
        if(ch=='A'){
          Serial.print("ADDRESS:");
          Serial.println(Adresse);          
        }
        if(ch=='S'){
          Serial.print("STATE:");
          Serial.print(Throttle.getState());
          Serial.print(":");
          Serial.println(Throttle.getStateStr(Throttle.getState()));
        }
        if(ch=='F'){
          Serial.print("dirf:");
          Serial.println(dirf,DEC);          
        }
        if(ch=='I'){  // addres をフリーにする
          Serial.println("setAddress");      
          Throttle.setAddress(Adresse);      //via Adresse
          ledMode = 1;
        }
        if(ch=='Z'){  // freeAdderss
          Serial.println("freeAddress");      
          Throttle.freeAddress(Adresse);      //via Adresse
        }
        if(ch=='R'){  // rereceAdderss
          Serial.println("releaseAddress");      
          Throttle.releaseAddress();      //via Adresse
        }
        if(ch=='O'){  // addres を取得する
          Throttle.dispatchAddress(Adresse);
          Throttle.freeAddress(Adresse);
          Throttle.releaseAddress();
        }
        if(ch=='D'){
          Throttle.acquireAddress();
        }
        if(ch=='1'){
          ledMode=SPEED_DISP;
        }
        if(ch=='3'){
          ledMode=FUNC_DISP;
        }
     }
}
#endif


//--------------------------------------------------------------------------------------------------
// main loop
//--------------------------------------------------------------------------------------------------
void loop() {
  AdrL = Adresse;        //Adresse in je 2x 8 Bit Speichen für EEPROM Sicherung
  AdrH = Adresse >> 8;

  EncoderState();         // エンコーダー処理
  DirKeyState();          // DIRキー処理
  ShiftKeyState();        // SHIFTキー処理  
  getTaster();            // ボタンキー処理
  ledState();             // 7seg LED処理
//debugstate();
  adrState(0);            // アドレス取得処理
  pointState(0);          // ポイント処理
        
  if (PowerOff == 1)   
    system_sleep(); //set system into sleep state

  // Check for any received LocoNet packets
  LnPacket = LocoNet.receive() ;
  if (LnPacket)  {
  //Serial.println("LnPacket");
    if (!LocoNet.processSwitchSensorMessage(LnPacket))
      Throttle.processMessage(LnPacket) ; 
  }
  
  if (PowerOff == 1)   
    system_sleep(); //set system into sleep state  

  if(isTime(&LastThrottleTimerTick, 100)) {       // Locoには100ms周期でアクセスが必要だそうです。
    Throttle.process100msActions(); 
    if (Throttle.getState() != TH_ST_IN_USE){     //no Slot - Blink LED F0
//     dp=0x02;
//     Serial.println("2");
//     TickLED(true);
//      ledMode=3;
    }

    if (Throttle.getState() == TH_ST_SLOT_MOVE){    //Blink off
//     dp=0x00;
     Serial.println("0");
//     TickLED(false);
    }
  }
  
  if (PowerOff == 1)   
    system_sleep(); //set system into sleep state

}



//--------------------------------------------------------------------------------------------------
// Check Button states
//--------------------------------------------------------------------------------------------------
void getTaster() {
  int State;
  int Shift = digitalRead(ShiftPin);    //Shift abfragen?（シフトクエリ？）
  if (butState[SHIFT] != Shift) {
    butState[SHIFT] = Shift;
  }

  analogkey_in();

  if((dirf == 1 && ledMode == 3)){  // LED アドレス選択モード,DIR長押し
    Serial.println("Connect");
    adrState(1);
    ledMode = SPEED_DISP;                       // 速度表示画面に変更
  } else  if((dirf == 3 && ledMode == 1)){  // LED 速度表示モード,DIR長押し
    Serial.println("Dis Connect");
    Throttle.releaseAddress();
    Throttle.freeAddress(Adresse);
    Throttle.releaseAddress();
    Slot = 0;
    Serial.println("");
    ledMode = ADRS_DISP;                 // 速度表示画面に変更
    dirf=0;
  }

  if (Throttle.getSpeed()==1 && dirf==2 ) {                  // 速度 0 の時、ダブルクリックでForward/Reverse切り替え
    Throttle.setDirection(!Throttle.getDirection());    //DIR
    ledMode = DIR_DISP;
    dirf = 0;
  }

  if (Throttle.getState() == TH_ST_IN_USE && dirf==1 ) {                 // Func=0 DIRクリック状態?
    dirf = 0;
    if (Throttle.getSpeed()-1 > 0){    //EMERGENCY STOP (停車時速度=1)
      Serial.println("EMG STOP");
      Throttle.setSpeed(1);
      ledMode = EMG_DISP;                // 00表示画面
    } else {
      dirf = 0;
    }
  }

  if(FuncKey!=0 && funcf == 0 && dp != 0x03){       // dp != 0x03 は 0x03の時はポイント切り替え用に使う。
    Throttle.setFunction(FuncKey-1, !Throttle.getFunction(FuncKey-1) ); //FuncKeyは+1されているので、Locoに設定するときは-1する。
    Serial.print("FuncKey:");
    Serial.println(FuncKey-1,DEC);
    funcf =1 ;       // 処理完了
  }         
  if(FuncKey!=0 && funcf == 0 && dp == 0x03){       // dp == 0x03 は 0x03の時はポイント切り替え用に使う。
    pointState(FuncKey-1);
    Serial.print("requestSwitch:");
    Serial.println(FuncKey-1,DEC);
    funcf =1 ;       // 処理完了
  }       

}

//--------------------------------------------------------------------------------------------------
// Encoderの処理ステート
//--------------------------------------------------------------------------------------------------
void EncoderState(){
    //Encoder:----------------------------------------------------------------------------------
    int EnAState = digitalRead(EnAPin);
    butState[EnB] = digitalRead(EnBPin);
    if ((butState[EnA] == LOW) && (EnAState == HIGH)) {
      if (butState[EnB] == LOW) {   //Down
        if (Throttle.getState() == TH_ST_IN_USE) {    //Adr. vorhanden? 利用可能？
          if (Throttle.getSpeed() > 1)
            if(lgf==0)
              Throttle.setSpeed(Throttle.getSpeed() - 1);  //slow スピードを落とす
            else
              Throttle.setSpeed(Throttle.getSpeed() - 5);  //slow スピードを落とす            
        }
        else {        //ändere Adresse（アドレスの変更）
          ledMode = 3;
          if (Adresse <= 1)
            Adresse = AdrMax;
          else {
            if(lgf==0)
              Adresse--;
            else
              Adresse=Adresse-5;
          }
          Serial.print("Adresse:");
          Serial.println(Adresse);
//        updateLCDAdr(Adresse);
        }
      } 
      else {              //Up
        if (Throttle.getState() == TH_ST_IN_USE) {    //Adr. vorhanden?
            if (Throttle.getSpeed() < SpeedMax)       // SpeedMax=127
              if(lgf==0)
                Throttle.setSpeed(Throttle.getSpeed() + 1);  //faster スピードを上げる
              else
                Throttle.setSpeed(Throttle.getSpeed() + 5);  //faster スピードを上げる              
          }
        else {          //ändere Adresse
          if (Adresse >= AdrMax)
            Adresse = 1;
          else {
            if(lgf==0)
              Adresse++;
            else
              Adresse=Adresse+5;
          }
          Serial.print("Adresse:");
          Serial.println(Adresse);
//        updateLCDAdr(Adresse);
        }
      }
    }
    butState[EnA] = EnAState;
}


//--------------------------------------------------------------------------------------------------
// SHIFTキーの処理ステート
//--------------------------------------------------------------------------------------------------
void ShiftKeyState(){
  static int state = 0;
  static int Shift;
  
  switch(state){
    case 0:
            Shift = digitalRead(ShiftPin);                // プルアップされているので押下でL
            if(Shift == 0){
              LastShiftTimerTick = millis();              // 押下時の時間セット
              state = 1;
            }
            break;
    case 1:
            if(millis()-LastShiftTimerTick >= 200 ){      //100ms超えた？
//            Serial.println("100");
              if(digitalRead(ShiftPin)==0){
//              Serial.println("Shift Long press");
                lgf=1;                                    // 長押しフラグ 
                state = 2;
              }
            }
            if(digitalRead(ShiftPin)==1){                 // 100ms以下はクリック処理
//            Serial.println("Shift Click!");
              dp++;
              if(dp>=4)                                   // SHIFT 1,2 の状態は7segLEDのデシマルポインタの位置で表示
              dp=0;
              state = 0;
            }
            break;
    case 2:
              if(digitalRead(ShiftPin)==1){               // 長押し解除？
                lgf = 0;
//              Serial.println("Shift Long press dis");
                state = 0;            
              }
            break;
    default:
          break;
    }
}

//--------------------------------------------------------------------------------------------------
// DIRキーの処理ステート
// シングルクリック、ダブルクリック、長押しを判定
//--------------------------------------------------------------------------------------------------
void DirKeyState(){
  static int state = 0;
  static char bufp = 0;
  static uint32_t Ti;
  static char cKeybuf[20];
  
  char pc = 0;
  char ps = 0;
  char i;
  char di;
  char Max = 10;

  switch(state){
    case 0:
            Ti = millis();
            state = 1;
            break;
    case 1:                                       // 50msec周期の監視
            if( millis()-Ti >50){
              di = digitalRead(DIR);               // ボタン読み込み
              if(di == 0 || bufp > 0) {           // 最初の検出か？ bufpが0以上
                cKeybuf[bufp++] = di;
                if(bufp >= Max){            
                  state = 2;
                  bufp = 0;
                  break;
                }
              }
              Ti = millis();
            }
            break;
    case 2:                                     // シングルクリック・ダブルクリック・長押し判定
            Serial.println("");
            Serial.print("Keybuf:");  
            for(i=0;i<=Max-1;i++)
              Serial.print(cKeybuf[i],DEC);
            Serial.print(":");
                  
            for( i = 0 ; i <= Max - 2 ; i++){   // 変化点を変数pcでカウント
              if(cKeybuf[i] != cKeybuf[i+1])
                pc = pc + 1;
              ps = ps + cKeybuf[i];             // 0が多いと長押し判定に使える
            }

            Serial.print(pc,DEC);
            Serial.print(":");   
            Serial.print(ps,DEC);
            Serial.print(":");  
            
            switch(pc){
              case 0: Serial.println("Long press");
                      dirf = 3;
                      break;
              case 1:
              case 2:if(ps<=5){
                        Serial.println("Long press");
                        dirf = 3;
                      } else {
                        Serial.println("single click");
                        dirf = 1;
                      }
                     break;
              case 3:
              case 4: Serial.println("Double click");
                      dirf = 2;
                      break;

            }
            Serial.println("");
            state = 0;
            break;
    default:
            break;
  }  
}



//--------------------------------------------------------------------------------------------------
// アドレス取得用ステートマシン
// freeAddress()->setAddress()と実行してもbusyを検出して処理ができないので、100msのウエイトを入れた
//--------------------------------------------------------------------------------------------------

void adrState( int foo ){
  static int state = 0;

//  Serial.print("adrState:");
//  Serial.println(state,DEC);

  switch(state){
    case 0:
            if( foo == 1 )
              state = 1;
            break;
    case 1:                                             // 取得しているスロットを解放
            Throttle.freeAddress(Adresse);
            LastAdrTimerTick = millis();
            state = 2;
            break;
    case 2:                                             // 100ms経過後、アドレスを取得
            if(millis()-LastAdrTimerTick>=100) {
            Throttle.setAddress(Adresse); 
            state = 3;
            }
            break;
    case 3:
            ledMode = SPEED_DISP;                       // 速度表示画面に変更
            dirf = 0;                                   // DIRボタンの処理解除
            state = 0;
            break;
    default:
            break;
  }
}

//--------------------------------------------------------------------------------------------------
// ポイント切り替え用ステートマシン
// 現状の状態を問い合わせてから、切り替える様にした。そうする事で、１ボタンでc/tの切り替えができて便利
//--------------------------------------------------------------------------------------------------

void pointState( int adr ){
  static int state = 0;
  static int pointAdr;
//  Serial.print("pointState:");
//  Serial.println(state,DEC);

  switch(state){
    case 0:
            if( adr != 0 ){
              pointAdr = adr;
              state = 1;
            }
            break;
    case 1:                                             // ポインタの状態を取得
            Loconet.reportSwitch(pointAdr);
            LastAdrTimerTick = millis();
            state = 2;
            break;
    case 2:                                             // 100ms経過後、アドレスを取得
            if(millis()-LastAdrTimerTick>=100) {
              if(PointDir == THROWN)
                Loconet.requestSwitch(pointAdr,1,CLOSED);
              else
                Loconet.requestSwitch(pointAdr,1,THROWN);
              state = 3;
            }
            break;
    case 3:
            ledMode = SPEED_DISP;                       // 速度表示画面に変更
            dirf = 0;                                   // DIRボタンの処理解除
            state = 0;
            break;
    default:
            break;
  }
}



//--------------------------------------------------------------------------------------------------
// 7segLEDの表示処理ステート
// 0:ブランク表示、1:スピード表示、2:ファンクッション番号表示、3:アドレス表示、4:Forard/Revers表示
//--------------------------------------------------------------------------------------------------
void ledState(){
  static int state = 0;
  static int cnt = 0;
  static char dispFuncKey;
  int disp;
  
//Serial.print("ledMode:");
//Serial.print(ledMode);
//Serial.print("_state:");
//Serial.println(state);

  switch(ledMode){
    case 0:                                     // 7seg消灯
            state = 0;
            break;
    case 1:                                     // スピード表示 
            state = 1;
            break;
    case 2:                                     // ファンクッション表示 
            if(state==3){
             
            }
            else
              state = 2;
            break;
    case 3:                                     // アドレス表示（ブリンク）
           if(state>=4 && state<=6){
           }
           else
              state = 4;
           
           break;
    case 4:                                      // Forard / Revers 表示
            if(state==11){
            }
            else
               state = 10;
            break;
    case 5:                                      // EMG実行時の00点滅処理
            if(state>=21 && state<=22){
            }
            else
               state = 20;
            break;
    
    
    default:
            break;
  }

  switch(state){
    case 0:                                     // 7seg消灯
            digitalWrite(ssegD1,LOW);
            digitalWrite(ssegD2,LOW);
            state = 0;
            break;
    case 1:                                     // スピードを表示
            ledUpdate(dispSpeed,dp);
//          ledUpdate(Throttle.getSpeed() + 1,dp);
            break;
    case 2:                                     // ファンクッションを表示
            LastFunctionTimerTick = millis();
            dispFuncKey = FuncKey-1;            // ファンクッション表示は実際のファンクッションの処理が終わっても表示させたいので、dispFuncKeyに値をコピーする
            ledUpdate2(15,dispFuncKey,0);
            state = 3;
            break;
    case 3:                                     // ファンクッヨンを0.5秒だけ表示
            if(millis()-LastFunctionTimerTick>=500) {
              state = 4;
              ledMode = 1;                      // 速度表示画面へ
            }
            ledUpdate2(15,dispFuncKey,0);
            break;
    case 4:                                     // アドレス表示（初回点灯）
            LastAddressTimerTick = millis();
            ledUpdate(Adresse,dp);
            state = 5;
            break;          
    case 5:                                     // アドレス表示（ブリンクのため消灯）
            if(millis()-LastAddressTimerTick>=250) {
              LastAddressTimerTick = millis();
              state = 6;
              break;
            }
            ledUpdate(Adresse,dp);
            break;
    case 6:                                     // アドレス表示（ブリンクのため点灯）
            if(millis()-LastAddressTimerTick>=250) {
              LastAddressTimerTick = millis();
              state = 4;
            } else {
              digitalWrite(ssegD1,LOW);
              digitalWrite(ssegD2,LOW);
            }
            break;
    case 10:                                     // Forward/Reverse表示
            LastDiTimerTick = millis();
            dispFuncKey = Throttle.getDirection();
            if(dispFuncKey==0){
              ledUpdate2(27,27,0);                // RR
            } else{
              ledUpdate2(15,15,0);                // FF
            }
           
            state = 11;
            break;
    case 11:                                     // Forward/Reverseを0.5秒だけ表示
            if(millis()-LastDiTimerTick>=500) {
              state = 1;
              ledMode = 1;                      // 速度表示画面へ
            }
            if(dispFuncKey==0){
              ledUpdate2(27,27,0);                // RR
            } else{
              ledUpdate2(15,15,0);                // FF
            }
            break;                   

    case 20:                                     // EMG STOP時の00点滅表示
            LastAddressTimerTick = millis();
            ledUpdate(00,dp);
            state = 21;
            cnt = 5;
            break;     
    case 21:                                     // 00消灯
            if(millis()-LastAddressTimerTick>=100) {
              LastAddressTimerTick = millis();
              state = 22;
              cnt--;
              if(cnt<=0){
                state = 1;
                ledMode = 1;                     // 速度表示画面へ
              }
              break;
            }
            ledUpdate(00,dp);
            break;                
    case 22:                                     // 00表示（ブリンクのため点灯）
            if(millis()-LastAddressTimerTick>=100) {
              LastAddressTimerTick = millis();
              state = 21;
            } else {
              digitalWrite(ssegD1,LOW);
              digitalWrite(ssegD2,LOW);
            }
            break;
    
    
    
    default:
            break;
  }
}

//--------------------------------------------------------------------------------------------------
// power OFF割り込み処理
//--------------------------------------------------------------------------------------------------
void powerISR() {
  if (digitalRead(PowerPin) == HIGH) {
    PowerOff = 0;      //aktiv
  }
  else {
    PowerOff = 1;      //System komplett herrunterfahren. Minimaler Energieverbrauch!
    DDRD = B00000000;    //sets Arduino pins 0 to 7 as input
    PORTD = B00000000;
    DDRB = B00000000;   //sets Arduino pins 8 to 13 as input
    PORTB = B00000000;
    DDRC = B00000000;    //sets Arduino pins A0 to A5 as input
    PORTC = B00000000;
   
    //Save Adresse und Slot
    if ((AdrL != EEPROM.read(EEPROMAdrL)) && (EEPROMOn == true))
      EEPROM.write(EEPROMAdrL, AdrL);
    if ((AdrH != EEPROM.read(EEPROMAdrH)) && (EEPROMOn == true))
      EEPROM.write(EEPROMAdrH, AdrH);
    if ((Slot != EEPROM.read(EEPROMSlot)) && (EEPROMOn == true))
      EEPROM.write(EEPROMSlot, Slot);  
  }
}

//--------------------------------------------------------------------------------------------------
// update Speed via tick
// boolean型はtrue/falseの２値を表現する型
//--------------------------------------------------------------------------------------------------
boolean isTime(unsigned long *timeMark, unsigned long timeInterval) {
    unsigned long timeNow = millis();
    if ( timeNow - *timeMark >= timeInterval) {
        *timeMark = timeNow;
        return true;
    }    
    return false;
}

//--------------------------------------------------------------------------------------------------
// Throttle notify Call-back functions
// if(関数名)で飛んでくるけど、どんな仕組み？
// loconetのメッセージが、LocoSlot -> DC50K -> LocoSlot に戻ってくる？その、戻ってきたら、更新している？
//--------------------------------------------------------------------------------------------------
void notifyThrottleSpeed( uint8_t UserData, TH_STATE State, uint8_t Speed )
{
  Serial.print("notifyThrottleSpeed():");
  Serial.println(Speed);
  dispSpeed=Speed-1;
};

//--------------------------------------------------------------------------------------------------
// Addressは、車両アドレス？ Slotは車両アドレスが格納されているSlot番号
//--------------------------------------------------------------------------------------------------
void notifyThrottleAddress( uint8_t UserData, TH_STATE State, uint16_t Address, uint8_t isSlot )
{
  Serial.print("notifyThrottleAddress():adress");
  Serial.print(Address);
  Serial.print("_slot:");

  Serial.println(isSlot);
  
//  updateLCDAdr(Address);
  Slot = isSlot;
};

//--------------------------------------------------------------------------------------------------
// forwardかreverseか？
//--------------------------------------------------------------------------------------------------
void notifyThrottleDirection( uint8_t UserData, TH_STATE State, uint8_t Direction )
{
  Serial.print("notifyThrottleDirection():");
  Serial.println(Direction);
};

//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
void notifyThrottleFunction( uint8_t UserData, uint8_t Function, uint8_t Value ) {
//  Serial.print("notifyThrottleFunction():");
//  Serial.print(Function);
//  Serial.print(":");
//  Serial.println(Value);
//  return;
 
  if (Function == 0) //F0受信？
    Serial.println("F0");
  else {
    Serial.print("F");
    Serial.print(Function);
    Serial.print(":");
    Serial.println(Value);

  }  
};

//--------------------------------------------------------------------------------------------------
void notifyThrottleSlotStatus( uint8_t UserData, uint8_t Status ) {};

//--------------------------------------------------------------------------------------------------
// 何が見える？
//--------------------------------------------------------------------------------------------------
void notifyThrottleState( uint8_t UserData, TH_STATE PrevState, TH_STATE State ) {
  Serial.print("notifyThrottleState():");
  Serial.println(Throttle.getStateStr(State));
};

//--------------------------------------------------------------------------------------------------
void notifyThrottleError( uint8_t UserData, TH_ERROR Error ) {
  Serial.print("notifyThrottleError():");
  Serial.println(Throttle.getErrorStr(Error));
}

void notifySwitchRequest( uint16_t Address, uint8_t outData, uint8_t dirData) {
  Serial.print("notifySwitchRequest():");
  Serial.print(Address,DEC);
  Serial.print(",");
  Serial.print(outData,DEC); // 常に16?
  Serial.print(",");
  Serial.print(dirData,DEC); // 0:t:分岐  32:c:直線
  Serial.println("");
  PointDir = dirData;
}


//--------------------------------------------------------------------------------------------------
void getEEPROMAdresse() {
  byte AdrH = EEPROM.read(EEPROMAdrH);
  byte AdrL = EEPROM.read(EEPROMAdrL);
  Adresse = 0;
  Adresse = AdrH << 8;
  Adresse = Adresse + AdrL;
}

//--------------------------------------------------------------------------------------------------
//Hole letzten Slot aus EEPROM falls aktiv
void getOldSlot() {
  if (Slot == 0xFF && EEPROMOn == true) {    //Prüfe auf Reset? リセットの確認？
    byte oldslot = EEPROM.read(EEPROMSlot);
    getEEPROMAdresse();
    if (oldslot == 255 || oldslot == 0)
      Slot = 0;   //kein Slot vorhanden zum beitreten
    else {
      Slot = oldslot;  
      Throttle.resumeAddress(Adresse, oldslot);
    }
  }
  else {
    if (Slot == 0xFF || Slot == 0)    //Slot nicht rückgesetzt?
      Slot = 0;
    else Throttle.resumeAddress(Adresse, Slot);
  }  
//  updateLCDAdr(Adresse);
}

//--------------------------------------------------------------------------------------------------
void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  
  /*
    SLEEP_MODE_IDLE                   – least power savings
    SLEEP_MODE_ADC
    SLEEP_MODE_PWR_SAVE
    SLEEP_MODE_STANDBY
    SLEEP_MODE_PWR_DOWN    – most power savings
  */
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  
   // Set sleep enable (SE) bit:
  sleep_enable();
  
  sleep_mode();                        // System sleeps here

  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
  
  setup();    //setzte alle PIN's nach sleep zurück
}

//--------------------------------------------------------------------------------------------------
// 7segLED更新処理(10進数表示用）
// ダイナミック点灯なので、右と左は交互に表示させている。
// 7segの書き換えはLOW(OFF)にしてから、Numprint()を実行してから、HIGH(on)にすると残像がなくキレイに表示される。
//--------------------------------------------------------------------------------------------------
void ledUpdate(int num7seg, char dp){
  static char sw = 0;

  switch(sw){
    case 0:                           // 下位の表示
      digitalWrite(ssegD1,LOW);       // 上位を消灯（やっておかないと残像が）
      NumPrint(NumParse(num7seg,1));
      digitalWrite(ssegD2,HIGH);
      if(dp&0x01)                     // デシマルポインタ表示処理
        digitalWrite(DP,HIGH);
      else
        digitalWrite(DP,LOW);      
      sw = 1;
      break;
      
    case 1:                           // 上位の表示
      digitalWrite(ssegD2,LOW);       // 下位を消灯（やっておかないと残像が）
      NumPrint(NumParse(num7seg,2));     
      digitalWrite(ssegD1,HIGH);
      if(dp&0x02)                     // デシマルポインタ表示処理
        digitalWrite(DP,HIGH);
      else
        digitalWrite(DP,LOW);   
      sw = 0;
      break;
   default:
      break;
  }
}
//--------------------------------------------------------------------------------------------------
// 7segLED更新処理(16進数表示用）
// ダイナミック点灯なので、右と左は交互に表示させている。
// 7segの書き換えはLOW(OFF)にしてから、Numprint()を実行してから、HIGH(on)にすると残像がなくキレイに表示される。
//--------------------------------------------------------------------------------------------------
void ledUpdate2(char hi,char low, char dp){
  static char sw = 0;
  
  switch(sw){
    case 0:
      digitalWrite(ssegD1,LOW);
      NumPrint(low); //1桁目の表示
      digitalWrite(ssegD2,HIGH);
      if(dp&0x01)
        digitalWrite(DP,HIGH);
      else
        digitalWrite(DP,LOW);      
      sw = 1;
      break;
      
    case 1:
      digitalWrite(ssegD2,LOW);
      NumPrint(hi); //2桁目の表示    
      digitalWrite(ssegD1,HIGH);
      if(dp&0x02)
        digitalWrite(DP,HIGH);
      else
        digitalWrite(DP,LOW);   
      sw = 0;
      break;
   default:
      break;
  }
}


//LED表示関数を定義
void NumPrint(int Number){
  for (int w=0; w<=6; w++){
  switch(w){
    case 6:               // 6だけOUTポートが飛んでいるから。＋1
      digitalWrite(w+2+1,-Num_Array[Number][w]);
      break;
    default:
      digitalWrite(w+2,-Num_Array[Number][w]);
      break;    
  }
  }
}

//2桁をそれぞれの桁で分解する関数
int NumParse(int Number,int s){
  if(s == 1){
    return Number % 10; //10で割ったあまり = 一桁目の値
  }
  else if(s == 2){
    return Number / 10; //10で割った値を整数にする = 二桁目の値
  }
  return 0;
}
//--------------------------------------------------------------------------------------------------
// アナログキーボードからのキー判定処理
// 敷居値は別途調整、初回検出値と連続に押下している場合で検出電圧が異なる。
//--------------------------------------------------------------------------------------------------
void analogkey_in() {
  int kin;
  int row;
  
  char FuncTable[5][4]={        // ファンクションテーブル SHIFT押してもF0を残したいというも可能 {F0,F0,F0},
  // 00 01 02 (SHIFT)
    {F0,F5,F10,C1},
    {F1,F6,F11,C2},
    {F2,F7,F12,C3},
    {F3,F8,F13,C4},
    {F4,F9,F14,C5}
  };

#if 0
  char FuncTable[5][4]={        // ファンクションテーブル SHIFT押してもF0を残したいというも可能 {F0,F0,F0},
  // 00 01 02 (SHIFT)
    {F0,F5,F10,F15},
    {F1,F6,F11,F16},
    {F2,F7,F12,F17},
    {F3,F8,F13,F18},
    {F4,F9,F14,F19}
  };
#endif
  
  kin = analogRead(analogkey);
  if( kin > 700 ){            // Funcボタンが押されなくなった？
    funcf = 0;
    FuncKey=0;
    return;
  } else if( funcf != 0){   // 押されっぱなし
    return;
  }
  ledMode = FUNC_DISP;
  
//  Serial.println(kin);
  if( kin < 100 ){          // F0,F5,F10
    row = 0;
  } else if( kin < 300 ){          // F1,F6,F11
    row = 1;
  } else  if( kin < 500 ){          // F2,F7,F12
    row = 2;
  } else  if( kin < 600 ){          // F3,F8,F13
    row = 3;
  } else  if( kin < 700 ){          // F4,F9,F14
    row = 4;
  }
    FuncKey = FuncTable[row][dp] + 1; //F0を認識させるために+1します

//  Serial.print("F:");
//  Serial.print(ret,DEC);
//  Serial.print(" row:");
//  Serial.print(row,DEC);
//  Serial.print(" dp:");
//  Serial.println(dp,DEC);

    return;
}
//--------------------------------------------------------------------------------------------------
