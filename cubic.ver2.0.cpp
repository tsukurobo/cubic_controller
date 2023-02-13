#include "Cubic.ver2.0.h"

//g_buf[0] = モーター0の方向(1or2)及びパリティー値
//g_buf[1] = モーター1~7の方向が一ビットづつ格納(モーター7の値は最下位2ビット使用 　正回転で0、負回転で

SPISettings Cubic_SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
uint8_t DC_motor::buf[DC_MOTOR_NUM + 2] = {1,0};
uint8_t Inc_enc::buf[INC_ENC_NUM*4];
uint8_t Abs_enc::buf[ABS_ENC_NUM*2];


// DCモータの初期化を行う関数
void DC_motor::begin(){
    pinMode(SS_DC_MOTOR,OUTPUT); 
    digitalWrite(SS_DC_MOTOR,HIGH);
    for(int i = 0; i < 10; i++) buf[i] = 0; // bufの値を0で初期化
    buf[0] = 1; // パリティの値を1で初期化
}

// モーターのdutyを代入する関数
void DC_motor::put(uint8_t num, int duty){
    // duty値を代入
    if(abs(duty) > 255) return; // 想定外の入力が来たら何もしない
    if(num >= DC_MOTOR_NUM) return;
    else if(abs(duty) < 3) buf[num+2] = 0; // パリティとして利用するので2以下の入力は0として出力
    else buf[num+2] = abs(duty); // その他の場合はduty値の絶対値を代入

    // 回転方向の値を代入
    // motor0の回転方向はパリティバイトbuf[0]に代入。正回転で1、負回転で2
    if(num == 0){
        if(duty < 0) buf[0] = 2;
        else buf[0] = 1;
    }

    // motor7の回転方向はbuf[1]の下位2ビットに代入。正回転で0,負回転で1
    else if(num == 7){ 
        if(duty < 0) buf[1] = (buf[1] & 0b11111100) | (0b11); 
        else buf[1] &= 0b11111100;
    }

    // その他の場合は最下位から(8-num)番目のビットに代入。正回転で0、負回転で1
    else{ 
        if(duty < 0) buf[1] |= (1<<(8-num)); //
        else buf[1] &= ((1<<(8-num)) ^ 0b11111111); 
    }
}

//配列データを送信する関数
void DC_motor::send(void){
    SPI.beginTransaction(Cubic_SPISettings);

    for (int i = 0; i < 10; i++) {//g_buf[]のデータをすべて送る
        digitalWrite(SS_DC_MOTOR,LOW);
        delayMicroseconds(SPI_DELAY);
        SPI.transfer(buf[i]);
        delayMicroseconds(SPI_DELAY);
        digitalWrite(SS_DC_MOTOR,HIGH);
    }

    SPI.endTransaction(); //通信終了
}

//シリアルモニターにbufの値を出力する関数
void DC_motor::check(void){
  for(int i = 0; i < DC_MOTOR_NUM + 2; i++){
    Serial.print(buf[i]);
    Serial.print("  ");
  }
  Serial.println();
}


void Inc_enc::begin(void){
    pinMode(SS_INC_ENC, OUTPUT); 
    digitalWrite(SS_INC_ENC, HIGH);
}

int32_t Inc_enc::get(uint8_t num){
    if(num >= INC_ENC_NUM) return 1;

    int32_t ret = 0;
    ret |= buf[num*4];
    ret |= buf[num*4+1] << 8;
    ret |= buf[num*4+2] << 16;
    ret |= buf[num*4+3] << 24;
    return ret;
}

void Inc_enc::receive(void){
    SPI.beginTransaction(Cubic_SPISettings);

    // データを受信
    for (int i = 0; i < INC_ENC_NUM*4; i++) {
        digitalWrite(SS_INC_ENC,LOW);
        delayMicroseconds(SPI_DELAY);
        buf[i] = SPI.transfer(0);
        delayMicroseconds(SPI_DELAY);
        digitalWrite(SS_INC_ENC,HIGH);
    }

    SPI.endTransaction(); //通信終了
}

void Inc_enc::check(void) {
    for (int i = 0; i < INC_ENC_NUM*4; i++) {
        Serial.print(buf[i], BIN);
        Serial.print(" ");
    }
    Serial.println();
}


void Abs_enc::begin(void){
    pinMode(SS_ABS_ENC, OUTPUT); 
    digitalWrite(SS_ABS_ENC, HIGH);
}

uint16_t Abs_enc::get(uint8_t num){
    if(num >= ABS_ENC_NUM) return 1;
    
    uint16_t ret = 0;
    ret |= buf[num*2];
    ret |= buf[num*2+1] << 8;
    return ret;
}

void Abs_enc::receive(void){
    //uint8_t dummy[ABS_ENC_NUM*2] = {0};
    SPI.beginTransaction(Cubic_SPISettings);

    // データを受信
    for (int i = 0; i < ABS_ENC_NUM*2; i++) {
        digitalWrite(SS_ABS_ENC,LOW);
        delayMicroseconds(SPI_DELAY);
        buf[i] = SPI.transfer(0);
        delayMicroseconds(SPI_DELAY);
        digitalWrite(SS_ABS_ENC,HIGH);
    }

    SPI.endTransaction(); //通信終了
}

void Abs_enc::check(void) {
    for (int i = 0; i < ABS_ENC_NUM*2; i++) {
        Serial.print(buf[i], BIN);
        Serial.print(" ");
    }
    Serial.println();
}


void Cubic::begin(){
    // SPI通信セットアップ
    SPI.begin();
    // Cubicの動作開始
    pinMode(ENABLE,OUTPUT);
    digitalWrite(ENABLE,HIGH);

    // DCモータの初期化
    DC_motor::begin();

    // インクリメントエンコーダの初期化
    Inc_enc::begin();

    // アブソリュートエンコーダの初期化
    Abs_enc::begin();
}

void Cubic::update() {
    DC_motor::send();
    Inc_enc::receive();
    Abs_enc::receive();
}