#include "Cubic_arduino_ver2.3.h"

SPISettings DC_motor_SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings Inc_enc_SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings Abs_enc_SPISettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
int16_t DC_motor::buf[DC_MOTOR_NUM+SOL_SUB_NUM];
uint8_t Inc_enc::buf[INC_ENC_NUM*INC_ENC_BYTES];
uint8_t Abs_enc::buf[ABS_ENC_NUM*ABS_ENC_BYTES];
unsigned long Solenoid::time_prev[SOL_SUB_NUM];

void DC_motor::begin(){
    pinMode(SS_DC_MOTOR,OUTPUT);
    digitalWrite(SS_DC_MOTOR,HIGH);
}

void DC_motor::put(uint8_t num, int16_t duty, uint16_t duty_max){
    // 想定外の入力が来たら何もしない
    if(duty_max > DUTY_SPI_MAX) return;
    if(abs(duty) > duty_max) return; 
    if(num >= DC_MOTOR_NUM) return;

    // duty値を代入
    buf[num] = (int16_t)((float)duty/(float)duty_max * (float)DUTY_SPI_MAX);
}

void DC_motor::send(void){
    uint8_t *l_buf = (uint8_t*)buf;

    SPI.beginTransaction(DC_motor_SPISettings);
    // スレーブ側で割り込み処理を検知させる
    digitalWrite(SS_DC_MOTOR,LOW);
    delayMicroseconds(SPI_DELAY);
    digitalWrite(SS_DC_MOTOR,HIGH);
    delayMicroseconds(SPI_DELAY);
    // データの送信
    for (int i = 0; i < (DC_MOTOR_NUM+SOL_SUB_NUM)*DC_MOTOR_BYTES; i++) {
        digitalWrite(SS_DC_MOTOR,LOW);
        SPI.transfer(l_buf[i]);
        digitalWrite(SS_DC_MOTOR,HIGH);
    }
    SPI.endTransaction();
}

void DC_motor::print(bool new_line){
    for (int i = 0; i < DC_MOTOR_NUM+SOL_SUB_NUM; i++) {
        if (abs(buf[i]) == DUTY_SPI_MAX + 1 && i >= DC_MOTOR_NUM) {
            //Serial.print(Solenoid::get(i-DC_MOTOR_NUM));
            Serial.print("SOL");
        }
        else {
            Serial.print(buf[i]);
        }
        Serial.print(" ");
    }
    if (new_line == true)
        Serial.println();
    else
        Serial.print(" ");
}


void Solenoid::begin(void) {
    for (int i = 0; i < SOL_SUB_NUM; i++) {
        time_prev[i] = millis();
    }
}

void Solenoid::put(uint8_t num, bool state) {
    if (num >= SOL_SUB_NUM) return;
    if (DC_motor::buf[DC_MOTOR_NUM+num] == (state ? DUTY_SPI_MAX + 1 : -(DUTY_SPI_MAX + 1))) return;

    unsigned long time_now = millis();
    if (time_now - time_prev[num] < SOL_TIME_MIN) return;

    DC_motor::buf[DC_MOTOR_NUM+num] = (state ? DUTY_SPI_MAX + 1 : -(DUTY_SPI_MAX + 1));
    time_prev[num] = time_now;
}

int8_t Solenoid::get(uint8_t num) {
    if (num >= SOL_SUB_NUM) return -1;
    int16_t raw_val = DC_motor::buf[DC_MOTOR_NUM+num];
    return (abs(raw_val) == DUTY_SPI_MAX + 1 ? (raw_val < 0 ? 0 : 1) : -1);
}

void Solenoid::print(bool new_line) {
    for (int i = 0; i < SOL_SUB_NUM; i++) {
        Serial.print(Solenoid::get(i));
        Serial.print(" ");
    }
    if (new_line == true)
        Serial.println();
    else
        Serial.print(" ");
}


void Inc_enc::begin(void){
    pinMode(SS_INC_ENC, OUTPUT); 
    digitalWrite(SS_INC_ENC, HIGH);
}

int16_t Inc_enc::get(uint8_t num){
    if(num >= INC_ENC_NUM) return 1;

    int16_t ret = 0;
    ret |= buf[num*INC_ENC_BYTES];
    ret |= buf[num*INC_ENC_BYTES+1] << 8;
    return ret;
}

void Inc_enc::receive(void){
    SPI.beginTransaction(Inc_enc_SPISettings);
    // スレーブ側で割り込み処理を検知させる
    digitalWrite(SS_INC_ENC,LOW);
    delayMicroseconds(SPI_DELAY);
    digitalWrite(SS_INC_ENC,HIGH);
    delayMicroseconds(SPI_DELAY);
    // データを受信
    for (int i = 0; i < INC_ENC_NUM*INC_ENC_BYTES; i++) {
        digitalWrite(SS_INC_ENC,LOW);
        buf[i] = SPI.transfer(0x88);
        digitalWrite(SS_INC_ENC,HIGH);
    }
    SPI.endTransaction();
}

void Inc_enc::print(bool new_line) {
    for (int i = 0; i < INC_ENC_NUM; i++) {
        Serial.print(get(i));
        Serial.print(" ");
    }
    if (new_line == true)
        Serial.println();
    else
        Serial.print(" ");
}


void Abs_enc::begin(void){
    pinMode(SS_ABS_ENC, OUTPUT); 
    digitalWrite(SS_ABS_ENC, HIGH);
}

uint16_t Abs_enc::get(uint8_t num){
    if(num >= ABS_ENC_NUM) return 1;
    
    uint16_t ret = 0;
    ret |= buf[num*ABS_ENC_BYTES];
    ret |= buf[num*ABS_ENC_BYTES+1] << 8;
    return ret;
}

void Abs_enc::receive(void){
    SPI.beginTransaction(Abs_enc_SPISettings);
    // スレーブ側で割り込み処理を検知させる
    digitalWrite(SS_ABS_ENC,LOW);
    delayMicroseconds(SPI_DELAY);
    digitalWrite(SS_ABS_ENC,HIGH);
    delayMicroseconds(SPI_DELAY);
    // データを受信
    for (int i = 0; i < ABS_ENC_NUM*ABS_ENC_BYTES; i++) {
        digitalWrite(SS_ABS_ENC,LOW);
        buf[i] = SPI.transfer(0x88);
        digitalWrite(SS_ABS_ENC,HIGH);
    }

    SPI.endTransaction();
}

void Abs_enc::print(bool new_line) {
    for (int i = 0; i < ABS_ENC_NUM; i++) {
        Serial.print(get(i));
        Serial.print(" ");
    }
    if (new_line == true)
        Serial.println();
    else
        Serial.print(" ");
}


void Cubic::begin(){
    // Cubicの動作開始
    pinMode(ENABLE,OUTPUT);
    digitalWrite(ENABLE,HIGH);
    // DCモータの初期化
    DC_motor::begin();
    // インクリメントエンコーダの初期化
    Inc_enc::begin();
    // アブソリュートエンコーダの初期化
    Abs_enc::begin();
    // ソレノイドの初期化
    Solenoid::begin();
    // SPI通信セットアップ
    SPI.begin();
    pinMode(MISO, INPUT_PULLUP);
    // ADCのSSの初期化
    pinMode(SS_ADC,OUTPUT);
    digitalWrite(SS_ADC,HIGH);
}

void Cubic::update() {
    DC_motor::send();
    Abs_enc::receive();
    Inc_enc::receive();
}