
#ifndef Cubic_h 
#define Cubic_h
#include "Arduino.h"
#include <SPI.h>

#define ENABLE 7 //cubicの仕様で、このピンをHIGHにすることによって動作開始

//SPI通信に用いるピン
#define MISO 12
#define MOSI 11 
#define SCK 13 
#define SS_INC_ENC 21 //インクリメントエンコーダのSS
#define SS_ABS_ENC 14 //アブソリュートエンコーダのSS
#define SS_DC_MOTOR 15   //DCモータのSS
#define SPI_DELAY 0 //SSの変化と情報送信との間のディレイ(us)

#define DC_MOTOR_NUM 8
#define INC_ENC_NUM 8
#define ABS_ENC_NUM 8

class DC_motor {
    public:
        static void begin(void); // 初期化
        static void put(uint8_t, int); //値を格納する関数
        static void send(void); //値をSPI通信で送信する関数
        static void check(void); //bufの値をSerial.print()で出力する関数
        //int current(void); //電流センサの値を返す関数
    private:
        static uint8_t buf[DC_MOTOR_NUM + 2]; //RP2040への送信データを格納する配列
};

class Inc_enc{
    public:
        static void begin(void);
        static int32_t get(uint8_t);
        static void receive(void);
        static void check(void);
    private:
        static uint8_t buf[INC_ENC_NUM*4];
};

class Abs_enc{
    public:
        static void begin(void);
        static uint16_t get(uint8_t);
        static void receive(void);
        static void check(void);
    private:
        static uint8_t buf[ABS_ENC_NUM*2];
};

class Cubic{
    public:
        static void begin(void);
        static void update(void);
};

#endif