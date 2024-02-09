/* This is a sample program for Cubic Control Library. */
#include <Arduino.h>
#include "cubic_arduino.h"
#include "PID.h"
#include "Cubic.controller.h"

void setup()
{
  Cubic::begin(true);
  Serial.begin(115200);
}

void loop()
{
  // using namespace Cubic_controller;

  uint8_t motorNo = 13; uint8_t encoderNo = 0;
  uint16_t CPR = 2048 * 4; // エンコーダのCPR(カウント/1回転) = PPR(パルス/1回転)
  double Kp = 4.0; double Ki = 0.0; double Kd = 0.0; // PIDゲイン
  double velTarget = 4.0; double posTarget = degToRad(90.0); // 目標角速度[rad/s]、目標角度[rad] (-PI<= target < PI)
  bool direction = true; // モータが正回転したときにエンコーダの値が+方向に増えるならtrue
  double capableDutyCycle = 0.5; // 最大許容デューティ比。0.0~1.0。省略可能で、デフォルトは1.0。
  double lowpassFilter = 1.0; // ローパスフィルタの係数。0.0~1.0。省略可能で、デフォルトは1.0(フィルタなし)。
  bool logging = true; // ログをSerial.printで出力するかどうか。省略可能で、デフォルトはfalse。

  static Cubic_controller::Velocity_PID velocityPID(motorNo, encoderNo, Cubic_controller::encoderType::inc, CPR, Kp, Ki, Kd, velTarget, direction, capableDutyCycle, lowpassFilter, logging);
  static Cubic_controller::Position_PID positionPID(motorNo, encoderNo, Cubic_controller::encoderType::abs, Cubic_controller::AMT22_CPR, Kp, Ki, Kd, posTarget, direction, capableDutyCycle, logging);

  static bool stopFlag = false;
  static bool confirmFlag = false;
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    double value = Serial.readStringUntil('\n').toDouble();
    if (c == 'p')
    {
      velocityPID.setKp(value);
      positionPID.setKp(value);
    }
    else if (c == 'i')
    {
      velocityPID.setKi(value);
      positionPID.setKi(value);
      velocityPID.reset();
      positionPID.reset();
    }
    else if (c == 'd')
    {
      velocityPID.setKd(value);
      positionPID.setKd(value);
    }
    else if (c == 's')
    {
      velocityPID.setTarget(value);
      positionPID.setTarget(degToRad(value));
    }
    else if (c == 'l'){
      velocityPID.setLPF(value);
    }
    else if (c == 'r'){
      velocityPID.reset();
      positionPID.reset();
    }
    else if (c == 'c')
    {
      confirmFlag = true;
    }
    else
    {
      stopFlag = true;
    }
  }
  if (stopFlag)
  {
    Serial.println("stopping...");
    for (int i = 0; i < 23; i++)
    {
      DC_motor::put(i, 0);
    }
  }
  else if (confirmFlag)
  {
    // モータの回転方向確認()
    for (int i = 0; i < 23; i++)
    {
      DC_motor::put(i, 100);
    }
    Serial.print("Inc: ");
    Inc_enc::print();
    Serial.print(", Abs: ");
    Abs_enc::print(true);
  }
  else
  {
    velocityPID.compute();
    // positionPID.compute();
  }
  Cubic::update();
}