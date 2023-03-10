/* This is a sample program for Cubic Control Library. */
#include <Arduino.h>
#include "cubic_arduino_ver2.5.h"
#include "PID.h"
#include "Cubic.controller.h"

void setup()
{
  Cubic::begin();
  Serial.begin(115200);
  /* ↓魔術（消すと最初の数回エンコーダの値が読めないので消すな！！） */
  Cubic::update();
  Cubic::update();
  Cubic::update();
}

void loop()
{
  using namespace Cubic_controller;
  static Velocity_PID velocityPID(6, 5, encoderType::inc, 0.5, 0.04, 0, 0, 15.0, true, true, 512);
  static Position_PID positionPID(7, 0, encoderType::abs, AMT22_PPR, 0.5, 0.4, 0.1, 0.0, degToRad(90.0), true, true);
  static bool stopFlag = false;
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
    else
    {
      stopFlag = true;
    }
  }
  if (stopFlag)
  {
    Serial.println("stopping...");
    for (int i = 0; i < 8; i++)
    {
      DC_motor::put(i, 0);
    }
  }
  else
  {
    // velocityPID.compute();
    positionPID.compute();
  }
  Cubic::update();
  delay(1);
}