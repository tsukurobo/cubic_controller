#include <Arduino.h>
#include "cubic_arduino_ver2.3.h"
#include "PID.h"
#include "Cubic.controller.h"
void setup()
{
  Cubic::begin();
  Serial.begin(115200);
}

void loop()
{
  static int cnt = 0;
  cnt++;
  static unsigned long preTime = micros();
  if(cnt==10000){
    Serial.println(micros()-preTime);
    preTime = micros();
    cnt = 0;
  }
  using namespace Cubic_controller;
  static Velocity_PID velocityPID(0, 0, encoderType::inc, 0.2, 900.0, 0.0, 0.0, 0.3, false, true, 2048);
  static Position_PID positionPID(1, 1, encoderType::abs, AMT22_PPR, 0.2, 350.0, 6.0, 8.0, degToRad(90.0), false, true, true);
  static bool stopFlag = false;
  if (Serial.available() > 0)
  {
    char c = Serial.read();
    double gain = Serial.readStringUntil('\n').toDouble();
    if (c == 'p')
    {
      velocityPID.setKp(gain);
      positionPID.setKp(gain);
    }
    else if (c == 'i')
    {
      velocityPID.setKi(gain);
      positionPID.setKi(gain);
    }
    else if (c == 'd')
    {
      velocityPID.setKd(gain);
      positionPID.setKd(gain);
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
    velocityPID.compute();
    // positionPID.compute();
  }
  Cubic::update();
  delay(1);
}