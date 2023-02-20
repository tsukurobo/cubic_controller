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
  using namespace Cubic_controller;
  static Velocity_PID velocityPID(6, 5, encoderType::inc, 0.5, 0.04, 0, 0, 15.0, true, true, 512);
  static Position_PID positionPID(7, 1, encoderType::abs, AMT22_PPR, 0.5, 0.2, 0.0, 0.0, degToRad(90.0), true, true);
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
    else if (c == 's')
    {
      velocityPID.setTarget(gain);
      positionPID.setTarget(degToRad(gain));
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