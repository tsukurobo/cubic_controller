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
  static Velocity_PID velocityPID(0, 0, encoderType::inc, 180, 3.0, 0.0, 0.0, 50.0, false, true, 2048);
  static Position_PID positionPID(1, 1, encoderType::abs, AMT22_PPR, 180, 350.0, 6.0, 8.0, degToRad(90.0), false, true);
  static bool stopFlag = false;
  if (Serial.available() > 0)
  {
    stopFlag = true;
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
    for (int i = 0; i < 8; i++)
    {
      // Serial.print(Abs_enc::get(i));
    }
  }
  Cubic::update();
  delay(1);
}