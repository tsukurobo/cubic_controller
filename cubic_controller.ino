#include <Arduino.h>
#include "cubic.ver2.0.h"
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
  static Velocity_PID velocityPID(0, 0, encoderType::inc, 180, 0.1, 0.1, 0.1, 100.0, false, true);
  static Position_PID positionPID(1, 1, encoderType::inc, 2048, 180, 0.1, 0.1, 0.1, 90.0, true, true);
  static bool stopFlag = false;
  if (Serial.available() > 0)
  {
    stopFlag = true;
  }
  if (stopFlag)
  {
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
  delay(1);
}