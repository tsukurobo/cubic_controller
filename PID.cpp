#include "PID.h"

namespace PID
{
  PID::PID(double capableDutyCycle, double Kp, double Ki, double Kd, double current, double target, bool direction)
      : Kp(Kp), Ki(Ki), Kd(Kd), capableDutyCycle(capableDutyCycle), current(current), target(target), direction(direction)
  {
    preMicros = micros();
    preDiff = 0;
    integral = 0;
  }

  double PID::compute_PID(double current, const bool logging)
  {
    /* Update dt */
    unsigned long nowMicros = micros();
    if (nowMicros < preMicros)
    {
      dt = MAX_MICROSECONDS - preMicros + nowMicros;
    }
    else
    {
      dt = nowMicros - preMicros;
    }
    dt /= 1000000.0;
    preMicros = nowMicros;

    this->current = current;
    diff = target - current;

    if (!direction)
    {
      diff *= -1.0;
    }

    /* Compute dutyCycle */
    integral += (diff + preDiff) * dt / 2.0;
    dutyCycle = Kp * diff + Ki * integral + Kd * (diff - preDiff) / dt;

    Serial.print("integral:");
    Serial.print(integral);
    Serial.print(",");

    if (dutyCycle > capableDutyCycle)
    {
      integral -= (diff + preDiff) * dt / 2.0;
      dutyCycle = capableDutyCycle;
    }
    else if (dutyCycle < -capableDutyCycle)
    {
      integral -= (diff + preDiff) * dt / 2.0;
      dutyCycle = -capableDutyCycle;
    }

    preDiff = diff;

    if (logging)
    {
      Serial.print("dt:");
      Serial.print(dt, 4);
      Serial.print(",");
      Serial.print("current:");
      Serial.print(current);
      Serial.print(",target:");
      Serial.print(target);
      Serial.print(",diff:");
      Serial.print(diff);
      Serial.print(",");
    }
    return dutyCycle;
  }
}