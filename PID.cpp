#include "PID.h"

namespace PID
{
PID::PID(int capableDuty, double Kp, double Ki, double Kd,double current, double target, bool direction)
: Kp(Kp), Ki(Ki), Kd(Kd), capableDuty(abs(limitInPermitedDutyRange(capableDuty))), current(current), target(target), direction(direction){
  preMicros = micros();
  preDiff = 0;
  integral = 0;
}

int PID::compute_PID(double current, const bool logging){

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
  // dt = 0.001;

  this->current = current;
  diff = target - current;


  if(!direction){
  diff *= -1.0;
  }

  /* Compute duty */
  integral += (diff + preDiff) * dt / 2.0;
  duty = Kp * diff + Ki * integral + Kd * (diff - preDiff) / dt;

  duty = dutyLimiter();

  preDiff = diff;

  if (logging)
  {
    Serial.print("dt:");
    Serial.print(dt, 5);
    Serial.print(",current:");
    Serial.print(current);
    Serial.print(",target:");
    Serial.print(target);
    Serial.print(",diff:");
    Serial.print(diff);
    Serial.print(",duty:");
    Serial.println(duty);
  }
  return duty;
}

}