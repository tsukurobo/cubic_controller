#include "Cubic.controller.h"

namespace Cubic_controller{
     Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int capableDuty, double Kp, double Ki, double Kd, double current, double target, bool direction, bool logging)  : motorNo(motorNo),encoderNo(encoderNo),encoderType(encoderType), capableDuty(capableDuty),  target(target), direction(direction),logging(logging) {
        pid = new PID::PID(capableDuty,Kp,Ki,Kd,current,target,direction);
    }

    void Velocity_PID::compute(){
        static int32_t prevEncoder = readEncoder(encoderNo,encoderType);
        static double velocity = 0;
        velocity = readEncoder(encoderNo,encoderType) - prevEncoder;
        duty = pid->compute_PID(velocity,logging);
    }
}