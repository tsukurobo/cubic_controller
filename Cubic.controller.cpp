#include "Cubic.controller.h"

namespace Cubic_controller{
     DC_motor_PID::DC_motor_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int capableDuty, double Kp, double Ki, double Kd, double current, double target, bool direction, bool logging)  : motorNo(motorNo),encoderNo(encoderNo),encoderType(encoderType), capableDuty(capableDuty),  target(target), direction(direction),logging(logging) {
        pid = new PID::PID(capableDuty,Kp,Ki,Kd,current,target,direction);
    }

    void DC_motor_PID::compute(){
        duty = pid->compute_PID(readEncoder(encoderNo,encoderType),logging);
    }
}