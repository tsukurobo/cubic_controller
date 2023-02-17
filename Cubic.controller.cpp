#include "Cubic.controller.h"

namespace Cubic_controller
{
    int32_t readEncoder(const int encoderNo, const enum encoderType encoderType)
    {
        if (encoderType == encoderType::inc)
        {
            return Inc_enc::get(encoderNo);
        }
        else if (encoderType == encoderType::abs)
        {
            return Abs_enc::get(encoderNo);
        }
    }
    Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int capableDuty, double Kp, double Ki, double Kd, double target, bool direction, bool logging, uint16_t PPR) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), capableDuty(capableDuty), direction(direction), logging(logging), CPR(4 * PPR)
    {
        constexpr double current = 0.0;
        pid = new PID::PID(capableDuty, Kp, Ki, Kd, current, target, direction);
    }

    int Velocity_PID::compute()
    {
        static int32_t prevEncoder = readEncoder(encoderNo, encoderType);
        static double velocity = 0;
        velocity = readEncoder(encoderNo, encoderType) - prevEncoder;
        duty = pid->compute_PID(velocity, logging);
        DC_motor::put(motorNo, duty);
        return duty;
    }

    Position_PID::Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, int capableDuty, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(4 * PPR), capableDuty(capableDuty), direction(direction), logging(logging)
    {
        this->base = readEncoder(encoderNo, encoderType);
        constexpr double current = 0.0;
        pid = new PID::PID(capableDuty, Kp, Ki, Kd, current, targetAngle, direction);
    }

    int Position_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        double currentAngle = encoderToAngle(encoder);
        if (currentAngle - this->targetAngle > 180.0)
        {
            pid->setTarget(this->targetAngle + 360.0);
        }
        else if (this->targetAngle - currentAngle > 180.0)
        {
            pid->setTarget(this->targetAngle - 360.0);
        }
        if (logging)
        {
            Serial.print("current enc: ");
            Serial.print(encoder);
            Serial.print(",");
        }
        duty = pid->compute_PID(currentAngle, logging);
        DC_motor::put(motorNo, duty);
        return duty;
    }
}