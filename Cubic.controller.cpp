#include "Cubic.controller.h"

namespace Cubic_controller
{
    int32_t readEncoder(const uint8_t encoderNo, const enum encoderType encoderType)
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

    double encoderToAngle(const int32_t encoder, const uint16_t CPR)
    {
        double angle = encoder * TWO_PI / (double)CPR;
        while (angle < -PI)
        {
            angle += TWO_PI;
        }
        while (angle >= PI)
        {
            angle -= TWO_PI;
        }
        return angle;
    }

    Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int16_t capableDuty, double Kp, double Ki, double Kd, double target, bool direction, bool logging, uint16_t PPR) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), capableDuty(capableDuty), direction(direction), logging(logging), CPR(4 * PPR)
    {
        constexpr double current = 0.0;
        pid = new PID::PID(capableDuty, Kp, Ki, Kd, current, target, direction);

        if(encoderType==encoderType::abs){
            Serial.println("ERROR!! encoderType is abs");
        }
    }

    int16_t Velocity_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        double velocity = encoderToAngle(encoder,this->CPR)/this->pid->getDt();
        duty = pid->compute_PID(velocity, logging);
        DC_motor::put(motorNo, duty);
        return duty;
    }

    Position_PID::Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, int16_t capableDuty, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(4 * PPR), capableDuty(capableDuty), targetAngle(targetAngle), direction(direction), logging(logging)
    {
        this->base = readEncoder(encoderNo, encoderType);
        constexpr double current = 0.0;
        pid = new PID::PID(capableDuty, Kp, Ki, Kd, current, targetAngle, direction);
        if(encoderType==encoderType::inc){
            Serial.println("ERROR!! encoderType is inc");
        }
    }

    int16_t Position_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        double currentAngle = this->encoderToAngle(encoder);
        if (logging)
        {
            Serial.print("current enc:");
            Serial.print(encoder);
            Serial.print(",");
            Serial.print("current angle:");
            Serial.print(currentAngle);
            Serial.print(",");
            Serial.print("target angle:");
            Serial.print(this->targetAngle);
            Serial.print(",");
        }

        while (currentAngle - this->targetAngle > PI)
        {
            this->targetAngle += TWO_PI;
            pid->setTarget(this->targetAngle);
        }
        while (this->targetAngle - currentAngle > PI)
        {
            this->targetAngle -= TWO_PI;
            pid->setTarget(this->targetAngle);
        }
        duty = pid->compute_PID(currentAngle, logging);
        DC_motor::put(motorNo, duty);
        return duty;
    }
}