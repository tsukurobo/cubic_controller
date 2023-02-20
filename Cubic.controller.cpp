/**
 * @file Cubic.controller.cpp
 */

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

    constexpr double encoderToAngle(const int32_t encoder, const uint16_t CPR)
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

    Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging, uint16_t PPR) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), capableDutyCycle(capableDutyCycle), direction(direction), logging(logging), CPR(4 * PPR)
    {
        constexpr double current = 0.0;
        pid = new PID::PID(this->capableDutyCycle, Kp, Ki, Kd, current, target, direction);

        if (encoderType == encoderType::abs)
        {
            Serial.println("ERROR!! encoderType is abs");
        }
    }

    double Velocity_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        if (logging)
        {
            Serial.print("encoder:");
            Serial.print(encoder);
            Serial.print(",");
        }
        double angle = encoderToAngle(encoder, this->CPR);
        double velocity = angle / this->pid->getDt();
        if (logging)
        {
            Serial.print("angle:");
            Serial.print(angle,4);
            Serial.print(",");
            Serial.print("velocity:");
            Serial.print(velocity);
            Serial.print(",");
        }
        dutyCycle = pid->compute_PID(velocity, logging);
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        return dutyCycle;
    }

    Position_PID::Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging, bool is_gear_ratio_two) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(4 * PPR), capableDutyCycle(capableDutyCycle), targetAngle(targetAngle), direction(direction), logging(logging), is_gear_ratio_two(is_gear_ratio_two)
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        double currentAngle = this->encoderToAngle(encoder);
        pid = new PID::PID(capableDutyCycle, Kp, Ki, Kd, currentAngle, targetAngle, direction);
        if (encoderType == encoderType::inc)
        {
            Serial.println("ERROR!! encoderType is inc");
        }
    }

    double Position_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        static int32_t prevEncoder = encoder;
        int32_t actualEncoder = encoder;
        double currentAngle;
        if (is_gear_ratio_two == true)
        {
            if (logging)
            {
                Serial.print("actual enc:");
                Serial.print(encoder);
                Serial.print(",");
                Serial.print("current cycle:");
                Serial.print(current_cycle);
                Serial.print(",");
            }
            if ((prevEncoder > CPR * 5 / 6 && encoder < CPR / 6) || (prevEncoder < CPR / 6 && encoder > CPR * 5 / 6))
            {
                current_cycle ^= 1;
            }
            if (current_cycle)
            {
                encoder += CPR;
            }
            encoder /= 2;
        }
        currentAngle = this->encoderToAngle(encoder);
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
        dutyCycle = pid->compute_PID(currentAngle, logging);
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        prevEncoder = actualEncoder;
        return dutyCycle;
    }
}