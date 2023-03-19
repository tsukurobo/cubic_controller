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

    Controller::Controller(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, double current, bool direction, bool logging) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(4 * PPR), capableDutyCycle(capableDutyCycle), direction(direction), logging(logging), pid(*(new PID::PID(capableDutyCycle, Kp, Ki, Kd, current, target, direction)))
    {
    }

    Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging) : Controller(motorNo, encoderNo, encoderType, PPR, capableDutyCycle, Kp, Ki, Kd, target, 0.0, direction, logging)
    {
        if (encoderType == encoderType::abs)
        {
            Serial.println("ERROR!! Absolute encoder can't be used for velocity PID.");
        }
    }

    double Velocity_PID::compute()
    {
        int32_t encoder = this->readEncoder();
        double angle = this->encoderToAngle(encoder);
        double velocity = angle / this->getDt();
        if (logging)
        {
            Serial.print("angle:");
            Serial.print(angle, 4);
            Serial.print(",");
            Serial.print("velocity:");
            Serial.print(velocity);
            Serial.print(",");
        }
        double dutyCycle = this->compute_PID(velocity);
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        return dutyCycle;
    }

    Position_PID::Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(4 * PPR), capableDutyCycle(capableDutyCycle), targetAngle(targetAngle), direction(direction), logging(logging)
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        double currentAngle = this->encoderToAngle(encoder);
        pid = new PID::PID(capableDutyCycle, Kp, Ki, Kd, currentAngle, targetAngle, direction);
        if (logging)
        {
            Serial.print("current angle:");
            Serial.print(currentAngle);
            Serial.print(",");
        }
        if (targetAngle - currentAngle >= 0)
        {
            if (logging)
            {
                Serial.print("isGoingForward:");
                Serial.println("true");
            }
            isGoingForward = true;
        }
        else
        {
            if (logging)
            {
                Serial.print("isGoingForward:");
                Serial.println("false");
            }
            isGoingForward = false;
        }
        if (encoderType == encoderType::inc)
        {
            Serial.println("ERROR!! encoderType is inc");
        }
    }

    double Position_PID::compute()
    {
        int32_t encoder = readEncoder(encoderNo, encoderType);
        if (encoder <= 16384)
        {
            double currentAngle = this->encoderToAngle(encoder);
            double actualAngle = currentAngle;
            static double prevAngle = currentAngle;
            if (logging)
            {
                Serial.print("current enc:");
                Serial.print(encoder);
                Serial.print(",");
                // Serial.print("current angle:");
                // Serial.print(currentAngle);
                // Serial.print(",");
                // Serial.print("target angle:");
                // Serial.print(this->targetAngle);
                // Serial.print(",");
                // Serial.print("isGoingForward:");
                // Serial.print(isGoingForward);
                // Serial.print(",");
            }

            if (isGoingForward)
            {
                if (currentAngle < -5 * PI / 6 && prevAngle > 5 * PI / 6)
                {
                    isOverMax = true;
                }
            }
            else
            {
                if (currentAngle > 5 * PI / 6 && prevAngle < -5 * PI / 6)
                {
                    isOverMin = true;
                }
            }
            if (logging)
            {
                Serial.print("isOverMax:");
                Serial.print(isOverMax);
                Serial.print(",");
                Serial.print("isOverMin:");
                Serial.print(isOverMin);
                Serial.print(",");
            }

            dutyCycle = pid->compute_PID(currentAngle, logging);
            if (isOverMax)
            {
                if (currentAngle > 5 * PI / 6)
                {
                    isOverMax = false;
                }
                dutyCycle = capableDutyCycle * (direction ? -1.0 : 1.0);
            }
            else if (isOverMin)
            {
                if (currentAngle < -5 * PI / 6)
                {
                    isOverMin = false;
                }
                dutyCycle = capableDutyCycle * (direction ? 1.0 : -1.0);
            }
            prevAngle = actualAngle;
        }
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        return dutyCycle;
    }
}