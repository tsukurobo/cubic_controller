/**
 * @file Cubic.controller.cpp
 */

#include "Cubic.controller.h"

namespace Cubic_controller
{
    Controller::Controller(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, double current, bool direction, bool logging) : motorNo(motorNo), encoderNo(encoderNo), encoderType(encoderType), CPR(CPR), capableDutyCycle(capableDutyCycle), direction(direction), logging(logging), pid(*(new PID::PID(capableDutyCycle, Kp, Ki, Kd, current, target, direction)))
    {
    }

    Velocity_PID::Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double p, double Kp, double Ki, double Kd, double target, bool direction, bool logging) : Controller(motorNo, encoderNo, encoderType, CPR, capableDutyCycle, Kp, Ki, Kd, target, 0.0, direction, logging), p(p)
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
        // low-pass filter
        static double vLPF = 0.0;
        vLPF = vLPF * (1.0 - p) + velocity * p;
        if (logging)
        {
            Serial.print("angle:");
            Serial.print(angle, 4);
            Serial.print(",");
        }
        double dutyCycle = this->compute_PID(vLPF);
        if (logging)
        {
            Serial.print("dutyCycle:");
            Serial.println(dutyCycle);
        }
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        return dutyCycle;
    }

    Position_PID::Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging) : Controller(motorNo, encoderNo, encoderType, CPR, capableDutyCycle, Kp, Ki, Kd, targetAngle, this->encoderToAngle(this->readEncoder()), direction, logging)
    {
        if (encoderType == encoderType::inc)
        {
            Serial.println("ERROR!! Incremental encoder can't be used for position PID.");
        }
        double currentAngle = this->getCurrent();
        if (logging)
        {
            Serial.print("current angle:");
            Serial.print(currentAngle);
            Serial.print(",");
        }
    }

    double Position_PID::compute()
    {
        int32_t encoder = this->readEncoder();
        double currentAngle = this->encoderToAngle(encoder);
        double actualAngle = currentAngle;
        static double prevAngle = currentAngle;
        static double dutyCycle = 0.0;
        if (encoder == ABS_ENC_ERR_RP2040)
        { // RP2040でエンコーダを正しく読めなかったとき e.g.)エンコーダが繋がっていない・線材の接触不良
            if (logging)
            {
                Serial.print("ERROR: ABS_ENC_ERR_RP2040. Skipping this loop.");
            }
        }
        else if (encoder == ABS_ENC_ERR)
        { // ArduinoとRP2040間のSPI通信でバグがある，または引数が間違っている
            if (logging)
            {
                Serial.print("ERROR: ABS_ENC_ERR. Skipping this loop.");
            }
        }
        else if (encoder > ABS_ENC_MAX)
        { // エンコーダの値が異常
            if (logging)
            {
                Serial.print("ERROR: encoder > ABS_ENC_MAX. Skipping this loop.");
            }
        }
        else
        {
            if (currentAngle < -LOOP_THRESHOLD && prevAngle > LOOP_THRESHOLD)
            {
                loopCount++;
            }
            else if (currentAngle > LOOP_THRESHOLD && prevAngle < -LOOP_THRESHOLD)
            {
                loopCount--;
            }
            currentAngle += loopCount * TWO_PI;

            if (logging)
            {
                Serial.print("actual angle:");
                Serial.print(actualAngle);
                Serial.print(",");
                Serial.print("loopCount:");
                Serial.print(loopCount);
                Serial.print(",");
            }

            dutyCycle = this->compute_PID(currentAngle);
            prevAngle = actualAngle;
        }
        if (logging)
        {
            Serial.print("dutyCycle:");
            Serial.println(dutyCycle);
        }
        DC_motor::put(motorNo, dutyCycle * DUTY_SPI_MAX, DUTY_SPI_MAX);
        return dutyCycle;
    }
}