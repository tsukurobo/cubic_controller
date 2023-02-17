#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic_arduino_ver2.3.h"

namespace Cubic_controller
{
    enum class encoderType
    {
        inc,
        abs
    };

    int32_t readEncoder(int encoderNo, enum encoderType encoderType);

    class Velocity_PID
    {
    private:
        PID::PID *pid;
        uint8_t motorNo;
        enum class encoderType encoderType;
        uint8_t encoderNo;

        int capableDuty;
        int duty;
        bool direction;
        bool logging;

    public:
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int capableDuty, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true);
        int compute();
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int getDuty() const;
    };

    class Position_PID
    {
    private:
        PID::PID *pid;
        uint8_t motorNo;
        enum class encoderType encoderType;
        uint8_t encoderNo;
        uint16_t CPR;
        int32_t base;

        int capableDuty;
        int duty;
        double targetAngle;
        bool direction;
        bool logging;

    public:
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, int capableDuty, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging = true);
        int compute();
        void setTarget(double target);
        void setTargetByRelative(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int getDuty() const;
        double encoderToAngle(int32_t encoder) const;
        int32_t angleToEncoder(double angle, int32_t currentEncoder) const;
    };

    inline void Velocity_PID::setTarget(const double target)
    {
        this->pid->setTarget(target);
    }
    inline void Velocity_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    inline double Velocity_PID::getTarget() const
    {
        return this->pid->getTarget();
    }
    inline int Velocity_PID::getDuty() const
    {
        return this->duty;
    }

    inline void Position_PID::setTarget(const double target)
    {
        this->targetAngle = target;
        this->pid->setTarget(target);
    }

    inline void Position_PID::setTargetByRelative(const double target)
    {
        // get current angle and add target
        this->targetAngle = encoderToAngle(readEncoder(this->encoderNo, this->encoderType)) + target;
        this->pid->setTarget(this->targetAngle);
    }

    inline void Position_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    inline double Position_PID::getTarget() const
    {
        return this->targetAngle;
    }
    inline int Position_PID::getDuty() const
    {
        return this->duty;
    }

    inline double Position_PID::encoderToAngle(const int32_t encoder) const
    {
        double angle = encoder * 360.0 / (double)CPR;
        while (angle < 0.0)
        {
            angle += 360.0;
        }
        while (angle >= 360.0)
        {
            angle -= 360.0;
        }
        return angle;
    }
    inline int32_t Position_PID::angleToEncoder(const double angle, const int32_t currentEncoder) const
    {
        int32_t encoder = base + (angle * CPR) / 360.0;
        encoder += (currentEncoder - encoder) / CPR;
        int32_t diff = encoder - currentEncoder;
        if (diff > 0)
        {
            if (diff > CPR / 2)
            {
                encoder -= CPR;
            }
        }
        else
        {
            if (diff < -CPR / 2)
            {
                encoder += CPR;
            }
        }
        return encoder;
    }
}