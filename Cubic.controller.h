#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic.ver2.0.h"

namespace Cubic_controller
{
    enum class encoderType
    {
        inc,
        abs
    };

    int32_t readEncoder(const int encoderNo, const enum encoderType encoderType){
        if(encoderType == encoderType::inc){
            return Inc_enc::get(encoderNo);
        }
        else if(encoderType == encoderType::abs){
            return Abs_enc::get(encoderNo);
        }
    }
    class Velocity_PID
    {
    private:
        PID::PID *pid;
        uint8_t motorNo;
        enum class encoderType encoderType;
        uint8_t encoderNo;

        int capableDuty;
        int duty;
        double target;
        bool direction;
        bool logging;


    public:
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo,enum class encoderType  encoderType, int capableDuty, double Kp, double Ki, double Kd, double current, double target, bool direction, bool logging=true);
        void compute();
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int getDuty() const;
    };

    inline void Velocity_PID::setTarget(const double target)
    {
        this->target = target;
    }
    inline void Velocity_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    inline double Velocity_PID::getTarget() const
    {
        return this->target;
    }
    inline int Velocity_PID::getDuty() const
    {
        return this->duty;
    }
}