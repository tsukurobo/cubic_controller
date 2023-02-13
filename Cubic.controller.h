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
    class DC_motor_PID
    {
    private:
        PID::PID *pid;
        uint8_t motorNo;
        enum class encoderType encoderType;
        uint8_t encoderNo;

        int capableDuty;
        int duty;
        int target;
        bool direction;
        bool logging;


    public:
        DC_motor_PID(uint8_t motorNo, uint8_t encoderNo,enum class encoderType  encoderType, int capableDuty, double Kp, double Ki, double Kd, double current, double target, bool direction, bool logging=true);
        void compute();
        void setTarget(int target);
        void setGains(double Kp, double Ki, double Kd);
        int getTarget() const;
        int getDuty() const;
    };

    inline void DC_motor_PID::setTarget(const int target)
    {
        this->target = target;
    }
    inline void DC_motor_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    inline int DC_motor_PID::getTarget() const
    {
        return this->target;
    }
    inline int DC_motor_PID::getDuty() const
    {
        return this->duty;
    }
}