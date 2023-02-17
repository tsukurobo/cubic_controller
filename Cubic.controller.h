#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic_arduino_ver2.3.h"

namespace Cubic_controller
{
    /**
     * @brief エンコーダの種類を示します
     *
     * @param inc incremental encoder
     * @param abs absolute encoder
     *
     */
    enum class encoderType
    {
        inc,
        abs
    };

    /**
     * @brief エンコーダを読みます
     *
     * @param encoderNo
     * @param encoderType
     * @return int32_t 値
     */
    int32_t readEncoder(int encoderNo, enum encoderType encoderType);

    /**
     * @brief 速度制御を行うためのクラスです。
     *
     */
    class Velocity_PID
    {
    private:
        PID::PID *pid;
        uint8_t motorNo;
        enum class encoderType encoderType;
        uint8_t encoderNo;
        uint16_t CPR;

        int capableDuty;
        int duty;
        bool direction;
        bool logging;

    public:
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int capableDuty, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true, uint16_t PPR = 1024);
        int compute();
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int getDuty() const;
    };

    /**
     * @brief 位置制御を行うためのクラスです。
     *
     */
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

    /**
     * @brief 目標速度を設定します。
     *
     * @param target エンコーダのカウント値/秒
     */
    inline void Velocity_PID::setTarget(const double target)
    {
        this->pid->setTarget(target);
    }
    /**
     * @brief PIDゲインを（再）設定します。
     *
     * @param Kp
     * @param Ki
     * @param Kd
     */
    inline void Velocity_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    /**
     * @brief 目標速度を取得します。
     *
     * @return double
     */
    inline double Velocity_PID::getTarget() const
    {
        return this->pid->getTarget();
    }
    /**
     * @brief Duty比を取得します。内部で計算は行われないので、マルチスレッドでもない限り使用は想定していません。
     *
     * @return int
     */
    inline int Velocity_PID::getDuty() const
    {
        return this->duty;
    }

    /**
     * @brief 目標角度を設定します。
     *
     * @param target [deg] 0~360
     */
    inline void Position_PID::setTarget(const double target)
    {
        this->targetAngle = target;
        this->pid->setTarget(target);
    }

    /**
     * @brief 目標角度を現在の角度に対して相対的に設定します。
     *
     * @param target [deg]
     */
    inline void Position_PID::setTargetByRelative(const double target)
    {
        // get current angle and add target
        this->targetAngle = encoderToAngle(readEncoder(this->encoderNo, this->encoderType)) + target;
        this->pid->setTarget(this->targetAngle);
    }

    /**
     * @brief PIDゲインを（再）設定します。
     *
     * @param Kp
     * @param Ki
     * @param Kd
     */
    inline void Position_PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid->setGains(Kp, Ki, Kd);
    }
    /**
     * @brief 目標角度を取得します。
     *
     * @return double [deg]
     */
    inline double Position_PID::getTarget() const
    {
        return this->targetAngle;
    }
    /**
     * @brief Duty比を取得します。内部で計算は行われないので、マルチスレッドでもない限り使用は想定していません。
     *
     * @return int
     */
    inline int Position_PID::getDuty() const
    {
        return this->duty;
    }
    /**
     * @brief エンコーダの値を角度に変換します。
     *
     * @param encoder
     * @return double [deg] 0~360
     */
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
    /**
     * @brief 角度をエンコーダの値に変換します。
     *
     * @param angle [deg] 0~360
     * @param currentEncoder 現在のエンコーダ値。
     * @return int32_t 現在のエンコーダ値に最も近い、angleを示す値を返します。
     */
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