#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic_arduino_ver2.3.h"

namespace Cubic_controller
{
    /**
     * @brief AMT22のPPRです
     */
    constexpr uint16_t AMT22_PPR = 16384 / 4;

    constexpr double degToRad(double deg)
    {
        return deg * DEG_TO_RAD;
    }

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
    int32_t readEncoder(uint8_t encoderNo, enum encoderType encoderType);

    double encoderToAngle(int32_t encoder, uint16_t CPR);

    int32_t angleToEncoder(double angle, uint16_t CPR);

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

        int16_t capableDuty;
        int16_t duty;
        bool direction;
        bool logging;

    public:
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, int16_t capableDuty, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true, uint16_t PPR = 1024);
        int16_t compute();
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int16_t getDuty() const;
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

        int16_t capableDuty;
        int16_t duty;
        double targetAngle;
        bool direction;
        bool logging;
        bool is_gear_ratio_two;
        bool current_cycle;

    public:
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, int16_t capableDuty, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging = true, bool is_gear_ratio_two = false);
        int16_t compute();
        void setTarget(double target);
        void setTargetByRelative(double target);
        void setGains(double Kp, double Ki, double Kd);
        double getTarget() const;
        int16_t getDuty() const;
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief 目標速度を設定します。
     *
     * @param target [rad/s] -360~360
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
     * @brief 目標角度を取得します。
     *
     * @return double [rad]
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
    inline int16_t Velocity_PID::getDuty() const
    {
        return this->duty;
    }

    /**
     * @brief 目標角度を設定します。
     *
     * @param target [rad] -PI~PI
     */
    inline void Position_PID::setTarget(const double target)
    {
        this->targetAngle = target;
        this->pid->setTarget(target);
    }

    /**
     * @brief 目標角度を現在の角度に対して相対的に設定します。
     *
     * @param target [rad]
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
     * @return double [rad]
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
    inline int16_t Position_PID::getDuty() const
    {
        return this->duty;
    }
    /**
     * @brief エンコーダの値を角度に変換します。
     *
     * @param encoder
     * @return double angle[rad] -PI~PI
     */
    inline double Position_PID::encoderToAngle(const int32_t encoder) const
    {
        return Cubic_controller::encoderToAngle(encoder, this->CPR);
    }
}