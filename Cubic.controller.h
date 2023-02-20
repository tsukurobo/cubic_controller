/**
 * @file Cubic.controller.h
 */

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

    /**
     * @brief 度数法から弧度法に変換します
     *
     * @param deg
     * @return constexpr double
     */
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

    /**
     * @brief 与えられたCPRのもと、エンコーダの値から角度を計算します
     *
     * @param encoder エンコーダの値
     * @param CPR counts per revolution(PPRの4倍)
     * @return constexpr double angle[rad](-PI<= angle < PI)
     */
    constexpr double encoderToAngle(int32_t encoder, uint16_t CPR);

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

        double capableDutyCycle;
        double dutyCycle;
        bool direction;
        bool logging;

    public:
        /**
         * @brief Construct a new Velocity_PID object
         *
         * @param motorNo モータ番号
         * @param encoderNo エンコーダ番号
         * @param encoderType エンコーダの種類
         * @param capableDutyCycle 最大許容デューティ比。0.0~1.0
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target 目標速度[rad/s]
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはtrue。
         * @param PPR エンコーダのPPR（CPRでないことに注意）
         *
         */
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true, uint16_t PPR = 1024);
        double compute();
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        double getTarget() const;
        double getDuty() const;
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

        double capableDutyCycle;
        double dutyCycle;
        double targetAngle;
        bool direction;
        bool logging;
        bool is_gear_ratio_two;
        bool current_cycle;

    public:
        /**
         * @brief Construct a new Position_PID object
         *
         * @param motorNo
         * @param encoderNo
         * @param encoderType
         * @param PPR エンコーダのPPR（CPRでないことに注意）
         * @param capableDutyCycle モーターに与えられる最大のduty比。0~1の範囲で指定する。
         * @param Kp
         * @param Ki
         * @param Kd
         * @param targetAngle 目標角度[rad]
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはtrue。
         * @param is_gear_ratio_two ギア比が2:1の場合はtrue。1:1の場合はfalse。省略可能で、デフォルトはfalse。
         */
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging = true, bool is_gear_ratio_two = false);

        double compute();
        void setTarget(double target);
        void setTargetByRelative(double target);
        void setGains(double Kp, double Ki, double Kd);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        double getTarget() const;
        double getDuty() const;
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief 目標速度を設定します。
     *
     * @param target [rad/s] -PI~PI
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
     * @return double [rad/s]
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
    inline double Velocity_PID::getDuty() const
    {
        return this->dutyCycle;
    }

    /**
     * @brief Kpを設定します。
     *
     * @param Kp
     */
    inline void Velocity_PID::setKp(const double Kp)
    {
        this->pid->setKp(Kp);
    }

    /**
     * @brief Kiを設定します。
     *
     * @param Ki
     */
    inline void Velocity_PID::setKi(const double Ki)
    {
        this->pid->setKi(Ki);
    }

    /**
     * @brief Kdを設定します。
     *
     * @param Kd
     */
    inline void Velocity_PID::setKd(const double Kd)
    {
        this->pid->setKd(Kd);
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
     * @brief Kpを設定します。
     *
     * @param Kp
     */
    inline void Position_PID::setKp(const double Kp)
    {
        this->pid->setKp(Kp);
    }
    /**
     * @brief Kiを設定します。
     *
     * @param Ki
     */
    inline void Position_PID::setKi(const double Ki)
    {
        this->pid->setKi(Ki);
    }
    /**
     * @brief Kdを設定します。
     *
     * @param Kd
     */
    inline void Position_PID::setKd(const double Kd)
    {
        this->pid->setKd(Kd);
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
    inline double Position_PID::getDuty() const
    {
        return this->dutyCycle;
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