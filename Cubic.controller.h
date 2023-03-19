/**
 * @file Cubic.controller.h
 */

#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic_arduino_ver2.5.h"

namespace Cubic_controller
{
    /**
     * @brief AMT22のPPRです
     * @details PPR: Pulse Per Revolution
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
    int32_t readEncoder(uint8_t encoderNo, const enum class encoderType encoderType);

    /**
     * @brief 与えられた角度を一定範囲（min<=angle<min+2pi）に収めます
     *
     * @param angle 角度[rad]
     * @param min 最低値[rad]。省略可能で、デフォルトは-PI
     * @return constexpr double 角度[rad](-PI<= angle < PI)
     */
    constexpr double limitAngle(double angle, double min = -PI);

    /**
     * @brief 与えられたCPRのもと、エンコーダの値から角度を計算します
     *
     * @param encoder エンコーダの値
     * @param CPR counts per revolution(PPRの4倍)
     * @param offset オフセット[rad]。省略可能で、デフォルトは0.0
     * @return constexpr double angle[rad](-PI<= angle < PI)
     */
    constexpr double encoderToAngle(int32_t encoder, uint16_t CPR, double offset = 0.0);

    /**
     * @brief Cubic制御器の抽象クラス
     *
     */
    class Controller
    {
    private:
        PID::PID &pid;

        double capableDutyCycle;
        double dutyCycle;

    public:
        Controller(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, double current, bool direction, bool logging = true);

        const uint8_t motorNo;
        const enum class encoderType encoderType;
        const uint8_t encoderNo;
        const uint16_t CPR;
        const bool direction;
        const bool logging;

        virtual double compute() = 0;
        double compute_PID(double current);
        virtual void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        int32_t readEncoder() const;
        double getTarget() const;
        double getCurrent() const;
        double getDutyCycle() const;
        double getDt() const;
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief インクリメンタルエンコーダを用いた、DCモータの速度制御を行うためのクラス
     *
     */
    class Velocity_PID : public Controller
    {
    public:
        /**
         * @brief Construct a new Velocity_PID object
         *
         * @param motorNo モータ番号
         * @param encoderNo エンコーダ番号
         * @param encoderType エンコーダの種類
         * @param PPR エンコーダのPPR（CPRでないことに注意）
         * @param capableDutyCycle 最大許容デューティ比。0.0~1.0
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target 目標速度[rad/s]
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはtrue。
         *
         */
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true);
        double compute() override;
    };

    /**
     * @brief アブソリュートエンコーダを用いた、DCモータの位置制御を行うためのクラス
     *
     */
    class Position_PID : public Controller
    {
    private:
        bool isGoingForward;
        int8_t loopCount = 0;

    public:
        /**
         * @brief Construct a new Position_PID object
         *
         * @param motorNo モータ番号
         * @param encoderNo エンコーダ番号
         * @param encoderType エンコーダの種類
         * @param PPR エンコーダのPPR（CPRでないことに注意）
         * @param capableDutyCycle 最大許容デューティ比。0.0~1.0
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target 目標角度[rad] (-PI<= target < PI)
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはtrue。
         */
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging = true);

        void setTarget(double target) override;
        double compute() override;
    };

    // Definition

    inline double Controller::compute_PID(const double current)
    {
        return dutyCycle = this->pid.compute(current, logging);
    }
    inline void Controller::setTarget(const double target)
    {
        this->pid.setTarget(target);
    }
    inline void Position_PID::setTarget(const double target)
    {
        this->isGoingForward = (target - this->getCurrent()) >= 0;
        Controller::setTarget(target);
    }
    inline void Controller::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->pid.setGains(Kp, Ki, Kd);
    }
    inline void Controller::setKp(const double Kp)
    {
        this->pid.setKp(Kp);
    }
    inline void Controller::setKi(const double Ki)
    {
        this->pid.setKi(Ki);
    }
    inline void Controller::setKd(const double Kd)
    {
        this->pid.setKd(Kd);
    }
    inline double Controller::getTarget() const
    {
        return this->pid.getTarget();
    }
    inline double Controller::getCurrent() const
    {
        return this->pid.getCurrent();
    }
    inline double Controller::getDutyCycle() const
    {
        return this->dutyCycle;
    }
    inline double Controller::getDt() const
    {
        return this->pid.getDt();
    }
    inline double Controller::encoderToAngle(const int32_t encoder) const
    {
        return Cubic_controller::encoderToAngle(encoder, this->CPR);
    }
    inline int32_t Controller::readEncoder() const
    {
        int32_t value = encoderType == encoderType::inc ? Inc_enc::get(encoderNo) : Abs_enc::get(encoderNo);
        if (logging)
        {
            Serial.print("encoder:");
            Serial.print(value);
            Serial.print(",");
        }
        return value;
    }
}