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
    int32_t readEncoder(uint8_t encoderNo, enum encoderType encoderType);

    /**
     * @brief 与えられた角度を一定範囲（min<=angle<min+2pi）に収めます
     *
     * @param angle 角度[rad]
     * @param min 最低値[rad]。省略可能で、デフォルトは-PI
     * @return constexpr double 角度[rad](-PI<= angle < PI)
     */
    constexpr double limitAngle(double angle, const double min = -PI)
    {
        while (angle < min)
        {
            angle += TWO_PI;
        }
        while (angle >= min + TWO_PI)
        {
            angle -= TWO_PI;
        }
        return angle;
    }

    /**
     * @brief 与えられたCPRのもと、エンコーダの値から角度を計算します
     *
     * @param encoder エンコーダの値
     * @param CPR counts per revolution(PPRの4倍)
     * @param offset オフセット[rad]。省略可能で、デフォルトは0.0
     * @return constexpr double angle[rad](-PI<= angle < PI)
     */
    constexpr double encoderToAngle(int32_t encoder, uint16_t CPR, double offset = 0.0)
    {
        double angle = offset + encoder * TWO_PI / (double)CPR;
        limitAngle(angle);
        return angle;
    }

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
        void setTarget(double target);
        void setGains(double Kp, double Ki, double Kd);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        int32_t readEncoder() const;
        double getTarget() const;
        double getDutyCycle() const;
        double getDt() const;
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief 速度制御を行うためのクラスです。
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
        double compute() override;
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
        bool isGoingForward;
        bool isOverMax = false;
        bool isOverMin = false;

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
         */
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t PPR, double capableDutyCycle, double Kp, double Ki, double Kd, double targetAngle, bool direction, bool logging = true);

        double compute();
        void setTarget(double targetAngle);
        void setGains(double Kp, double Ki, double Kd);
        void setKp(double Kp);
        void setKi(double Ki);
        void setKd(double Kd);
        double getTarget() const;
        double getDuty() const;
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief 目標角度を設定します。
     *
     * @param targetAngle [rad] -PI~PI
     */
    inline void Position_PID::setTarget(double targetAngle)
    {
        targetAngle = limitAngle(targetAngle);
        this->targetAngle = targetAngle;
        this->pid->setTarget(targetAngle);
        double currentAngle = this->encoderToAngle(readEncoder(encoderNo, encoderType));
        if (targetAngle - currentAngle >= 0)
        {
            isGoingForward = true;
        }
        else
        {
            isGoingForward = false;
        }
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
        return Cubic_controller::encoderToAngle(encoder, CPR, -PI);
    }


    // Definition

    inline double Controller::compute_PID(const double current)
    {
        return dutyCycle = this->pid.compute(current, logging);
    }
    inline void Controller::setTarget(const double target)
    {
        this->pid.setTarget(target);
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