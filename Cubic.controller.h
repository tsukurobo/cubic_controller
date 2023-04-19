/**
 * @file Cubic.controller.h
 */

#pragma once
#include <Arduino.h>
#include "PID.h"
#include "cubic_arduino.h"

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
 * @brief 弧度法から度数法に変換します
 *
 * @param rad
 * @return constexpr double
 */
constexpr double radToDeg(double rad)
{
    return rad * RAD_TO_DEG;
}

namespace Cubic_controller
{
    /**
     * @brief AMT22のCPRです
     * @details CPR: Counts Per Revolution
     */
    constexpr uint16_t AMT22_CPR = 16384;

    /// @brief アブソリュートエンコーダのループ閾値
    constexpr double LOOP_THRESHOLD = 5.0 * PI / 6.0;

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
    constexpr double encoderToAngle(const int32_t encoder, const uint16_t CPR, const double offset = 0.0)
    {
        return limitAngle(offset + encoder * (TWO_PI / (double)CPR));
    }

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

    protected:
        /// @brief モータ番号
        const uint8_t motorNo;
        /// @brief エンコーダの種類
        const enum class encoderType encoderType;
        /// @brief エンコーダ番号
        const uint8_t encoderNo;
        /**
         * @brief CPR(Counts Per Revolution)
         * @details CPR = PPR * 4
         */
        const uint16_t CPR;
        /// @brief モータをプラスの方向に回したとき、エンコーダが増加するかどうか
        const bool direction;
        /// @brief ログを出力するかどうか
        const bool logging;

        /**
         * @brief pid.compute_PID()を呼ぶだけの関数です。
         *
         * @param current
         * @return double
         */
        double compute_PID(double current);

    public:
        /**
         * @brief Construct a new Controller object
         *
         * @param motorNo
         * @param encoderNo
         * @param encoderType
         * @param CPR
         * @param capableDutyCycle
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target
         * @param current
         * @param direction
         * @param logging
         */
        Controller(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, double current, bool direction, bool logging = false);

        /**
         * @brief duty比を計算します。各ループで一回呼び出してください。このduty比は、DUTY_SPI_MAXに対する比です。計算された値は、この関数内部で、DC_motor::put()されます。
         *
         * @return double dutyCycle
         */
        virtual double compute() = 0;
        /**
         * @brief 制御量の目標値を設定します。
         *
         * @param target
         */
        virtual void setTarget(double target);
        /**
         * @brief PIDゲインを設定します。
         *
         * @param Kp
         * @param Ki
         * @param Kd
         */
        void setGains(double Kp, double Ki, double Kd);
        /**
         * @brief Pゲインを設定します。
         *
         * @param Kp
         */
        void setKp(double Kp);
        /**
         * @brief Iゲインを設定します。
         *
         * @param Ki
         */
        void setKi(double Ki);
        /**
         * @brief Dゲインを設定します。
         *
         * @param Kd
         */
        void setKd(double Kd);
        /**
         * @brief エンコーダを読みだします。
         *
         * @return int32_t encoder
         */
        int32_t readEncoder() const;
        /**
         * @brief 目標値を返します。
         *
         * @return double target
         */
        double getTarget() const;
        /**
         * @brief 直前に計算したデューティ比を返します。
         *
         * @return double dutyCycle
         */
        double getDutyCycle() const;
        /**
         * @brief 直前のループにおける経過時間dtを返します
         *
         * @return double dt[s]
         */
        double getDt() const;
        /**
         * @brief 直前に読んだ制御量を返します。
         *
         * @return double
         */
        double getCurrent() const;
        /**
         * @brief エンコーダの値から角度を計算します。設定したCPR(Count Per Revolution)に依存します。
         *
         * @param encoder
         * @return double angle[rad](-PI<= angle < PI)
         */
        double encoderToAngle(int32_t encoder) const;
    };

    /**
     * @brief インクリメンタルエンコーダを用いた、DCモータの速度制御を行うためのクラス
     * @details このクラスでは、エンコーダの角速度[rad/s]を制御量とします。
     */
    class Velocity_PID : public Controller
    {
    private:
        double p;

    public:
        /**
         * @brief Construct a new Velocity_PID object
         *
         * @param motorNo モータ番号
         * @param encoderNo エンコーダ番号
         * @param encoderType エンコーダの種類
         * @param CPR エンコーダのCPR（PPRでないことに注意。CPR=PPR*4）
         * @param capableDutyCycle 最大許容デューティ比。0.0~1.0
         * @param p ローパスフィルタの係数。0.0~1.0
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target 目標速度[rad/s]
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはfalse。
         *
         */
        Velocity_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double p, double Kp, double Ki, double Kd, double target, bool direction, bool logging = false);
        /**
         * @brief ローパスフィルタの係数pを設定します。
         *
         * @param p
         */
        void setLPF(double p);
        double compute() override;
    };

    /**
     * @brief アブソリュートエンコーダを用いた、DCモータの位置制御を行うためのクラス
     *
     */
    class Position_PID : public Controller
    {
    private:
        int8_t loopCount = 0;

    public:
        /**
         * @brief Construct a new Position_PID object
         *
         * @param motorNo モータ番号
         * @param encoderNo エンコーダ番号
         * @param encoderType エンコーダの種類
         * @param CPR エンコーダのCPR（PPRでないことに注意。CPR=PPR*4）
         * @param capableDutyCycle 最大許容デューティ比。0.0~1.0
         * @param Kp
         * @param Ki
         * @param Kd
         * @param target 目標角度[rad] (-PI<= target < PI)
         * @param direction モーターに正のdutyを与えたときに、エンコーダが正方向に回転するかどうか。trueなら正方向、falseなら負方向。
         * @param logging ログをSerial.printで出力するかどうか。省略可能で、デフォルトはfalse。
         */
        Position_PID(uint8_t motorNo, uint8_t encoderNo, enum class encoderType encoderType, uint16_t CPR, double capableDutyCycle, double Kp, double Ki, double Kd, double target, bool direction, bool logging = false);

        void setTarget(double target) override;
        double compute() override;
    };

    // Definition

    inline double Controller::compute_PID(const double current)
    {
        return dutyCycle = this->pid.compute_PID(current, logging);
    }
    inline void Controller::setTarget(const double target)
    {
        this->pid.setTarget(target);
    }
    inline void Position_PID::setTarget(const double target)
    {
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
        int32_t value = encoderType == encoderType::inc ? Inc_enc::get_diff(encoderNo) : Abs_enc::get(encoderNo);
        if (logging)
        {
            Serial.print("encoder:");
            Serial.print(value);
            Serial.print(",");
        }
        return value;
    }
    inline void Velocity_PID::setLPF(const double p)
    {
        this->p = p;
    }
}