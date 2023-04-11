#pragma once
#include <Arduino.h>
#include <limits.h>

namespace PID
{
    constexpr bool EXCEED_MICROS_LIMIT = false;

    constexpr unsigned long MAX_MICROSECONDS = ULONG_MAX;
    constexpr double MICROSECONDS_TO_SECONDS = 1.0 / 1000000.0;

    class PID
    {
    private:
        double Kp;
        double Ki;
        double Kd;
        double current;
        double target;
        double diff;
        double preDiff;
        double integral;
        unsigned long preMicros;

        double dutyCycle = 0;
        double capableDutyCycle;

        bool direction;

    public:
        /// @brief dt[s]
        double dt;

        /**
         * @brief コントローラのコンストラクタ
         *
         * @param capableDutyCycle 出力最大Duty比（絶対値）
         * @param Kp 比例ゲイン
         * @param Ki 積分ゲイン
         * @param Kd 微分ゲイン
         * @param current 現在値
         * @param target 目標
         * @param direction 方向。trueで正方向、falseで負方向。
         */
        PID(double capableDutyCycle, double Kp, double Ki, double Kd, double current, double target, bool direction);

        /**
         * @brief ゲインを変更する。
         *
         * @param Kp 比例ゲイン
         * @param Ki 積分ゲイン
         * @param Kd 微分ゲイン
         */
        void setGains(double Kp, double Ki, double Kd);

        /**
         * @brief Set the Kp object
         *
         * @param Kp
         */
        void setKp(double Kp);

        /**
         * @brief Set the Ki object
         *
         * @param Ki
         */
        void setKi(double Ki);

        /**
         * @brief Set the Kd object
         *
         * @param Kd
         */
        void setKd(double Kd);

        /**
         * @brief 目標を変更する。
         *
         * @param target 目標
         */
        void setTarget(double target);

        /**
         * @brief 目標を取得する。
         *
         * @return double 目標
         */
        double getTarget() const;

        /**
         * @brief 現在値を取得する。
         *
         * @return double 現在値
         */
        double getCurrent() const;

        /**
         * @brief Duty比の取得
         * @details この関数は、compute_PID()によって計算されるduty比を取得するのに使用する。この関数内では、計算は行われない。基本的にこの関数を使用しなければならない場面は、マルチスレッドでもない限り想定されない。
         *
         * @return int duty比
         */
        double getDutyCycle() const;

        /**
         * @brief PID制御を行う関数
         *
         * @param current 現在値
         * @param logging ログを出力するかどうか。省略可能で、デフォルトではfalse。
         * @return int duty比
         */
        double compute_PID(double current, bool logging = false);

        /**
         * @brief Get the Dt object
         *
         * @return double dt[s]
         */
        double getDt() const
        {
            return dt;
        }
    };

    inline void PID::setGains(const double Kp, const double Ki, const double Kd)
    {
        this->Kp = Kp;
        this->Ki = Ki;
        this->Kd = Kd;
    }
    inline void PID::setKp(const double Kp)
    {
        this->Kp = Kp;
    }
    inline void PID::setKi(const double Ki)
    {
        this->Ki = Ki;
    }
    inline void PID::setKd(const double Kd)
    {
        this->Kd = Kd;
    }
    inline void PID::setTarget(const double target)
    {
        this->target = target;
    }
    inline double PID::getTarget() const
    {
        return this->target;
    }
    inline double PID::getCurrent() const
    {
        return this->current;
    }
    inline double PID::getDutyCycle() const
    {
        return this->dutyCycle;
    }

}