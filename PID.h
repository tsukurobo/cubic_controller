#pragma once
#include <Arduino.h>
#include <limits.h>

namespace PID
{
    constexpr unsigned long MAX_MICROSECONDS = ULONG_MAX;

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
        long double integral;
        unsigned long preMicros;

        double dutyCycle = 0;
        double capableDutyCycle;

        double dutyCycleLimiter(); /* Limit the dutyCycle and reset integral if limited. */

        bool direction;

    public:
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
         * @brief 制御量（モーターのduty比）の計算を行う。loop内で呼び出すことを想定している。
         *
         * @param ifPrint 関数中で情報をSerial.print()するかどうか。省略可能（デフォルトはfalse）主にデバッグ時の使用を想定している。
         * @return int 計算されたduty比を返す。
         */
        int compute(double current, bool ifPrint = false) {}

        /**
         * @brief ゲインを変更する。
         *
         * @param Kp 比例ゲイン
         * @param Ki 積分ゲイン
         * @param Kd 微分ゲイン
         */
        void setGains(double Kp, double Ki, double Kd);

        void setKp(double Kp);

        void setKi(double Ki);

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
         * @brief Duty比の取得
         * @details この関数は、compute()によって計算されるduty比を取得するのに使用する。この関数内では、計算は行われない。基本的にこの関数を使用しなければならない場面は、マルチスレッドでもない限り想定されない。
         *
         * @return int duty比
         */
        double getDuty() const;

        /**
         * @brief PID制御を行う関数
         *
         * @param current 現在値
         * @param ifPrint ログを出力するかどうか
         * @return int duty比
         */
        double compute_PID(double current, bool ifPrint);

        /**
         * @brief Get the Dt object
         *
         * @return double dt
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
    inline double PID::getDuty() const
    {
        return this->dutyCycle;
    }
    inline double PID::dutyCycleLimiter()
    {
        if (abs(dutyCycle) > capableDutyCycle)
            integral = 0; // Anti-windup
        dutyCycle = dutyCycle > capableDutyCycle    ? capableDutyCycle
                    : dutyCycle < -capableDutyCycle ? -capableDutyCycle
                                                    : dutyCycle;
        return dutyCycle;
    }

}