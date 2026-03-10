#pragma once
#include <algorithm>
#include <cmath>

class ScrewPID {
public:
    ScrewPID(double p, double i, double d, double deadzone, double max_out) 
        : Kp(p), Ki(i), Kd(d), deadzone_pixels(deadzone), max_output(max_out) {}
    
    void set_params(double p, double i, double d, double deadzone, double max_out) {
        Kp = p;
        Ki = i;
        Kd = d;
        deadzone_pixels = deadzone;
        max_output = max_out;
    }

    double compute(double current_error, double dt) {
        if (dt <= 0.0001) return 0.0;

        // 1. 丝杠专属：死区与静摩擦刹车
        // 只要误差在死区内，彻底清空所有动力和蓄力（积分）
        if (std::abs(current_error) <= deadzone_pixels) {
            integral_ = 0.0;
            last_error_ = current_error;
            return 0.0; // 电机死锁
        }

        // 2. 比例 P
        double p_out = Kp * current_error;

        // 3. 积分 I (带抗饱和，最多只允许蓄力到最大输出的 20%)
        integral_ += current_error * dt;
        double max_i = max_output * 0.2; 
        integral_ = std::clamp(integral_, -max_i, max_i);
        double i_out = Ki * integral_;

        // 4. 微分 D (丝杠专属：抗 70Hz 视觉高频抖动滤波)
        double raw_d = (current_error - last_error_) / dt;
        double d_out = Kd * raw_d;

        // 5. 汇总输出与限幅
        double total_out = p_out + i_out + d_out;
        total_out = std::clamp(total_out, -max_output, max_output);

        last_error_ = current_error;
        return total_out;
    }

    void reset() {
        integral_ = 0.0;
        last_error_ = 0.0;
    }

private:
    double Kp, Ki, Kd, deadzone_pixels, max_output;
    double integral_ = 0.0;
    double last_error_ = 0.0;
};