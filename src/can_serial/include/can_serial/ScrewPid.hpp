/**
 * @file ScrewPid.hpp
 * @brief 增量式 PID 控制器，用于云台速度控制
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <algorithm>
#include <cmath>

namespace can_serial
{

/**
 * @brief 增量式 PID 控制器
 *
 * 用于根据像素误差计算云台控制速度。
 * 包含积分抗饱和和死区处理。
 */
class ScrewPID
{
public:
  /**
   * @brief 构造函数
   * @param p 比例系数
   * @param i 积分系数
   * @param d 微分系数
   * @param deadzone 死区像素阈值，误差小于此值时输出 0
   * @param max_out 最大输出限幅
   */
  ScrewPID(double p, double i, double d, double deadzone, double max_out)
  : kp_(p), ki_(i), kd_(d), deadzone_pixels_(deadzone), max_output_(max_out)
  {
  }

  /**
   * @brief 动态设置 PID 参数
   * @param p 比例系数
   * @param i 积分系数
   * @param d 微分系数
   * @param deadzone 死区像素阈值
   * @param max_out 最大输出限幅
   */
  void set_params(double p, double i, double d, double deadzone, double max_out)
  {
    kp_ = p;
    ki_ = i;
    kd_ = d;
    deadzone_pixels_ = deadzone;
    max_output_ = max_out;
  }

  /**
   * @brief 计算 PID 输出
   * @param current_error 当前误差（像素差）
   * @param dt 距离上次计算的时间间隔（秒）
   * @return PID 输出速度值，已限幅到 [-max_output, max_output]
   */
  double compute(double current_error, double dt)
  {
    if (dt <= 0.0001) return 0.0;

    // 死区判断：误差在死区范围内则不输出
    if (std::abs(current_error) <= deadzone_pixels_) {
      integral_ = 0.0;
      last_error_ = current_error;
      return 0.0;
    }

    // 比例项 P
    double p_out = kp_ * current_error;

    // 积分项 I（带抗饱和，最多允许蓄力到最大输出的 20%）
    integral_ += current_error * dt;
    double max_i = max_output_ * 0.2;
    integral_ = std::clamp(integral_, -max_i, max_i);
    double i_out = ki_ * integral_;

    // 微分项 D
    double raw_d = (current_error - last_error_) / dt;
    double d_out = kd_ * raw_d;

    // 汇总输出与限幅
    double total_out = p_out + i_out + d_out;
    total_out = std::clamp(total_out, -max_output_, max_output_);

    last_error_ = current_error;
    return total_out;
  }

  /** @brief 重置积分项和上次误差，用于状态切换时清零 */
  void reset()
  {
    integral_ = 0.0;
    last_error_ = 0.0;
  }

private:
  double kp_, ki_, kd_, deadzone_pixels_, max_output_;
  double integral_ = 0.0;
  double last_error_ = 0.0;
};

}  // namespace can_serial
