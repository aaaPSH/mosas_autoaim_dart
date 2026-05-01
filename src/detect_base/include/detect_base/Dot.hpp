/**
 * @file Dot.hpp
 * @brief 检测目标结构体定义
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <opencv2/opencv.hpp>

namespace detect_base
{

  /**
   * @brief 单个检测目标
   *
   * 包含偏航角和去畸变后的像素坐标。
   */
  struct Dot
  {
    double yaw = 0.0;   ///< 偏航角 (度)
    cv::Point2f center; ///< 去畸变后的像素坐标
  };

} // namespace detect_base
