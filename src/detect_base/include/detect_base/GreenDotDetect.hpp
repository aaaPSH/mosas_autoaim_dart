/**
 * @file GreenDotDetect.hpp
 * @brief 绿色灯点检测算法核心类
 * @author zllc <zllc@todo.todo>
 * @copyright Copyright (c) 2026 MOSAS Team
 */

#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>
#include <vector>

#include "detect_base/Dot.hpp"

namespace detect_base
{

   // ==========================================
   // 数据结构定义
   // ==========================================

   /**
    * @brief 检测参数配置结构体
    *
    * 针对 25m 距离、55mm 目标给出了建议默认值。
    * 参数可通过 ROS 参数服务器动态更新。
    */
   struct DetectParams
   {
      // --- 几何与 ROI 参数 ---
      double target_height = 600.0; ///< 目标离地高度 (mm)
      double camera_height = 0.0;   ///< 相机离地高度 (mm)
      double distance = 25000.0;    ///< 预估距离 (mm)，默认 25 米
      double detect_scale = 10.0;   ///< 搜索条带水平半视场角 (度)，总水平搜索角为 2*detect_scale
      int search_strip_min_h = 50;  ///< 搜索条带最小像素高度 (防止过窄)

      // --- 阈值与形态学 ---
      int v_low = 210; ///< 绿色通道二值化阈值 (需根据曝光调整)

      // --- 几何筛选 (针对半分辨率图) ---
      double min_area = 2.0;         ///< 最小面积 (对应原图上的像素数，代码中会 /4 处理)
      double max_area = 30.0;        ///< 最大面积
      double min_aspect_ratio = 0.4; ///< 最小长宽比
      double max_aspect_ratio = 2.5; ///< 最大长宽比
      double min_circularity = 0.4;  ///< 最小圆形度 (小目标像素化严重，圆形度不会太高)

      // --- 颜色比率 (Bayer 域) ---
      double min_gr_ratio = 3.0; ///< Green/Red 能量比
      double min_gb_ratio = 0.8; ///< Green/Blue 能量比

      // --- 标定修正 ---
      double calibrated_pixel_x = 0.0; ///< 校准的像素点 x 坐标，用于计算像素差和偏航角
   };

   // ==========================================
   // 核心检测类
   // ==========================================

   /**
    * @brief 绿色灯点检测器
    *
    * 从 Bayer Raw 图像中检测绿色 LED 灯点。
    * 流程：Bayer 降维 → ROI 计算 → 二值化 → 轮廓筛选 → 颜色比率验证 → 亚像素精定位 → 去畸变
    */
   class GreenDotDetect
   {
   public:
      GreenDotDetect() = default;
      ~GreenDotDetect() = default;

      /**
       * @brief 初始化相机内参
       * @param camera_matrix 3x3 内参矩阵
       * @param dist_coeffs 畸变系数
       */
      void init_camera(const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs);

      /**
       * @brief 更新检测参数
       * @param new_params 新的检测参数
       */
      void update_params(const DetectParams &new_params);

      /**
       * @brief 执行检测
       * @param raw_image 输入图像 (必须是单通道 Bayer Raw，假设为 BayerRG 格式)
       * @param dots 输出检测到的灯点列表
       * @param debug 是否开启可视化调试窗口
       * @return true 表示检测到至少一个目标
       */
      bool detect(const cv::Mat &raw_image, std::vector<Dot> &dots, bool debug = false);

      /**
       * @brief 计算所有目标的偏航角
       *
       * 结果直接写入 dots 结构体中的 yaw 字段。
       * @param dots 待计算偏航角的目标列表
       * @return true 表示计算成功
       */
      bool calculate_dots_yaw(std::vector<Dot> &dots);

   private:
      /**
       * @brief 计算亚像素质心 (灰度重心法)
       * @param roi_img ROI 灰度图像
       * @param local_rect 轮廓在 ROI 中的局部外接矩形
       * @return 亚像素精度的质心坐标
       */
      cv::Point2f compute_sub_pixel_center(const cv::Mat &roi_img, const cv::Rect &local_rect);

      // --- 内部成员变量 ---
      cv::Mat camera_matrix_; ///< 3x3 相机内参矩阵
      cv::Mat dist_coeffs_;   ///< 畸变系数
      DetectParams params_;   ///< 当前检测参数

      // --- 内存复用缓存 (零拷贝优化) ---
      cv::Mat half_green_; ///< 半分辨率绿色通道缓存
      cv::Mat mask_;       ///< 二值化掩码缓存
   };

} // namespace detect_base
