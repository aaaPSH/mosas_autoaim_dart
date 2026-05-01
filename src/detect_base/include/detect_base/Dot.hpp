#pragma once
#include <opencv2/opencv.hpp>
struct Dot
{
  double yaw;           // 偏航角 (度)
  cv::Point2f center;   // 去畸变后的像素坐标
};