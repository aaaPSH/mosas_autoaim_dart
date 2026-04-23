#ifndef SYSTEM_MONITOR__SYSTEM_MONITOR_NODE_HPP_
#define SYSTEM_MONITOR__SYSTEM_MONITOR_NODE_HPP_

#include <fstream>
#include <memory>
#include <string>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/string.hpp"

namespace system_monitor
{

class SystemMonitorNode : public rclcpp::Node
{
public:
  explicit SystemMonitorNode(const rclcpp::NodeOptions & options);
  ~SystemMonitorNode();

private:
  // 回调函数
  void greenDotCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);
  void canStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void timerCallback();

  // 日志记录
  void writeLog(const std::string & message);
  void checkAndWriteLog();
  std::string generateLogFilename();

  // 订阅者
  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dot_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr can_status_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // 状态
  std::string can_status_;
  bool green_dot_detected_;
  double green_dot_x_;
  double green_dot_y_;
  bool can_status_received_;
  bool green_dot_received_;

  // 日志文件
  std::ofstream log_file_;
  std::string log_path_;

  // 参数
  int check_interval_ms_;
  int can_timeout_ms_;

  // 时间跟踪
  rclcpp::Time last_can_msg_time_;
  rclcpp::Time last_green_dot_time_;
  rclcpp::Time last_log_time_;
};

}  // namespace system_monitor

#endif  // SYSTEM_MONITOR__SYSTEM_MONITOR_NODE_HPP_
