#include "system_monitor/SystemMonitorNode.hpp"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

namespace system_monitor
{

SystemMonitorNode::SystemMonitorNode(const rclcpp::NodeOptions & options)
: Node("system_monitor_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing SystemMonitorNode...");

  // 声明参数
  log_path_ = this->declare_parameter("log_path", "./logs");
  check_interval_ms_ = this->declare_parameter("check_interval_ms", 1000);
  can_timeout_ms_ = this->declare_parameter("can_timeout_ms", 2000);

  // 初始化状态
  can_status_ = "UNKNOWN";
  green_dot_detected_ = false;
  green_dot_x_ = 0.0;
  green_dot_y_ = 0.0;
  can_status_received_ = false;
  green_dot_received_ = false;

  // 生成带时间戳的日志文件名
  std::string log_filename = generateLogFilename();
  std::filesystem::create_directories(log_path_);
  
  // 打开日志文件
  log_file_.open(log_filename, std::ios::app);
  if (!log_file_.is_open()) {
    RCLCPP_WARN(this->get_logger(), "Failed to open log file: %s", log_filename.c_str());
  } else {
    RCLCPP_INFO(this->get_logger(), "Log file opened: %s", log_filename.c_str());
    writeLog("=== System Monitor Started ===");
  }

  // 订阅绿灯检测话题
  green_dot_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
    "/detections/green_dots", rclcpp::SensorDataQoS(),
    std::bind(&SystemMonitorNode::greenDotCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /detections/green_dots");

  // 订阅CAN状态话题
  can_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/can_status", rclcpp::SensorDataQoS(),
    std::bind(&SystemMonitorNode::canStatusCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to /can_status");

  // 创建定时器，定期检查状态并写入日志
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(check_interval_ms_),
    std::bind(&SystemMonitorNode::timerCallback, this));

  RCLCPP_INFO(this->get_logger(), "SystemMonitorNode initialized successfully");
}

SystemMonitorNode::~SystemMonitorNode()
{
  if (log_file_.is_open()) {
    writeLog("=== System Monitor Stopped ===");
    log_file_.close();
  }
  RCLCPP_INFO(this->get_logger(), "SystemMonitorNode destroyed");
}

void SystemMonitorNode::greenDotCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
  green_dot_received_ = true;
  last_green_dot_time_ = this->now();

  // 判断是否检测到绿灯（x >= 0 表示检测到目标）
  if (msg->x >= 0.0) {
    green_dot_detected_ = true;
    green_dot_x_ = msg->x;
    green_dot_y_ = msg->y;
  } else {
    green_dot_detected_ = false;
    green_dot_x_ = 0.0;
    green_dot_y_ = 0.0;
  }
}

void SystemMonitorNode::canStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  can_status_received_ = true;
  last_can_msg_time_ = this->now();
  can_status_ = msg->data;
}

void SystemMonitorNode::timerCallback()
{
  // 检查CAN超时
  if (can_status_received_) {
    auto now = this->now();
    auto elapsed_ms = (now - last_can_msg_time_).seconds() * 1000.0;
    if (elapsed_ms > can_timeout_ms_) {
      can_status_ = "OFFLINE";
    }
  }

  // 检查绿灯检测超时（如果超过2秒没有收到检测消息，认为丢失）
  if (green_dot_received_) {
    auto now = this->now();
    auto elapsed_ms = (now - last_green_dot_time_).seconds() * 1000.0;
    if (elapsed_ms > 2000) {
      green_dot_detected_ = false;
    }
  }

  // 写入日志
  checkAndWriteLog();
}

void SystemMonitorNode::checkAndWriteLog()
{
  // 构建状态字符串
  std::string green_dot_status;
  if (!green_dot_received_) {
    green_dot_status = "NO_DATA";
  } else if (green_dot_detected_) {
    std::ostringstream oss;
    oss << "DETECTED (x=" << std::fixed << std::setprecision(1)
        << green_dot_x_ << ", y=" << green_dot_y_ << ")";
    green_dot_status = oss.str();
  } else {
    green_dot_status = "LOST";
  }

  std::string log_message = "CAN: " + can_status_ + " | GREEN_DOT: " + green_dot_status;

  // 写入日志
  writeLog(log_message);

  // 同时输出到ROS日志
  RCLCPP_INFO(this->get_logger(), "%s", log_message.c_str());
}

std::string SystemMonitorNode::generateLogFilename()
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  std::stringstream ss;
  ss << log_path_ << "/system_monitor_"
     << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << "_" << std::setfill('0') << std::setw(3) << ms.count()
     << ".log";
  return ss.str();
}

void SystemMonitorNode::writeLog(const std::string & message)
{
  if (!log_file_.is_open()) {
    return;
  }

  // 获取当前时间
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;

  // 格式化时间戳
  std::stringstream ss;
  ss << "[" << std::put_time(std::localtime(&time_t), "%Y-%m-%d %H:%M:%S")
     << "." << std::setfill('0') << std::setw(3) << ms.count() << "] "
     << message;

  // 写入文件
  log_file_ << ss.str() << std::endl;
  log_file_.flush();
}

}  // namespace system_monitor

RCLCPP_COMPONENTS_REGISTER_NODE(system_monitor::SystemMonitorNode)
