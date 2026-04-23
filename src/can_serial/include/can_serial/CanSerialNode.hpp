#ifndef CAN_SERIAL__CAN_SERIAL_NODE_HPP_
#define CAN_SERIAL__CAN_SERIAL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <vector>
#include "autoaim_interfaces/msg/green_dot.hpp"
#include "CanSerialCore.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace can_serial
{
class CanSerialNode : public rclcpp::Node
{
public:
  explicit CanSerialNode(const rclcpp::NodeOptions & options);
  void send_command(const double& d_pixel);
  void handle_can_frame(const can_frame& frame);

private:
  double current_error;

  void green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);
   std::string to_binary_string(uint8_t value);

  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dots_sub_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  std::unique_ptr<CanSerial> can_core_;
  can_frame frame_;

  void parse_received_data();
  bool send_flag_;

  // CAN状态发布
  void checkCanHealth();
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr can_status_pub_;
  rclcpp::TimerBase::SharedPtr can_health_timer_;
  std::chrono::steady_clock::time_point last_rx_time_;
  bool can_initialized_;

  // CAN通信常量
  static constexpr uint32_t CAN_TX_ID = 0x106;      // 发送CAN帧ID
  static constexpr uint32_t CAN_RX_ID = 0x100;      // 接收CAN帧ID  
  static constexpr uint8_t CAN_FRAME_DLC = 8;       // CAN帧数据长度
  static constexpr uint8_t CAN_MIN_RX_DLC = 7;      // 最小接收数据长度

};
// 辅助函数：将 uint8_t 转换为二进制字符串
std::string CanSerialNode::to_binary_string(uint8_t value)
{
  std::ostringstream oss;
  for (int i = 7; i >= 0; --i)  // 从最高位到最低位
  {
    oss << ((value >> i) & 1);  // 提取第 i 位
  }
  return oss.str();
}
} 
#endif
