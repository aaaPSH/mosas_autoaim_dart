#ifndef UART_SERIAL_NODE_HPP_
#define UART_SERIAL_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "autoaim_interfaces/msg/green_dot.hpp"

namespace uart_serial
{
class UartSerialNode : public rclcpp::Node
{
public:
    explicit UartSerialNode(const rclcpp::NodeOptions & options);
    ~UartSerialNode();

private:
    // 视觉数据回调函数
    void green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);

    // ROS 2 订阅者与串口对象
    rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr green_dots_sub_;
    serial::Serial serial_;

    // =============== 核心状态机变量 ===============
    bool has_locked_before_ = false;  // 是否曾成功锁定目标
    int lost_count_ = 0;              // 连续丢帧计数器
    int16_t last_valid_yaw_ = 0;      // 上一帧有效的 Yaw (放大100倍后)
    int16_t last_valid_pitch_ = 0;    // 上一帧有效的 Pitch (放大100倍后)
};
} // namespace uart_serial

#endif  // UART_SERIAL_NODE_HPP_