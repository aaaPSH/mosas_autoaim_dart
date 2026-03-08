#include "uart_serial/UartSerialNode.hpp"
#include <cstring>

namespace uart_serial
{

UartSerialNode::UartSerialNode(const rclcpp::NodeOptions & options)
: Node("uart_serial_node", options)
{
    // ================= [ 1. 声明并读取参数 ] =================
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("debug_level", 1);

    std::string port_name = this->get_parameter("port_name").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();

    // ================= [ 2. 初始化串口 ] =================
    try {
        serial_.setPort(port_name);
        serial_.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        serial_.setTimeout(to);
        serial_.open();
    } catch (serial::IOException & e) {
        RCLCPP_FATAL(this->get_logger(), "串口开启失败！请检查 %s 是否存在且具备读写权限。", port_name.c_str());
        throw; // 硬件没准备好，直接抛出异常阻止节点启动
    }

    if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "串口初始化成功! 端口: %s, 波特率: %d", port_name.c_str(), baud_rate);
    }

    // ================= [ 3. 初始化订阅者 ] =================
    green_dots_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
        "/detections/green_dots", rclcpp::SensorDataQoS(),
        std::bind(&UartSerialNode::green_dots_callback, this, std::placeholders::_1));
}

UartSerialNode::~UartSerialNode()
{
    // 节点销毁时安全关闭串口
    if (serial_.isOpen()) {
        serial_.close();
    }
}

void UartSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
    const float angle_scale = 100.0f;
    int debug_level = this->get_parameter("debug_level").as_int();

    // 初始化 8 字节发送缓冲区，并清零
    uint8_t tx_buffer[5];
    std::memset(tx_buffer, 0, sizeof(tx_buffer));

    // 填入固定的帧头
    tx_buffer[0] = 0x55;
    tx_buffer[1] = 0xAA;

    // 判定条件：如果 x, y 小于 0，视为目标丢失
    bool no_dot = (msg->x < 0 && msg->y < 0);

    // ================= [ 4. 核心状态机逻辑 ] =================
    if (!no_dot) 
    {
        // 【状态 1：看到目标】
        has_locked_before_ = true;
        lost_count_ = 0;

        double target_yaw = msg->angle_yaw;

        // 浮点数转 int16_t (保留0.01精度)
        int16_t send_yaw = static_cast<int16_t>(target_yaw * angle_scale);

        // 记忆当前帧
        last_valid_yaw_ = send_yaw;

        tx_buffer[2] = send_yaw & 0xFF;
        tx_buffer[3] = (send_yaw >> 8) & 0xFF;
        tx_buffer[4] = 0x01; // Flag = 0x01 (正常跟踪)

        if (debug_level >= 1) {
            RCLCPP_INFO(get_logger(), "Targeted! Yaw: %.2f°", target_yaw);
        }
    } 
    else 
    {
        lost_count_++;

        if (has_locked_before_ && lost_count_ <= 5) 
        {
            // 【状态 2：容忍期】 -> 维持上一帧的有效数据
            tx_buffer[2] = last_valid_yaw_ & 0xFF;
            tx_buffer[3] = (last_valid_yaw_ >> 8) & 0xFF;
            tx_buffer[4] = 0x01; 

            if (debug_level >= 1) {
                RCLCPP_WARN(get_logger(), "Lost! (%d/5)", lost_count_);
            }
        } 
        else 
        {
            // 【状态 3：彻底脱靶 / 刚开机未找到】 -> 下发全零急刹车
            tx_buffer[2] = 0x00;
            tx_buffer[3] = 0x00;
            tx_buffer[4] = 0x00; // Flag = 0x00 (刹车标志)

            has_locked_before_ = false;

            if (debug_level >= 1) {
                RCLCPP_ERROR(get_logger(), "No Target");
            }
        }
    }

    // ================= [ 5. 计算校验和 (Checksum) ] =================
    uint8_t checksum = 0;
    for (int i = 0; i < 7; i++) {
        checksum += tx_buffer[i];
    }
    tx_buffer[7] = checksum;

    // ================= [ 6. 执行串口发送 ] =================
    try {
        if (serial_.isOpen()) {
            serial_.write(tx_buffer, 8);
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "串口写入异常: %s", e.what());
    }
}

} // namespace uart_serial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uart_serial::UartSerialNode)