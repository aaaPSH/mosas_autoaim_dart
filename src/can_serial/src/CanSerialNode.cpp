#include "can_serial/CanSerialNode.hpp"

#include <cstring>

namespace can_serial
{
CanSerialNode::CanSerialNode(const rclcpp::NodeOptions & options)
: Node("can_serial_node", options)
{
  std::memset(&frame_, 0, sizeof(frame_)); // 初始化CAN帧数据为0
  frame_.can_id = CAN_TX_ID;
  frame_.can_dlc = CAN_FRAME_DLC;


  // ================= [ 初始化can ] =================
  // 初始化CAN驱动
  can_core_ = std::make_unique<CanSerial>("can0");
  can_initialized_ = false;
  try {
    can_core_->init();
    can_core_->set_frame_callback(
      std::bind(&CanSerialNode::handle_can_frame, this, std::placeholders::_1));
    can_core_->async_read();
    // 启动Boost.Asio事件循环线程
    can_core_->start_io_service();
    can_initialized_ = true;
    std::cout << "Boost.Asio线程已启动" << std::endl;
  } catch (const std::exception & e) {
    RCLCPP_FATAL(get_logger(), "CAN初始化失败: %s", e.what());
    throw;
  }

  // ================= [ CAN状态发布 ] =================
  can_status_pub_ = this->create_publisher<std_msgs::msg::String>("/can_status", 10);
  last_rx_time_ = std::chrono::steady_clock::now();
  can_health_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(1000),
    std::bind(&CanSerialNode::checkCanHealth, this));

  // ================= [ 初始化订阅者 ] =================
  green_dots_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
    "/detections/green_dots", rclcpp::SensorDataQoS(),
    std::bind(&CanSerialNode::green_dots_callback, this, std::placeholders::_1));

}
void CanSerialNode::parse_received_data()
{
  
}

void CanSerialNode::handle_can_frame(const can_frame & frame){

  // 更新CAN接收时间
  last_rx_time_ = std::chrono::steady_clock::now();

  // RCLCPP_INFO(this->get_logger(),"已经收到消息");
  // RCLCPP_INFO(get_logger(), "收到CAN帧 - ID: 0x%x, 长度: %d", frame.can_id, frame.can_dlc);
  //   // 仅处理ID为0xA0的帧
  if (frame.can_id == CAN_RX_ID && frame.can_dlc >= CAN_MIN_RX_DLC)
  {
    //根据传输帧改变发送标志位
    send_flag_ = true;
  }
  //     RCLCPP_INFO(this->get_logger(), "CAN Received -> %s", ss.str().c_str());
  //   }
}

void CanSerialNode::checkCanHealth()
{
  auto msg = std_msgs::msg::String();
  
  if (!can_initialized_) {
    msg.data = "OFFLINE";
    can_status_pub_->publish(msg);
    return;
  }
  
  // 检查是否超过2秒没有收到CAN帧
  auto now = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now - last_rx_time_).count();
  
  if (elapsed_ms > 2000) {
    msg.data = "OFFLINE";
  } else {
    msg.data = "ONLINE";
  }
  
  can_status_pub_->publish(msg);
}

void CanSerialNode::send_command(const double& d_pixel)
{
  // 将像素差转换为整数（使用SCALE=100）
  const double SCALE = 100.0;
  int16_t pixel_int = static_cast<int16_t>(d_pixel * SCALE);
  
  // 按照大端字节序赋值到frame.data[0]和data[1]
  // data[0]存储高位字节，data[1]存储低位字节
  frame_.data[0] = (pixel_int >> 8) & 0xFF; // 高位字节
  frame_.data[1] = pixel_int & 0xFF;        // 低位字节
  
  // 其他数据位清零
  for (int i = 2; i < 8; i++) {
    frame_.data[i] = 0x00;
  }
  
  try {
    can_core_->send_frame(frame_);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "CAN 发送异常: %s", e.what());
  }
}


void CanSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
  this->current_error = msg->d_pixel;
  // RCLCPP_INFO(this->get_logger(), "发送像素差: %.2f", current_error);
  if(send_flag_){
    send_command(current_error);
    send_flag_ = false;
  }
}

} 

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(can_serial::CanSerialNode)
