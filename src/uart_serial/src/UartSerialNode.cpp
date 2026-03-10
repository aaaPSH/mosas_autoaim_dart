#include "uart_serial/UartSerialNode.hpp"
#include <cstring>

namespace uart_serial
{

bool UartSerialNode::send_command(double speed, bool fire) {
    uint8_t tx_buffer[6];
    std::memset(tx_buffer, 0, sizeof(tx_buffer));

    tx_buffer[0] = 0x55;
    tx_buffer[1] = 0xAA;

    int16_t scaled_speed = static_cast<int16_t>(speed * SCALE); // 注意确保 SCALE 已定义
    tx_buffer[2] = scaled_speed & 0xFF;
    tx_buffer[3] = (scaled_speed >> 8) & 0xFF;
    tx_buffer[4] = fire ? 0x01 : 0x00;

    // 【修复 Bug】：只对前 5 个字节 (0~4) 求和
    uint8_t checksum = 0;
    for (int i = 0; i < 5; i++) { 
        checksum += tx_buffer[i];
    }
    tx_buffer[5] = checksum;

    if (serial_.isOpen()) {
        serial_.write(tx_buffer, sizeof(tx_buffer));
        return true;
    } else {
        return false;
    }
}

UartSerialNode::UartSerialNode(const rclcpp::NodeOptions & options)
: Node("uart_serial_node", options)
{
    // ================= [ 1. 声明并读取参数 ] =================
    this->declare_parameter("port_name", "/dev/ttyUSB0");
    this->declare_parameter("baud_rate", 115200);
    this->declare_parameter("debug_level", 1);

    this->declare_parameter("Kp", 0.01);
    this->declare_parameter("Ki", 0.0);
    this->declare_parameter("Kd", 0.0);
    this->declare_parameter("max_speed", 100.0);
    this->declare_parameter("deadzone", 1.0);

    std::string port_name = this->get_parameter("port_name").as_string();
    int baud_rate = this->get_parameter("baud_rate").as_int();

    pid_params_.Kp = this->get_parameter("Kp").as_double();
    pid_params_.Ki = this->get_parameter("Ki").as_double();
    pid_params_.Kd = this->get_parameter("Kd").as_double();
    pid_params_.max_speed = this->get_parameter("max_speed").as_double();
    pid_params_.deadzone = this->get_parameter("deadzone").as_double();

    // 统一初始化您的 PID 对象
    my_pid_.set_params(pid_params_.Kp, pid_params_.Ki, pid_params_.Kd, pid_params_.deadzone, pid_params_.max_speed);

    // ================= [ 2. 注册动态参数回调 ] =================
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&UartSerialNode::parameters_callback, this, std::placeholders::_1));

    // ================= [ 3. 初始化串口 ] =================
    // ... (保留您原本的串口初始化逻辑不变) ...
    try {
        serial_.setPort(port_name);
        serial_.setBaudrate(baud_rate);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        serial_.setTimeout(to);
        serial_.open();
    } catch (serial::IOException & e) {
        RCLCPP_FATAL(this->get_logger(), "串口开启失败！请检查 %s", port_name.c_str());
        throw; 
    }

    if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "串口初始化成功!");
    }

    // ================= [ 4. 初始化订阅者 ] =================
    green_dots_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
        "/detections/green_dots", rclcpp::SensorDataQoS(),
        std::bind(&UartSerialNode::green_dots_callback, this, std::placeholders::_1));
}

UartSerialNode::~UartSerialNode()
{
    if (serial_.isOpen()) {
        serial_.close();
    }
}

// ==========================================================
// 【核心新增】动态参数回调处理函数
// ==========================================================
rcl_interfaces::msg::SetParametersResult UartSerialNode::parameters_callback(
    const std::vector<rclcpp::Parameter> &parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "Parameters updated successfully";

    for (const auto &param : parameters) {
        if (param.get_name() == "Kp") {
            pid_params_.Kp = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Kp updated to: %.4f", pid_params_.Kp);
        } else if (param.get_name() == "Ki") {
            pid_params_.Ki = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Ki updated to: %.4f", pid_params_.Ki);
        } else if (param.get_name() == "Kd") {
            pid_params_.Kd = param.as_double();
            RCLCPP_INFO(this->get_logger(), "Kd updated to: %.4f", pid_params_.Kd);
        } else if (param.get_name() == "max_speed") {
            pid_params_.max_speed = param.as_double();
            RCLCPP_INFO(this->get_logger(), "max_speed updated to: %.1f", pid_params_.max_speed);
        } else if (param.get_name() == "deadzone") {
            pid_params_.deadzone = param.as_double();
            RCLCPP_INFO(this->get_logger(), "deadzone updated to: %.2f", pid_params_.deadzone);
        }
    }

    // 将新参数推送到 PID 控制器中
    my_pid_.set_params(pid_params_.Kp, pid_params_.Ki, pid_params_.Kd, pid_params_.deadzone, pid_params_.max_speed);

    return result;
}

void UartSerialNode::green_dots_callback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
    rclcpp::Time current_time = msg->header.stamp;
    double current_x = msg->x;
    double current_error = msg->d_pixel;

    // 0. 首次运行初始化时间戳
    if (first_run_) {
        last_time_ = current_time;
        first_run_ = false;
        return;
    }

    // ==========================================
    // 1. [绝对防御层] 丢帧拦截与惯性滑行机制
    // ==========================================
    if (current_x <= 0.0) { // 假设 x<=0 代表视觉丢失
        lost_frames_count_++;
        if (lost_frames_count_ <= MAX_LOST_TOLERANCE) {
            // 短暂瞎眼：保持惯性，不更新时间，不进 PID
            send_command(last_valid_speed_, false); 
        } else {
            // 彻底丢失：急刹车，清零状态
            send_command(0, false);
            my_pid_.reset();
            current_state_ = AimState::TRACKING; // 随时准备重新抓取
        }
        return; // 拦截！不往下走，last_time_ 停留在上一次成功时！
    }

    // ==========================================
    // 2. [时间差计算] 自动吸收丢帧时间
    // ==========================================
    lost_frames_count_ = 0; // 能走到这说明看到了目标，计数器清零
    double dt = (current_time - last_time_).seconds();
    
    // 防御机制：如果断连太久（比如被挡了 1 秒），重置 dt 防止求导爆炸
    if (dt > 0.5) { 
        dt = 0.014; 
        my_pid_.reset(); 
    }
    // 更新记忆：只有有效帧才配成为“过去”
    last_time_ = current_time; 

    // ==========================================
    // 3. [大脑核心] 动静分离状态机
    // ==========================================
    switch (current_state_) {
        
        case AimState::TRACKING: {
            // 【追击模式】：吃单帧，要的是 0 延迟！
            double target_speed = my_pid_.compute(current_error, dt);

            if (target_speed == 0.0) {
                // PID 算出来 0，说明单帧掉进粗略死区了。电机刹车！
                send_command(0, false);
                current_state_ = AimState::VERIFYING; // 切换到核实模式
                history_x_.clear();
                // RCLCPP_INFO(this->get_logger(), "进入死区，刹车并开始多帧核实...");
            } else {
                send_command(target_speed, false);
                last_valid_speed_ = target_speed; // 记住当前速度，给丢帧滑行用
            }
            break;
        }

        case AimState::VERIFYING: {
            // 【核实模式】：电机已死锁，收集纯净的静止帧
            history_x_.push_back(current_x);
            
            if (history_x_.size() >= VERIFY_FRAMES) {
                // 攒够了，算平均值抗空气扰动
                double sum = 0;
                for (double x : history_x_) sum += x;
                double avg_x = sum / VERIFY_FRAMES;
                
                if (std::abs(avg_x) <= pid_params_.deadzone) { 
                    // 真的对准了！
                    current_state_ = AimState::LOCKED;
                    RCLCPP_INFO(this->get_logger(), "完美锁定！真理均值坐标: %.2f", avg_x); 
                } else {
                    // 被假噪点骗了，打回原形重新微调
                    current_state_ = AimState::TRACKING;
                    history_x_.clear();
                    my_pid_.reset(); // 必须清空 PID 历史积分！防止带着旧力量启动
                    RCLCPP_WARN(this->get_logger(), "假锁定(均值%.2f)，打回 TRACKING 继续微调", avg_x);
                }
            }
            break;
        }

        case AimState::LOCKED: {
            // 【锁定模式】：死死咬住，持续发 0 保持丝杠自锁
            send_command(0, true);
            
            // 破局机制：如果在锁定状态下，有人撞了机器，或者目标真动了
            // 容忍度设大一点（比如 3.0），防止微小底噪频繁打破锁定
            if (std::abs(current_error) > 3.0) {
                current_state_ = AimState::TRACKING;
                my_pid_.reset();
                RCLCPP_WARN(this->get_logger(), "受外力干扰脱锁，重新追击！");
            }
            break;
        }
    }
}

} // namespace uart_serial

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uart_serial::UartSerialNode)