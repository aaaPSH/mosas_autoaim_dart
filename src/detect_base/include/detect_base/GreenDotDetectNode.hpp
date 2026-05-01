#include <chrono>
#include <limits>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "cv_bridge/cv_bridge.h"
#include "detect_base/Dot.hpp"
#include "detect_base/GreenDotDetect.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <filesystem>

class GreenDotDetectNode : public rclcpp::Node
{
private:
  // 订阅者 & 算法对象
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  rclcpp::Publisher<autoaim_interfaces::msg::GreenDot>::SharedPtr target_pub;
  std::shared_ptr<GreenDotDetect> detector_;
  OnSetParametersCallbackHandle::SharedPtr callback_handle_;

  // 静态相机参数 (只读一次)
  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // FPS 统计
  int fps_counter_ = 0;
  std::chrono::steady_clock::time_point last_fps_time_;
  //储存计时器
  std::chrono::steady_clock::time_point last_save_time_;

  // 保存帧率（可通过yaml配置）
  double save_fps_ = 60.0;
  std::chrono::milliseconds save_interval_{1000 / 60};

  // 保存路径
  std::string save_path_ = "/home/nvidia/saved_green_dots";

  // 调试开关
  bool debug_mode_ = false;
  bool save_images = false;

  // 异步保存线程
  std::thread save_thread_;
  std::mutex save_mutex_;
  std::condition_variable save_cv_;
  std::queue<cv::Mat> save_queue_;
  std::atomic<bool> stop_save_thread_{false};


  DetectParams p;

public:
  GreenDotDetectNode(const rclcpp::NodeOptions & options) : Node("green_dot_detect_node", options)
  {
    last_fps_time_ = std::chrono::steady_clock::now();
    last_save_time_ = std::chrono::steady_clock::now();

    detector_ = std::make_shared<GreenDotDetect>();

    // ==========================================
    // 1. 声明并初始化相机内参 (Static)
    // ==========================================
    std::vector<double> default_k = {1500.0, 0.0, 640.0, 0.0, 1500.0, 360.0, 0.0, 0.0, 1.0};
    std::vector<double> default_d = {0.0, 0.0, 0.0, 0.0, 0.0};

    this->declare_parameter("camera.matrix", default_k);
    this->declare_parameter("camera.dist_coeffs", default_d);

    std::vector<double> k_data, d_data;
    this->get_parameter("camera.matrix", k_data);
    this->get_parameter("camera.dist_coeffs", d_data);

    camera_matrix_ = cv::Mat(3, 3, CV_64F, k_data.data()).clone();
    dist_coeffs_ = cv::Mat(d_data.size(), 1, CV_64F, d_data.data()).clone();

    // 初始化算法的相机部分
    detector_->init_camera(camera_matrix_, dist_coeffs_);
    RCLCPP_INFO(this->get_logger(), "Camera parameters loaded.");

    // ==========================================
    // 2. 声明动态参数 (对应 DetectParams)
    // ==========================================
    this->declare_parameter("debug_mode", true);
    this->declare_parameter("save_images", false);
    this->declare_parameter("save_fps", 60.0);
    this->declare_parameter("save_path", save_path_);

    // 阈值与形态
    this->declare_parameter("detect.v_low", 100);
    this->declare_parameter("detect.min_area", 2.0);
    this->declare_parameter("detect.max_area", 200.0);
    this->declare_parameter("detect.min_aspect_ratio", 0.4);
    this->declare_parameter("detect.max_aspect_ratio", 2.5);
    this->declare_parameter("detect.min_circularity", 0.5);

    // 抗干扰
    this->declare_parameter("detect.min_gr_ratio", 2.3);
    this->declare_parameter("detect.min_gb_ratio", 0.8);
    this->declare_parameter("detect.search_strip_min_h", 20);

    // 物理场景
    this->declare_parameter("detect.camera_height", 0.0);  // mm
    this->declare_parameter("detect.target_height", 0.0);  // mm
    this->declare_parameter("detect.detect_scale", 10.0);  // deg
    this->declare_parameter("detect.distance", 25000.0);   // mm

    this->declare_parameter("detect.calibrated_pixel_x", 0.0);

    // 首次同步参数
    refresh_params();

    // ==========================================
    // 3. 注册回调
    // ==========================================
    callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GreenDotDetectNode::parametersCallback, this, std::placeholders::_1));

    // 订阅 Raw 图像
    sub_image_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&GreenDotDetectNode::imageCallback, this, std::placeholders::_1));

    target_pub = this->create_publisher<autoaim_interfaces::msg::GreenDot>(
      "/detections/green_dots", rclcpp::SensorDataQoS().keep_last(1));

    RCLCPP_INFO(this->get_logger(), "Node initialized. Waiting for images...");

    // 启动异步保存线程
    save_thread_ = std::thread(&GreenDotDetectNode::saveThreadLoop, this);
  }

  ~GreenDotDetectNode() override
  {
    stop_save_thread_ = true;
    save_cv_.notify_one();
    if (save_thread_.joinable()) {
      save_thread_.join();
    }
  }

private:
  void save_image_to_disk(const cv::Mat & image)
  {
      // 1. 按节点启动时间创建子目录（只在第一次创建）
      std::string base_path = save_path_;
      static std::string save_dir = [base_path]() {
          auto now = std::chrono::system_clock::now();
          auto time_t = std::chrono::system_clock::to_time_t(now);
          std::stringstream ss;
          ss << base_path << "/"
             << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
          std::string dir = ss.str();
          std::filesystem::create_directories(dir);
          return dir;
      }();

      auto now = std::chrono::system_clock::now();
      auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
      std::time_t t = std::chrono::system_clock::to_time_t(now);

      std::stringstream ss;
      ss << save_dir << "/DOT_"
        << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S") << "_"
        << std::setfill('0') << std::setw(3) << ms.count() << ".jpg";

      // 2. 写入并检查返回值 (image 已由调用方 clone，无需再次拷贝)
      bool success = cv::imwrite(ss.str(), image);
      if (!success) {
          RCLCPP_ERROR(this->get_logger(), "Failed to save image to: %s", ss.str().c_str());
      }
  }
  void saveThreadLoop()
  {
    while (!stop_save_thread_) {
      cv::Mat frame;
      {
        std::unique_lock<std::mutex> lock(save_mutex_);
        save_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
          return !save_queue_.empty() || stop_save_thread_;
        });
        if (stop_save_thread_ && save_queue_.empty()) break;
        if (save_queue_.empty()) continue;
        frame = std::move(save_queue_.front());
        save_queue_.pop();
      }
      save_image_to_disk(frame);
    }
  }

  // -----------------------------------------------------------------
  // 从 ROS 参数服务器读取最新值 -> 打包进 DetectParams -> 传给算法
  // -----------------------------------------------------------------
  void refresh_params()
  {

    // 读取基础控制
    this->get_parameter("debug_mode", debug_mode_);
    this->get_parameter("save_images", save_images);
    this->get_parameter("save_fps", save_fps_);
    this->get_parameter("save_path", save_path_);
    save_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / save_fps_));

    // 读取算法参数
    this->get_parameter("detect.v_low", p.v_low);
    this->get_parameter("detect.min_area", p.min_area);
    this->get_parameter("detect.max_area", p.max_area);
    this->get_parameter("detect.min_aspect_ratio", p.min_aspect_ratio);
    this->get_parameter("detect.max_aspect_ratio", p.max_aspect_ratio);
    this->get_parameter("detect.min_circularity", p.min_circularity);

    this->get_parameter("detect.min_gr_ratio", p.min_gr_ratio);
    this->get_parameter("detect.min_gb_ratio", p.min_gb_ratio);
    this->get_parameter("detect.search_strip_min_h", p.search_strip_min_h);

    this->get_parameter("detect.camera_height", p.camera_height);
    this->get_parameter("detect.target_height", p.target_height);
    this->get_parameter("detect.detect_scale", p.detect_scale);
    this->get_parameter("detect.distance", p.distance);

    this->get_parameter("detect.calibrated_pixel_x", p.calibrated_pixel_x);

    // 更新算法内部状态
    detector_->update_params(p);

    RCLCPP_INFO(
      this->get_logger(),
      "[Params Updated] Debug:%d | V_Low:%d | Area:%.1f-%.1f | Dist:%.0f | CalPixelX:%.1f",
      debug_mode_, p.v_low, p.min_area, p.max_area, p.distance, p.calibrated_pixel_x);
  }

  // -----------------------------------------------------------------
  // 动态参数回调
  // -----------------------------------------------------------------
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "success";

    bool algo_params_changed = false;

    for (const auto & param : parameters) {
      const std::string & name = param.get_name();

      // 1. 基础控制
      if (name == "debug_mode") {
        debug_mode_ = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Debug Mode Set: %d", debug_mode_);
      } else if (name == "save_images") {
        save_images = param.as_bool();
        RCLCPP_INFO(this->get_logger(), "Save Images Set: %d", save_images);
      } else if (name == "save_fps") {
        save_fps_ = param.as_double();
        save_interval_ = std::chrono::milliseconds(static_cast<int>(1000.0 / save_fps_));
        RCLCPP_INFO(this->get_logger(), "Save FPS Set: %.1f", save_fps_);
      } else if (name == "save_path") {
        save_path_ = param.as_string();
        RCLCPP_INFO(this->get_logger(), "Save Path Set: %s", save_path_.c_str());
      }
      // 2. 算法参数匹配 
      else if (name == "detect.v_low") {
        p.v_low = param.as_int();
        algo_params_changed = true;
      } else if (name == "detect.min_area") {
        p.min_area = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.max_area") {
        p.max_area = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_aspect_ratio") {
        p.min_aspect_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.max_aspect_ratio") {
        p.max_aspect_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_circularity") {
        p.min_circularity = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_gr_ratio") {
        p.min_gr_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.min_gb_ratio") {
        p.min_gb_ratio = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.search_strip_min_h") {
        p.search_strip_min_h = param.as_int();
        algo_params_changed = true;
      } else if (name == "detect.distance") {
        p.distance = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.calibrated_pixel_x") {
        p.calibrated_pixel_x = param.as_double();
        algo_params_changed = true;
      } else if (name == "detect.detect_scale") {
        p.detect_scale = param.as_double();
        algo_params_changed = true;
      }
    }

    // 只有当算法相关的参数变化时，才调用 update_params
    if (algo_params_changed) {
      detector_->update_params(p);
      RCLCPP_INFO(
        this->get_logger(), "[Dynamic] Algorithm Params Updated! V_Low: %d, Area: %.1f",
        p.v_low, p.min_area);
    }

    return result;
  }

  // -----------------------------------------------------------------
  // 图像回调
  // -----------------------------------------------------------------
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    cv::Mat raw_frame;
    try {
      if (
        msg->encoding.find("bayer") != std::string::npos ||
        msg->encoding == sensor_msgs::image_encodings::MONO8) {
        auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        raw_frame = cv_ptr->image;
      } else {
        auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8");
        cv::Mat bgr = cv_ptr->image;
        std::vector<cv::Mat> chs;
        cv::split(bgr, chs);
        raw_frame = chs[1];  // Green Channel
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    // 3. 执行检测
    std::vector<Dot> dots;
    bool found = detector_->detect(raw_frame, dots, debug_mode_);

    // 4. 解算与输出
    if (found) {
      detector_->calculate_dots_yaw(dots);

      // 假设这是在 GreenDotDetect::detect 的末尾，此时您已经有了 std::vector<Dot> dots
      if (dots.size() > 1) {
        // 1. 获取相机内参和物理配置参数
        double fy = camera_matrix_.at<double>(1, 1);
        double cy = camera_matrix_.at<double>(1, 2);

        double delta_H = p.target_height - p.camera_height;
        double Z = p.distance;

        // 2. 计算理论上绿点应该出现的 Y 像素坐标
        // (如果 target 比 camera 高，图像上 Y 应该小于 cy，所以用减法)
        double expected_y_pixel = cy - fy * (delta_H / Z);

        // 3. 遍历所有的候选绿点，找到 Y 坐标最接近理论值的那个
        double min_y_diff = std::numeric_limits<double>::max();
        Dot best_dot;

        for (const auto & dot : dots) {
          // dot.center.y 是您前面代码算出并映射回原图的绝对像素坐标
          double y_diff = std::abs(dot.center.y - expected_y_pixel);

          if (y_diff < min_y_diff) {
            min_y_diff = y_diff;
            best_dot = dot;
          }
        }

        // 4. 清空原来的 dots 数组，只保留最完美匹配的那一个！
        dots.clear();
        dots.push_back(best_dot);
      }

      // 输出所有检测到的点
      autoaim_interfaces::msg::GreenDot target;
      auto & d = dots[0];
      // RCLCPP_INFO(this->get_logger(),
      //     "Target Pos:(%.2f, %.2f) Yaw:%.3f deg",
      //     d.center.x, d.center.y, d.yaw);

      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = d.center.x;
      target.y = d.center.y;
      target.angle_yaw = d.yaw;
      target.d_pixel = target.x - p.calibrated_pixel_x;  // 使用校准的像素点x坐标计算像素差

      target_pub->publish(target);
    } else {
      // RCLCPP_INFO(this->get_logger(),"No Target!");
      autoaim_interfaces::msg::GreenDot target;
      target.header.stamp = msg->header.stamp;
      target.header.frame_id = msg->header.frame_id;
      target.x = -1;
      target.y = -1;

      target_pub->publish(target);
    }

    auto now = std::chrono::steady_clock::now();

    if (debug_mode_) {
      fps_counter_++;
      auto fps_diff =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_fps_time_).count();
      if (fps_diff >= 1000) {
        RCLCPP_INFO(this->get_logger(), "FPS: %d", fps_counter_);
        fps_counter_ = 0;
        last_fps_time_ = now;
      }
    }

    auto save_diff =
      std::chrono::duration_cast<std::chrono::milliseconds>(now - last_save_time_).count();
    if (save_images && save_diff >= save_interval_.count()) {
      {
        std::lock_guard<std::mutex> lock(save_mutex_);
        if (save_queue_.size() >= 30) save_queue_.pop();
        save_queue_.push(raw_frame.clone());
      }
      save_cv_.notify_one();
      last_save_time_ = now;
    }
  }
};
