#include "save_frame/SaveFrameNode.hpp"

#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

#include "cv_bridge/cv_bridge.h"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include <rosbag2_compression_zstd/zstd_compressor.hpp>
#include "rosbag2_compression/compression_options.hpp"
#include "sensor_msgs/msg/camera_info.hpp"

namespace save_frame
{

SaveFrameNode::SaveFrameNode(const rclcpp::NodeOptions & options)
: Node("save_frame_node", options),
  last_stat_time_(std::chrono::steady_clock::now())
{
  RCLCPP_INFO(this->get_logger(), "Initializing SaveFrameNode...");

  // 初始化参数
  initParameters();

  // 初始化订阅器
  initSubscribers();

  // 初始化录制
  initRecording();

  RCLCPP_INFO(this->get_logger(), "SaveFrameNode initialized successfully");
  RCLCPP_INFO(this->get_logger(), "Save path: %s", save_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "Image topic: %s", image_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Detection topic: %s", detection_topic_.c_str());
  RCLCPP_INFO(this->get_logger(), "Match status topic: %s", match_status_topic_.c_str());
}

SaveFrameNode::~SaveFrameNode()
{
  stop_thread_ = true;
  buffer_cv_.notify_all();

  if (recording_thread_.joinable()) {
    recording_thread_.join();
  }

  if (is_recording_) {
    stopRecording();
  }

  RCLCPP_INFO(this->get_logger(), "SaveFrameNode destroyed");
}

void SaveFrameNode::initParameters()
{
  // 基本配置
  save_path_ = this->declare_parameter("save_path", "./recordings");
  image_topic_ = this->declare_parameter("image_topic", "/save_frame");
  detection_topic_ = this->declare_parameter("detection_topic", "/detections/green_dots");
  match_status_topic_ = this->declare_parameter("match_status_topic", "/match_status");
  
  record_images_ = this->declare_parameter("record_images", true);
  record_detections_ = this->declare_parameter("record_detections", true);
  auto_start_ = this->declare_parameter("auto_start", false);
  
  // 性能配置
  buffer_size_ = this->declare_parameter("buffer_size", 1000);
  batch_size_ = this->declare_parameter("batch_size", 50);
  use_compression_ = this->declare_parameter("use_compression", true);
  
  // 间隔取帧配置
  frame_interval_ = this->declare_parameter("frame_interval", 1);  // 每N帧保存一帧，1=每帧都保存
  time_interval_ms_ = this->declare_parameter("time_interval_ms", static_cast<int64_t>(0));  // 时间间隔（毫秒），0=不按时间间隔
  
  // 存储管理
  max_file_size_mb_ = this->declare_parameter("max_file_size_mb", 1024);  // 1GB
  max_total_size_gb_ = this->declare_parameter("max_total_size_gb", 50);   // 50GB
  disk_space_threshold_gb_ = this->declare_parameter("disk_space_threshold_gb", 5);  // 5GB
  
  // 图像处理
  downsample_ratio_ = this->declare_parameter("downsample_ratio", 1.0);
  image_quality_ = this->declare_parameter("image_quality", 95);
  
  // 相机内参将从/camera_info话题获取，这里只声明默认值（用于初始化）
  camera_matrix_ = {1500.0, 0.0, 640.0, 0.0, 1500.0, 360.0, 0.0, 0.0, 1.0};
  distortion_coeffs_ = {0.0, 0.0, 0.0, 0.0, 0.0};
  distortion_model_ = "plumb_bob";
  
  // 初始化间隔取帧状态
  frame_counter_ = 0;
  last_saved_time_ = std::chrono::steady_clock::now();
  
  // 创建保存目录
  std::filesystem::create_directories(save_path_);
}

void SaveFrameNode::initSubscribers()
{
  // 订阅相机信息话题
  camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info", rclcpp::SensorDataQoS(),
    std::bind(&SaveFrameNode::cameraInfoCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to camera info topic: /camera_info");
  
  if (record_images_) {
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SaveFrameNode::imageCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to image topic: %s", image_topic_.c_str());
  }
  
  if (record_detections_) {
    detection_sub_ = this->create_subscription<autoaim_interfaces::msg::GreenDot>(
      detection_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SaveFrameNode::detectionCallback, this, std::placeholders::_1));
    RCLCPP_INFO(this->get_logger(), "Subscribed to detection topic: %s", detection_topic_.c_str());
  }
  
  // 比赛状态订阅（使用std_msgs::msg::String作为通用接口）
  match_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    match_status_topic_, rclcpp::SensorDataQoS(),
    std::bind(&SaveFrameNode::matchStatusCallback, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Subscribed to match status topic: %s", match_status_topic_.c_str());
}

void SaveFrameNode::initRecording()
{
  // 启动录制线程
  recording_thread_ = std::thread(&SaveFrameNode::recordingThread, this);
  
  // 如果配置为自动开始，则开始录制
  if (auto_start_) {
    startRecording();
  }
}

void SaveFrameNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  frames_received_++;
  
  if (!is_recording_ || !record_images_) {
    return;
  }
  
  // 检查磁盘空间
  uint64_t available_space = getAvailableDiskSpace(save_path_);
  if (available_space < disk_space_threshold_gb_ * 1024ULL * 1024ULL * 1024ULL) {
    RCLCPP_WARN(this->get_logger(), "Low disk space: %lu MB available. Stopping recording.",
                available_space / (1024 * 1024));
    stopRecording();
    return;
  }
  
  // 间隔取帧逻辑
  bool should_save = true;
  
  // 基于帧间隔的过滤
  if (frame_interval_ > 1) {
    frame_counter_++;
    if (frame_counter_ % frame_interval_ != 0) {
      should_save = false;
    }
  }
  
  // 基于时间间隔的过滤（如果同时启用，取更严格的限制）
  if (time_interval_ms_ > 0) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
      now - last_saved_time_).count();
    
    if (elapsed_ms < time_interval_ms_) {
      should_save = false;
    } else {
      last_saved_time_ = now;
    }
  }
  
  // 如果不满足保存条件，直接返回
  if (!should_save) {
    // 即使不保存，也更新性能统计
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time_).count();
    if (elapsed >= 5) {
      double fps = static_cast<double>(frames_received_) / elapsed;
      RCLCPP_INFO(this->get_logger(), "Image FPS: %.1f, Buffer size: %zu (filtered)", fps, message_buffer_.size());
      frames_received_ = 0;
      last_stat_time_ = now;
    }
    return;
  }
  
  // 序列化消息
  rclcpp::Serialization<sensor_msgs::msg::Image> serialization;
  rclcpp::SerializedMessage serialized_msg;
  serialization.serialize_message(msg.get(), &serialized_msg);
  
  // 添加到缓冲区
  addToBuffer(image_topic_, serialized_msg);
  
  // 性能统计
  auto now = std::chrono::steady_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - last_stat_time_).count();
  if (elapsed >= 5) {
    double fps = static_cast<double>(frames_received_) / elapsed;
    RCLCPP_INFO(this->get_logger(), "Image FPS: %.1f, Buffer size: %zu", fps, message_buffer_.size());
    frames_received_ = 0;
    last_stat_time_ = now;
  }
}

void SaveFrameNode::detectionCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg)
{
  if (!is_recording_ || !record_detections_) {
    return;
  }
  
  // 序列化消息
  rclcpp::Serialization<autoaim_interfaces::msg::GreenDot> serialization;
  rclcpp::SerializedMessage serialized_msg;
  serialization.serialize_message(msg.get(), &serialized_msg);
  
  // 添加到缓冲区
  addToBuffer(detection_topic_, serialized_msg);
}

void SaveFrameNode::matchStatusCallback(const std_msgs::msg::String::SharedPtr msg)
{
  std::string status = msg->data;
  RCLCPP_INFO(this->get_logger(), "Received match status: %s", status.c_str());
  
  // 简单的状态解析：如果消息包含"start"，则开始录制；包含"stop"，则停止录制
  if (status.find("start") != std::string::npos || status.find("START") != std::string::npos) {
    if (!is_recording_) {
      startRecording();
      // 尝试从消息中提取比赛ID
      size_t id_start = status.find("id:");
      if (id_start != std::string::npos) {
        current_match_id_ = status.substr(id_start + 3);
      }
    }
  } else if (status.find("stop") != std::string::npos || status.find("STOP") != std::string::npos) {
    if (is_recording_) {
      stopRecording();
    }
  }
}

void SaveFrameNode::addToBuffer(const std::string & topic_name, const rclcpp::SerializedMessage & msg)
{
  std::lock_guard<std::mutex> lock(buffer_mutex_);
  
  // 如果缓冲区已满，丢弃最旧的消息
  if (message_buffer_.size() >= buffer_size_) {
    message_buffer_.pop_front();
    messages_dropped_++;
    
    if (messages_dropped_ % 100 == 0) {
      RCLCPP_WARN(this->get_logger(), "Buffer full, dropped %zu messages", messages_dropped_);
    }
  }
  
  // 添加新消息到缓冲区
  BufferedMessage buffered_msg;
  buffered_msg.timestamp = this->now();
  buffered_msg.topic_name = topic_name;
  buffered_msg.serialized_msg = std::make_shared<rclcpp::SerializedMessage>(msg);
  
  message_buffer_.push_back(buffered_msg);
  
  // 通知录制线程
  buffer_cv_.notify_one();
}

void SaveFrameNode::recordingThread()
{
  while (!stop_thread_) {
    std::unique_lock<std::mutex> lock(buffer_mutex_);
    
    // 等待缓冲区有数据或停止信号
    buffer_cv_.wait_for(lock, std::chrono::milliseconds(100), [this]() {
      return !message_buffer_.empty() || stop_thread_;
    });
    
    if (stop_thread_) {
      break;
    }
    
    if (!message_buffer_.empty() && is_recording_) {
      // 处理一批消息
      processBufferBatch();
    }
    
    lock.unlock();
    
    // 检查是否需要创建新文件
    if (is_recording_ && current_file_size_ > max_file_size_mb_ * 1024ULL * 1024ULL) {
      createNewBagFile();
    }
  }
}

void SaveFrameNode::processBufferBatch()
{
  if (!bag_writer_) {
    return;
  }
  
  size_t processed = 0;
  std::vector<BufferedMessage> batch;
  
  // 获取一批消息
  {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    size_t count = std::min(batch_size_, message_buffer_.size());
    batch.reserve(count);
    
    for (size_t i = 0; i < count; ++i) {
      batch.push_back(std::move(message_buffer_.front()));
      message_buffer_.pop_front();
    }
  }
  
  // 写入消息
  for (auto & msg : batch) {
    try {
      rosbag2_storage::SerializedBagMessage serialized_bag_msg;
      serialized_bag_msg.serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        const_cast<rcutils_uint8_array_t*>(&msg.serialized_msg->get_rcl_serialized_message()),
        [](rcutils_uint8_array_t* /* data */) {});
      
      serialized_bag_msg.topic_name = msg.topic_name;
      serialized_bag_msg.time_stamp = msg.timestamp.nanoseconds();
      
      bag_writer_->write(std::make_shared<rosbag2_storage::SerializedBagMessage>(serialized_bag_msg));
      
      // 更新文件大小估计（粗略估计）
      current_file_size_ += msg.serialized_msg->size();
      total_recorded_size_ += msg.serialized_msg->size();
      frames_recorded_++;
      
      processed++;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to write message: %s", e.what());
    }
  }
  
  if (processed > 0) {
    // 定期清理旧文件
    if (total_recorded_size_ > max_total_size_gb_ * 1024ULL * 1024ULL * 1024ULL) {
      cleanupOldFiles();
    }
  }
}

void SaveFrameNode::startRecording()
{
  if (is_recording_) {
    RCLCPP_WARN(this->get_logger(), "Already recording");
    return;
  }
  
  try {
    createNewBagFile();
    is_recording_ = true;
    
    RCLCPP_INFO(this->get_logger(), "Started recording to: %s", save_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "Match ID: %s", current_match_id_.c_str());
    
    // 重置统计
    frames_recorded_ = 0;
    messages_dropped_ = 0;
    current_file_size_ = 0;
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start recording: %s", e.what());
  }
}

void SaveFrameNode::stopRecording()
{
  if (!is_recording_) {
    return;
  }
  
  is_recording_ = false;
  
  // 处理缓冲区中剩余的消息
  processBufferBatch();
  
  if (bag_writer_) {
    bag_writer_.reset();
  }
  
  RCLCPP_INFO(this->get_logger(), "Stopped recording");
  RCLCPP_INFO(this->get_logger(), "Total frames recorded: %zu", frames_recorded_);
  RCLCPP_INFO(this->get_logger(), "Messages dropped: %zu", messages_dropped_);
}

void SaveFrameNode::createNewBagFile()
{
  if (bag_writer_) {
    bag_writer_.reset();
  }
  
  std::string filename = generateFilename();
  std::string full_path = (std::filesystem::path(save_path_) / filename).string();
  
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = full_path;
  storage_options.storage_id = "sqlite3";
  
  rosbag2_cpp::ConverterOptions converter_options;
  converter_options.output_serialization_format = "cdr";
  
  if (use_compression_) {
    // 使用压缩写入器
    rosbag2_compression::CompressionOptions compression_options;
    compression_options.compression_mode = rosbag2_compression::CompressionMode::FILE;
    compression_options.compression_format = "zstd";
    
    auto compression_writer = std::make_unique<rosbag2_compression::SequentialCompressionWriter>(compression_options);
    bag_writer_ = std::move(compression_writer);
    
    RCLCPP_INFO(this->get_logger(), "Using compression: format=zstd, mode=FILE");
  } else {
    // 使用普通写入器
    bag_writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();
  }
  
  bag_writer_->open(storage_options, converter_options);
  
  // 注册话题
  if (record_images_) {
    bag_writer_->create_topic(
      {image_topic_, "sensor_msgs/msg/Image", "cdr", ""});
  }
  
  if (record_detections_) {
    bag_writer_->create_topic(
      {detection_topic_, "autoaim_interfaces/msg/GreenDot", "cdr", ""});
  }
  
  current_file_size_ = 0;
  file_counter_++;
  
  RCLCPP_INFO(this->get_logger(), "Created new bag file: %s", filename.c_str());
}

std::string SaveFrameNode::generateFilename() const
{
  auto now = std::chrono::system_clock::now();
  auto time_t = std::chrono::system_clock::to_time_t(now);
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
    now.time_since_epoch()) % 1000;
  
  std::stringstream ss;
  ss << "match_" << current_match_id_ << "_"
     << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S")
     << "_" << std::setfill('0') << std::setw(3) << ms.count()
     << "_part" << std::setfill('0') << std::setw(3) << file_counter_
     << ".db3";
  
  return ss.str();
}

void SaveFrameNode::cleanupOldFiles()
{
  try {
    std::vector<std::filesystem::path> files;
    
    // 收集所有bag文件
    for (const auto & entry : std::filesystem::directory_iterator(save_path_)) {
      if (entry.is_regular_file() && entry.path().extension() == ".db3") {
        files.push_back(entry.path());
      }
    }
    
    // 按修改时间排序（最旧的在前面）
    std::sort(files.begin(), files.end(),
              [](const std::filesystem::path & a, const std::filesystem::path & b) {
                return std::filesystem::last_write_time(a) < std::filesystem::last_write_time(b);
              });
    
    // 计算当前总大小
    uint64_t total_size = 0;
    for (const auto & file : files) {
      total_size += std::filesystem::file_size(file);
    }
    
    // 删除最旧的文件直到满足大小限制
    size_t deleted_count = 0;
    while (!files.empty() && total_size > max_total_size_gb_ * 1024ULL * 1024ULL * 1024ULL) {
      const auto & oldest_file = files.front();
      uint64_t file_size = std::filesystem::file_size(oldest_file);
      
      std::filesystem::remove(oldest_file);
      total_size -= file_size;
      files.erase(files.begin());
      deleted_count++;
    }
    
    if (deleted_count > 0) {
      RCLCPP_INFO(this->get_logger(), "Cleaned up %zu old bag files", deleted_count);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to cleanup old files: %s", e.what());
  }
}

uint64_t SaveFrameNode::getAvailableDiskSpace(const std::string & path) const
{
  try {
    std::filesystem::space_info space = std::filesystem::space(path);
    return space.available;
  } catch (const std::exception & e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get disk space: %s", e.what());
    return UINT64_MAX;
  }
}

void SaveFrameNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  // 提取相机内参矩阵
  if (msg->k.size() == 9) {
    camera_matrix_.clear();
    camera_matrix_.reserve(9);
    for (size_t i = 0; i < 9; ++i) {
      camera_matrix_.push_back(msg->k[i]);
    }
  }
  
  // 提取畸变系数
  distortion_coeffs_.clear();
  distortion_coeffs_.reserve(msg->d.size());
  for (const auto & coeff : msg->d) {
    distortion_coeffs_.push_back(coeff);
  }
  
  // 提取畸变模型
  distortion_model_ = msg->distortion_model;
  
  if (!camera_info_received_) {
    camera_info_received_ = true;
    RCLCPP_INFO(this->get_logger(), "Received camera info: model=%s, matrix_size=%zu, dist_size=%zu",
                distortion_model_.c_str(), camera_matrix_.size(), distortion_coeffs_.size());
  }
}

}  // namespace save_frame

RCLCPP_COMPONENTS_REGISTER_NODE(save_frame::SaveFrameNode)
