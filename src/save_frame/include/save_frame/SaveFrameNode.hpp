#pragma once

#include <atomic>
#include <chrono>
#include <deque>
#include <filesystem>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "autoaim_interfaces/msg/green_dot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_compression/sequential_compression_writer.hpp"
#include "rosbag2_compression/compression_options.hpp"
#include <rosbag2_compression_zstd/zstd_compressor.hpp>
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "rclcpp/parameter_client.hpp"

namespace save_frame
{

struct BufferedMessage
{
  rclcpp::Time timestamp;
  std::string topic_name;
  std::shared_ptr<rclcpp::SerializedMessage> serialized_msg;
};

class SaveFrameNode : public rclcpp::Node
{
public:
  explicit SaveFrameNode(const rclcpp::NodeOptions & options);
  ~SaveFrameNode();

private:
  // 初始化
  void initParameters();
  void initSubscribers();
  void initRecording();

  // 回调函数
  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  void detectionCallback(const autoaim_interfaces::msg::GreenDot::SharedPtr msg);
  void matchStatusCallback(const std_msgs::msg::String::SharedPtr msg);
  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // 录制控制
  void startRecording();
  void stopRecording();
  void recordingThread();

  // 缓冲区管理
  void addToBuffer(const std::string & topic_name, const rclcpp::SerializedMessage & msg);
  void processBufferBatch();

  // 文件管理
  void createNewBagFile();
  void cleanupOldFiles();
  std::string generateFilename() const;
  uint64_t getAvailableDiskSpace(const std::string & path) const;
  
  // 录制目录管理
  void createRecordingDir();
  void saveDetectParams();
  std::string recording_dir_;

  // 配置参数
  std::string save_path_;
  std::string image_topic_;
  std::string detection_topic_;
  std::string match_status_topic_;
  bool record_images_;
  bool record_detections_;
  bool auto_start_;
  size_t buffer_size_;
  size_t batch_size_;
  size_t max_file_size_mb_;
  size_t max_total_size_gb_;
  size_t disk_space_threshold_gb_;
  bool use_compression_;
  double downsample_ratio_;
  int image_quality_;
  
  // 间隔取帧参数
  size_t frame_interval_;
  size_t frame_counter_;
  int64_t time_interval_ms_;
  std::chrono::steady_clock::time_point last_saved_time_;

  // 相机内参（从话题获取）
  std::vector<double> camera_matrix_;
  std::vector<double> distortion_coeffs_;
  std::string distortion_model_;
  std::atomic<bool> camera_info_received_{false};

  // 状态变量
  std::atomic<bool> is_recording_{false};
  std::atomic<bool> stop_thread_{false};
  std::atomic<size_t> current_file_size_{0};
  std::atomic<size_t> total_recorded_size_{0};
  size_t file_counter_{0};
  std::string current_match_id_{"default"};

  // 线程和同步
  std::thread recording_thread_;
  std::mutex buffer_mutex_;
  std::condition_variable buffer_cv_;
  std::deque<BufferedMessage> message_buffer_;

  // ROS 组件
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<autoaim_interfaces::msg::GreenDot>::SharedPtr detection_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr match_status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> bag_writer_;

  // 性能统计
  std::chrono::steady_clock::time_point last_stat_time_;
  size_t frames_received_{0};
  size_t frames_recorded_{0};
  size_t messages_dropped_{0};
};

}  // namespace save_frame
