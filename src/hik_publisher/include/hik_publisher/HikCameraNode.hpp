#include <algorithm>
#include <iostream>
#include <thread>
#include <vector>

#include "MvCameraControl.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

class HikCameraNode : public rclcpp::Node
{
private:
  // --- Raw data publisher (zero-copy via loaned messages) ---
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;

  // --- RGB recording publisher ---
  sensor_msgs::msg::Image rgb_msg_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr save_pub_;

  // Raw buffer for RGB conversion (copy before freeing SDK buffer)
  std::vector<uint8_t> raw_buffer_;

  OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;

private:
  std::thread capture_thread_;
  int nRet = MV_OK;
  MV_IMAGE_BASIC_INFO img_info_;
  MV_CC_PIXEL_CONVERT_PARAM ConvertParam_;
  void * camera_handle_;

  // Hardware timestamp
  uint64_t tick_freq_hz_ = 1000000000;  // 1 tick = 1ns (GigE Vision default)

public:
  explicit HikCameraNode(const rclcpp::NodeOptions & options) : Node("hik_camera", options)
  {
    RCLCPP_INFO(this->get_logger(), "Starting HikCameraNode!");

    // 1. Enumerate and open device
    MV_CC_DEVICE_INFO_LIST DeviceList;
    nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &DeviceList);
    while (DeviceList.nDeviceNum == 0 && rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found! Retrying...");
      std::this_thread::sleep_for(std::chrono::seconds(1));
      nRet = MV_CC_EnumDevices(MV_USB_DEVICE | MV_GIGE_DEVICE, &DeviceList);
    }
    MV_CC_CreateHandle(&camera_handle_, DeviceList.pDeviceInfo[0]);
    MV_CC_OpenDevice(camera_handle_);

    // 2. Force BayerRG8 output
    int tempRet = MV_CC_SetEnumValue(camera_handle_, "PixelFormat", PixelType_Gvsp_BayerRG8);
    if (tempRet != MV_OK) {
      RCLCPP_WARN(this->get_logger(), "Failed to set PixelFormat to BayerRG8.");
    }

    MV_CC_GetImageInfo(camera_handle_, &img_info_);

    // 3. Query device timestamp tick frequency
    MVCC_INTVALUE tick_freq;
    if (MV_CC_GetIntValue(camera_handle_, "DeviceTimestampTickFrequency", &tick_freq) == MV_OK &&
        tick_freq.nCurValue > 0) {
      tick_freq_hz_ = tick_freq.nCurValue;
      RCLCPP_INFO(this->get_logger(), "Timestamp tick frequency: %lu Hz", tick_freq_hz_);
    } else {
      tick_freq_hz_ = 1000000000;  // GigE Vision: 1 tick = 1 ns
      RCLCPP_INFO(this->get_logger(), "Using default tick frequency: 1 ns/tick");
    }

    // 4. Pre-allocate buffers (once, never resized in hot path)
    size_t raw_size = img_info_.nHeightMax * img_info_.nWidthMax;
    raw_buffer_.resize(raw_size);
    rgb_msg_.data.resize(raw_size * 3);

    // Init conversion params
    ConvertParam_.nWidth = img_info_.nWidthMax;
    ConvertParam_.nHeight = img_info_.nHeightMax;
    ConvertParam_.enDstPixelType = PixelType_Gvsp_RGB8_Packed;

    // 5. Publishers
    raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/image_raw", rclcpp::SensorDataQoS());

    save_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
        "/save_frame", rclcpp::SensorDataQoS());

    declareParams();
    MV_CC_StartGrabbing(camera_handle_);

    params_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&HikCameraNode::parametersCallback, this, std::placeholders::_1));

    capture_thread_ = std::thread(&HikCameraNode::captureLoop, this);
  }

  ~HikCameraNode()
  {
    if (capture_thread_.joinable()) capture_thread_.join();
    if (camera_handle_) {
      MV_CC_StopGrabbing(camera_handle_);
      MV_CC_CloseDevice(camera_handle_);
      MV_CC_DestroyHandle(&camera_handle_);
    }
    RCLCPP_INFO(this->get_logger(), "HikCameraNode destroyed!");
  }

  void captureLoop()
  {
    MV_FRAME_OUT OutFrame;
    int fail_count = 0;

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);
    MV_CC_StartGrabbing(camera_handle_);

    while (rclcpp::ok()) {
      nRet = MV_CC_GetImageBuffer(camera_handle_, &OutFrame, 1000);

      if (nRet == MV_OK) {
        fail_count = 0;

        // Hardware timestamp
        uint64_t dev_ticks = OutFrame.stFrameInfo.nDevTimeStamp;
        uint64_t dev_ns = dev_ticks * (1000000000ULL / tick_freq_hz_);
        rclcpp::Time stamp(dev_ns, RCL_SYSTEM_TIME);

        // Determine Bayer encoding
        std::string bayer_encoding;
        bool is_bayer = true;

        switch (OutFrame.stFrameInfo.enPixelType) {
          case PixelType_Gvsp_BayerRG8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_RGGB8;
            break;
          case PixelType_Gvsp_BayerGB8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_GBRG8;
            break;
          case PixelType_Gvsp_BayerGR8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
            break;
          case PixelType_Gvsp_BayerBG8:
            bayer_encoding = sensor_msgs::image_encodings::BAYER_BGGR8;
            break;
          default:
            is_bayer = false;
            break;
        }

        uint32_t frame_len = OutFrame.stFrameInfo.nFrameLen;
        uint32_t height = OutFrame.stFrameInfo.nHeight;
        uint32_t width = OutFrame.stFrameInfo.nWidth;

        bool need_rgb = save_pub_->get_subscription_count() > 0;

        if (is_bayer) {
          // Save raw copy for RGB conversion (before freeing SDK buffer)
          if (need_rgb && frame_len <= raw_buffer_.size()) {
            memcpy(raw_buffer_.data(), OutFrame.pBufAddr, frame_len);
          }

          // Publish /image_raw — zero-copy via loaned message
          auto loaned_msg = raw_pub_->borrow_loaned_message();
          if (loaned_msg.is_valid()) {
            auto & msg = loaned_msg.get();
            msg.header.stamp = stamp;
            msg.header.frame_id = "camera_optical_frame";
            msg.encoding = bayer_encoding;
            msg.height = height;
            msg.width = width;
            msg.step = width;
            msg.data.resize(frame_len);
            memcpy(msg.data.data(), OutFrame.pBufAddr, frame_len);
            raw_pub_->publish(std::move(loaned_msg));
          } else {
            auto msg = std::make_unique<sensor_msgs::msg::Image>();
            msg->header.stamp = stamp;
            msg->header.frame_id = "camera_optical_frame";
            msg->encoding = bayer_encoding;
            msg->height = height;
            msg->width = width;
            msg->step = width;
            msg->data.resize(frame_len);
            memcpy(msg->data.data(), OutFrame.pBufAddr, frame_len);
            raw_pub_->publish(std::move(msg));
          }
        }

        // Free SDK buffer immediately — camera can start next exposure
        MV_CC_FreeImageBuffer(camera_handle_, &OutFrame);

        // Publish /save_frame (from saved raw buffer)
        if (need_rgb) {
          rgb_msg_.header.stamp = stamp;
          rgb_msg_.header.frame_id = "camera_optical_frame";
          rgb_msg_.encoding = "rgb8";
          rgb_msg_.height = height;
          rgb_msg_.width = width;
          rgb_msg_.step = width * 3;
          rgb_msg_.data.resize(height * width * 3);

          ConvertParam_.pDstBuffer = rgb_msg_.data.data();
          ConvertParam_.nDstBufferSize = rgb_msg_.data.size();
          ConvertParam_.pSrcData = raw_buffer_.data();
          ConvertParam_.nSrcDataLen = frame_len;
          ConvertParam_.enSrcPixelType = OutFrame.stFrameInfo.enPixelType;

          MV_CC_ConvertPixelType(camera_handle_, &ConvertParam_);

          save_pub_->publish(rgb_msg_);
        }
      } else {
        fail_count++;
        if (fail_count >= 100) {
          RCLCPP_WARN(this->get_logger(), "Get buffer failed! Resetting...");
          MV_CC_StopGrabbing(camera_handle_);
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          MV_CC_StartGrabbing(camera_handle_);
          fail_count = 0;
        }
      }
    }
  }

  void declareParams()
  {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    MVCC_FLOATVALUE fValue;
    param_desc.integer_range.resize(1);
    param_desc.integer_range[0].step = 1;

    // Exposure time
    param_desc.description = "Exposure time in microseconds";
    MV_CC_GetFloatValue(camera_handle_, "ExposureTime", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double exposure_time = this->declare_parameter("exposure_time", 5000.0, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "ExposureTime", exposure_time);

    // Gain
    MV_CC_GetFloatValue(camera_handle_, "Gain", &fValue);
    param_desc.integer_range[0].from_value = fValue.fMin;
    param_desc.integer_range[0].to_value = fValue.fMax;
    double gain = this->declare_parameter("gain", fValue.fCurValue, param_desc);
    nRet = MV_CC_SetFloatValue(camera_handle_, "Gain", gain);

    MV_CC_SetEnumValue(camera_handle_, "TriggerMode", 0);
    nRet = MV_CC_SetBoolValue(camera_handle_, "AcquisitionFrameRateEnable", false);
  }

  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> & parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : parameters) {
      if (param.get_name() == "exposure_time") {
        MV_CC_SetFloatValue(camera_handle_, "ExposureTime", param.as_double());
      } else if (param.get_name() == "gain") {
        MV_CC_SetFloatValue(camera_handle_, "Gain", param.as_double());
      }
    }
    return result;
  }
};
