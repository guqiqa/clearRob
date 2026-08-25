#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

#include "CameraApi.h"
#include "cleaning_robot_interfaces/msg/stereo_frame_status.hpp"
#include "log.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace {

using namespace std::chrono_literals;

builtin_interfaces::msg::Time toRosTime(uint64_t timestamp_ns) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(timestamp_ns / 1000000000ULL);
  stamp.nanosec = static_cast<uint32_t>(timestamp_ns % 1000000000ULL);
  return stamp;
}

robot_sensor::CameraDeviceParam makeDevice(
    const std::string &name, robot_sensor::CameraStreamRole role, int sensor_index,
    const std::string &gdc_file, uint32_t stream_id, int sensor_fps,
    int output_fps, int width, int height, int pool_capacity) {
  robot_sensor::CameraDeviceParam device;
  device.m_strCameraName = name;
  device.m_enumCameraType = robot_sensor::CameraType_SC132GS_V1;
  device.m_enumStreamRole = role;
  device.m_strBinFilePath = gdc_file;
  device.m_nSensorIndex = sensor_index;
  device.m_nSensorMode = 0;
  device.m_nFps = sensor_fps;
  device.m_nI2cBus = 0;
  device.m_nI2cAddr = 0;
  device.m_cImgWidth = static_cast<uint16_t>(width);
  device.m_cImgHeight = static_cast<uint16_t>(height);
  device.m_cImgDepth = 8;
  device.m_cImgFormat = 0;
  device.m_nFramePoolCapacity = static_cast<uint32_t>(pool_capacity);

  robot_sensor::CameraOutputParam output;
  output.output_stream_id = stream_id;
  output.width = static_cast<uint16_t>(width);
  output.height = static_cast<uint16_t>(height);
  output.fps = static_cast<uint16_t>(output_fps);
  device.m_vecOutputParams.push_back(output);
  return device;
}

}  // namespace

class NativeStereoCameraNode final : public rclcpp::Node {
 public:
  NativeStereoCameraNode() : Node("native_stereo_camera") {
    declareParameters();
    robot::Log::Config log_config;
    log_config.filename = "/tmp/native_stereo_camera.log";
    log_config.enable_async = false;
    log_config.enable_console = true;
    robot::Log::init(log_config);
    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    left_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("left_topic").as_string(), qos);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(
        get_parameter("right_topic").as_string(), qos);
    status_pub_ = create_publisher<cleaning_robot_interfaces::msg::StereoFrameStatus>(
        get_parameter("status_topic").as_string(), qos);
    worker_ = std::thread([this]() { runCamera(); });
  }

  ~NativeStereoCameraNode() override {
    stopping_.store(true);
    if (camera_) {
      camera_->requestStop(robot_sensor::CameraStopMode::Abort);
    }
    if (worker_.joinable()) {
      worker_.join();
    }
  }

 private:
  void declareParameters() {
    declare_parameter("left_topic", "camera_native/left/image_raw");
    declare_parameter("right_topic", "camera_native/right/image_raw");
    declare_parameter("status_topic", "camera_native/stereo/status");
    declare_parameter("left_frame_id", "stereo_left_camera_optical_frame");
    declare_parameter("right_frame_id", "stereo_right_camera_optical_frame");
    declare_parameter("left_sensor_index", 4);
    declare_parameter("right_sensor_index", 5);
    declare_parameter("left_output_stream_id", 1);
    declare_parameter("right_output_stream_id", 2);
    declare_parameter("left_gdc_file", "/root/LawnMower/config/SC132GS_V1_left_config.bin");
    declare_parameter("right_gdc_file", "/root/LawnMower/config/SC132GS_V1_right_config.bin");
    declare_parameter("width", 1088);
    declare_parameter("height", 1280);
    declare_parameter("sensor_fps", 60);
    declare_parameter("output_fps", 30);
    declare_parameter("publish_fps", 15);
    declare_parameter("pool_capacity", 16);
    declare_parameter("reader_queue_capacity", 8);
    declare_parameter("max_pair_delta_ms", 5.0);
    declare_parameter("ready_timeout_s", 20.0);
  }

  robot_sensor::CameraSystemParam cameraParams() const {
    const int sensor_fps = get_parameter("sensor_fps").as_int();
    const int output_fps = get_parameter("output_fps").as_int();
    const int width = get_parameter("width").as_int();
    const int height = get_parameter("height").as_int();
    const int pool_capacity = get_parameter("pool_capacity").as_int();

    robot_sensor::CameraSystemParam params;
    params.m_enumMode = robot_sensor::CameraSystemMode_Stereo;
    params.m_memStereoSyncParam.max_delta_ns = static_cast<uint64_t>(
        get_parameter("max_pair_delta_ms").as_double() * 1000000.0);
    params.m_memStereoSyncParam.max_frame_age_ns = 200000000ULL;
    params.m_memStereoSyncParam.max_queue_size = 16;
    params.m_memStereoSyncParam.poll_interval_ms = 1;
    params.m_memStereoSyncParam.left_output_stream_id =
        static_cast<uint32_t>(get_parameter("left_output_stream_id").as_int());
    params.m_memStereoSyncParam.right_output_stream_id =
        static_cast<uint32_t>(get_parameter("right_output_stream_id").as_int());

    params.m_vecCameraParams.push_back(makeDevice(
        "left", robot_sensor::CameraStreamRole_StereoLeft,
        get_parameter("left_sensor_index").as_int(),
        get_parameter("left_gdc_file").as_string(),
        params.m_memStereoSyncParam.left_output_stream_id, sensor_fps, output_fps,
        width, height, pool_capacity));
    params.m_vecCameraParams.push_back(makeDevice(
        "right", robot_sensor::CameraStreamRole_StereoRight,
        get_parameter("right_sensor_index").as_int(),
        get_parameter("right_gdc_file").as_string(),
        params.m_memStereoSyncParam.right_output_stream_id, sensor_fps, output_fps,
        width, height, pool_capacity));
    return params;
  }

  void runCamera() {
    try {
      camera_ = std::make_unique<robot_sensor::CameraManager>();
      if (!camera_->configure(cameraParams())) {
        throw std::runtime_error("CameraManager configure failed: " +
                                 camera_->statusSnapshot().last_error.message);
      }

      const int output_fps = get_parameter("output_fps").as_int();
      const int publish_fps = get_parameter("publish_fps").as_int();
      robot_sensor::FrameReaderConfig reader_config;
      reader_config.delivery_mode = robot_sensor::FrameDeliveryMode::Latest;
      reader_config.queue_capacity = static_cast<uint32_t>(
          get_parameter("reader_queue_capacity").as_int());
      reader_config.deterministic_sample_period = static_cast<uint32_t>(
          std::max(1, output_fps / std::max(1, publish_fps)));
      reader_config.deterministic_sample_phase = 0;
      reader_ = camera_->subscribeStereo(reader_config);
      if (!reader_) {
        throw std::runtime_error("subscribeStereo failed: " +
                                 camera_->statusSnapshot().last_error.message);
      }
      if (!camera_->prepare()) {
        throw std::runtime_error("CameraManager prepare failed: " +
                                 camera_->statusSnapshot().last_error.message);
      }

      auto jobs = camera_->getThreadRunJobs();
      if (jobs.empty()) {
        throw std::runtime_error("CameraManager returned no thread jobs");
      }
      camera_threads_.reserve(jobs.size());
      for (auto &job : jobs) {
        RCLCPP_INFO(get_logger(), "starting camera job: %s", std::get<0>(job).c_str());
        camera_threads_.emplace_back(std::move(std::get<1>(job)));
      }

      const auto ready_timeout = std::chrono::milliseconds(static_cast<int64_t>(
          get_parameter("ready_timeout_s").as_double() * 1000.0));
      if (!camera_->waitReady(ready_timeout)) {
        throw std::runtime_error("camera readiness timeout: " +
                                 camera_->statusSnapshot().last_error.message);
      }
      RCLCPP_INFO(get_logger(), "native stereo camera ready; publish=%d fps", publish_fps);

      while (rclcpp::ok() && !stopping_.load()) {
        robot_sensor::StereoFrame frame;
        const auto result = reader_->readFrame(frame, 500ms);
        if (result == robot_sensor::FrameReadStatus::FrameReady) {
          publishFrame(frame);
        } else if (result == robot_sensor::FrameReadStatus::DeliveryError) {
          ++delivery_errors_;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                               "stereo reader delivery error");
        } else if (result == robot_sensor::FrameReadStatus::Closed) {
          break;
        }
      }
      stopCamera(robot_sensor::CameraStopMode::Drain);
    } catch (const std::exception &error) {
      RCLCPP_ERROR(get_logger(), "native stereo camera failed: %s", error.what());
      stopCamera(robot_sensor::CameraStopMode::Abort);
      if (rclcpp::ok() && !stopping_.load()) {
        rclcpp::shutdown();
      }
    }
  }

  void stopCamera(robot_sensor::CameraStopMode mode) {
    if (camera_) {
      camera_->requestStop(mode);
    }
    for (auto &thread : camera_threads_) {
      if (thread.joinable()) {
        thread.join();
      }
    }
    camera_threads_.clear();
  }

  sensor_msgs::msg::Image makeMonoImage(
      const robot_sensor::FrameHandle &handle, uint64_t pair_timestamp_ns,
      const std::string &frame_id) const {
    const auto &metadata = handle.metadata();
    if (!handle.valid() || handle.data() == nullptr || metadata.img_width == 0 ||
        metadata.img_height == 0 || metadata.img_stride < metadata.img_width) {
      throw std::runtime_error("invalid native camera frame");
    }
    const size_t required = static_cast<size_t>(metadata.img_stride) * metadata.img_height;
    if (handle.size() < required) {
      throw std::runtime_error("native NV12 frame is shorter than its Y plane");
    }

    sensor_msgs::msg::Image image;
    image.header.stamp = toRosTime(pair_timestamp_ns);
    image.header.frame_id = frame_id;
    image.height = metadata.img_height;
    image.width = metadata.img_width;
    image.encoding = "mono8";
    image.is_bigendian = false;
    image.step = metadata.img_width;
    image.data.resize(static_cast<size_t>(image.width) * image.height);
    for (uint32_t row = 0; row < image.height; ++row) {
      std::memcpy(image.data.data() + static_cast<size_t>(row) * image.width,
                  handle.data() + static_cast<size_t>(row) * metadata.img_stride,
                  image.width);
    }
    return image;
  }

  void publishFrame(const robot_sensor::StereoFrame &frame) {
    const auto max_delta_ns = static_cast<int64_t>(
        get_parameter("max_pair_delta_ms").as_double() * 1000000.0);
    if (!frame.left.valid() || !frame.right.valid() || frame.timestamp_ns == 0 ||
        std::llabs(frame.delta_ns) > max_delta_ns) {
      throw std::runtime_error("invalid or out-of-threshold stereo pair");
    }

    auto left = makeMonoImage(frame.left, frame.timestamp_ns,
                              get_parameter("left_frame_id").as_string());
    auto right = makeMonoImage(frame.right, frame.timestamp_ns,
                               get_parameter("right_frame_id").as_string());
    left_pub_->publish(left);
    right_pub_->publish(right);

    const auto snapshot = camera_->statusSnapshot();
    cleaning_robot_interfaces::msg::StereoFrameStatus status;
    status.header.stamp = left.header.stamp;
    status.header.frame_id = "stereo_camera_link";
    status.pair_seq = frame.pair_seq;
    status.left_timestamp_ns = frame.left.metadata().timestamp_ns;
    status.right_timestamp_ns = frame.right.metadata().timestamp_ns;
    status.delta_ns = frame.delta_ns;
    status.source_epoch = frame.source_epoch;
    status.paired_frames = snapshot.stereo.paired_frames;
    status.dropped_left = snapshot.stereo.dropped_left;
    status.dropped_right = snapshot.stereo.dropped_right;
    status.delivery_errors = delivery_errors_ + snapshot.stereo.output_delivery_errors;
    status_pub_->publish(status);
  }

  std::atomic_bool stopping_{false};
  uint64_t delivery_errors_{0};
  std::unique_ptr<robot_sensor::CameraManager> camera_;
  std::unique_ptr<robot_sensor::StereoFrameReader> reader_;
  std::thread worker_;
  std::vector<std::thread> camera_threads_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
  rclcpp::Publisher<cleaning_robot_interfaces::msg::StereoFrameStatus>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NativeStereoCameraNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
