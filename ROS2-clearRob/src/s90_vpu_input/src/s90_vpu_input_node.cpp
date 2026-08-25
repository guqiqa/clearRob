#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <deque>
#include <cstdio>
#include <functional>
#include <memory>
#include <mutex>
#include <netdb.h>
#include <optional>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <sys/socket.h>
#include <unistd.h>
#include <vector>

#include "cleaning_robot_interfaces/msg/stereo_frame_status.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

#ifdef S90_VPU_HAS_SDK
#include "hb_media_codec.h"
#endif

namespace {
using Clock = std::chrono::steady_clock;
using Nanoseconds = std::chrono::nanoseconds;

struct DecodedFrame {
  std::vector<uint8_t> luma;
  uint64_t arrival_ns{0};
};

uint64_t nowNs() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<Nanoseconds>(Clock::now().time_since_epoch()).count());
}

class ByteStream {
 public:
  explicit ByteStream(std::string source) : source_(std::move(source)) {}
  ~ByteStream() { close(); }

  bool open() {
    close();
    if (source_.rfind("file://", 0) == 0) {
      file_ = ::fopen(source_.c_str() + 7, "rb");
      return file_ != nullptr;
    }
    if (source_.rfind("http://", 0) != 0) return false;
    std::string rest = source_.substr(7);
    const auto slash = rest.find('/');
    const auto host_port = slash == std::string::npos ? rest : rest.substr(0, slash);
    path_ = slash == std::string::npos ? "/" : rest.substr(slash);
    std::string host = host_port;
    std::string port = "80";
    const auto colon = host_port.rfind(':');
    if (colon != std::string::npos) {
      host = host_port.substr(0, colon);
      port = host_port.substr(colon + 1);
    }
    addrinfo hints{};
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_family = AF_UNSPEC;
    addrinfo *resolved = nullptr;
    if (::getaddrinfo(host.c_str(), port.c_str(), &hints, &resolved) != 0) return false;
    for (addrinfo *p = resolved; p != nullptr; p = p->ai_next) {
      fd_ = ::socket(p->ai_family, p->ai_socktype, p->ai_protocol);
      if (fd_ >= 0 && ::connect(fd_, p->ai_addr, p->ai_addrlen) == 0) break;
      if (fd_ >= 0) ::close(fd_);
      fd_ = -1;
    }
    ::freeaddrinfo(resolved);
    if (fd_ < 0) return false;
    const std::string request = "GET " + path_ + " HTTP/1.0\r\nHost: " + host +
                                "\r\nConnection: close\r\n\r\n";
    if (::send(fd_, request.data(), request.size(), 0) < 0) {
      close();
      return false;
    }
    std::string headers;
    char c = 0;
    while (headers.size() < 16384 && ::recv(fd_, &c, 1, 0) == 1) {
      headers.push_back(c);
      if (headers.size() >= 4 && headers.compare(headers.size() - 4, 4, "\r\n\r\n") == 0) break;
    }
    if (headers.find(" 200 ") == std::string::npos) {
      close();
      return false;
    }
    return true;
  }

  ssize_t read(uint8_t *dst, size_t capacity) {
    if (file_) return static_cast<ssize_t>(::fread(dst, 1, capacity, file_));
    if (fd_ >= 0) return ::recv(fd_, dst, capacity, 0);
    return -1;
  }

  void close() {
    if (file_) {
      ::fclose(file_);
      file_ = nullptr;
    }
    if (fd_ >= 0) {
      ::shutdown(fd_, SHUT_RDWR);
      ::close(fd_);
      fd_ = -1;
    }
  }

 private:
  std::string source_;
  std::string path_;
  FILE *file_{nullptr};
  int fd_{-1};
};

#ifdef S90_VPU_HAS_SDK
class VpuDecoder {
 public:
  VpuDecoder(const std::string &source, int output_width, int output_height,
             int chunk_bytes, int bitstream_bytes, int bitstream_count,
             int frame_count, int vpf_channel, double max_fps,
             std::function<void(DecodedFrame)> callback,
             rclcpp::Logger logger)
      : source_(source), output_width_(output_width), output_height_(output_height),
        chunk_bytes_(chunk_bytes), bitstream_bytes_(bitstream_bytes),
        bitstream_count_(bitstream_count), frame_count_(frame_count), vpf_channel_(vpf_channel),
        period_(std::chrono::duration<double>(1.0 / std::max(max_fps, 1.0))),
        callback_(std::move(callback)), logger_(logger) {}

  void stop() { stopping_.store(true); }
  void run(double reconnect_s) {
    while (!stopping_.load() && rclcpp::ok()) {
      try {
        decodeOnce();
      } catch (const std::exception &e) {
        RCLCPP_WARN(logger_, "VPU stream %s stopped: %s", source_.c_str(), e.what());
      }
      if (!stopping_.load()) std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_s));
    }
  }

 private:
  void check(hb_s32 rc, const char *what) {
    if (rc != 0) throw std::runtime_error(std::string(what) + " rc=" + std::to_string(rc));
  }

  void decodeOnce() {
    ByteStream input(source_);
    if (!input.open()) throw std::runtime_error("unable to open source");
    media_codec_context_t context{};
    check(hb_mm_mc_get_default_context(MEDIA_CODEC_ID_H264, 0, &context), "get_default_context");
    context.video_dec_params.feed_mode = MC_FEEDING_MODE_STREAM_SIZE;
    context.video_dec_params.pix_fmt = MC_PIXEL_FORMAT_NV21;
    context.video_dec_params.bitstream_buf_size = static_cast<hb_u32>(bitstream_bytes_);
    context.video_dec_params.bitstream_buf_count = static_cast<hb_u32>(bitstream_count_);
    context.video_dec_params.external_bitstream_buf = 0;
    context.video_dec_params.frame_buf_count = static_cast<hb_u32>(frame_count_);
    context.video_dec_params.h264_dec_config.reorder_enable = 1;
    context.video_dec_params.h264_dec_config.skip_mode = 0;
    context.video_dec_params.h264_dec_config.bandwidth_Opt = 0;
    check(hb_mm_mc_initialize(&context), "initialize");
    bool configured = false;
    bool started = false;
    try {
      check(hb_mm_mc_vpf_init(&context, vpf_channel_), "vpf_init");
      check(hb_mm_mc_configure(&context), "configure");
      configured = true;
      mc_av_codec_startup_params_t startup{};
      check(hb_mm_mc_start(&context, &startup), "start");
      started = true;
      std::vector<uint8_t> chunk(static_cast<size_t>(chunk_bytes_));
      auto next_frame = Clock::now();
      while (!stopping_.load() && rclcpp::ok()) {
        ssize_t count = input.read(chunk.data(), chunk.size());
        if (count <= 0) break;
        size_t offset = 0;
        while (offset < static_cast<size_t>(count) && !stopping_.load()) {
          media_codec_buffer_t in{};
          check(hb_mm_mc_dequeue_input_buffer(&context, &in, 1000), "dequeue_input_buffer");
          if (in.type != MC_VIDEO_STREAM_BUFFER || in.vstream_buf.vir_ptr == nullptr || in.vstream_buf.size == 0) {
            throw std::runtime_error("invalid VPU input buffer");
          }
          const size_t amount = std::min<size_t>(in.vstream_buf.size, static_cast<size_t>(count) - offset);
          std::memcpy(in.vstream_buf.vir_ptr, chunk.data() + offset, amount);
          in.vstream_buf.size = static_cast<hb_u32>(amount);
          in.vstream_buf.pts = nowNs();
          in.vstream_buf.stream_end = 0;
          check(hb_mm_mc_queue_input_buffer(&context, &in, 1000), "queue_input_buffer");
          offset += amount;
          drain(context, next_frame);
        }
      }
      drain(context, next_frame);
    } catch (...) {
      if (started) hb_mm_mc_stop(&context);
      if (configured) hb_mm_mc_release(&context);
      input.close();
      throw;
    }
    hb_mm_mc_stop(&context);
    hb_mm_mc_release(&context);
    input.close();
  }

  void drain(media_codec_context_t &context, Clock::time_point &next_frame) {
    while (!stopping_.load()) {
      media_codec_buffer_t out{};
      media_codec_output_buffer_info_t info{};
      const hb_s32 rc = hb_mm_mc_dequeue_output_buffer(&context, &out, &info, 0);
      if (rc != 0) break;
      if (out.type == MC_VIDEO_FRAME_BUFFER && info.video_frame_info.frame_display_index >= 0 &&
          out.vframe_buf.vir_ptr[0] != nullptr) {
        const auto now = Clock::now();
        if (now < next_frame) {
          hb_mm_mc_queue_output_buffer(&context, &out, 1000);
          continue;
        }
        const int width = out.vframe_buf.width;
        const int height = out.vframe_buf.height;
        const int stride = out.vframe_buf.stride > 0 ? out.vframe_buf.stride : width;
        if (width <= 0 || height <= 0 || stride < width) throw std::runtime_error("invalid VPU frame");
        DecodedFrame frame;
        frame.luma.resize(static_cast<size_t>(output_width_) * output_height_);
        for (int y = 0; y < output_height_; ++y) {
          const int sy = std::min(height - 1, y * height / output_height_);
          for (int x = 0; x < output_width_; ++x) {
            const int sx = std::min(width - 1, x * width / output_width_);
            frame.luma[static_cast<size_t>(y) * output_width_ + x] =
                out.vframe_buf.vir_ptr[0][static_cast<size_t>(sy) * stride + sx];
          }
        }
        frame.arrival_ns = nowNs();
        callback_(std::move(frame));
        next_frame = now + std::chrono::duration_cast<Clock::duration>(period_);
      }
      hb_mm_mc_queue_output_buffer(&context, &out, 1000);
    }
  }

  std::string source_;
  int output_width_, output_height_, chunk_bytes_, bitstream_bytes_, bitstream_count_, frame_count_, vpf_channel_;
  std::chrono::duration<double> period_;
  std::function<void(DecodedFrame)> callback_;
  rclcpp::Logger logger_;
  std::atomic_bool stopping_{false};
};
#endif

bool vpuDecodeNodeEnabled() {
  std::ifstream status("/sys/firmware/devicetree/base/soc/vpu_v4l2@3B000000/status");
  std::string value;
  if (!status.good() || !std::getline(status, value, '\0')) return false;
  return value == "okay" || value == "ok";
}
}  // namespace

class S90VpuInputNode final : public rclcpp::Node {
 public:
  S90VpuInputNode() : Node("s90_vpu_input") {
    declare_parameter("left_source", "http://127.0.0.1:8081/left.h264");
    declare_parameter("right_source", "http://127.0.0.1:8081/right.h264");
    declare_parameter("left_topic", "camera_native/left/image_raw");
    declare_parameter("right_topic", "camera_native/right/image_raw");
    declare_parameter("status_topic", "camera_native/stereo/status");
    declare_parameter("left_frame_id", "stereo_left_camera_optical_frame");
    declare_parameter("right_frame_id", "stereo_right_camera_optical_frame");
    declare_parameter("output_width", 544);
    declare_parameter("output_height", 640);
    declare_parameter("max_fps", 5.0);
    declare_parameter("reconnect_s", 1.0);
    declare_parameter("max_pair_delta_ms", 5.0);
    declare_parameter("max_pair_age_ms", 200.0);
    declare_parameter("input_chunk_bytes", 65536);
    declare_parameter("bitstream_buffer_bytes", 1048576);
    declare_parameter("bitstream_buffer_count", 4);
    declare_parameter("frame_buffer_count", 4);
    declare_parameter("enable_vpu", false);
    declare_parameter("require_vpu_v4l2", true);
    auto qos = rclcpp::SensorDataQoS().keep_last(2);
    left_pub_ = create_publisher<sensor_msgs::msg::Image>(get_parameter("left_topic").as_string(), qos);
    right_pub_ = create_publisher<sensor_msgs::msg::Image>(get_parameter("right_topic").as_string(), qos);
    status_pub_ = create_publisher<cleaning_robot_interfaces::msg::StereoFrameStatus>(
        get_parameter("status_topic").as_string(), qos);
#ifndef S90_VPU_HAS_SDK
    RCLCPP_ERROR(get_logger(), "Hobot VPU SDK is unavailable; install the board SDK before starting this node");
#else
    if (get_parameter("enable_vpu").as_bool() &&
        (!get_parameter("require_vpu_v4l2").as_bool() || vpuDecodeNodeEnabled())) {
      startDecoders();
    } else if (get_parameter("enable_vpu").as_bool()) {
      RCLCPP_ERROR(get_logger(), "VPU decode requested but vpu_v4l2 is disabled; refusing unsafe MediaCodec start");
    } else {
      RCLCPP_WARN(get_logger(), "VPU input is disabled by default; use s90_h264_input until vdec_render is validated");
    }
#endif
  }

  ~S90VpuInputNode() override {
#ifdef S90_VPU_HAS_SDK
    stopping_.store(true);
    if (left_decoder_) left_decoder_->stop();
    if (right_decoder_) right_decoder_->stop();
    if (left_thread_.joinable()) left_thread_.join();
    if (right_thread_.joinable()) right_thread_.join();
#endif
  }

 private:
#ifdef S90_VPU_HAS_SDK
  void startDecoders() {
    const auto callback_left = [this](DecodedFrame frame) { onFrame(true, std::move(frame)); };
    const auto callback_right = [this](DecodedFrame frame) { onFrame(false, std::move(frame)); };
    const int w = get_parameter("output_width").as_int();
    const int h = get_parameter("output_height").as_int();
    const int chunk = get_parameter("input_chunk_bytes").as_int();
    const int bs = get_parameter("bitstream_buffer_bytes").as_int();
    const int bc = get_parameter("bitstream_buffer_count").as_int();
    const int fc = get_parameter("frame_buffer_count").as_int();
    const double fps = get_parameter("max_fps").as_double();
    left_decoder_ = std::make_unique<VpuDecoder>(get_parameter("left_source").as_string(), w, h, chunk, bs, bc, fc, 0, fps, callback_left, get_logger());
    right_decoder_ = std::make_unique<VpuDecoder>(get_parameter("right_source").as_string(), w, h, chunk, bs, bc, fc, 1, fps, callback_right, get_logger());
    left_thread_ = std::thread([this] { left_decoder_->run(get_parameter("reconnect_s").as_double()); });
    right_thread_ = std::thread([this] { right_decoder_->run(get_parameter("reconnect_s").as_double()); });
  }

  void onFrame(bool left, DecodedFrame frame) {
    std::lock_guard<std::mutex> lock(pair_mutex_);
    (left ? pending_left_ : pending_right_) = std::move(frame);
    if (!pending_left_ || !pending_right_) return;
    const int64_t delta = static_cast<int64_t>(pending_left_->arrival_ns) - static_cast<int64_t>(pending_right_->arrival_ns);
    const uint64_t max_delta = static_cast<uint64_t>(get_parameter("max_pair_delta_ms").as_double() * 1000000.0);
    if (std::llabs(delta) > static_cast<int64_t>(max_delta)) {
      if (delta < 0) { pending_left_.reset(); ++dropped_left_; }
      else { pending_right_.reset(); ++dropped_right_; }
      return;
    }
    const auto stamp = get_clock()->now();
    auto make = [this, &stamp](const DecodedFrame &f, const std::string &frame_id) {
      sensor_msgs::msg::Image msg;
      msg.header.stamp = stamp;
      msg.header.frame_id = frame_id;
      msg.height = static_cast<uint32_t>(get_parameter("output_height").as_int());
      msg.width = static_cast<uint32_t>(get_parameter("output_width").as_int());
      msg.encoding = "mono8";
      msg.step = msg.width;
      msg.data = f.luma;
      return msg;
    };
    left_pub_->publish(make(*pending_left_, get_parameter("left_frame_id").as_string()));
    right_pub_->publish(make(*pending_right_, get_parameter("right_frame_id").as_string()));
    cleaning_robot_interfaces::msg::StereoFrameStatus status;
    status.header.stamp = stamp;
    status.header.frame_id = "stereo_camera_link";
    status.pair_seq = ++pair_seq_;
    status.left_timestamp_ns = pending_left_->arrival_ns;
    status.right_timestamp_ns = pending_right_->arrival_ns;
    status.delta_ns = delta;
    status.paired_frames = pair_seq_;
    status.dropped_left = dropped_left_;
    status.dropped_right = dropped_right_;
    status.delivery_errors = 0;
    status_pub_->publish(status);
    pending_left_.reset();
    pending_right_.reset();
  }

  std::unique_ptr<VpuDecoder> left_decoder_, right_decoder_;
  std::thread left_thread_, right_thread_;
  std::atomic_bool stopping_{false};
  std::mutex pair_mutex_;
  std::optional<DecodedFrame> pending_left_, pending_right_;
  uint64_t pair_seq_{0}, dropped_left_{0}, dropped_right_{0};
#endif
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_, right_pub_;
  rclcpp::Publisher<cleaning_robot_interfaces::msg::StereoFrameStatus>::SharedPtr status_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<S90VpuInputNode>();
  rclcpp::spin(node);
  node.reset();
  if (rclcpp::ok()) rclcpp::shutdown();
  return 0;
}
