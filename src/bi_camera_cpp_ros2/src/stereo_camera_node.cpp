#include "bi_camera_cpp_ros2/camera_support.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>
#include <utility>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

namespace bi_camera_cpp_ros2
{

namespace
{

std::unordered_set<std::string> parameter_override_names(const rclcpp::NodeOptions & options)
{
  std::unordered_set<std::string> names;
  for (const auto & parameter : options.parameter_overrides()) {
    names.insert(parameter.get_name());
  }
  return names;
}

sensor_msgs::msg::Image frame_to_msg(
  const cv::Mat & frame,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & frame_id)
{
  sensor_msgs::msg::Image msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.height = static_cast<uint32_t>(frame.rows);
  msg.width = static_cast<uint32_t>(frame.cols);
  msg.encoding = "bgr8";
  msg.is_bigendian = false;
  msg.step = static_cast<sensor_msgs::msg::Image::_step_type>(frame.cols * frame.elemSize());
  msg.data.resize(static_cast<std::size_t>(msg.step) * frame.rows);

  for (int row = 0; row < frame.rows; ++row) {
    const auto * row_ptr = frame.ptr<uint8_t>(row);
    std::memcpy(msg.data.data() + static_cast<std::size_t>(row) * msg.step, row_ptr, msg.step);
  }

  return msg;
}

cv::Mat center_crop_square(const cv::Mat & frame)
{
  const int square_size = std::min(frame.cols, frame.rows);
  const int offset_x = (frame.cols - square_size) / 2;
  const int offset_y = (frame.rows - square_size) / 2;
  return frame(cv::Rect(offset_x, offset_y, square_size, square_size));
}

rclcpp::QoS make_image_qos(const RuntimeConfig & config)
{
  rclcpp::QoS qos(rclcpp::KeepLast(std::max(1, config.ros.image_qos_depth)));
  if (config.ros.image_qos_reliability == "best_effort") {
    qos.best_effort();
  } else {
    qos.reliable();
  }
  return qos;
}

}  // namespace

class StereoCameraNode : public rclcpp::Node
{
public:
  explicit StereoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("bi_camera_publisher", options),
    override_names_(parameter_override_names(options))
  {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<std::string>("device_path", "");
    declare_parameter<std::string>("vendor_id", "");
    declare_parameter<std::string>("product_id", "");
    declare_parameter<std::string>("usb_serial", "");
    declare_parameter<std::string>("pixel_format", "");
    declare_parameter<int>("width", 0);
    declare_parameter<int>("height", 0);
    declare_parameter<double>("fps", 0.0);
    declare_parameter<double>("publish_fps", 0.0);
    declare_parameter<int>("read_failure_threshold", 0);
    declare_parameter<int>("read_retry_delay_ms", 0);
    declare_parameter<int>("reconnect_delay_ms", 0);
    declare_parameter<bool>("split_stereo", true);
    declare_parameter<std::string>("stereo_layout", "");
    declare_parameter<bool>("publish_combined", true);
    declare_parameter<std::string>("combined_topic", "");
    declare_parameter<std::string>("left_topic", "");
    declare_parameter<std::string>("right_topic", "");
    declare_parameter<std::string>("fps_topic", "");
    declare_parameter<std::string>("image_qos_reliability", "");
    declare_parameter<int>("image_qos_depth", 0);
    declare_parameter<std::string>("left_frame_id", "");
    declare_parameter<std::string>("right_frame_id", "");
    declare_parameter<std::string>("combined_frame_id", "");

    config_ = load_runtime_config();
    const auto image_qos = make_image_qos(config_).get_rmw_qos_profile();

    if (config_.camera.publish_combined) {
      combined_pub_ = image_transport::create_publisher(this, config_.ros.combined_topic, image_qos);
    }
    if (config_.camera.split_stereo) {
      left_pub_ = image_transport::create_publisher(this, config_.ros.left_topic, image_qos);
      right_pub_ = image_transport::create_publisher(this, config_.ros.right_topic, image_qos);
    }
    fps_pub_ = create_publisher<std_msgs::msg::Float32>(config_.ros.fps_topic, 10);

    capture_thread_ = std::thread(&StereoCameraNode::capture_loop, this);
  }

  ~StereoCameraNode() override
  {
    stop_requested_.store(true);
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (capture_.isOpened()) {
      capture_.release();
    }
  }

private:
  RuntimeConfig load_runtime_config()
  {
    RuntimeConfig config = load_config_file(get_parameter("config_path").as_string());

    auto maybe_override_string = [&](const char * name, std::string & target) {
        const auto value = get_parameter(name).as_string();
        if (!value.empty()) {
          target = value;
        }
      };

    maybe_override_string("device_path", config.camera.preferred_device);
    maybe_override_string("vendor_id", config.camera.vendor_id);
    maybe_override_string("product_id", config.camera.product_id);
    maybe_override_string("usb_serial", config.camera.usb_serial);
    maybe_override_string("pixel_format", config.camera.pixel_format);
    maybe_override_string("stereo_layout", config.camera.stereo_layout);
    maybe_override_string("left_frame_id", config.camera.left_frame_id);
    maybe_override_string("right_frame_id", config.camera.right_frame_id);
    maybe_override_string("combined_frame_id", config.camera.combined_frame_id);
    maybe_override_string("combined_topic", config.ros.combined_topic);
    maybe_override_string("left_topic", config.ros.left_topic);
    maybe_override_string("right_topic", config.ros.right_topic);
    maybe_override_string("fps_topic", config.ros.fps_topic);
    maybe_override_string("image_qos_reliability", config.ros.image_qos_reliability);

    if (override_names_.count("width") > 0U && get_parameter("width").as_int() > 0) {
      config.camera.width = static_cast<int>(get_parameter("width").as_int());
    }
    if (override_names_.count("height") > 0U && get_parameter("height").as_int() > 0) {
      config.camera.height = static_cast<int>(get_parameter("height").as_int());
    }
    if (override_names_.count("fps") > 0U && get_parameter("fps").as_double() > 0.0) {
      config.camera.fps = get_parameter("fps").as_double();
    }
    if (
      override_names_.count("publish_fps") > 0U &&
      get_parameter("publish_fps").as_double() > 0.0)
    {
      config.camera.publish_fps = get_parameter("publish_fps").as_double();
    }
    if (
      override_names_.count("read_failure_threshold") > 0U &&
      get_parameter("read_failure_threshold").as_int() > 0)
    {
      config.camera.read_failure_threshold =
        static_cast<int>(get_parameter("read_failure_threshold").as_int());
    }
    if (
      override_names_.count("read_retry_delay_ms") > 0U &&
      get_parameter("read_retry_delay_ms").as_int() > 0)
    {
      config.camera.read_retry_delay_ms =
        static_cast<int>(get_parameter("read_retry_delay_ms").as_int());
    }
    if (
      override_names_.count("reconnect_delay_ms") > 0U &&
      get_parameter("reconnect_delay_ms").as_int() > 0)
    {
      config.camera.reconnect_delay_ms =
        static_cast<int>(get_parameter("reconnect_delay_ms").as_int());
    }
    if (override_names_.count("split_stereo") > 0U) {
      config.camera.split_stereo = get_parameter("split_stereo").as_bool();
    }
    if (override_names_.count("publish_combined") > 0U) {
      config.camera.publish_combined = get_parameter("publish_combined").as_bool();
    }
    if (override_names_.count("image_qos_depth") > 0U && get_parameter("image_qos_depth").as_int() > 0) {
      config.ros.image_qos_depth = static_cast<int>(get_parameter("image_qos_depth").as_int());
    }

    return config;
  }

  bool sleep_interruptibly(int duration_ms)
  {
    constexpr int sleep_slice_ms = 50;
    int remaining_ms = std::max(0, duration_ms);
    while (remaining_ms > 0 && !stop_requested_.load() && rclcpp::ok()) {
      const int current_slice_ms = std::min(sleep_slice_ms, remaining_ms);
      std::this_thread::sleep_for(std::chrono::milliseconds(current_slice_ms));
      remaining_ms -= current_slice_ms;
    }
    return !stop_requested_.load() && rclcpp::ok();
  }

  void close_camera()
  {
    if (capture_.isOpened()) {
      capture_.release();
    }
    current_device_path_.clear();
    actual_width_ = 0;
    actual_height_ = 0;
  }

  bool open_camera()
  {
    close_camera();
    const auto matched = find_matching_device(
      config_.camera.preferred_device,
      config_.camera.vendor_id,
      config_.camera.product_id,
      config_.camera.usb_serial);
    if (!matched.has_value()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "No matching stereo camera is present; waiting for USB device %s:%s serial=%s.",
        config_.camera.vendor_id.c_str(), config_.camera.product_id.c_str(),
        config_.camera.usb_serial.c_str());
      return false;
    }

    capture_.open(matched->video_path, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Failed to open stereo camera %s; it may still be resetting or in use.",
        matched->video_path.c_str());
      close_camera();
      return false;
    }

    capture_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc_from_string(config_.camera.pixel_format)));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config_.camera.width));
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config_.camera.height));
    capture_.set(cv::CAP_PROP_FPS, config_.camera.fps);

    actual_width_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    actual_height_ = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
    const int actual_fourcc = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));
    current_device_path_ = matched->video_path;

    if (
      actual_width_ != config_.camera.width || actual_height_ != config_.camera.height ||
      fourcc_to_string(static_cast<unsigned int>(actual_fourcc)) != config_.camera.pixel_format)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Stereo camera rejected requested mode %s %dx%d; driver selected %s %dx%d. "
        "Update the YAML to a mode reported by camera_config_wizard.",
        config_.camera.pixel_format.c_str(), config_.camera.width, config_.camera.height,
        fourcc_to_string(static_cast<unsigned int>(actual_fourcc)).c_str(), actual_width_, actual_height_);
      close_camera();
      return false;
    }

    RCLCPP_INFO(
      get_logger(),
      "Open %s | USB=%s:%s serial=%s | requested=%s %dx%d @ %.3f fps | "
      "actual=%s %dx%d @ %.3f fps | publish limit=%.3f fps",
      matched->video_path.c_str(),
      matched->vendor_id.c_str(),
      matched->product_id.c_str(),
      matched->serial.c_str(),
      config_.camera.pixel_format.c_str(),
      config_.camera.width,
      config_.camera.height,
      config_.camera.fps,
      fourcc_to_string(static_cast<unsigned int>(actual_fourcc)).c_str(),
      actual_width_,
      actual_height_,
      actual_fps,
      config_.camera.publish_fps);
    return true;
  }

  bool valid_frame(const cv::Mat & frame) const
  {
    return !frame.empty() && frame.type() == CV_8UC3 && frame.cols == actual_width_ &&
           frame.rows == actual_height_ && frame.cols >= 2 && frame.rows >= 2;
  }

  std::pair<cv::Mat, cv::Mat> split_frame(const cv::Mat & frame) const
  {
    if (config_.camera.stereo_layout == "vertical") {
      const int half = frame.rows / 2;
      return {
        frame(cv::Rect(0, 0, frame.cols, half)),
        frame(cv::Rect(0, half, frame.cols, frame.rows - half))};
    }
    if (config_.camera.stereo_layout == "mono") {
      return {frame, frame};
    }

    const int half = frame.cols / 2;
    return {
      frame(cv::Rect(0, 0, half, frame.rows)),
      frame(cv::Rect(half, 0, frame.cols - half, frame.rows))};
  }

  void capture_loop()
  {
    const auto publish_period = config_.camera.publish_fps > 0.0 ?
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / config_.camera.publish_fps)) :
      std::chrono::steady_clock::duration::zero();
    auto last_report = std::chrono::steady_clock::now();
    auto next_publish = std::chrono::steady_clock::time_point{};
    int frames_since_report = 0;
    int consecutive_read_failures = 0;

    while (!stop_requested_.load() && rclcpp::ok()) {
      if (!capture_.isOpened()) {
        if (!open_camera()) {
          sleep_interruptibly(config_.camera.reconnect_delay_ms);
          continue;
        }
        consecutive_read_failures = 0;
        frames_since_report = 0;
        last_report = std::chrono::steady_clock::now();
        next_publish = last_report;
      }

      cv::Mat frame;
      bool read_succeeded = false;
      try {
        read_succeeded = capture_.read(frame);
      } catch (const cv::Exception & error) {
        RCLCPP_WARN(get_logger(), "OpenCV failed while reading %s: %s", current_device_path_.c_str(), error.what());
      }

      if (!read_succeeded || !valid_frame(frame)) {
        ++consecutive_read_failures;
        if (consecutive_read_failures == 1) {
          RCLCPP_WARN(
            get_logger(),
            "Stereo camera frame read failed or returned invalid geometry; retrying before reconnect.");
        }
        if (consecutive_read_failures >= std::max(1, config_.camera.read_failure_threshold)) {
          RCLCPP_ERROR(
            get_logger(),
            "Stereo camera %s failed %d consecutive reads; closing stale fd and re-enumerating USB device.",
            current_device_path_.c_str(), consecutive_read_failures);
          close_camera();
          consecutive_read_failures = 0;
          sleep_interruptibly(config_.camera.reconnect_delay_ms);
        } else {
          sleep_interruptibly(config_.camera.read_retry_delay_ms);
        }
        continue;
      }
      consecutive_read_failures = 0;

      const auto captured_at = std::chrono::steady_clock::now();
      if (publish_period > std::chrono::steady_clock::duration::zero()) {
        if (captured_at < next_publish) {
          continue;
        }
        do {
          next_publish += publish_period;
        } while (next_publish <= captured_at);
      }

      const auto stamp = now();
      const auto [left_frame, right_frame] = split_frame(frame);
      const cv::Mat left_square = center_crop_square(left_frame);
      const cv::Mat right_square = center_crop_square(right_frame);

      try {
        if (combined_pub_) {
          if (config_.camera.stereo_layout == "vertical") {
            cv::Mat combined_square;
            cv::vconcat(left_square, right_square, combined_square);
            combined_pub_.publish(frame_to_msg(combined_square, stamp, config_.camera.combined_frame_id));
          } else if (config_.camera.stereo_layout == "mono") {
            combined_pub_.publish(frame_to_msg(left_square, stamp, config_.camera.combined_frame_id));
          } else {
            cv::Mat combined_square;
            cv::hconcat(left_square, right_square, combined_square);
            combined_pub_.publish(frame_to_msg(combined_square, stamp, config_.camera.combined_frame_id));
          }
        }
        if (left_pub_ && right_pub_) {
          left_pub_.publish(frame_to_msg(left_square, stamp, config_.camera.left_frame_id));
          right_pub_.publish(frame_to_msg(right_square, stamp, config_.camera.right_frame_id));
        }
      } catch (const std::exception &) {
        if (stop_requested_.load() || !rclcpp::ok()) {
          break;
        }
        throw;
      }

      ++frames_since_report;
      const auto current = captured_at;
      const auto elapsed = std::chrono::duration<double>(current - last_report).count();
      if (elapsed >= 1.0) {
        std_msgs::msg::Float32 fps_msg;
        fps_msg.data = static_cast<float>(frames_since_report / elapsed);
        fps_pub_->publish(fps_msg);
        RCLCPP_INFO(get_logger(), "Actual publish FPS: %.2f", fps_msg.data);
        frames_since_report = 0;
        last_report = current;
      }
    }
  }

  const std::unordered_set<std::string> override_names_;
  RuntimeConfig config_;
  cv::VideoCapture capture_;
  std::string current_device_path_;
  int actual_width_{0};
  int actual_height_{0};
  std::atomic_bool stop_requested_{false};
  std::thread capture_thread_;
  image_transport::Publisher combined_pub_;
  image_transport::Publisher left_pub_;
  image_transport::Publisher right_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
};

}  // namespace bi_camera_cpp_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<bi_camera_cpp_ros2::StereoCameraNode>();
    rclcpp::spin(node);
  } catch (const rclcpp::exceptions::RCLError &) {
  } catch (const std::exception & error) {
    std::cerr << error.what() << std::endl;
    if (rclcpp::ok()) {
      rclcpp::shutdown();
    }
    return 1;
  }

  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return 0;
}
