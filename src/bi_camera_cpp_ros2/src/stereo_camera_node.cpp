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
    const auto image_qos = make_image_qos(config_);

    if (config_.camera.publish_combined) {
      combined_pub_ = create_publisher<sensor_msgs::msg::Image>(
        config_.ros.combined_topic, image_qos);
    }
    if (config_.camera.split_stereo) {
      left_pub_ = create_publisher<sensor_msgs::msg::Image>(
        config_.ros.left_topic, image_qos);
      right_pub_ = create_publisher<sensor_msgs::msg::Image>(
        config_.ros.right_topic, image_qos);
    }
    fps_pub_ = create_publisher<std_msgs::msg::Float32>(config_.ros.fps_topic, 10);

    open_camera();
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
        if (override_names_.count(name) == 0U) {
          return;
        }
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

  void open_camera()
  {
    const auto matched = find_matching_device(
      config_.camera.preferred_device,
      config_.camera.vendor_id,
      config_.camera.product_id,
      config_.camera.usb_serial);
    if (!matched.has_value()) {
      throw std::runtime_error("没有找到匹配的采集设备，请先运行 camera_config_wizard 更新 YAML。");
    }

    capture_.open(matched->video_path, cv::CAP_V4L2);
    if (!capture_.isOpened()) {
      throw std::runtime_error("无法打开相机设备: " + matched->video_path);
    }

    capture_.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc_from_string(config_.camera.pixel_format)));
    capture_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config_.camera.width));
    capture_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config_.camera.height));
    capture_.set(cv::CAP_PROP_FPS, config_.camera.fps);

    const int actual_width = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_height = static_cast<int>(capture_.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actual_fps = capture_.get(cv::CAP_PROP_FPS);
    const int actual_fourcc = static_cast<int>(capture_.get(cv::CAP_PROP_FOURCC));

    RCLCPP_INFO(
      get_logger(),
      "Open %s | requested=%s %dx%d @ %.3f fps | actual=%s %dx%d @ %.3f fps",
      matched->video_path.c_str(),
      config_.camera.pixel_format.c_str(),
      config_.camera.width,
      config_.camera.height,
      config_.camera.fps,
      fourcc_to_string(static_cast<unsigned int>(actual_fourcc)).c_str(),
      actual_width,
      actual_height,
      actual_fps);
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
    auto last_report = std::chrono::steady_clock::now();
    int frames_since_report = 0;

    while (!stop_requested_.load() && rclcpp::ok()) {
      cv::Mat frame;
      if (!capture_.read(frame)) {
        RCLCPP_WARN(get_logger(), "读取相机帧失败，100ms 后重试。");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
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
            combined_pub_->publish(frame_to_msg(combined_square, stamp, config_.camera.combined_frame_id));
          } else if (config_.camera.stereo_layout == "mono") {
            combined_pub_->publish(frame_to_msg(left_square, stamp, config_.camera.combined_frame_id));
          } else {
            cv::Mat combined_square;
            cv::hconcat(left_square, right_square, combined_square);
            combined_pub_->publish(frame_to_msg(combined_square, stamp, config_.camera.combined_frame_id));
          }
        }
        if (left_pub_ && right_pub_) {
          left_pub_->publish(frame_to_msg(left_square, stamp, config_.camera.left_frame_id));
          right_pub_->publish(frame_to_msg(right_square, stamp, config_.camera.right_frame_id));
        }
      } catch (const std::exception &) {
        if (stop_requested_.load() || !rclcpp::ok()) {
          break;
        }
        throw;
      }

      ++frames_since_report;
      const auto current = std::chrono::steady_clock::now();
      const auto elapsed = std::chrono::duration<double>(current - last_report).count();
      if (elapsed >= 1.0) {
        std_msgs::msg::Float32 fps_msg;
        fps_msg.data = static_cast<float>(frames_since_report / elapsed);
        fps_pub_->publish(fps_msg);
        RCLCPP_INFO(get_logger(), "Actual capture FPS: %.2f", fps_msg.data);
        frames_since_report = 0;
        last_report = current;
      }
    }
  }

  const std::unordered_set<std::string> override_names_;
  RuntimeConfig config_;
  cv::VideoCapture capture_;
  std::atomic_bool stop_requested_{false};
  std::thread capture_thread_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr combined_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr left_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr right_pub_;
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
