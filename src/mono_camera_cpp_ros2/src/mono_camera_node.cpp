#include "mono_camera_cpp_ros2/camera_support.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <unordered_set>

#include <opencv2/opencv.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>

namespace mono_camera_cpp_ros2
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

class MonoCameraNode : public rclcpp::Node
{
public:
  explicit MonoCameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("mono_camera_publisher", options),
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
    declare_parameter<std::string>("image_topic", "");
    declare_parameter<std::string>("fps_topic", "");
    declare_parameter<std::string>("image_qos_reliability", "");
    declare_parameter<int>("image_qos_depth", 0);
    declare_parameter<std::string>("frame_id", "");

    config_ = load_runtime_config();
    image_pub_ = create_publisher<sensor_msgs::msg::Image>(config_.ros.image_topic, make_image_qos(config_));
    fps_pub_ = create_publisher<std_msgs::msg::Float32>(config_.ros.fps_topic, 10);

    open_camera();
    capture_thread_ = std::thread(&MonoCameraNode::capture_loop, this);
  }

  ~MonoCameraNode() override
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
    maybe_override_string("frame_id", config.camera.frame_id);
    maybe_override_string("image_topic", config.ros.image_topic);
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

      try {
        image_pub_->publish(frame_to_msg(frame, now(), config_.camera.frame_id));
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
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr fps_pub_;
};

}  // namespace mono_camera_cpp_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mono_camera_cpp_ros2::MonoCameraNode>();
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
