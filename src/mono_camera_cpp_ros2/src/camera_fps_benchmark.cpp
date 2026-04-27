#include "mono_camera_cpp_ros2/camera_support.hpp"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <memory>
#include <numeric>
#include <stdexcept>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mono_camera_cpp_ros2
{

class CameraFpsBenchmarkNode : public rclcpp::Node
{
public:
  CameraFpsBenchmarkNode()
  : Node("mono_camera_fps_benchmark")
  {
    declare_parameter<std::string>("config_path", "");
    declare_parameter<double>("duration_sec", 10.0);
    declare_parameter<double>("warmup_sec", 1.0);
  }

  void run()
  {
    const RuntimeConfig config = load_config_file(get_parameter("config_path").as_string());
    const auto matched = find_matching_device(
      config.camera.preferred_device,
      config.camera.vendor_id,
      config.camera.product_id,
      config.camera.usb_serial);
    if (!matched.has_value()) {
      throw std::runtime_error("没有找到匹配的采集设备，请先运行 camera_config_wizard 更新 YAML。");
    }

    cv::VideoCapture capture(matched->video_path, cv::CAP_V4L2);
    if (!capture.isOpened()) {
      throw std::runtime_error("无法打开相机设备: " + matched->video_path);
    }

    capture.set(cv::CAP_PROP_FOURCC, static_cast<double>(fourcc_from_string(config.camera.pixel_format)));
    capture.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(config.camera.width));
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(config.camera.height));
    capture.set(cv::CAP_PROP_FPS, config.camera.fps);

    const int actual_width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
    const int actual_height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
    const double actual_fps = capture.get(cv::CAP_PROP_FPS);
    RCLCPP_INFO(
      get_logger(),
      "Benchmark start on %s | actual=%dx%d @ %.3f fps",
      matched->video_path.c_str(),
      actual_width,
      actual_height,
      actual_fps);

    const auto warmup_deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(get_parameter("warmup_sec").as_double());
    while (std::chrono::steady_clock::now() < warmup_deadline) {
      cv::Mat frame;
      capture.read(frame);
    }

    std::vector<double> timestamps;
    const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::duration<double>(get_parameter("duration_sec").as_double());
    while (std::chrono::steady_clock::now() < deadline) {
      cv::Mat frame;
      if (!capture.read(frame)) {
        RCLCPP_WARN(get_logger(), "读取帧失败，跳过。");
        continue;
      }
      timestamps.push_back(std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count());
    }

    capture.release();

    if (timestamps.size() < 2U) {
      throw std::runtime_error("采样帧数不足，无法计算 FPS。");
    }

    std::vector<double> intervals;
    intervals.reserve(timestamps.size() - 1U);
    for (std::size_t i = 1; i < timestamps.size(); ++i) {
      intervals.push_back(timestamps[i] - timestamps[i - 1]);
    }

    const double avg_fps = static_cast<double>(timestamps.size() - 1U) /
      (timestamps.back() - timestamps.front());
    const double min_interval = *std::min_element(intervals.begin(), intervals.end());
    const double max_interval = *std::max_element(intervals.begin(), intervals.end());

    std::sort(intervals.begin(), intervals.end());
    const std::size_t middle = intervals.size() / 2U;
    const double median_interval = intervals.size() % 2U == 0U ?
      (intervals[middle - 1U] + intervals[middle]) / 2.0 : intervals[middle];

    RCLCPP_INFO(
      get_logger(),
      "Frames captured: %zu in %.3f sec",
      timestamps.size(),
      timestamps.back() - timestamps.front());
    RCLCPP_INFO(get_logger(), "Average FPS: %.3f", avg_fps);
    RCLCPP_INFO(get_logger(), "Median FPS : %.3f", 1.0 / median_interval);
    RCLCPP_INFO(get_logger(), "Min FPS    : %.3f", 1.0 / max_interval);
    RCLCPP_INFO(get_logger(), "Max FPS    : %.3f", 1.0 / min_interval);
  }
};

}  // namespace mono_camera_cpp_ros2

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<mono_camera_cpp_ros2::CameraFpsBenchmarkNode>();
    node->run();
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
