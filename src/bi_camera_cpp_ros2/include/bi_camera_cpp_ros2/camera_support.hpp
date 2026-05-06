#pragma once

#include <optional>
#include <string>
#include <vector>

namespace bi_camera_cpp_ros2
{

struct FrameInterval
{
  bool is_discrete{true};
  double fps{0.0};
  double fps_min{0.0};
  double fps_max{0.0};
  double fps_step{0.0};
};

struct CaptureMode
{
  std::string pixel_format;
  std::string description;
  int width{0};
  int height{0};
  std::vector<FrameInterval> intervals;
};

struct DeviceSummary
{
  std::string video_path;
  std::string name;
  std::string vendor_id;
  std::string product_id;
  std::string serial;
  std::string manufacturer;
  std::string product_name;
  std::string usb_path;
  bool supports_capture{false};
  bool supports_streaming{false};
};

struct CameraConfig
{
  std::string preferred_device;
  std::string vendor_id;
  std::string product_id;
  std::string usb_serial;
  std::string manufacturer;
  std::string product_name;
  std::string pixel_format{"MJPG"};
  int width{1280};
  int height{480};
  double fps{30.0};
  bool split_stereo{true};
  std::string stereo_layout{"horizontal"};
  bool publish_combined{true};
  std::string left_frame_id{"stereo_left_optical_frame"};
  std::string right_frame_id{"stereo_right_optical_frame"};
  std::string combined_frame_id{"stereo_combined_optical_frame"};
};

struct RosConfig
{
  std::string node_name{"bi_camera_publisher"};
  std::string combined_topic{"image_raw"};
  std::string left_topic{"left/image_raw"};
  std::string right_topic{"right/image_raw"};
  std::string fps_topic{"actual_fps"};
  std::string image_qos_reliability{"reliable"};
  int image_qos_depth{10};
};

struct RuntimeConfig
{
  CameraConfig camera;
  RosConfig ros;
};

RuntimeConfig default_config();
RuntimeConfig load_config_file(const std::string & path);
void save_config_file(const std::string & path, const RuntimeConfig & config);

std::vector<DeviceSummary> discover_video_devices();
std::optional<DeviceSummary> find_matching_device(
  const std::string & preferred_device,
  const std::string & vendor_id,
  const std::string & product_id,
  const std::string & serial);
std::vector<CaptureMode> enumerate_capture_modes(const std::string & device_path);

std::string fourcc_to_string(unsigned int code);
int fourcc_from_string(const std::string & code);

}  // namespace bi_camera_cpp_ros2
