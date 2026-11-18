#pragma once

#include <optional>
#include <string>
#include <vector>

namespace mono_camera_cpp_ros2
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
  int width{640};
  int height{480};
  double fps{30.0};
  bool exposure_dynamic_framerate{false};
  int read_failure_threshold{5};
  int read_retry_delay_ms{100};
  int reconnect_delay_ms{1000};
  std::string frame_id{"mono_camera_optical_frame"};
};

struct RosConfig
{
  std::string node_name{"mono_camera_publisher"};
  std::string image_topic{"image_raw"};
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
void set_exposure_dynamic_framerate(const std::string & device_path, bool enabled);

std::string fourcc_to_string(unsigned int code);
int fourcc_from_string(const std::string & code);

}  // namespace mono_camera_cpp_ros2
