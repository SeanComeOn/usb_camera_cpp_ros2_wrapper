#include "bi_camera_cpp_ros2/camera_support.hpp"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace fs = std::filesystem;

namespace bi_camera_cpp_ros2
{

namespace
{

struct CapabilitySummary
{
  bool supports_capture{false};
  bool supports_streaming{false};
};

std::string trim_copy(const std::string & value)
{
  const auto begin = value.find_first_not_of(" \t\r\n");
  if (begin == std::string::npos) {
    return "";
  }
  const auto end = value.find_last_not_of(" \t\r\n");
  return value.substr(begin, end - begin + 1);
}

std::string lowercase_copy(const std::string & value)
{
  std::string result = value;
  std::transform(result.begin(), result.end(), result.begin(), [](unsigned char c) {return std::tolower(c);});
  return result;
}

std::string read_text_file(const fs::path & path)
{
  std::ifstream stream(path);
  if (!stream.is_open()) {
    return "";
  }
  std::string content((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return trim_copy(content);
}

fs::path find_usb_root(const fs::path & device_path)
{
  std::error_code ec;
  fs::path current = fs::weakly_canonical(device_path, ec);
  if (ec) {
    current = device_path;
  }

  while (!current.empty() && current != current.parent_path()) {
    if (fs::exists(current / "idVendor") && fs::exists(current / "idProduct")) {
      return current;
    }
    current = current.parent_path();
  }
  return {};
}

CapabilitySummary query_capabilities(const std::string & device_path)
{
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("Failed to open " + device_path);
  }

  v4l2_capability capability{};
  if (::ioctl(fd, VIDIOC_QUERYCAP, &capability) < 0) {
    ::close(fd);
    throw std::runtime_error("VIDIOC_QUERYCAP failed for " + device_path);
  }
  ::close(fd);

  const uint32_t caps = (capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0U ?
    capability.device_caps : capability.capabilities;

  CapabilitySummary summary;
  summary.supports_capture =
    (caps & V4L2_CAP_VIDEO_CAPTURE) != 0U || (caps & V4L2_CAP_VIDEO_CAPTURE_MPLANE) != 0U;
  summary.supports_streaming = (caps & V4L2_CAP_STREAMING) != 0U;
  return summary;
}

double fract_to_fps(const v4l2_fract & fract)
{
  if (fract.numerator == 0U) {
    return 0.0;
  }
  return static_cast<double>(fract.denominator) / static_cast<double>(fract.numerator);
}

std::vector<FrameInterval> enumerate_frame_intervals(int fd, uint32_t pixel_format, int width, int height)
{
  std::vector<FrameInterval> intervals;
  for (uint32_t index = 0;; ++index) {
    v4l2_frmivalenum interval{};
    interval.index = index;
    interval.pixel_format = pixel_format;
    interval.width = static_cast<uint32_t>(width);
    interval.height = static_cast<uint32_t>(height);

    if (::ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &interval) < 0) {
      if (errno == EINVAL) {
        break;
      }
      throw std::runtime_error("VIDIOC_ENUM_FRAMEINTERVALS failed");
    }

    if (interval.type == V4L2_FRMIVAL_TYPE_DISCRETE) {
      const double fps = fract_to_fps(interval.discrete);
      if (fps > 0.0) {
        FrameInterval value;
        value.is_discrete = true;
        value.fps = fps;
        intervals.push_back(value);
      }
    } else {
      FrameInterval value;
      value.is_discrete = false;
      value.fps_min = fract_to_fps(interval.stepwise.max);
      value.fps_max = fract_to_fps(interval.stepwise.min);
      value.fps_step = fract_to_fps(interval.stepwise.step);
      intervals.push_back(value);
    }
  }

  std::sort(intervals.begin(), intervals.end(), [](const FrameInterval & lhs, const FrameInterval & rhs) {
    const double lhs_value = lhs.is_discrete ? lhs.fps : lhs.fps_max;
    const double rhs_value = rhs.is_discrete ? rhs.fps : rhs.fps_max;
    return lhs_value > rhs_value;
  });
  return intervals;
}

void append_stepwise_sizes(
  int fd,
  uint32_t pixel_format,
  const v4l2_frmsize_stepwise & stepwise,
  std::vector<CaptureMode> & modes,
  const std::string & pixel_format_name,
  const std::string & description)
{
  const int step_width = std::max(1U, stepwise.step_width);
  const int step_height = std::max(1U, stepwise.step_height);

  for (int width = static_cast<int>(stepwise.min_width);
    width <= static_cast<int>(stepwise.max_width);
    width += step_width)
  {
    for (int height = static_cast<int>(stepwise.min_height);
      height <= static_cast<int>(stepwise.max_height);
      height += step_height)
    {
      auto intervals = enumerate_frame_intervals(fd, pixel_format, width, height);
      if (intervals.empty()) {
        continue;
      }
      CaptureMode mode;
      mode.pixel_format = pixel_format_name;
      mode.description = description;
      mode.width = width;
      mode.height = height;
      mode.intervals = std::move(intervals);
      modes.push_back(mode);
    }
  }
}

template<typename T>
void apply_if_present(const YAML::Node & node, const char * key, T & output)
{
  if (node[key]) {
    output = node[key].as<T>();
  }
}

}  // namespace

RuntimeConfig default_config()
{
  return RuntimeConfig{};
}

RuntimeConfig load_config_file(const std::string & path)
{
  RuntimeConfig config = default_config();
  if (path.empty()) {
    return config;
  }

  if (!fs::exists(path)) {
    throw std::runtime_error("Config file not found: " + path);
  }

  const YAML::Node root = YAML::LoadFile(path);
  const YAML::Node camera = root["camera"];
  const YAML::Node ros = root["ros"];

  if (camera) {
    apply_if_present(camera, "preferred_device", config.camera.preferred_device);
    apply_if_present(camera, "vendor_id", config.camera.vendor_id);
    apply_if_present(camera, "product_id", config.camera.product_id);
    apply_if_present(camera, "usb_serial", config.camera.usb_serial);
    apply_if_present(camera, "manufacturer", config.camera.manufacturer);
    apply_if_present(camera, "product_name", config.camera.product_name);
    apply_if_present(camera, "pixel_format", config.camera.pixel_format);
    apply_if_present(camera, "width", config.camera.width);
    apply_if_present(camera, "height", config.camera.height);
    apply_if_present(camera, "fps", config.camera.fps);
    apply_if_present(camera, "split_stereo", config.camera.split_stereo);
    apply_if_present(camera, "stereo_layout", config.camera.stereo_layout);
    apply_if_present(camera, "publish_combined", config.camera.publish_combined);
    apply_if_present(camera, "left_frame_id", config.camera.left_frame_id);
    apply_if_present(camera, "right_frame_id", config.camera.right_frame_id);
    apply_if_present(camera, "combined_frame_id", config.camera.combined_frame_id);
  }

  if (ros) {
    apply_if_present(ros, "node_name", config.ros.node_name);
    apply_if_present(ros, "combined_topic", config.ros.combined_topic);
    apply_if_present(ros, "left_topic", config.ros.left_topic);
    apply_if_present(ros, "right_topic", config.ros.right_topic);
    apply_if_present(ros, "fps_topic", config.ros.fps_topic);
    apply_if_present(ros, "image_qos_reliability", config.ros.image_qos_reliability);
    apply_if_present(ros, "image_qos_depth", config.ros.image_qos_depth);
  }

  return config;
}

void save_config_file(const std::string & path, const RuntimeConfig & config)
{
  fs::create_directories(fs::path(path).parent_path());

  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "preferred_device" << YAML::Value << config.camera.preferred_device;
  out << YAML::Key << "vendor_id" << YAML::Value << config.camera.vendor_id;
  out << YAML::Key << "product_id" << YAML::Value << config.camera.product_id;
  out << YAML::Key << "usb_serial" << YAML::Value << config.camera.usb_serial;
  out << YAML::Key << "manufacturer" << YAML::Value << config.camera.manufacturer;
  out << YAML::Key << "product_name" << YAML::Value << config.camera.product_name;
  out << YAML::Key << "pixel_format" << YAML::Value << config.camera.pixel_format;
  out << YAML::Key << "width" << YAML::Value << config.camera.width;
  out << YAML::Key << "height" << YAML::Value << config.camera.height;
  out << YAML::Key << "fps" << YAML::Value << config.camera.fps;
  out << YAML::Key << "split_stereo" << YAML::Value << config.camera.split_stereo;
  out << YAML::Key << "stereo_layout" << YAML::Value << config.camera.stereo_layout;
  out << YAML::Key << "publish_combined" << YAML::Value << config.camera.publish_combined;
  out << YAML::Key << "left_frame_id" << YAML::Value << config.camera.left_frame_id;
  out << YAML::Key << "right_frame_id" << YAML::Value << config.camera.right_frame_id;
  out << YAML::Key << "combined_frame_id" << YAML::Value << config.camera.combined_frame_id;
  out << YAML::EndMap;

  out << YAML::Key << "ros" << YAML::Value << YAML::BeginMap;
  out << YAML::Key << "node_name" << YAML::Value << config.ros.node_name;
  out << YAML::Key << "combined_topic" << YAML::Value << config.ros.combined_topic;
  out << YAML::Key << "left_topic" << YAML::Value << config.ros.left_topic;
  out << YAML::Key << "right_topic" << YAML::Value << config.ros.right_topic;
  out << YAML::Key << "fps_topic" << YAML::Value << config.ros.fps_topic;
  out << YAML::Key << "image_qos_reliability" << YAML::Value << config.ros.image_qos_reliability;
  out << YAML::Key << "image_qos_depth" << YAML::Value << config.ros.image_qos_depth;
  out << YAML::EndMap;
  out << YAML::EndMap;

  std::ofstream stream(path);
  if (!stream.is_open()) {
    throw std::runtime_error("Failed to write config file: " + path);
  }
  stream << out.c_str() << '\n';
}

std::vector<DeviceSummary> discover_video_devices()
{
  std::vector<DeviceSummary> devices;
  const fs::path root("/sys/class/video4linux");
  if (!fs::exists(root)) {
    return devices;
  }

  std::vector<fs::path> entries;
  for (const auto & entry : fs::directory_iterator(root)) {
    if (entry.is_directory()) {
      entries.push_back(entry.path());
    }
  }
  std::sort(entries.begin(), entries.end());

  for (const auto & video_dir : entries) {
    const std::string video_name = video_dir.filename().string();
    const std::string video_path = "/dev/" + video_name;
    if (!fs::exists(video_path)) {
      continue;
    }

    CapabilitySummary caps;
    try {
      caps = query_capabilities(video_path);
    } catch (const std::exception &) {
      continue;
    }

    const fs::path usb_root = find_usb_root(video_dir / "device");
    DeviceSummary summary;
    summary.video_path = video_path;
    summary.name = read_text_file(video_dir / "name");
    summary.vendor_id = usb_root.empty() ? "" : read_text_file(usb_root / "idVendor");
    summary.product_id = usb_root.empty() ? "" : read_text_file(usb_root / "idProduct");
    summary.serial = usb_root.empty() ? "" : read_text_file(usb_root / "serial");
    summary.manufacturer = usb_root.empty() ? "" : read_text_file(usb_root / "manufacturer");
    summary.product_name = usb_root.empty() ? "" : read_text_file(usb_root / "product");
    summary.usb_path = usb_root.empty() ? "" : usb_root.string();
    summary.supports_capture = caps.supports_capture;
    summary.supports_streaming = caps.supports_streaming;
    devices.push_back(summary);
  }

  return devices;
}

std::optional<DeviceSummary> find_matching_device(
  const std::string & preferred_device,
  const std::string & vendor_id,
  const std::string & product_id,
  const std::string & serial)
{
  const auto devices = discover_video_devices();

  if (!preferred_device.empty()) {
    for (const auto & device : devices) {
      if (device.video_path == preferred_device && device.supports_capture) {
        return device;
      }
    }
  }

  for (const auto & device : devices) {
    if (!device.supports_capture) {
      continue;
    }
    if (!vendor_id.empty() && lowercase_copy(device.vendor_id) != lowercase_copy(vendor_id)) {
      continue;
    }
    if (!product_id.empty() && lowercase_copy(device.product_id) != lowercase_copy(product_id)) {
      continue;
    }
    if (!serial.empty() && !device.serial.empty() && device.serial != serial) {
      continue;
    }
    if (!vendor_id.empty() || !product_id.empty() || !serial.empty()) {
      return device;
    }
  }

  for (const auto & device : devices) {
    if (device.supports_capture) {
      return device;
    }
  }
  return std::nullopt;
}

std::vector<CaptureMode> enumerate_capture_modes(const std::string & device_path)
{
  const int fd = ::open(device_path.c_str(), O_RDWR | O_NONBLOCK);
  if (fd < 0) {
    throw std::runtime_error("Failed to open " + device_path + " for mode enumeration");
  }

  std::vector<CaptureMode> modes;
  try {
    for (uint32_t index = 0;; ++index) {
      v4l2_fmtdesc format{};
      format.index = index;
      format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

      if (::ioctl(fd, VIDIOC_ENUM_FMT, &format) < 0) {
        if (errno == EINVAL) {
          break;
        }
        throw std::runtime_error("VIDIOC_ENUM_FMT failed");
      }

      const std::string pixel_format_name = fourcc_to_string(format.pixelformat);
      const std::string description = reinterpret_cast<char *>(format.description);

      for (uint32_t size_index = 0;; ++size_index) {
        v4l2_frmsizeenum size{};
        size.index = size_index;
        size.pixel_format = format.pixelformat;

        if (::ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &size) < 0) {
          if (errno == EINVAL) {
            break;
          }
          throw std::runtime_error("VIDIOC_ENUM_FRAMESIZES failed");
        }

        if (size.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
          CaptureMode mode;
          mode.pixel_format = pixel_format_name;
          mode.description = description;
          mode.width = static_cast<int>(size.discrete.width);
          mode.height = static_cast<int>(size.discrete.height);
          mode.intervals = enumerate_frame_intervals(fd, format.pixelformat, mode.width, mode.height);
          modes.push_back(mode);
        } else {
          append_stepwise_sizes(fd, format.pixelformat, size.stepwise, modes, pixel_format_name, description);
        }
      }
    }
  } catch (...) {
    ::close(fd);
    throw;
  }

  ::close(fd);

  std::sort(modes.begin(), modes.end(), [](const CaptureMode & lhs, const CaptureMode & rhs) {
    const int lhs_priority = lhs.pixel_format == "MJPG" ? 0 : 1;
    const int rhs_priority = rhs.pixel_format == "MJPG" ? 0 : 1;
    if (lhs_priority != rhs_priority) {
      return lhs_priority < rhs_priority;
    }
    if (lhs.width != rhs.width) {
      return lhs.width > rhs.width;
    }
    return lhs.height > rhs.height;
  });
  return modes;
}

std::string fourcc_to_string(unsigned int code)
{
  std::string result(4, '\0');
  result[0] = static_cast<char>(code & 0xFFU);
  result[1] = static_cast<char>((code >> 8U) & 0xFFU);
  result[2] = static_cast<char>((code >> 16U) & 0xFFU);
  result[3] = static_cast<char>((code >> 24U) & 0xFFU);
  return trim_copy(result);
}

int fourcc_from_string(const std::string & code)
{
  const std::string padded = (code + "    ").substr(0, 4);
  return v4l2_fourcc(padded[0], padded[1], padded[2], padded[3]);
}

}  // namespace bi_camera_cpp_ros2
