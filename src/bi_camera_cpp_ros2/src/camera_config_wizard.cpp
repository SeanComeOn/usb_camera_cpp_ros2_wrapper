#include "bi_camera_cpp_ros2/camera_support.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace bi_camera_cpp_ros2
{

namespace
{

struct WizardArgs
{
  std::string output{"config/bi_camera.yaml"};
  std::string device;
  std::string pixel_format{"MJPG"};
  std::string layout{"horizontal"};
};

WizardArgs parse_args(int argc, char ** argv)
{
  WizardArgs args;
  for (int i = 1; i < argc; ++i) {
    const std::string current = argv[i];
    auto consume_value = [&](const std::string & flag) -> std::string {
        if (i + 1 >= argc) {
          throw std::runtime_error("缺少参数值: " + flag);
        }
        return argv[++i];
      };

    if (current == "--output") {
      args.output = consume_value(current);
    } else if (current == "--device") {
      args.device = consume_value(current);
    } else if (current == "--pixel-format") {
      args.pixel_format = consume_value(current);
    } else if (current == "--layout") {
      args.layout = consume_value(current);
    } else if (current == "--help" || current == "-h") {
      std::cout << "Usage: camera_config_wizard [--output path] [--device /dev/videoX] "
                << "[--pixel-format MJPG] [--layout horizontal|vertical|mono]\n";
      std::exit(0);
    } else {
      throw std::runtime_error("未知参数: " + current);
    }
  }
  return args;
}

std::size_t prompt_choice(const std::string & prompt, std::size_t upper_bound, std::size_t default_index)
{
  while (true) {
    std::cout << prompt << " [" << (default_index + 1) << "]: ";
    std::string line;
    if (!std::getline(std::cin, line)) {
      return default_index;
    }
    if (line.empty()) {
      return default_index;
    }
    std::istringstream stream(line);
    std::size_t value = 0;
    if ((stream >> value) && value >= 1 && value <= upper_bound) {
      return value - 1;
    }
    std::cout << "请输入 1 到 " << upper_bound << " 之间的编号。\n";
  }
}

std::string prompt_text(const std::string & prompt, const std::string & default_value)
{
  std::cout << prompt << " [" << default_value << "]: ";
  std::string line;
  if (!std::getline(std::cin, line) || line.empty()) {
    return default_value;
  }
  return line;
}

std::size_t default_mode_index(const std::vector<CaptureMode> & modes)
{
  const std::vector<std::pair<int, int>> preferred_sizes = {
    {1280, 480},
    {2560, 720},
    {3840, 1080},
    {640, 480},
  };

  for (const auto & [width, height] : preferred_sizes) {
    for (std::size_t index = 0; index < modes.size(); ++index) {
      const auto & mode = modes[index];
      if (mode.pixel_format != "MJPG" || mode.width != width || mode.height != height) {
        continue;
      }
      for (const auto & interval : mode.intervals) {
        if (interval.is_discrete && std::abs(interval.fps - 30.0) < 1e-6) {
          return index;
        }
      }
    }
  }

  for (std::size_t index = 0; index < modes.size(); ++index) {
    if (modes[index].pixel_format == "MJPG") {
      return index;
    }
  }
  return 0;
}

double default_fps(const std::vector<FrameInterval> & intervals)
{
  for (const auto & interval : intervals) {
    if (interval.is_discrete && std::abs(interval.fps - 30.0) < 1e-6) {
      return 30.0;
    }
  }
  for (const auto & interval : intervals) {
    if (interval.is_discrete) {
      return interval.fps;
    }
  }
  if (!intervals.empty()) {
    return intervals.front().fps_max;
  }
  return 30.0;
}

std::string format_interval(const FrameInterval & interval)
{
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3);
  if (interval.is_discrete) {
    stream << interval.fps << " fps";
  } else {
    stream << "range " << interval.fps_min << " - " << interval.fps_max << " fps step " << interval.fps_step;
  }
  return stream.str();
}

DeviceSummary select_device(const std::vector<DeviceSummary> & devices, const std::string & requested_path)
{
  std::vector<DeviceSummary> capture_devices;
  for (const auto & device : devices) {
    if (device.supports_capture) {
      capture_devices.push_back(device);
    }
  }

  if (capture_devices.empty()) {
    throw std::runtime_error("没有发现可用于采集的 /dev/video* 设备。");
  }

  if (!requested_path.empty()) {
    for (const auto & device : capture_devices) {
      if (device.video_path == requested_path) {
        return device;
      }
    }
    throw std::runtime_error("指定设备不存在或不可采集: " + requested_path);
  }

  std::cout << "发现以下可采集视频设备：\n";
  for (std::size_t index = 0; index < capture_devices.size(); ++index) {
    const auto & device = capture_devices[index];
    std::cout << "  " << (index + 1) << ". " << device.video_path << " | " << device.name
              << " | VID:PID=" << device.vendor_id << ":" << device.product_id
              << " | " << device.manufacturer << " " << device.product_name << "\n";
  }

  return capture_devices[prompt_choice("请选择相机设备编号", capture_devices.size(), 0)];
}

std::pair<CaptureMode, double> select_mode(const DeviceSummary & device, const std::string & requested_pixel_format)
{
  std::vector<CaptureMode> filtered;
  for (const auto & mode : enumerate_capture_modes(device.video_path)) {
    if (requested_pixel_format.empty() || mode.pixel_format == requested_pixel_format) {
      filtered.push_back(mode);
    }
  }

  if (filtered.empty()) {
    throw std::runtime_error(device.video_path + " 不支持像素格式 " + requested_pixel_format);
  }

  std::cout << "\n" << device.video_path << " 支持的模式如下：\n";
  for (std::size_t index = 0; index < filtered.size(); ++index) {
    std::ostringstream fps_text;
    for (std::size_t interval_index = 0; interval_index < filtered[index].intervals.size(); ++interval_index) {
      if (interval_index > 0) {
        fps_text << ", ";
      }
      fps_text << format_interval(filtered[index].intervals[interval_index]);
    }
    std::cout << "  " << (index + 1) << ". "
              << filtered[index].pixel_format << " "
              << filtered[index].width << "x" << filtered[index].height
              << " | " << filtered[index].description
              << " | " << fps_text.str() << "\n";
  }

  const std::size_t mode_index = prompt_choice(
    "请选择分辨率/格式编号", filtered.size(), default_mode_index(filtered));
  const auto & mode = filtered[mode_index];
  std::ostringstream default_fps_text;
  default_fps_text << std::fixed << std::setprecision(3) << default_fps(mode.intervals);
  const double fps = std::stod(prompt_text("请输入目标 FPS", default_fps_text.str()));
  return {mode, fps};
}

}  // namespace

}  // namespace bi_camera_cpp_ros2

int main(int argc, char ** argv)
{
  using namespace bi_camera_cpp_ros2;

  try {
    const WizardArgs args = parse_args(argc, argv);
    const auto devices = discover_video_devices();
    const DeviceSummary device = select_device(devices, args.device);
    auto [mode, fps] = select_mode(device, args.pixel_format);

    RuntimeConfig config = default_config();
    config.camera.preferred_device = device.video_path;
    config.camera.vendor_id = device.vendor_id;
    config.camera.product_id = device.product_id;
    config.camera.usb_serial = device.serial;
    config.camera.manufacturer = device.manufacturer;
    config.camera.product_name = device.product_name;
    config.camera.pixel_format = mode.pixel_format;
    config.camera.width = mode.width;
    config.camera.height = mode.height;
    config.camera.fps = fps;
    config.camera.split_stereo = true;
    config.camera.stereo_layout = args.layout;
    config.camera.publish_combined = true;

    save_config_file(args.output, config);

    std::cout << "\n已写入配置: " << fs::absolute(args.output) << "\n";
    std::cout << "选择设备: " << device.video_path << " | VID:PID=" << device.vendor_id
              << ":" << device.product_id << "\n";
    std::cout << "选择模式: " << mode.pixel_format << " " << mode.width << "x" << mode.height
              << " @ " << std::fixed << std::setprecision(3) << fps
              << " fps | layout=" << args.layout << "\n";
  } catch (const std::exception & error) {
    std::cerr << error.what() << "\n";
    return 1;
  }

  return 0;
}
