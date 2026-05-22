# usb_camera_cpp_ros2_wrapper

一个 ROS 2 相机工作区，当前包含 3 个包，分别对应不同阶段和不同相机形态的封装。

## 包关系

### 1. `mono_camera_cpp_ros2`

单目 USB 相机的 C++ 版封装。

适合：

- 普通 UVC USB 单目相机
- 需要稳定发布 `image_raw`
- 需要在 Jetson 或普通 Ubuntu 主机上跑

当前推荐优先使用这个包。

能力：

- 枚举 `/dev/video*`
- 读取 `vendor_id / product_id`
- 列出分辨率和 FPS 并写入 YAML
- 发布 `/image_raw`
- 发布 `/actual_fps`
- 测实际可达 FPS

目录：

- [src/mono_camera_cpp_ros2](src/mono_camera_cpp_ros2)

### 2. `bi_camera_cpp_ros2`

双目 USB 相机的 C++ 版封装。

适合：

- 左右拼接输出的双目 USB 相机
- 需要同时发布合并图和左右图
- 需要比 Python 版更高的发布性能

能力：

- 枚举 `/dev/video*`
- 读取 `vendor_id / product_id`
- 列出分辨率和 FPS 并写入 YAML
- 发布 `/image_raw`
- 发布 `/left/image_raw`
- 发布 `/right/image_raw`
- 发布 `/actual_fps`
- 测实际可达 FPS

目录：

- [src/bi_camera_cpp_ros2](src/bi_camera_cpp_ros2)

### 3. `bi_camera_py_ros2`

双目 USB 相机的 Python 版封装。

适合：

- 早期原型
- 对照参考
- 想快速看逻辑实现

不建议作为正式高吞吐方案。

原因：

- Python 版虽然能正常采集和发图
- 但双目原始图发布时吞吐明显不如 C++ 版

目录：

- [src/bi_camera_py_ros2](src/bi_camera_py_ros2)

## 推荐选择

- 单目 USB 相机：优先用 `mono_camera_cpp_ros2`
- 双目 USB 相机：优先用 `bi_camera_cpp_ros2`
- Python 版：仅保留作参考和对照
- 如果需要在远端机器上一键同时启动当前验证过的稳定单目+双目组合，优先用 `camera_bringup_ros2`

## 工作区用法

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

只编译某一个包时，例如：

```bash
colcon build --packages-select mono_camera_cpp_ros2
```

## 当前稳定启动组合

仓库里额外提供了一个专门的 bringup 包：

- [src/camera_bringup_ros2](src/camera_bringup_ros2)

这个包只负责 launch 和默认参数，不重复实现采集逻辑。

当前默认稳态参数是：

- 单目
  - `/dev/video0`
  - `MJPG`
  - `640x480 @ 30`
- 双目
  - `/dev/video3`
  - `MJPG`
  - `2560x720 @ 30`

同时启动：

```bash
colcon build --packages-select camera_bringup_ros2
source install/setup.bash
ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py
```

默认会同时启动：

- 单目相机节点
- 双目相机节点
- 一份预配置好的 `rviz2`

## 说明

- 工作区级构建产物 `build/`、`install/`、`log/` 已在 `.gitignore` 中忽略
- 每个包自己的详细用法、FAQ 和示例配置请看各自目录下的 `README.md`
