# camera_bringup_ros2

这个包只负责启动，不包含相机采集实现。

作用：

- 存放单目和双目相机的稳态默认参数
- 提供同时启动两个相机的 ROS 2 launch
- 提供默认 RViz 配置

当前默认稳态配置基于 Jetson Orin Nano 上的实际验证结果：

- 单目 USB 相机
  - `/dev/video0`
  - `MJPG`
  - `640x480`
  - `30 FPS`
- 双目 USB 相机
  - `/dev/video3`
  - `MJPG`
  - `640x480`
  - `30 FPS`

## 用法

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select camera_bringup_ros2
source install/setup.bash

ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py
```

如果只想起相机，不起 RViz：

```bash
ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py start_rviz:=false
```

## 说明

- 如果 Jetson 重枚举导致双目不再是 `/dev/video3`，需要更新 `config/stereo_stable.yaml`
- 默认 RViz 配置文件在 `rviz/dual_usb_cameras_stable.rviz`
- 这个包依赖：
  - `mono_camera_cpp_ros2`
  - `bi_camera_cpp_ros2`
