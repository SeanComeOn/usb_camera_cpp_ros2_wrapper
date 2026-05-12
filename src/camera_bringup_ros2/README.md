# camera_bringup_ros2

这个包只负责启动，不包含相机采集实现。

作用：

- 存放单目和双目相机的稳态默认参数
- 提供同时启动两个相机的 ROS 2 launch
- 提供默认 RViz 配置

当前默认稳态配置基于 Jetson Orin Nano 上的实际验证结果：

- 单目 USB 相机
  - `VID:PID=1e45:0209`
  - `USB serial=CN02KX4NLG0004ABK00`
  - `MJPG`
  - `640x480`
  - `30 FPS`
- 双目 USB 相机
  - `VID:PID=1bcf:0b15`
  - `USB serial=01.00.00`
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

- 默认配置优先通过 `vendor_id / product_id / usb_serial` 匹配设备，`preferred_device` 为空时不会依赖 `/dev/video*` 编号
- 只有在设备身份信息缺失或需要强制指定某个节点时，才建议填写 `preferred_device`
- 默认 RViz 配置文件在 `rviz/dual_usb_cameras_stable.rviz`
- 额外提供了一份“相机 + Franka 本体”的 RViz 模板：
  `rviz/dual_usb_cameras_with_franka.rviz`
- 如果你在 `orin_nano` 上用上层脚本启动 RViz，默认会先复制模板到
  `bi_camera_ws/rviz/remote_*.rviz`，之后你在远端保存的修改会保留，不会在下次同步时被覆盖
- 这个包依赖：
  - `mono_camera_cpp_ros2`
  - `bi_camera_cpp_ros2`
