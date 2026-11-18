# camera_bringup_ros2

这个包只负责启动，不包含相机采集实现。

作用：

- 存放单目和双目相机的稳态默认参数
- 提供同时启动两个相机的 ROS 2 launch
- 提供默认 RViz 配置

`beside` 同时使用三台相机：腕部单目、腕部双目、第三视角双目。两台双目拥有完全相同的
VID/PID 和 USB serial，因此 `beside_third_person_stereo_5883.yaml` 与
`beside_wrist_stereo_5883.yaml` 必须分别通过 `usb-0:8`、`usb-0:4` 物理路径绑定，不能使用
会冲突的 `/dev/v4l/by-id` 双目链接。

当前默认稳态配置基于 Jetson Orin Nano 上的实际验证结果：

- 单目 USB 相机
  - `VID:PID=1e45:0209`
  - `USB serial=CN02KX4NLG0004ABK00`
  - `MJPG`
  - `1920x1080`
  - `30 FPS`
  - 发布前中央裁剪为 `1080x1080`
- 双目 USB 相机
  - `VID:PID=1bcf:0b15`
  - `USB serial=01.00.00`
  - `MJPG`
  - `2560x720`
  - `30 FPS`
  - 左右图原始拆分为 `1280x720` 两路，发布前各自裁成 `720x720`
  - 默认仅发布左右图，不重复发布合并图

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

launch 默认同时启动 `camera_stream_watchdog`。watchdog 使用轻量的
`/mono/actual_fps`、`/wrist/stereo/actual_fps` 和 `/stereo/actual_fps` 心跳，不额外订阅大尺寸原始图像：

- 连续 `5s` 没有新 FPS 心跳：ROS 2 `ERROR`，并自动重启对应采集节点；
- 实际帧率连续 `10s` 低于 `27 FPS`：ROS 2 `ERROR`，并自动重启、重连；
- USB 设备没有枚举：每 `10s` 输出 ROS 2 `ERROR`，等待系统 USB guard 恢复端口；
- 节点退出：launch 在 `2s` 后自动 respawn。

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
