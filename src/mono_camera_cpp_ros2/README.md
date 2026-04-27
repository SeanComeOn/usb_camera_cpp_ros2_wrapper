# mono_camera_cpp_ros2

用于 USB 单目相机的 ROS 2 C++ 包，包含 3 个核心能力：

1. 枚举 `/dev/video*`、读取 USB `vendor_id/device_id`，列出支持的分辨率和 FPS，并把选择结果写进 YAML。
2. 以 `MJPG` 模式打开相机，把单目原始图发布到 ROS 2 topic。
3. 基准测试实际可达 FPS。

## 依赖

- ROS 2 Humble
- OpenCV C++
- `yaml-cpp`

## 1. 生成配置 YAML

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
colcon build --packages-select mono_camera_cpp_ros2
source install/setup.bash
ros2 run mono_camera_cpp_ros2 camera_config_wizard --output src/mono_camera_cpp_ros2/config/mono_camera.yaml
```

## 2. 启动相机发布节点

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mono_camera_cpp_ros2 mono_camera.launch.py config_path:=/home/good/projects/robot_projects/hardware/bi_camera/src/mono_camera_cpp_ros2/config/mono_camera.yaml
```

默认发布：

- `image_raw`
- `actual_fps`

默认图像 QoS 使用 `reliable`，这样 RViz 更容易直接看到图。
如果你后面更关心吞吐而不是 RViz 兼容性，可以把 YAML 里的 `image_qos_reliability` 改成 `best_effort`。

## 3. 测试实际 FPS

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run mono_camera_cpp_ros2 camera_fps_benchmark --ros-args -p config_path:=/home/good/projects/robot_projects/hardware/bi_camera/src/mono_camera_cpp_ros2/config/mono_camera.yaml -p duration_sec:=10.0
```

## FAQ

### 1. RViz 里显示 `No Image`，但 topic 明明存在，为什么？

最常见原因是 QoS 不匹配。

例如：

- 节点按 `best_effort` 发布图像
- RViz 的 Image display 按 `reliable` 订阅

这时两边虽然 topic 名一样，但订阅不会真正连上，RViz 就会显示 `No Image`。

### 2. 怎么解决？

有两种常见办法：

1. 在 RViz 里把对应 Image display 的 Reliability 改成 `Best Effort`。
这适合节点当前就是按 `best_effort` 发布的情况。

2. 把节点发布端改成 `reliable`。
在本包里可以直接在 YAML 里设置：

```yaml
ros:
  image_qos_reliability: reliable
```

如果你更关心吞吐，也可以改成：

```yaml
ros:
  image_qos_reliability: best_effort
```

这时记得把 RViz 里的 Reliability 也改成 `Best Effort`。

### 3. 怎么快速判断是不是 QoS 问题？

如果你看到下面这类日志，基本就是 QoS 不兼容：

- `requesting incompatible QoS`
- `No messages will be sent`

另外也可以先确认：

- `ros2 topic list` 能看到 `/image_raw` 或 `/left/image_raw`
- 但 RViz 仍然显示 `No Image`

这种情况优先检查 QoS，而不是先怀疑相机没出图。

### 4. Jetson 上这颗 USB 相机当前识别结果是什么？

这次在 `orin_nano` 上实际探测到的是一颗 USB UVC 相机，不是 CSI 相机。

- `VID:PID = 1e45:0209`
- 设备节点有 `/dev/video0` 和 `/dev/video1`
- 产品名是 `USB Camera`

当前实测可用的典型 `MJPG` 模式包括：

- `1920x1080 @ 30 fps`
- `1280x1024 @ 30 fps`
- `1280x800 @ 30 fps`
- `1280x720 @ 30 fps`
- `640x480 @ 30 fps`

当前默认 YAML 选择的是 `640x480 @ 30 fps`，这样更稳，也更适合先验证链路。

### 5. 为什么有时会报“无法打开相机设备: /dev/video0”？

最常见原因是设备被别的进程占用了。

比如：

- 你已经启动了 `mono_camera_node`
- 同时又跑了 `camera_fps_benchmark`
- 或者别的程序正在占着 `/dev/video0`

这时新的进程就可能打不开相机。

处理办法：

1. 先停掉已经在跑的相机节点。
2. 再启动 benchmark 或新的发布节点。
3. 如果不确定是谁占着，可以在远端执行：

```bash
fuser /dev/video0
```
