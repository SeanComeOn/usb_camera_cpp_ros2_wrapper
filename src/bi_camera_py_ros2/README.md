# bi_camera_py_ros2

用于 USB 双目相机的 ROS 2 包，包含 3 个核心能力：

1. 枚举 `/dev/video*`、读取 USB `vendor_id/device_id`，列出支持的分辨率和 FPS，并把选择结果写进 YAML。
2. 以 `MJPG` 模式打开相机，把拼接双目图发布到 ROS 2 topic，同时拆成左右图。
3. 基准测试实际可达 FPS。

## 依赖

- ROS 2 Humble
- `python3-opencv`
- `PyYAML`

## 1. 生成配置 YAML

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
colcon build --packages-select bi_camera_py_ros2
source install/setup.bash
ros2 run bi_camera_py_ros2 camera_config_wizard --output src/bi_camera_py_ros2/config/bi_camera.yaml
```

向导会：

- 扫描所有可采集的 `/dev/video*`
- 读取 `VID:PID`
- 列出支持的 `MJPG` / `YUYV` 分辨率和 FPS
- 让你交互选择，并写到 YAML

## 2. 启动相机发布节点

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch bi_camera_py_ros2 bi_camera.launch.py config_path:=/home/good/projects/robot_projects/hardware/bi_camera/src/bi_camera_py_ros2/config/bi_camera.yaml
```

默认发布：

- `left/image_raw`
- `right/image_raw`
- `actual_fps`

默认按 `horizontal` 拆分双目，也就是左右拼接图。如果你的相机是上下拼接，可以把 YAML 里的 `stereo_layout` 改成 `vertical`。
如果你还想额外发布完整拼接图，把 YAML 里的 `publish_combined` 改成 `true`。

## 3. 测试实际 FPS

```bash
cd /home/good/projects/robot_projects/hardware/bi_camera
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run bi_camera_py_ros2 camera_fps_benchmark --ros-args -p config_path:=/home/good/projects/robot_projects/hardware/bi_camera/src/bi_camera_py_ros2/config/bi_camera.yaml -p duration_sec:=10.0
```

该节点会直接打开相机，统计一段时间内的实际采样 FPS，并输出平均、最大、最小和中位数 FPS。

## FAQ

### 1. RViz 里显示 `No Image`，但 topic 明明存在，为什么？

最常见原因是 QoS 不匹配。

这个 Python 包的图像发布使用的是 sensor data 风格 QoS，RViz 如果按 `reliable` 去订阅，常常就会看到 `No Image`。

### 2. 怎么解决？

最直接的办法是在 RViz 里把对应 Image display 的 Reliability 改成 `Best Effort`。

如果你看到 topic 存在，但 RViz 没图，先优先检查这个设置。
