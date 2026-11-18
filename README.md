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

当前默认稳态参数按 USB 身份匹配，不依赖会随重插变化的 `/dev/videoN`：

- 腕部单目
  - USB `1e45:0209`，序列号 `CN02KX4NLG0004ABK00`
  - `MJPG`
  - `1920x1080 @ 30`（设备当前可验证的最高 30 FPS 模式，发布前中央裁剪为 `1080x1080`）
- `beside` 第三视角双目
  - `beside` USB `0bda:5883`，序列号 `20000001`
  - `MJPG`
  - `1920x540 @ 60`，限速发布 `30 FPS`（左右各发布完整 `540x540`）
  - 默认不重复发布合并图，降低内存带宽与 CPU 负载
- `beside` 腕部第一视角双目
  - 与第三视角双目具有相同的 `0bda:5883 / 20000001` 身份，必须用 USB 物理路径区分
  - `2560x960 @ 30`（左右各发布 `960x960`）

图像 QoS 默认使用 `best_effort`、深度 `2`，避免慢订阅者或网络抖动反向阻塞采集线程。
`beside` 的设备专用配置固定第三视角为 `usb-0:8`、腕部双目为 `usb-0:4`。预览、RViz
和录制默认走 `image_transport/compressed`；五个 raw topic 继续保留给模型和其他工具。

同时启动：

```bash
colcon build --packages-select camera_bringup_ros2
source install/setup.bash
ros2 launch camera_bringup_ros2 dual_usb_cameras_stable.launch.py
```

默认会同时启动：

- 单目相机节点
- 双目相机节点
- 相机流 watchdog：无新数据、USB 断联或帧率持续低于 `27 FPS` 时输出 ROS 2
  `ERROR`（终端红色），并自动重启、重连对应采集进程
- 一份预配置好的 `rviz2`

## Orin USB 稳定性安装

Orin 上首次部署或 USB 拓扑调整后执行：

```bash
cd /home/nv/projects/bi_camera_ws
./scripts/install_orin_camera_usb_stability.sh
```

安装器也可直接在 `beside` 上运行；它会按主机名自动选用
`config/default/beside-usb-camera-guard`。beside 不自动复位 USB2/USB3 根总线，避免影响同总线的
双目、IMU 和键盘。仅允许按 VID/PID 和序列号复位已配置的腕部单目设备。两台机器都会安装同名的
`orin-usb-camera-guard.service`，这是为兼容已有部署保留的历史名称。

安装内容包括：

- 相机和已知上游 USB 2/3 Realtek Hub 的 udev 防休眠规则
- `orin-usb-camera-guard.service` USB 守护服务（不启动相机或 ROS 2）
- Orin 上先按相机端口独立断电重上电，连续失败时再复位共同上游 Hub
- `beside` 上保持防休眠；单目图像过期至少 20 秒、单目发布进程仍在运行，且内核最近 60 秒
  有该设备的 USB `-110` 超时证据时，才执行一次单设备复位。单纯 ROS/预览过期不会复位 USB。
  两次尝试至少间隔 120 秒，连续失败两次后停止尝试；连续健康出帧 60 秒才重新计数。
  设备已消失时仅告警，不猜测设备号，也不升级为共享 Hub 复位。

Orin 的 Hub 恢复每轮失败后等待 `120s` 再尝试；beside 使用上述有次数上限的单设备恢复。
相机一旦重新枚举，ROS 2 采集节点会自动连接并恢复发布。软件复位不等于物理断电：
beside 当前 root hub 报告不支持电源切换；设备卡死且无法枚举时需要现场拔插。
查看日志：

```bash
journalctl -u orin-usb-camera-guard.service -f
```

相机保持手动启动；启动后，launch 内的 watchdog 和节点自重连负责运行期恢复：

```bash
cd /home/nv/projects/bi_camera_ws
START_RVIZ=false BUILD_ON_START=false ./scripts/run_camera_rviz_local.sh
```

验证腕部单目、腕部双目左右眼和第三视角双目左右眼五个 ROS 图像流：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/check_dual_camera_health.py \
  --include-wrist-stereo --duration 30 --min-fps 27 \
  --stereo-width 540 --stereo-height 540 \
  --wrist-stereo-width 960 --wrist-stereo-height 960
```

## 录制五路相机视频

相机节点启动后，可以同时录制腕部单目、腕部双目左右眼和第三视角双目左右眼。

直接在目标机本地启动：

```bash
cd /home/nv/projects/bi_camera_ws
./scripts/record_dual_camera_videos_local.sh
```

默认：

- 第三视角双目：`/stereo/left/image_raw`、`/stereo/right/image_raw`
- 腕部双目：`/wrist/stereo/left/image_raw`、`/wrist/stereo/right/image_raw`
- 腕部单目：`/mono/image_raw`
- 关节 topic：`/joint_states`
- 末端 TF：`fr3_link0 -> fr3_hand_tcp`
- 保存目录：`$HOME/data/camera_recordings/YYYY-MM-DD/HH-MM-SS/`
- 轨迹采样频率：`100 Hz`
- 录像格式：`fr3-multimodal-recording` v4（v1-v3 读取兼容）
- 默认通过标准 `image_transport/compressed` 订阅五路相机，raw topics 保持可用，避免预览、RViz
  和录制同时运行时重复搬运高分辨率原始帧。

稳定配置会把 wrist 相机的 `exposure_dynamic_framerate` 设为 `false`。否则 UVC 自动曝光会在
暗光下把已经协商为 30 FPS 的流降到约 10--15 FPS；需要更亮的画面时应先补光，再考虑开启动态
帧率，不能只看驱动报告的标称 FPS。

录制时按键：

- `r` 开始录制
- `s` 结束并保存
- `d` 丢弃当前录制
- `q` 退出程序

无人值守短录制可设置 `RECORD_SECONDS=10`；它会等待五路图像、自动开始、保存并退出，且不读取
调用脚本的标准输入。

保存内容默认包含：

- `stereo_left.mp4`
- `stereo_right.mp4`
- `wrist_stereo_left.mp4`
- `wrist_stereo_right.mp4`
- `wrist.mp4`
- `combined.mp4`
- `video_frames.csv`
- `trajectory.csv`
- `control_inputs.csv`
- `metadata.json`

v4 的 `metadata.json` 会记录格式/最低读取器版本、请求与实测频率、ROS 时间与单调调度时钟
的职责、轨迹和视频对齐策略，以及关节、末端 TF 和五路图像源的唯一时间戳数量与时间戳年龄。
`trajectory.csv` 同时保存显式 `record_elapsed_sec`、`video_time_sec`、关节消息序号和各源
`*_age_ns`，因此下游不需要从行号猜采样频率，也可以识别“100 行/秒但源状态实际只有 30 Hz”
的伪高频数据；每个关节还保留 position、velocity 与 effort，末端位姿来自录制时的最新 TF。
`control_inputs.csv` 订阅 `/fr3_web_teleop/input`，记录实际送入 FPV worker 的键盘、鼠标、六自由度
和夹爪控制样本，并使用同一 `record_elapsed_sec`/`video_time_sec` 对齐视频。格式定义见
`schemas/recording_metadata_v4.schema.json`。

录完或同步数据后，先验证单条会话或整个数据集根目录：

```bash
python3 scripts/validate_recording_dataset.py \
  "$HOME/data/camera_recordings/2026-08-17/12-00-00"
python3 scripts/validate_recording_dataset.py --json "$HOME/data/camera_recordings"
```

v1-v3 会话仍可通过兼容检查；未知的未来版本会
明确失败，避免处理脚本静默误读新列。v3/v4 默认要求实测频率达到请求值的 `90%`，且源时间戳
年龄不超过 `100 ms`；可用 `--minimum-rate-ratio` 和 `--maximum-source-age-ms` 按经过验证的
部署条件调整。v4 的五路图像分别是腕部单目、腕部双目左右眼和第三视角双目左右眼。

常用覆盖参数示例：

```bash
STEREO_LEFT_TOPIC=/stereo/left/image_raw \
STEREO_RIGHT_TOPIC=/stereo/right/image_raw \
WRIST_TOPIC=/mono/image_raw \
JOINT_STATES_TOPIC=/joint_states \
BASE_FRAME=fr3_link0 \
END_EFFECTOR_FRAME=fr3_hand_tcp \
TRAJECTORY_HZ=100 \
VIDEO_OUTPUT_ROOT=/home/nv/data/camera_recordings \
SESSION_PREFIX=teleop \
./scripts/record_dual_camera_videos_local.sh
```

## 说明

- 工作区级构建产物 `build/`、`install/`、`log/` 已在 `.gitignore` 中忽略
- 每个包自己的详细用法、FAQ 和示例配置请看各自目录下的 `README.md`
