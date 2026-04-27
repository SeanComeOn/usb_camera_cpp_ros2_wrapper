from __future__ import annotations

import threading
import time
from typing import Any

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32

from .camera_config import load_config_file
from .device_discovery import find_matching_device


def _fourcc_to_int(code: str) -> int:
    return cv2.VideoWriter_fourcc(*code[:4])


class StereoCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("bi_camera_publisher")
        self._declare_parameters()
        self.capture = None
        self.stop_event = threading.Event()
        self.capture_thread: threading.Thread | None = None
        self.frames_since_report = 0
        self.last_report_time = time.monotonic()
        self.parameter_overrides = set(getattr(self, "_parameter_overrides", {}).keys())

        runtime_config = self._load_runtime_config()
        self.camera_cfg = runtime_config["camera"]
        self.ros_cfg = runtime_config["ros"]

        self.combined_pub = None
        if self.camera_cfg["publish_combined"]:
            self.combined_pub = self.create_publisher(Image, self.ros_cfg["combined_topic"], qos_profile_sensor_data)

        self.left_pub = None
        self.right_pub = None
        if self.camera_cfg["split_stereo"]:
            self.left_pub = self.create_publisher(Image, self.ros_cfg["left_topic"], qos_profile_sensor_data)
            self.right_pub = self.create_publisher(Image, self.ros_cfg["right_topic"], qos_profile_sensor_data)
        self.fps_pub = self.create_publisher(Float32, self.ros_cfg["fps_topic"], 10)

        self.capture = self._open_camera()
        self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.capture_thread.start()

    def _declare_parameters(self) -> None:
        self.declare_parameter("config_path", "")
        self.declare_parameter("device_path", "")
        self.declare_parameter("vendor_id", "")
        self.declare_parameter("product_id", "")
        self.declare_parameter("usb_serial", "")
        self.declare_parameter("pixel_format", "")
        self.declare_parameter("width", 0)
        self.declare_parameter("height", 0)
        self.declare_parameter("fps", 0.0)
        self.declare_parameter("split_stereo", True)
        self.declare_parameter("stereo_layout", "")
        self.declare_parameter("publish_combined", True)
        self.declare_parameter("combined_topic", "")
        self.declare_parameter("left_topic", "")
        self.declare_parameter("right_topic", "")
        self.declare_parameter("fps_topic", "")
        self.declare_parameter("left_frame_id", "")
        self.declare_parameter("right_frame_id", "")
        self.declare_parameter("combined_frame_id", "")

    def _load_runtime_config(self) -> dict[str, Any]:
        config_path = self.get_parameter("config_path").get_parameter_value().string_value
        config = load_config_file(config_path)

        camera = config["camera"]
        ros_cfg = config["ros"]

        string_overrides = {
            "preferred_device": self.get_parameter("device_path").value,
            "vendor_id": self.get_parameter("vendor_id").value,
            "product_id": self.get_parameter("product_id").value,
            "usb_serial": self.get_parameter("usb_serial").value,
            "pixel_format": self.get_parameter("pixel_format").value,
            "stereo_layout": self.get_parameter("stereo_layout").value,
            "left_frame_id": self.get_parameter("left_frame_id").value,
            "right_frame_id": self.get_parameter("right_frame_id").value,
            "combined_frame_id": self.get_parameter("combined_frame_id").value,
        }
        for key, value in string_overrides.items():
            if value:
                camera[key] = value

        if self.get_parameter("width").value > 0:
            camera["width"] = int(self.get_parameter("width").value)
        if self.get_parameter("height").value > 0:
            camera["height"] = int(self.get_parameter("height").value)
        if self.get_parameter("fps").value > 0.0:
            camera["fps"] = float(self.get_parameter("fps").value)

        if "split_stereo" in self.parameter_overrides:
            camera["split_stereo"] = bool(self.get_parameter("split_stereo").value)
        if "publish_combined" in self.parameter_overrides:
            camera["publish_combined"] = bool(self.get_parameter("publish_combined").value)

        topic_overrides = {
            "combined_topic": self.get_parameter("combined_topic").value,
            "left_topic": self.get_parameter("left_topic").value,
            "right_topic": self.get_parameter("right_topic").value,
            "fps_topic": self.get_parameter("fps_topic").value,
        }
        for key, value in topic_overrides.items():
            if value:
                ros_cfg[key] = value

        return config

    def _open_camera(self) -> cv2.VideoCapture:
        matched = find_matching_device(
            preferred_device=self.camera_cfg["preferred_device"],
            vendor_id=self.camera_cfg["vendor_id"],
            product_id=self.camera_cfg["product_id"],
            serial=self.camera_cfg["usb_serial"],
        )
        if matched is None:
            raise RuntimeError("没有找到匹配的采集设备，请先运行 camera_config_wizard 更新 YAML。")

        device_path = matched.video_path
        capture = cv2.VideoCapture(device_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"无法打开相机设备: {device_path}")

        pixel_format = str(self.camera_cfg["pixel_format"] or "MJPG")
        capture.set(cv2.CAP_PROP_FOURCC, _fourcc_to_int(pixel_format))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.camera_cfg["width"]))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.camera_cfg["height"]))
        capture.set(cv2.CAP_PROP_FPS, float(self.camera_cfg["fps"]))

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
        actual_fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))

        self.get_logger().info(
            f"Open {device_path} | requested={pixel_format} {self.camera_cfg['width']}x{self.camera_cfg['height']} "
            f"@ {self.camera_cfg['fps']} fps | actual={self._fourcc_string(actual_fourcc)} "
            f"{actual_width}x{actual_height} @ {actual_fps:.3f} fps"
        )
        return capture

    def _fourcc_string(self, code: int) -> str:
        return "".join(chr((code >> (8 * idx)) & 0xFF) for idx in range(4)).strip("\x00")

    def _split_frame(self, frame):
        layout = self.camera_cfg["stereo_layout"]
        if layout == "vertical":
            half = frame.shape[0] // 2
            return frame[:half, :, :], frame[half:, :, :]
        if layout == "mono":
            return frame, frame
        half = frame.shape[1] // 2
        return frame[:, :half, :], frame[:, half:, :]

    def _frame_to_msg(self, frame, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height = int(frame.shape[0])
        msg.width = int(frame.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = int(frame.shape[1] * frame.shape[2])
        msg.data = frame.tobytes()
        return msg

    def _capture_loop(self) -> None:
        while not self.stop_event.is_set() and rclpy.ok():
            ok, frame = self.capture.read()
            if not ok:
                self.get_logger().warning("读取相机帧失败，100ms 后重试。")
                time.sleep(0.1)
                continue

            stamp = self.get_clock().now().to_msg()
            left_frame, right_frame = self._split_frame(frame)

            try:
                if self.combined_pub is not None:
                    combined_msg = self._frame_to_msg(frame, stamp, self.camera_cfg["combined_frame_id"])
                    self.combined_pub.publish(combined_msg)

                if self.left_pub is not None and self.right_pub is not None:
                    left_msg = self._frame_to_msg(left_frame, stamp, self.camera_cfg["left_frame_id"])
                    self.left_pub.publish(left_msg)

                    right_msg = self._frame_to_msg(right_frame, stamp, self.camera_cfg["right_frame_id"])
                    self.right_pub.publish(right_msg)
            except Exception:
                if self.stop_event.is_set() or not rclpy.ok():
                    break
                raise

            self.frames_since_report += 1
            now = time.monotonic()
            elapsed = now - self.last_report_time
            if elapsed >= 1.0:
                actual_fps = self.frames_since_report / elapsed
                if rclpy.ok():
                    self.fps_pub.publish(Float32(data=float(actual_fps)))
                    self.get_logger().info(f"Actual capture FPS: {actual_fps:.2f}")
                self.frames_since_report = 0
                self.last_report_time = now

    def destroy_node(self) -> bool:
        self.stop_event.set()
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        if self.capture is not None:
            self.capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = StereoCameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
