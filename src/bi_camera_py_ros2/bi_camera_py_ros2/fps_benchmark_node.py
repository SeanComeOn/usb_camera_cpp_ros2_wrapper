from __future__ import annotations

import statistics
import time

import cv2
import rclpy
from rclpy.node import Node

from .camera_config import load_config_file
from .device_discovery import find_matching_device


def _fourcc_to_int(code: str) -> int:
    return cv2.VideoWriter_fourcc(*code[:4])


class CameraFpsBenchmarkNode(Node):
    def __init__(self) -> None:
        super().__init__("bi_camera_fps_benchmark")
        self.declare_parameter("config_path", "")
        self.declare_parameter("duration_sec", 10.0)
        self.declare_parameter("warmup_sec", 1.0)

    def run(self) -> None:
        config_path = self.get_parameter("config_path").value
        duration_sec = float(self.get_parameter("duration_sec").value)
        warmup_sec = float(self.get_parameter("warmup_sec").value)
        config = load_config_file(config_path)
        camera = config["camera"]

        matched = find_matching_device(
            preferred_device=camera["preferred_device"],
            vendor_id=camera["vendor_id"],
            product_id=camera["product_id"],
            serial=camera["usb_serial"],
        )
        if matched is None:
            raise RuntimeError("没有找到匹配的采集设备，请先运行 camera_config_wizard 更新 YAML。")

        capture = cv2.VideoCapture(matched.video_path, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise RuntimeError(f"无法打开相机设备: {matched.video_path}")

        capture.set(cv2.CAP_PROP_FOURCC, _fourcc_to_int(str(camera["pixel_format"] or "MJPG")))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(camera["width"]))
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(camera["height"]))
        capture.set(cv2.CAP_PROP_FPS, float(camera["fps"]))

        actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
        self.get_logger().info(
            f"Benchmark start on {matched.video_path} | actual={actual_width}x{actual_height} @ {actual_fps:.3f} fps"
        )

        warmup_deadline = time.monotonic() + warmup_sec
        while time.monotonic() < warmup_deadline:
            capture.read()

        timestamps: list[float] = []
        deadline = time.monotonic() + duration_sec
        while time.monotonic() < deadline:
            ok, _frame = capture.read()
            if not ok:
                self.get_logger().warning("读取帧失败，跳过。")
                continue
            timestamps.append(time.monotonic())

        capture.release()

        if len(timestamps) < 2:
            raise RuntimeError("采样帧数不足，无法计算 FPS。")

        intervals = [b - a for a, b in zip(timestamps[:-1], timestamps[1:])]
        avg_fps = (len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
        min_fps = 1.0 / max(intervals)
        max_fps = 1.0 / min(intervals)
        median_fps = 1.0 / statistics.median(intervals)

        self.get_logger().info(f"Frames captured: {len(timestamps)} in {timestamps[-1] - timestamps[0]:.3f} sec")
        self.get_logger().info(f"Average FPS: {avg_fps:.3f}")
        self.get_logger().info(f"Median FPS : {median_fps:.3f}")
        self.get_logger().info(f"Min FPS    : {min_fps:.3f}")
        self.get_logger().info(f"Max FPS    : {max_fps:.3f}")


def main() -> None:
    rclpy.init()
    node = CameraFpsBenchmarkNode()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
