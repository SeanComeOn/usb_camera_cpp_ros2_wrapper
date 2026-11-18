#!/usr/bin/env python3
"""Verify sustained delivery for wrist mono and both stereo camera pairs."""

from __future__ import annotations

import argparse
import statistics
import sys
import threading
import time
from dataclasses import dataclass, field

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


@dataclass
class StreamStats:
    image_received: bool = False
    image_valid: bool = False
    width: int = 0
    height: int = 0
    fps_samples: list[float] = field(default_factory=list)
    last_fps_received: float | None = None
    image_count: int = 0
    first_image_received: float | None = None
    last_image_received: float | None = None

    def record_image(self, msg: Image, received_at: float) -> None:
        valid = (
            msg.width > 0
            and msg.height > 0
            and msg.step > 0
            and len(msg.data) >= int(msg.step) * int(msg.height)
        )
        if self.image_count == 0:
            self.first_image_received = received_at
            self.image_valid = valid
        else:
            self.image_valid = self.image_valid and valid
        self.image_received = True
        self.image_count += 1
        self.width = int(msg.width)
        self.height = int(msg.height)
        self.last_image_received = received_at

    def record_fps(self, msg: Float32, received_at: float) -> None:
        self.fps_samples.append(float(msg.data))
        self.last_fps_received = received_at

    def received_image_fps(self) -> float:
        if (
            self.image_count < 2
            or self.first_image_received is None
            or self.last_image_received is None
            or self.last_image_received <= self.first_image_received
        ):
            return 0.0
        return (self.image_count - 1) / (
            self.last_image_received - self.first_image_received
        )


@dataclass(frozen=True)
class StreamConfig:
    image_topic: str
    fps_topic: str
    expected_width: int
    expected_height: int


class CameraHealthNode(Node):
    def __init__(self, streams: dict[str, StreamConfig]) -> None:
        super().__init__("dual_camera_health_check")
        self.stats = {name: StreamStats() for name in streams}
        self.image_callback_groups = {
            name: MutuallyExclusiveCallbackGroup() for name in streams
        }
        self.fps_callback_group = ReentrantCallbackGroup()
        self.image_subscriptions = []
        self.fps_subscriptions = []
        for name, config in streams.items():
            self.image_subscriptions.append(
                self.create_subscription(
                    Image,
                    config.image_topic,
                    lambda msg, stream_name=name: self.stats[stream_name].record_image(
                        msg, time.monotonic()
                    ),
                    qos_profile_sensor_data,
                    callback_group=self.image_callback_groups[name],
                )
            )
            self.fps_subscriptions.append(
                self.create_subscription(
                    Float32,
                    config.fps_topic,
                    lambda msg, stream_name=name: self.stats[stream_name].record_fps(msg, time.monotonic()),
                    10,
                    callback_group=self.fps_callback_group,
                )
            )

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mono-topic", default="/mono/image_raw")
    parser.add_argument(
        "--stereo-left-topic",
        "--stereo-topic",
        dest="stereo_left_topic",
        default="/stereo/left/image_raw",
    )
    parser.add_argument("--stereo-right-topic", default="/stereo/right/image_raw")
    parser.add_argument("--wrist-stereo-left-topic", default="/wrist/stereo/left/image_raw")
    parser.add_argument("--wrist-stereo-right-topic", default="/wrist/stereo/right/image_raw")
    parser.add_argument("--mono-fps-topic", default="/mono/actual_fps")
    parser.add_argument("--stereo-fps-topic", default="/stereo/actual_fps")
    parser.add_argument("--wrist-stereo-fps-topic", default="/wrist/stereo/actual_fps")
    parser.add_argument(
        "--include-wrist-stereo",
        action="store_true",
        help="Also check both streams from the wrist stereo camera.",
    )
    parser.add_argument(
        "--skip-mono",
        action="store_true",
        help="Check only the stereo left and right streams.",
    )
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--image-timeout", type=float, default=10.0)
    parser.add_argument("--min-fps", type=float, default=27.0)
    parser.add_argument("--min-fps-samples", type=int, default=10)
    parser.add_argument("--min-image-samples", type=int, default=10)
    parser.add_argument("--max-stall", type=float, default=2.0)
    parser.add_argument("--stereo-width", type=int, default=720)
    parser.add_argument("--stereo-height", type=int, default=720)
    parser.add_argument("--wrist-stereo-width", type=int, default=960)
    parser.add_argument("--wrist-stereo-height", type=int, default=960)
    parser.add_argument("--wrist-width", type=int, default=1080)
    parser.add_argument("--wrist-height", type=int, default=1080)
    return parser.parse_args()


def build_streams(args: argparse.Namespace) -> dict[str, StreamConfig]:
    streams = {
        "stereo_left": StreamConfig(
            args.stereo_left_topic,
            args.stereo_fps_topic,
            args.stereo_width,
            args.stereo_height,
        ),
        "stereo_right": StreamConfig(
            args.stereo_right_topic,
            args.stereo_fps_topic,
            args.stereo_width,
            args.stereo_height,
        ),
    }
    if not getattr(args, "skip_mono", False):
        streams["wrist"] = StreamConfig(
            args.mono_topic,
            args.mono_fps_topic,
            args.wrist_width,
            args.wrist_height,
        )
    if getattr(args, "include_wrist_stereo", False):
        streams["wrist_stereo_left"] = StreamConfig(
            args.wrist_stereo_left_topic,
            args.wrist_stereo_fps_topic,
            args.wrist_stereo_width,
            args.wrist_stereo_height,
        )
        streams["wrist_stereo_right"] = StreamConfig(
            args.wrist_stereo_right_topic,
            args.wrist_stereo_fps_topic,
            args.wrist_stereo_width,
            args.wrist_stereo_height,
        )
    return streams


def stream_is_healthy(
    stats: StreamStats,
    config: StreamConfig,
    *,
    finished_at: float,
    min_fps: float,
    min_fps_samples: int,
    min_image_samples: int,
    max_stall: float,
) -> bool:
    mean_fps = statistics.fmean(stats.fps_samples) if stats.fps_samples else 0.0
    median_fps = statistics.median(stats.fps_samples) if stats.fps_samples else 0.0
    image_stall = (
        finished_at - stats.last_image_received
        if stats.last_image_received is not None
        else float("inf")
    )
    fps_stall = (
        finished_at - stats.last_fps_received
        if stats.last_fps_received is not None
        else float("inf")
    )
    return (
        stats.image_received
        and stats.image_valid
        and stats.width == config.expected_width
        and stats.height == config.expected_height
        and stats.image_count >= min_image_samples
        and len(stats.fps_samples) >= min_fps_samples
        and mean_fps >= min_fps
        and median_fps >= min_fps
        and image_stall <= max_stall
        and fps_stall <= max_stall
    )


def main() -> int:
    args = parse_args()
    streams = build_streams(args)
    rclpy.init()
    node = CameraHealthNode(streams)
    executor = MultiThreadedExecutor(num_threads=max(4, len(streams) + 1))
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    started_at = time.monotonic()
    deadline = started_at + max(args.image_timeout, args.duration)

    try:
        while rclpy.ok() and time.monotonic() < deadline:
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))
    finally:
        finished_at = time.monotonic()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join(timeout=2.0)

    failed = False
    for name, stats in node.stats.items():
        config = streams[name]
        samples = stats.fps_samples
        mean_fps = statistics.fmean(samples) if samples else 0.0
        median_fps = statistics.median(samples) if samples else 0.0
        minimum_fps = min(samples) if samples else 0.0
        image_stall = (
            finished_at - stats.last_image_received
            if stats.last_image_received is not None
            else float("inf")
        )
        fps_stall = (
            finished_at - stats.last_fps_received
            if stats.last_fps_received is not None
            else float("inf")
        )
        healthy = stream_is_healthy(
            stats,
            config,
            finished_at=finished_at,
            min_fps=args.min_fps,
            min_fps_samples=args.min_fps_samples,
            min_image_samples=args.min_image_samples,
            max_stall=args.max_stall,
        )
        status = "PASS" if healthy else "FAIL"
        print(
            f"{status} {name}: size={stats.width}x{stats.height} valid={stats.image_valid} "
            f"image_samples={stats.image_count} image_fps={stats.received_image_fps():.2f} "
            f"last_image_age={image_stall:.2f}s fps_samples={len(samples)} "
            f"mean_fps={mean_fps:.2f} median_fps={median_fps:.2f} "
            f"min_fps={minimum_fps:.2f} last_fps_age={fps_stall:.2f}s"
        )
        failed = failed or not healthy

    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
