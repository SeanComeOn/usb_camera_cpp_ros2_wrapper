#!/usr/bin/env python3
"""Monitor camera image flow and restart unhealthy camera processes."""

from __future__ import annotations

import os
import signal
import time
from dataclasses import dataclass
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float32


USB_SYSFS = Path("/sys/bus/usb/devices")
PROC_ROOT = Path("/proc")


@dataclass(frozen=True)
class StreamTarget:
    name: str
    fps_topic: str
    executable: str
    node_name: str
    usb_identity: str
    usb_device_path: str = ""


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="ascii").strip()
    except (FileNotFoundError, PermissionError, OSError):
        return ""


def usb_identity_present(identity: str) -> bool:
    try:
        entries = USB_SYSFS.iterdir()
    except OSError:
        return False
    for entry in entries:
        vendor_id = read_text(entry / "idVendor").lower()
        product_id = read_text(entry / "idProduct").lower()
        if f"{vendor_id}:{product_id}" == identity:
            return True
    return False


def usb_device_present(device_path: str, identity: str) -> bool:
    if device_path:
        return Path(device_path).exists()
    return usb_identity_present(identity)


def process_ids(executable: str, node_name: str = "") -> list[int]:
    matches: list[int] = []
    try:
        entries = PROC_ROOT.iterdir()
    except OSError:
        return matches
    for entry in entries:
        if not entry.name.isdigit():
            continue
        try:
            resolved_executable = Path(os.readlink(entry / "exe")).name
        except (FileNotFoundError, PermissionError, OSError):
            continue
        if resolved_executable == executable:
            if node_name:
                try:
                    command = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(
                        "utf-8", errors="replace"
                    )
                except (FileNotFoundError, PermissionError, OSError):
                    continue
                if f"__node:={node_name}" not in command:
                    continue
            matches.append(int(entry.name))
    return matches


class CameraStreamWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("camera_stream_watchdog")
        self.declare_parameter("startup_grace_sec", 20.0)
        self.declare_parameter("stall_timeout_sec", 5.0)
        self.declare_parameter("minimum_fps", 27.0)
        self.declare_parameter("low_fps_grace_sec", 10.0)
        self.declare_parameter("restart_backoff_sec", 30.0)
        self.declare_parameter("terminate_grace_sec", 3.0)
        self.declare_parameter("problem_log_interval_sec", 10.0)
        self.declare_parameter("wrist_usb_identity", "1e45:0209")
        self.declare_parameter("stereo_usb_identity", "1bcf:0b15")
        self.declare_parameter("wrist_usb_device", "")
        self.declare_parameter("stereo_usb_device", "")
        self.declare_parameter("wrist_stereo_enabled", False)
        self.declare_parameter("wrist_stereo_usb_identity", "")
        self.declare_parameter("wrist_stereo_usb_device", "")

        self.startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self.stall_timeout = float(self.get_parameter("stall_timeout_sec").value)
        self.minimum_fps = float(self.get_parameter("minimum_fps").value)
        self.low_fps_grace = float(self.get_parameter("low_fps_grace_sec").value)
        self.restart_backoff = float(self.get_parameter("restart_backoff_sec").value)
        self.terminate_grace = float(self.get_parameter("terminate_grace_sec").value)
        self.problem_log_interval = float(self.get_parameter("problem_log_interval_sec").value)
        wrist_usb_identity = str(self.get_parameter("wrist_usb_identity").value).lower()
        stereo_usb_identity = str(self.get_parameter("stereo_usb_identity").value).lower()
        wrist_usb_device = str(self.get_parameter("wrist_usb_device").value)
        stereo_usb_device = str(self.get_parameter("stereo_usb_device").value)
        wrist_stereo_enabled = bool(self.get_parameter("wrist_stereo_enabled").value)
        wrist_stereo_usb_identity = str(
            self.get_parameter("wrist_stereo_usb_identity").value
        ).lower()
        wrist_stereo_usb_device = str(self.get_parameter("wrist_stereo_usb_device").value)

        targets = [
            StreamTarget(
                "wrist_mono",
                "/mono/actual_fps",
                "mono_camera_node",
                "mono_camera_publisher",
                wrist_usb_identity,
                wrist_usb_device,
            ),
            StreamTarget(
                "third_person_stereo",
                "/stereo/actual_fps",
                "stereo_camera_node",
                "third_person_stereo_camera_publisher",
                stereo_usb_identity,
                stereo_usb_device,
            ),
        ]
        if wrist_stereo_enabled:
            targets.append(
                StreamTarget(
                    "wrist_stereo",
                    "/wrist/stereo/actual_fps",
                    "stereo_camera_node",
                    "wrist_stereo_camera_publisher",
                    wrist_stereo_usb_identity,
                    wrist_stereo_usb_device,
                )
            )
        self.targets = tuple(targets)
        now = time.monotonic()
        self.started_at = now
        self.last_frame: dict[str, float | None] = {target.name: None for target in self.targets}
        self.actual_fps: dict[str, float | None] = {target.name: None for target in self.targets}
        self.low_fps_since: dict[str, float | None] = {target.name: None for target in self.targets}
        self.last_restart = {target.name: 0.0 for target in self.targets}
        self.last_problem_log = {target.name: 0.0 for target in self.targets}
        self.unhealthy_reason: dict[str, str | None] = {target.name: None for target in self.targets}
        self.pending_kills: dict[int, float] = {}
        self._fps_subscriptions = [
            self.create_subscription(
                Float32,
                target.fps_topic,
                lambda msg, stream_name=target.name: self._record_fps(stream_name, msg),
                10,
            )
            for target in self.targets
        ]
        self._watchdog_timer = self.create_timer(1.0, self._check_streams)
        self.get_logger().info(
            f"Camera watchdog active: stall_timeout={self.stall_timeout:.1f}s, "
            f"minimum_fps={self.minimum_fps:.1f}"
        )

    def _record_fps(self, stream_name: str, msg: Float32) -> None:
        self.last_frame[stream_name] = time.monotonic()
        self.actual_fps[stream_name] = float(msg.data)

    def _log_problem(self, stream_name: str, message: str, now: float, *, force: bool = False) -> None:
        if force or now - self.last_problem_log[stream_name] >= self.problem_log_interval:
            # ROS 2 renders ERROR in red, making camera faults visible in launch output.
            self.get_logger().error(message)
            self.last_problem_log[stream_name] = now

    def _mark_healthy(self, stream_name: str) -> None:
        previous_reason = self.unhealthy_reason[stream_name]
        if previous_reason is not None:
            self.get_logger().info(f"{stream_name} camera stream recovered from {previous_reason}")
        self.unhealthy_reason[stream_name] = None

    def _finish_stuck_terminations(self, now: float) -> None:
        for pid, deadline in list(self.pending_kills.items()):
            if not (PROC_ROOT / str(pid)).exists():
                self.pending_kills.pop(pid, None)
                continue
            if now < deadline:
                continue
            try:
                os.kill(pid, signal.SIGKILL)
                self.get_logger().error(f"Camera process {pid} ignored SIGTERM; sent SIGKILL")
            except ProcessLookupError:
                pass
            except PermissionError as error:
                self.get_logger().error(f"Cannot SIGKILL camera process {pid}: {error}")
            self.pending_kills.pop(pid, None)

    def _restart_target(self, target: StreamTarget, reason: str, now: float) -> None:
        if now - self.last_restart[target.name] < self.restart_backoff:
            return

        pids = process_ids(target.executable, target.node_name)
        if not pids:
            self._log_problem(
                target.name,
                f"{target.name} camera is unhealthy ({reason}), but {target.executable} is not running; "
                "waiting for ros2 launch respawn",
                now,
            )
            self.last_restart[target.name] = now
            return

        for pid in pids:
            try:
                os.kill(pid, signal.SIGTERM)
                self.pending_kills[pid] = now + max(0.5, self.terminate_grace)
            except ProcessLookupError:
                continue
            except PermissionError as error:
                self.get_logger().error(f"Cannot restart {target.executable} pid={pid}: {error}")
        self._log_problem(
            target.name,
            f"{target.name} camera is unhealthy ({reason}); restarting {target.executable} pid(s)={pids}",
            now,
            force=True,
        )
        self.last_restart[target.name] = now
        self.last_frame[target.name] = None
        self.actual_fps[target.name] = None
        self.low_fps_since[target.name] = None

    def _check_streams(self) -> None:
        now = time.monotonic()
        self._finish_stuck_terminations(now)
        if now - self.started_at < self.startup_grace:
            return

        for target in self.targets:
            stream_name = target.name
            last_frame = self.last_frame[stream_name]
            frame_age = now - last_frame if last_frame is not None else now - self.started_at

            if not usb_device_present(target.usb_device_path, target.usb_identity):
                identity = target.usb_device_path or target.usb_identity
                reason = f"USB {identity} disconnected or not enumerated"
                self.unhealthy_reason[stream_name] = reason
                self.low_fps_since[stream_name] = None
                self._log_problem(
                    stream_name,
                    f"{stream_name} camera has no new data: {reason}; the USB recovery service will retry",
                    now,
                )
                continue

            if last_frame is None or frame_age > self.stall_timeout:
                reason = f"no new image for {frame_age:.1f}s"
                self.unhealthy_reason[stream_name] = reason
                self.low_fps_since[stream_name] = None
                self._log_problem(stream_name, f"{stream_name} camera stalled: {reason}", now)
                self._restart_target(target, reason, now)
                continue

            measured_fps = self.actual_fps[stream_name]
            if measured_fps is None:
                continue
            if measured_fps < self.minimum_fps:
                if self.low_fps_since[stream_name] is None:
                    self.low_fps_since[stream_name] = now
                low_duration = now - self.low_fps_since[stream_name]
                reason = (
                    f"frame rate {measured_fps:.2f} FPS is below {self.minimum_fps:.2f} FPS "
                    f"for {low_duration:.1f}s"
                )
                self.unhealthy_reason[stream_name] = reason
                self._log_problem(stream_name, f"{stream_name} camera low frame rate: {reason}", now)
                if low_duration >= self.low_fps_grace:
                    self._restart_target(target, reason, now)
                continue

            self.low_fps_since[stream_name] = None
            self._mark_healthy(stream_name)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = CameraStreamWatchdog()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except (KeyboardInterrupt, ExternalShutdownException):
            pass


if __name__ == "__main__":
    main()
