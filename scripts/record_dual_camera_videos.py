#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import json
from pathlib import Path
import queue
import select
import sys
import termios
import threading
import time
import tty
from typing import Callable

import cv2
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
import rclpy.time
from sensor_msgs.msg import CompressedImage, Image, JointState
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


ARM_JOINT_NAMES = [
    "fr3_joint1",
    "fr3_joint2",
    "fr3_joint3",
    "fr3_joint4",
    "fr3_joint5",
    "fr3_joint6",
    "fr3_joint7",
]

GRIPPER_JOINT_NAMES = [
    "fr3_finger_joint1",
    "fr3_finger_joint2",
]

RECORDING_SCHEMA = "fr3-multimodal-recording"
RECORDING_FORMAT_VERSION = 4
MINIMUM_READER_VERSION = 2
CONTROL_KEY_CODES = frozenset(
    (
        "KeyW", "KeyA", "KeyS", "KeyD", "KeyQ", "KeyE",
        "KeyU", "KeyI", "KeyO", "KeyJ", "KeyK", "KeyL",
        "KeyC", "KeyV",
    )
)


@dataclass
class FrameSlot:
    image: np.ndarray | None = None
    stamp_sec: float = 0.0
    stamp_ns: int = 0
    received_count: int = 0


@dataclass
class FrameSnapshot:
    image: np.ndarray
    stamp_sec: float
    stamp_ns: int
    received_count: int


@dataclass
class JointStateSlot:
    arm_positions: np.ndarray | None = None
    arm_velocities: np.ndarray | None = None
    arm_efforts: np.ndarray | None = None
    gripper_positions: np.ndarray | None = None
    stamp_sec: float = 0.0
    stamp_ns: int = 0
    received_count: int = 0


@dataclass
class EndEffectorPose:
    position: np.ndarray
    quaternion_xyzw: np.ndarray
    stamp_sec: float
    stamp_ns: int


@dataclass
class TrajectorySample:
    record_time_sec: float
    record_time_ns: int
    latest_video_frame_index: int
    joint_state_received_count: int
    joint_state_stamp_sec: float
    joint_state_stamp_ns: int
    ee_transform_stamp_sec: float
    ee_transform_stamp_ns: int
    stereo_left_image_stamp_sec: float
    stereo_left_image_stamp_ns: int
    stereo_right_image_stamp_sec: float
    stereo_right_image_stamp_ns: int
    wrist_stereo_left_image_stamp_sec: float
    wrist_stereo_left_image_stamp_ns: int
    wrist_stereo_right_image_stamp_sec: float
    wrist_stereo_right_image_stamp_ns: int
    wrist_image_stamp_sec: float
    wrist_image_stamp_ns: int
    joint_positions: np.ndarray
    joint_velocities: np.ndarray
    joint_efforts: np.ndarray
    ee_position: np.ndarray
    ee_quaternion_xyzw: np.ndarray
    gripper_positions: np.ndarray
    gripper_width: float
    gripper_open_ratio: float


@dataclass
class ControlInputSample:
    record_time_sec: float
    record_time_ns: int
    sequence: int
    client_id: str
    active: bool
    pointer_locked: bool
    keys: list[str]
    mouse_delta_x: float
    mouse_delta_y: float
    forward: float
    right: float
    up: float
    roll: float
    pitch: float
    yaw: float
    look_yaw: float
    look_pitch: float
    speed_scale: float
    gripper: str


class StereoWristRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("stereo_wrist_video_recorder")
        self._args = args
        self._lock = threading.Lock()
        self._stereo_left = FrameSlot()
        self._stereo_right = FrameSlot()
        self._wrist_stereo_left = FrameSlot()
        self._wrist_stereo_right = FrameSlot()
        self._wrist = FrameSlot()
        self._joint_state = JointStateSlot()
        self._writer: RecordingSessionWriter | None = None
        self._recording = False
        self._paused = False
        self._session_dir: Path | None = None
        self._warning_timestamps_ns: dict[str, int] = {}
        self._stereo_left_callback_group = MutuallyExclusiveCallbackGroup()
        self._stereo_right_callback_group = MutuallyExclusiveCallbackGroup()
        self._wrist_stereo_left_callback_group = MutuallyExclusiveCallbackGroup()
        self._wrist_stereo_right_callback_group = MutuallyExclusiveCallbackGroup()
        self._wrist_callback_group = MutuallyExclusiveCallbackGroup()
        self._joint_state_callback_group = MutuallyExclusiveCallbackGroup()
        self._control_input_callback_group = MutuallyExclusiveCallbackGroup()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        image_qos = make_image_qos(args.image_qos_reliability)
        image_type = CompressedImage if args.image_transport == "compressed" else Image

        self.create_subscription(
            image_type,
            image_transport_topic(args.stereo_left_topic, args.image_transport),
            self._stereo_left_callback,
            image_qos,
            callback_group=self._stereo_left_callback_group,
        )
        self.create_subscription(
            image_type,
            image_transport_topic(args.stereo_right_topic, args.image_transport),
            self._stereo_right_callback,
            image_qos,
            callback_group=self._stereo_right_callback_group,
        )
        self.create_subscription(
            image_type,
            image_transport_topic(args.wrist_stereo_left_topic, args.image_transport),
            self._wrist_stereo_left_callback,
            image_qos,
            callback_group=self._wrist_stereo_left_callback_group,
        )
        self.create_subscription(
            image_type,
            image_transport_topic(args.wrist_stereo_right_topic, args.image_transport),
            self._wrist_stereo_right_callback,
            image_qos,
            callback_group=self._wrist_stereo_right_callback_group,
        )
        self.create_subscription(
            image_type,
            image_transport_topic(args.wrist_topic, args.image_transport),
            self._wrist_callback,
            image_qos,
            callback_group=self._wrist_callback_group,
        )
        self.create_subscription(
            JointState,
            args.joint_states_topic,
            self._joint_state_callback,
            qos_profile_sensor_data,
            callback_group=self._joint_state_callback_group,
        )
        self.create_subscription(
            String,
            args.control_input_topic,
            self._control_input_callback,
            50,
            callback_group=self._control_input_callback_group,
        )

        self.get_logger().info(f"Third-person stereo left topic: {args.stereo_left_topic}")
        self.get_logger().info(f"Third-person stereo right topic: {args.stereo_right_topic}")
        self.get_logger().info(f"Wrist stereo left topic: {args.wrist_stereo_left_topic}")
        self.get_logger().info(f"Wrist stereo right topic: {args.wrist_stereo_right_topic}")
        self.get_logger().info(f"Wrist mono topic: {args.wrist_topic}")
        self.get_logger().info(f"Image transport: {args.image_transport}")
        self.get_logger().info(f"Joint states topic: {args.joint_states_topic}")
        self.get_logger().info(f"Web control input topic: {args.control_input_topic}")
        self.get_logger().info(
            f"TF end-effector pose: {args.base_frame} -> {args.end_effector_frame}"
        )

    def _stereo_left_callback(self, msg: Image | CompressedImage) -> None:
        image = ros_image_to_bgr(msg, self.get_logger().warning)
        if image is None:
            return
        with self._lock:
            self._stereo_left = FrameSlot(
                image=image,
                stamp_sec=stamp_to_sec(msg),
                stamp_ns=stamp_to_ns(msg),
                received_count=self._stereo_left.received_count + 1,
            )

    def _stereo_right_callback(self, msg: Image | CompressedImage) -> None:
        image = ros_image_to_bgr(msg, self.get_logger().warning)
        if image is None:
            return
        with self._lock:
            self._stereo_right = FrameSlot(
                image=image,
                stamp_sec=stamp_to_sec(msg),
                stamp_ns=stamp_to_ns(msg),
                received_count=self._stereo_right.received_count + 1,
            )

    def _wrist_callback(self, msg: Image | CompressedImage) -> None:
        image = ros_image_to_bgr(msg, self.get_logger().warning)
        if image is None:
            return
        with self._lock:
            self._wrist = FrameSlot(
                image=image,
                stamp_sec=stamp_to_sec(msg),
                stamp_ns=stamp_to_ns(msg),
                received_count=self._wrist.received_count + 1,
            )

    def _wrist_stereo_left_callback(self, msg: Image | CompressedImage) -> None:
        image = ros_image_to_bgr(msg, self.get_logger().warning)
        if image is None:
            return
        with self._lock:
            self._wrist_stereo_left = FrameSlot(
                image=image,
                stamp_sec=stamp_to_sec(msg),
                stamp_ns=stamp_to_ns(msg),
                received_count=self._wrist_stereo_left.received_count + 1,
            )

    def _wrist_stereo_right_callback(self, msg: Image | CompressedImage) -> None:
        image = ros_image_to_bgr(msg, self.get_logger().warning)
        if image is None:
            return
        with self._lock:
            self._wrist_stereo_right = FrameSlot(
                image=image,
                stamp_sec=stamp_to_sec(msg),
                stamp_ns=stamp_to_ns(msg),
                received_count=self._wrist_stereo_right.received_count + 1,
            )

    def _joint_state_callback(self, msg: JointState) -> None:
        if len(msg.name) != len(msg.position):
            self._throttled_warn(
                "joint_state_mismatch",
                "JointState name/position length mismatch, ignoring message.",
            )
            return

        positions_by_name = dict(zip(msg.name, msg.position))
        arm_positions = None
        arm_velocities = None
        arm_efforts = None
        gripper_positions = None

        if all(joint_name in positions_by_name for joint_name in ARM_JOINT_NAMES):
            arm_positions = np.asarray(
                [positions_by_name[joint_name] for joint_name in ARM_JOINT_NAMES],
                dtype=np.float64,
            )
            if len(msg.velocity) == len(msg.name):
                velocities_by_name = dict(zip(msg.name, msg.velocity))
                arm_velocities = np.asarray(
                    [velocities_by_name[joint_name] for joint_name in ARM_JOINT_NAMES],
                    dtype=np.float64,
                )
            if len(msg.effort) == len(msg.name):
                efforts_by_name = dict(zip(msg.name, msg.effort))
                arm_efforts = np.asarray(
                    [efforts_by_name[joint_name] for joint_name in ARM_JOINT_NAMES],
                    dtype=np.float64,
                )

        if all(joint_name in positions_by_name for joint_name in GRIPPER_JOINT_NAMES):
            gripper_positions = np.asarray(
                [positions_by_name[joint_name] for joint_name in GRIPPER_JOINT_NAMES],
                dtype=np.float64,
            )

        if arm_positions is None and gripper_positions is None:
            return

        with self._lock:
            if arm_positions is not None:
                self._joint_state.arm_positions = arm_positions
                self._joint_state.arm_velocities = (
                    arm_velocities
                    if arm_velocities is not None
                    else np.full(len(ARM_JOINT_NAMES), np.nan, dtype=np.float64)
                )
                self._joint_state.arm_efforts = (
                    arm_efforts
                    if arm_efforts is not None
                    else np.full(len(ARM_JOINT_NAMES), np.nan, dtype=np.float64)
                )
            if gripper_positions is not None:
                self._joint_state.gripper_positions = gripper_positions
            self._joint_state.stamp_sec = stamp_to_sec(msg)
            self._joint_state.stamp_ns = stamp_to_ns(msg)
            self._joint_state.received_count += 1

    def _control_input_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            sample = control_input_sample(payload, self.get_clock().now().nanoseconds)
        except (json.JSONDecodeError, TypeError, ValueError) as exc:
            self._throttled_warn("malformed_control_input", f"Ignoring malformed web control input: {exc}")
            return
        writer = self._writer
        if self._recording and not self._paused and writer is not None:
            writer.write_control_input(sample)

    def latest_counts(self) -> tuple[int, int, int, int, int]:
        with self._lock:
            return (
                self._stereo_left.received_count,
                self._stereo_right.received_count,
                self._wrist_stereo_left.received_count,
                self._wrist_stereo_right.received_count,
                self._wrist.received_count,
            )

    def latest_frames(
        self,
    ) -> tuple[
        FrameSnapshot | None,
        FrameSnapshot | None,
        FrameSnapshot | None,
        FrameSnapshot | None,
        FrameSnapshot | None,
    ]:
        with self._lock:
            stereo_left = copy_frame_slot(self._stereo_left)
            stereo_right = copy_frame_slot(self._stereo_right)
            wrist_stereo_left = copy_frame_slot(self._wrist_stereo_left)
            wrist_stereo_right = copy_frame_slot(self._wrist_stereo_right)
            wrist = copy_frame_slot(self._wrist)
        return stereo_left, stereo_right, wrist_stereo_left, wrist_stereo_right, wrist

    def start_recording(self) -> Path | None:
        if self._recording:
            self.get_logger().warning("Already recording. Press 's' to save or 'd' to discard first.")
            return self._session_dir

        stereo_left, stereo_right, wrist_stereo_left, wrist_stereo_right, wrist = self.latest_frames()
        if any(
            frame is None
            for frame in (stereo_left, stereo_right, wrist_stereo_left, wrist_stereo_right, wrist)
        ):
            self.get_logger().warning(
                "Cannot start yet: waiting for third-person stereo, wrist stereo, and wrist mono streams."
            )
            return None
        assert wrist_stereo_left is not None and wrist_stereo_right is not None
        assert stereo_left is not None and stereo_right is not None and wrist is not None

        if self._args.require_trajectory and not self._trajectory_inputs_ready():
            return None

        session_dir = make_session_dir(Path(self._args.output_root), self._args.session_prefix)
        writer = RecordingSessionWriter(
            session_dir=session_dir,
            fps=self._args.fps,
            trajectory_hz=self._args.trajectory_hz,
            codec=self._args.codec,
            write_combined=self._args.write_combined,
            stereo_left_topic=self._args.stereo_left_topic,
            stereo_right_topic=self._args.stereo_right_topic,
            wrist_stereo_left_topic=self._args.wrist_stereo_left_topic,
            wrist_stereo_right_topic=self._args.wrist_stereo_right_topic,
            wrist_topic=self._args.wrist_topic,
            joint_states_topic=self._args.joint_states_topic,
            base_frame=self._args.base_frame,
            end_effector_frame=self._args.end_effector_frame,
            gripper_max_width=self._args.gripper_max_width,
            trajectory_required=self._args.require_trajectory,
            control_input_topic=self._args.control_input_topic,
            image_transport=self._args.image_transport,
        )
        writer.open(
            stereo_left.image,
            stereo_right.image,
            wrist_stereo_left.image,
            wrist_stereo_right.image,
            wrist.image,
        )

        self._writer = writer
        self._session_dir = session_dir
        self._recording = True
        self._paused = False
        self.get_logger().info(f"Recording started: {session_dir}")
        return session_dir

    def toggle_pause(self) -> bool | None:
        if not self._recording or self._writer is None:
            self.get_logger().warning("Not recording. Press 'r' to start.")
            return None
        now_ns = self.get_clock().now().nanoseconds
        if self._paused:
            self._writer.resume(now_ns)
            self._paused = False
            self.get_logger().info(f"Recording resumed: {self._session_dir}")
        else:
            self._writer.pause(now_ns)
            self._paused = True
            self.get_logger().info(f"Recording paused: {self._session_dir}")
        return self._paused

    def stop_and_save(self) -> Path | None:
        if not self._recording:
            self.get_logger().warning("Not recording. Press 'r' to start.")
            return None
        session_dir = self._close_writer(save=True)
        self.get_logger().info(f"Recording saved: {session_dir}")
        return session_dir

    def discard(self) -> None:
        if not self._recording:
            self.get_logger().warning("Not recording. Nothing to discard.")
            return
        session_dir = self._close_writer(save=False)
        if session_dir is not None:
            remove_session_dir(session_dir)
            self.get_logger().info(f"Recording discarded: {session_dir}")

    def tick_video(self) -> None:
        if not self._recording or self._paused or self._writer is None:
            return

        stereo_left, stereo_right, wrist_stereo_left, wrist_stereo_right, wrist = self.latest_frames()
        if any(
            frame is None
            for frame in (stereo_left, stereo_right, wrist_stereo_left, wrist_stereo_right, wrist)
        ):
            return
        assert wrist_stereo_left is not None and wrist_stereo_right is not None
        assert stereo_left is not None and stereo_right is not None and wrist is not None

        record_time_ns = self.get_clock().now().nanoseconds
        self._writer.write_video_frame(
            stereo_left,
            stereo_right,
            wrist_stereo_left,
            wrist_stereo_right,
            wrist,
            record_time_ns=record_time_ns,
        )

    def tick_trajectory(self) -> None:
        if not self._recording or self._paused or self._writer is None:
            return

        sample = self._build_trajectory_sample()
        if sample is None:
            return

        self._writer.write_trajectory_sample(sample)

    def shutdown(self) -> None:
        if self._recording:
            self.stop_and_save()

    def _build_trajectory_sample(self) -> TrajectorySample | None:
        with self._lock:
            arm_positions = None
            if self._joint_state.arm_positions is not None:
                arm_positions = self._joint_state.arm_positions.copy()
            arm_velocities = None
            if self._joint_state.arm_velocities is not None:
                arm_velocities = self._joint_state.arm_velocities.copy()
            arm_efforts = None
            if self._joint_state.arm_efforts is not None:
                arm_efforts = self._joint_state.arm_efforts.copy()
            gripper_positions = None
            if self._joint_state.gripper_positions is not None:
                gripper_positions = self._joint_state.gripper_positions.copy()
            joint_state_stamp_sec = self._joint_state.stamp_sec
            joint_state_stamp_ns = self._joint_state.stamp_ns
            joint_state_received_count = self._joint_state.received_count
            stereo_left_stamp_sec = self._stereo_left.stamp_sec
            stereo_left_stamp_ns = self._stereo_left.stamp_ns
            stereo_right_stamp_sec = self._stereo_right.stamp_sec
            stereo_right_stamp_ns = self._stereo_right.stamp_ns
            wrist_stereo_left_stamp_sec = self._wrist_stereo_left.stamp_sec
            wrist_stereo_left_stamp_ns = self._wrist_stereo_left.stamp_ns
            wrist_stereo_right_stamp_sec = self._wrist_stereo_right.stamp_sec
            wrist_stereo_right_stamp_ns = self._wrist_stereo_right.stamp_ns
            wrist_stamp_sec = self._wrist.stamp_sec
            wrist_stamp_ns = self._wrist.stamp_ns

        if arm_positions is None:
            self._throttled_warn(
                "missing_arm_joint_state",
                "Arm joint states are not ready yet; trajectory samples are waiting for /joint_states.",
            )
            return None
        if arm_velocities is None:
            arm_velocities = np.full(len(ARM_JOINT_NAMES), np.nan, dtype=np.float64)
        if arm_efforts is None:
            arm_efforts = np.full(len(ARM_JOINT_NAMES), np.nan, dtype=np.float64)

        if gripper_positions is None:
            self._throttled_warn(
                "missing_gripper_joint_state",
                "Gripper joint states are not ready yet; trajectory samples will contain NaN gripper values.",
            )
            gripper_positions = np.asarray([np.nan, np.nan], dtype=np.float64)

        end_effector_pose = self._lookup_end_effector_pose()
        if end_effector_pose is None:
            return None

        record_time_ns = self.get_clock().now().nanoseconds
        gripper_width = float(np.sum(gripper_positions))
        if np.any(np.isnan(gripper_positions)):
            gripper_width = float("nan")
            gripper_open_ratio = float("nan")
        elif self._args.gripper_max_width > 0.0:
            gripper_open_ratio = float(
                np.clip(gripper_width / self._args.gripper_max_width, 0.0, 1.0)
            )
        else:
            gripper_open_ratio = float("nan")

        latest_video_frame_index = -1
        if self._writer is not None:
            latest_video_frame_index = self._writer.latest_video_frame_index

        return TrajectorySample(
            record_time_sec=record_time_ns / 1e9,
            record_time_ns=record_time_ns,
            latest_video_frame_index=latest_video_frame_index,
            joint_state_received_count=joint_state_received_count,
            joint_state_stamp_sec=joint_state_stamp_sec,
            joint_state_stamp_ns=joint_state_stamp_ns,
            ee_transform_stamp_sec=end_effector_pose.stamp_sec,
            ee_transform_stamp_ns=end_effector_pose.stamp_ns,
            stereo_left_image_stamp_sec=stereo_left_stamp_sec,
            stereo_left_image_stamp_ns=stereo_left_stamp_ns,
            stereo_right_image_stamp_sec=stereo_right_stamp_sec,
            stereo_right_image_stamp_ns=stereo_right_stamp_ns,
            wrist_stereo_left_image_stamp_sec=wrist_stereo_left_stamp_sec,
            wrist_stereo_left_image_stamp_ns=wrist_stereo_left_stamp_ns,
            wrist_stereo_right_image_stamp_sec=wrist_stereo_right_stamp_sec,
            wrist_stereo_right_image_stamp_ns=wrist_stereo_right_stamp_ns,
            wrist_image_stamp_sec=wrist_stamp_sec,
            wrist_image_stamp_ns=wrist_stamp_ns,
            joint_positions=arm_positions,
            joint_velocities=arm_velocities,
            joint_efforts=arm_efforts,
            ee_position=end_effector_pose.position,
            ee_quaternion_xyzw=end_effector_pose.quaternion_xyzw,
            gripper_positions=gripper_positions,
            gripper_width=gripper_width,
            gripper_open_ratio=gripper_open_ratio,
        )

    def _trajectory_inputs_ready(self) -> bool:
        with self._lock:
            arm_ready = self._joint_state.arm_positions is not None
        if not arm_ready:
            self._throttled_warn(
                "recording_waiting_for_arm_joint_state",
                "Cannot start yet: waiting for FR3 arm joints on "
                f"{self._args.joint_states_topic}.",
            )
            return False

        if self._lookup_end_effector_pose() is None:
            self._throttled_warn(
                "recording_waiting_for_tf",
                "Cannot start yet: waiting for end-effector TF "
                f"{self._args.base_frame} -> {self._args.end_effector_frame}.",
            )
            return False
        return True

    def _lookup_end_effector_pose(self) -> EndEffectorPose | None:
        try:
            transform = self._tf_buffer.lookup_transform(
                self._args.base_frame,
                self._args.end_effector_frame,
                rclpy.time.Time(),
            )
        except TransformException as exc:
            self._throttled_warn(
                "missing_tf",
                f"Unable to lookup TF {self._args.base_frame} -> {self._args.end_effector_frame}: {exc}",
            )
            return None

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp_ns = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
        return EndEffectorPose(
            position=np.asarray(
                [translation.x, translation.y, translation.z],
                dtype=np.float64,
            ),
            quaternion_xyzw=np.asarray(
                [rotation.x, rotation.y, rotation.z, rotation.w],
                dtype=np.float64,
            ),
            stamp_sec=stamp_ns / 1e9,
            stamp_ns=stamp_ns,
        )

    def _close_writer(self, *, save: bool) -> Path | None:
        writer = self._writer
        session_dir = self._session_dir
        if writer is not None and self._paused:
            writer.resume(self.get_clock().now().nanoseconds)
        self._writer = None
        self._session_dir = None
        self._recording = False
        self._paused = False
        if writer is not None:
            writer.close(save=save)
        return session_dir

    def _throttled_warn(self, key: str, message: str, *, interval_sec: float = 2.0) -> None:
        now_ns = self.get_clock().now().nanoseconds
        last_ns = self._warning_timestamps_ns.get(key, 0)
        if now_ns - last_ns < int(interval_sec * 1e9):
            return
        self._warning_timestamps_ns[key] = now_ns
        self.get_logger().warning(message)


class RecordingSessionWriter:
    def __init__(
        self,
        *,
        session_dir: Path,
        fps: float,
        trajectory_hz: float,
        codec: str,
        write_combined: bool,
        stereo_left_topic: str,
        stereo_right_topic: str,
        wrist_stereo_left_topic: str,
        wrist_stereo_right_topic: str,
        wrist_topic: str,
        joint_states_topic: str,
        base_frame: str,
        end_effector_frame: str,
        gripper_max_width: float,
        trajectory_required: bool,
        control_input_topic: str = "/fr3_web_teleop/input",
        image_transport: str = "raw",
    ) -> None:
        self._session_dir = session_dir
        self._fps = fps
        self._trajectory_hz = trajectory_hz
        self._codec = codec
        self._write_combined = write_combined
        self._stereo_left_writer: cv2.VideoWriter | None = None
        self._stereo_right_writer: cv2.VideoWriter | None = None
        self._wrist_stereo_left_writer: cv2.VideoWriter | None = None
        self._wrist_stereo_right_writer: cv2.VideoWriter | None = None
        self._wrist_writer: cv2.VideoWriter | None = None
        self._combined_writer: cv2.VideoWriter | None = None
        self._stereo_left_size: tuple[int, int] | None = None
        self._stereo_right_size: tuple[int, int] | None = None
        self._wrist_stereo_left_size: tuple[int, int] | None = None
        self._wrist_stereo_right_size: tuple[int, int] | None = None
        self._wrist_size: tuple[int, int] | None = None
        self._combined_size: tuple[int, int] | None = None
        self._video_frames_file = None
        self._video_frames_csv = None
        self._trajectory_file = None
        self._trajectory_csv = None
        self._control_inputs_file = None
        self._control_inputs_csv = None
        self._started_at = datetime.now().isoformat(timespec="seconds")
        self._video_frame_count = 0
        self._trajectory_sample_count = 0
        self._control_input_count = 0
        self._first_video_record_time_ns: int | None = None
        self._last_video_record_time_ns: int | None = None
        self._first_trajectory_record_time_ns: int | None = None
        self._last_trajectory_record_time_ns: int | None = None
        self._pause_started_record_time_ns: int | None = None
        self._pause_intervals: list[dict[str, int | float]] = []
        self._source_timing = {
            name: {
                "previous_stamp_ns": None,
                "first_stamp_ns": None,
                "last_stamp_ns": None,
                "unique_timestamp_count": 0,
                "age_count": 0,
                "age_sum_ns": 0,
                "minimum_age_ns": None,
                "maximum_age_ns": None,
            }
            for name in (
                "joint_state",
                "end_effector_transform",
                "stereo_left_image",
                "stereo_right_image",
                "wrist_stereo_left_image",
                "wrist_stereo_right_image",
                "wrist_image",
            )
        }
        self._metadata = {
            "recording_schema": RECORDING_SCHEMA,
            "recording_format_version": RECORDING_FORMAT_VERSION,
            "minimum_reader_version": MINIMUM_READER_VERSION,
            "backward_compatible_with": [2, 3],
            "stereo_left_topic": stereo_left_topic,
            "stereo_right_topic": stereo_right_topic,
            "third_person_topic": stereo_left_topic,
            "third_person_stereo_left_topic": stereo_left_topic,
            "third_person_stereo_right_topic": stereo_right_topic,
            "wrist_stereo_left_topic": wrist_stereo_left_topic,
            "wrist_stereo_right_topic": wrist_stereo_right_topic,
            "wrist_topic": wrist_topic,
            "wrist_mono_topic": wrist_topic,
            "image_transport": image_transport,
            "joint_states_topic": joint_states_topic,
            "base_frame": base_frame,
            "end_effector_frame": end_effector_frame,
            "arm_joint_names": ARM_JOINT_NAMES,
            "joint_state_fields": ["position", "velocity", "effort"],
            "gripper_joint_names": GRIPPER_JOINT_NAMES,
            "gripper_max_width": gripper_max_width,
            "trajectory_required": trajectory_required,
            "control_input_topic": control_input_topic,
            "end_effector_pose_source": "tf2_latest_transform",
            "timing_contract": {
                "scheduler_clock": "steady_monotonic",
                "record_time_clock": "ros_node_clock",
                "source_timestamp_clock": "ros_message_header_or_tf_header",
                "timestamp_unit": "nanoseconds",
                "trajectory_alignment": "latest_observation_hold_at_recorder_tick",
                "video_alignment": "latest_encoded_video_frame",
                "control_input_alignment": "ros_receive_time_to_recording_origin",
                "video_playback_time": "video_frame_index / encoded_fps",
                "trajectory_requested_hz": trajectory_hz,
                "video_requested_fps": fps,
            },
        }

    @property
    def latest_video_frame_index(self) -> int:
        return self._video_frame_count - 1

    def pause(self, record_time_ns: int) -> None:
        if self._pause_started_record_time_ns is None:
            self._pause_started_record_time_ns = record_time_ns

    def resume(self, record_time_ns: int) -> None:
        if self._pause_started_record_time_ns is None:
            return
        start_ns = self._pause_started_record_time_ns
        end_ns = max(start_ns, record_time_ns)
        self._pause_intervals.append(
            {
                "start_record_time_ns": start_ns,
                "end_record_time_ns": end_ns,
                "duration_sec": (end_ns - start_ns) / 1e9,
            }
        )
        self._pause_started_record_time_ns = None

    def open(
        self,
        stereo_left: np.ndarray,
        stereo_right: np.ndarray,
        wrist_stereo_left: np.ndarray,
        wrist_stereo_right: np.ndarray,
        wrist: np.ndarray,
    ) -> None:
        self._session_dir.mkdir(parents=True, exist_ok=False)

        self._stereo_left_size = frame_size(stereo_left)
        self._stereo_right_size = frame_size(stereo_right)
        self._wrist_stereo_left_size = frame_size(wrist_stereo_left)
        self._wrist_stereo_right_size = frame_size(wrist_stereo_right)
        self._wrist_size = frame_size(wrist)
        fourcc = cv2.VideoWriter_fourcc(*self._codec[:4])

        self._stereo_left_writer = cv2.VideoWriter(
            str(self._session_dir / "stereo_left.mp4"),
            fourcc,
            self._fps,
            self._stereo_left_size,
        )
        self._stereo_right_writer = cv2.VideoWriter(
            str(self._session_dir / "stereo_right.mp4"),
            fourcc,
            self._fps,
            self._stereo_right_size,
        )
        self._wrist_stereo_left_writer = cv2.VideoWriter(
            str(self._session_dir / "wrist_stereo_left.mp4"),
            fourcc,
            self._fps,
            self._wrist_stereo_left_size,
        )
        self._wrist_stereo_right_writer = cv2.VideoWriter(
            str(self._session_dir / "wrist_stereo_right.mp4"),
            fourcc,
            self._fps,
            self._wrist_stereo_right_size,
        )
        self._wrist_writer = cv2.VideoWriter(
            str(self._session_dir / "wrist.mp4"),
            fourcc,
            self._fps,
            self._wrist_size,
        )

        if not self._stereo_left_writer.isOpened():
            raise RuntimeError("Failed to open stereo_left.mp4 VideoWriter.")
        if not self._stereo_right_writer.isOpened():
            raise RuntimeError("Failed to open stereo_right.mp4 VideoWriter.")
        if not self._wrist_stereo_left_writer.isOpened():
            raise RuntimeError("Failed to open wrist_stereo_left.mp4 VideoWriter.")
        if not self._wrist_stereo_right_writer.isOpened():
            raise RuntimeError("Failed to open wrist_stereo_right.mp4 VideoWriter.")
        if not self._wrist_writer.isOpened():
            raise RuntimeError("Failed to open wrist.mp4 VideoWriter.")

        if self._write_combined:
            combined = make_combined_frame(
                stereo_left,
                stereo_right,
                wrist_stereo_left,
                wrist_stereo_right,
                wrist,
            )
            self._combined_size = frame_size(combined)
            self._combined_writer = cv2.VideoWriter(
                str(self._session_dir / "combined.mp4"),
                fourcc,
                self._fps,
                self._combined_size,
            )
            if not self._combined_writer.isOpened():
                raise RuntimeError("Failed to open combined.mp4 VideoWriter.")

        self._video_frames_file = (self._session_dir / "video_frames.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._video_frames_csv = csv.writer(self._video_frames_file)
        self._video_frames_csv.writerow(
            [
                "video_frame_index",
                "record_time_ns",
                "record_time_sec",
                "record_elapsed_sec",
                "stereo_left_image_stamp_ns",
                "stereo_left_image_stamp_sec",
                "stereo_left_received_count",
                "stereo_right_image_stamp_ns",
                "stereo_right_image_stamp_sec",
                "stereo_right_received_count",
                "wrist_stereo_left_image_stamp_ns",
                "wrist_stereo_left_image_stamp_sec",
                "wrist_stereo_left_received_count",
                "wrist_stereo_right_image_stamp_ns",
                "wrist_stereo_right_image_stamp_sec",
                "wrist_stereo_right_received_count",
                "third_person_image_stamp_ns",
                "third_person_image_stamp_sec",
                "third_person_received_count",
                "wrist_image_stamp_ns",
                "wrist_image_stamp_sec",
                "wrist_received_count",
            ]
        )

        self._trajectory_file = (self._session_dir / "trajectory.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._trajectory_csv = csv.writer(self._trajectory_file)
        self._trajectory_csv.writerow(
            [
                "sample_index",
                "record_time_ns",
                "record_time_sec",
                "record_elapsed_sec",
                "latest_video_frame_index",
                "video_time_sec",
                "joint_state_received_count",
                "joint_state_stamp_ns",
                "joint_state_stamp_sec",
                "joint_state_age_ns",
                "ee_transform_stamp_ns",
                "ee_transform_stamp_sec",
                "ee_transform_age_ns",
                "stereo_left_image_stamp_ns",
                "stereo_left_image_stamp_sec",
                "stereo_left_image_age_ns",
                "stereo_right_image_stamp_ns",
                "stereo_right_image_stamp_sec",
                "stereo_right_image_age_ns",
                "wrist_stereo_left_image_stamp_ns",
                "wrist_stereo_left_image_stamp_sec",
                "wrist_stereo_left_image_age_ns",
                "wrist_stereo_right_image_stamp_ns",
                "wrist_stereo_right_image_stamp_sec",
                "wrist_stereo_right_image_age_ns",
                "third_person_image_stamp_ns",
                "third_person_image_stamp_sec",
                "wrist_image_stamp_ns",
                "wrist_image_stamp_sec",
                "wrist_image_age_ns",
                *ARM_JOINT_NAMES,
                "ee_x",
                "ee_y",
                "ee_z",
                "ee_qx",
                "ee_qy",
                "ee_qz",
                "ee_qw",
                "gripper_left",
                "gripper_right",
                "gripper_width",
                "gripper_open_ratio",
                *(f"{name}_velocity" for name in ARM_JOINT_NAMES),
                *(f"{name}_effort" for name in ARM_JOINT_NAMES),
            ]
        )

        self._control_inputs_file = (self._session_dir / "control_inputs.csv").open(
            "w", encoding="utf-8", newline=""
        )
        self._control_inputs_csv = csv.writer(self._control_inputs_file)
        self._control_inputs_csv.writerow(
            [
                "input_index",
                "record_time_ns",
                "record_time_sec",
                "record_elapsed_sec",
                "latest_video_frame_index",
                "video_time_sec",
                "sequence",
                "client_id",
                "active",
                "pointer_locked",
                "keys",
                "mouse_delta_x",
                "mouse_delta_y",
                "forward",
                "right",
                "up",
                "roll",
                "pitch",
                "yaw",
                "look_yaw",
                "look_pitch",
                "speed_scale",
                "gripper",
            ]
        )

    def write_video_frame(
        self,
        stereo_left: FrameSnapshot,
        stereo_right: FrameSnapshot,
        wrist_stereo_left: FrameSnapshot,
        wrist_stereo_right: FrameSnapshot,
        wrist: FrameSnapshot,
        *,
        record_time_ns: int,
    ) -> None:
        if (
            self._stereo_left_writer is None
            or self._stereo_right_writer is None
            or self._wrist_stereo_left_writer is None
            or self._wrist_stereo_right_writer is None
            or self._wrist_writer is None
            or self._video_frames_csv is None
        ):
            return

        stereo_left_image = ensure_size(stereo_left.image, self._stereo_left_size)
        stereo_right_image = ensure_size(stereo_right.image, self._stereo_right_size)
        wrist_stereo_left_image = ensure_size(
            wrist_stereo_left.image, self._wrist_stereo_left_size
        )
        wrist_stereo_right_image = ensure_size(
            wrist_stereo_right.image, self._wrist_stereo_right_size
        )
        wrist_image = ensure_size(wrist.image, self._wrist_size)
        self._stereo_left_writer.write(stereo_left_image)
        self._stereo_right_writer.write(stereo_right_image)
        self._wrist_stereo_left_writer.write(wrist_stereo_left_image)
        self._wrist_stereo_right_writer.write(wrist_stereo_right_image)
        self._wrist_writer.write(wrist_image)

        if self._combined_writer is not None:
            combined = ensure_size(
                make_combined_frame(
                    stereo_left_image,
                    stereo_right_image,
                    wrist_stereo_left_image,
                    wrist_stereo_right_image,
                    wrist_image,
                ),
                self._combined_size,
            )
            self._combined_writer.write(combined)

        self._video_frames_csv.writerow(
            [
                self._video_frame_count,
                record_time_ns,
                record_time_ns / 1e9,
                self._elapsed_sec(record_time_ns),
                stereo_left.stamp_ns,
                stereo_left.stamp_sec,
                stereo_left.received_count,
                stereo_right.stamp_ns,
                stereo_right.stamp_sec,
                stereo_right.received_count,
                wrist_stereo_left.stamp_ns,
                wrist_stereo_left.stamp_sec,
                wrist_stereo_left.received_count,
                wrist_stereo_right.stamp_ns,
                wrist_stereo_right.stamp_sec,
                wrist_stereo_right.received_count,
                stereo_left.stamp_ns,
                stereo_left.stamp_sec,
                stereo_left.received_count,
                wrist.stamp_ns,
                wrist.stamp_sec,
                wrist.received_count,
            ]
        )
        self._video_frames_file.flush()
        if self._first_video_record_time_ns is None:
            self._first_video_record_time_ns = record_time_ns
        self._last_video_record_time_ns = record_time_ns
        for source_name, snapshot in (
            ("stereo_left_image", stereo_left),
            ("stereo_right_image", stereo_right),
            ("wrist_stereo_left_image", wrist_stereo_left),
            ("wrist_stereo_right_image", wrist_stereo_right),
            ("wrist_image", wrist),
        ):
            self._track_source_timing(source_name, snapshot.stamp_ns, record_time_ns)
        self._video_frame_count += 1

    def write_trajectory_sample(self, sample: TrajectorySample) -> None:
        if self._trajectory_csv is None:
            return

        self._trajectory_csv.writerow(
            [
                self._trajectory_sample_count,
                sample.record_time_ns,
                sample.record_time_sec,
                self._elapsed_sec(sample.record_time_ns),
                sample.latest_video_frame_index,
                (
                    sample.latest_video_frame_index / self._fps
                    if sample.latest_video_frame_index >= 0
                    else None
                ),
                sample.joint_state_received_count,
                sample.joint_state_stamp_ns,
                sample.joint_state_stamp_sec,
                sample.record_time_ns - sample.joint_state_stamp_ns,
                sample.ee_transform_stamp_ns,
                sample.ee_transform_stamp_sec,
                sample.record_time_ns - sample.ee_transform_stamp_ns,
                sample.stereo_left_image_stamp_ns,
                sample.stereo_left_image_stamp_sec,
                sample.record_time_ns - sample.stereo_left_image_stamp_ns,
                sample.stereo_right_image_stamp_ns,
                sample.stereo_right_image_stamp_sec,
                sample.record_time_ns - sample.stereo_right_image_stamp_ns,
                sample.wrist_stereo_left_image_stamp_ns,
                sample.wrist_stereo_left_image_stamp_sec,
                sample.record_time_ns - sample.wrist_stereo_left_image_stamp_ns,
                sample.wrist_stereo_right_image_stamp_ns,
                sample.wrist_stereo_right_image_stamp_sec,
                sample.record_time_ns - sample.wrist_stereo_right_image_stamp_ns,
                sample.stereo_left_image_stamp_ns,
                sample.stereo_left_image_stamp_sec,
                sample.wrist_image_stamp_ns,
                sample.wrist_image_stamp_sec,
                sample.record_time_ns - sample.wrist_image_stamp_ns,
                *sample.joint_positions.tolist(),
                *sample.ee_position.tolist(),
                *sample.ee_quaternion_xyzw.tolist(),
                *sample.gripper_positions.tolist(),
                sample.gripper_width,
                sample.gripper_open_ratio,
                *sample.joint_velocities.tolist(),
                *sample.joint_efforts.tolist(),
            ]
        )
        self._trajectory_file.flush()
        if self._first_trajectory_record_time_ns is None:
            self._first_trajectory_record_time_ns = sample.record_time_ns
        self._last_trajectory_record_time_ns = sample.record_time_ns
        for source_name, stamp_ns in (
            ("joint_state", sample.joint_state_stamp_ns),
            ("end_effector_transform", sample.ee_transform_stamp_ns),
        ):
            self._track_source_timing(source_name, stamp_ns, sample.record_time_ns)
        self._trajectory_sample_count += 1

    def write_control_input(self, sample: ControlInputSample) -> None:
        if self._control_inputs_csv is None:
            return
        latest_video_frame_index = self.latest_video_frame_index
        self._control_inputs_csv.writerow(
            [
                self._control_input_count,
                sample.record_time_ns,
                sample.record_time_sec,
                self._elapsed_sec(sample.record_time_ns),
                latest_video_frame_index,
                latest_video_frame_index / self._fps if latest_video_frame_index >= 0 else None,
                sample.sequence,
                sample.client_id,
                int(sample.active),
                int(sample.pointer_locked),
                json.dumps(sample.keys, separators=(",", ":")),
                sample.mouse_delta_x,
                sample.mouse_delta_y,
                sample.forward,
                sample.right,
                sample.up,
                sample.roll,
                sample.pitch,
                sample.yaw,
                sample.look_yaw,
                sample.look_pitch,
                sample.speed_scale,
                sample.gripper,
            ]
        )
        self._control_inputs_file.flush()
        self._control_input_count += 1

    def _elapsed_sec(self, record_time_ns: int) -> float:
        origin_ns = self._first_video_record_time_ns
        if origin_ns is None:
            origin_ns = record_time_ns
        paused_ns = self._paused_overlap_ns(origin_ns, record_time_ns)
        return max(0.0, (record_time_ns - origin_ns - paused_ns) / 1e9)

    def _paused_overlap_ns(self, start_ns: int, end_ns: int) -> int:
        return sum(
            max(
                0,
                min(end_ns, int(interval["end_record_time_ns"]))
                - max(start_ns, int(interval["start_record_time_ns"])),
            )
            for interval in self._pause_intervals
        )

    def _track_source_timing(self, source_name: str, stamp_ns: int, record_time_ns: int) -> None:
        if stamp_ns <= 0:
            return
        timing = self._source_timing[source_name]
        if timing["previous_stamp_ns"] != stamp_ns:
            timing["previous_stamp_ns"] = stamp_ns
            timing["unique_timestamp_count"] += 1
            if timing["first_stamp_ns"] is None:
                timing["first_stamp_ns"] = stamp_ns
            timing["last_stamp_ns"] = stamp_ns

        age_ns = record_time_ns - stamp_ns
        timing["age_count"] += 1
        timing["age_sum_ns"] += age_ns
        minimum_age_ns = timing["minimum_age_ns"]
        maximum_age_ns = timing["maximum_age_ns"]
        timing["minimum_age_ns"] = age_ns if minimum_age_ns is None else min(minimum_age_ns, age_ns)
        timing["maximum_age_ns"] = age_ns if maximum_age_ns is None else max(maximum_age_ns, age_ns)

    def _rate_statistics(
        self,
        sample_count: int,
        first_time_ns: int | None,
        last_time_ns: int | None,
        requested_hz: float | None = None,
    ) -> dict[str, float | int | None]:
        duration_sec = None
        observed_hz = None
        if first_time_ns is not None and last_time_ns is not None and last_time_ns > first_time_ns:
            paused_ns = self._paused_overlap_ns(first_time_ns, last_time_ns)
            duration_sec = max(0.0, (last_time_ns - first_time_ns - paused_ns) / 1e9)
            if sample_count > 1:
                observed_hz = (sample_count - 1) / duration_sec if duration_sec > 0 else None
        return {
            "requested_hz": requested_hz,
            "sample_count": sample_count,
            "duration_sec": duration_sec,
            "observed_hz": observed_hz,
        }

    def _timing_statistics(self) -> dict[str, object]:
        sources = {}
        for source_name, timing in self._source_timing.items():
            source_stats = self._rate_statistics(
                int(timing["unique_timestamp_count"]),
                timing["first_stamp_ns"],
                timing["last_stamp_ns"],
            )
            age_count = int(timing["age_count"])
            source_stats.update(
                {
                    "unique_timestamp_count": int(timing["unique_timestamp_count"]),
                    "age_ms": {
                        "minimum": (
                            timing["minimum_age_ns"] / 1e6
                            if timing["minimum_age_ns"] is not None
                            else None
                        ),
                        "mean": timing["age_sum_ns"] / age_count / 1e6 if age_count else None,
                        "maximum": (
                            timing["maximum_age_ns"] / 1e6
                            if timing["maximum_age_ns"] is not None
                            else None
                        ),
                    },
                }
            )
            sources[source_name] = source_stats
        return {
            "video": self._rate_statistics(
                self._video_frame_count,
                self._first_video_record_time_ns,
                self._last_video_record_time_ns,
                self._fps,
            ),
            "trajectory": self._rate_statistics(
                self._trajectory_sample_count,
                self._first_trajectory_record_time_ns,
                self._last_trajectory_record_time_ns,
                self._trajectory_hz,
            ),
            "sources": sources,
        }

    def close(self, *, save: bool) -> None:
        ended_at = datetime.now().isoformat(timespec="seconds")

        for writer in (
            self._stereo_left_writer,
            self._stereo_right_writer,
            self._wrist_stereo_left_writer,
            self._wrist_stereo_right_writer,
            self._wrist_writer,
            self._combined_writer,
        ):
            if writer is not None:
                writer.release()

        for file_obj in (self._video_frames_file, self._trajectory_file, self._control_inputs_file):
            if file_obj is not None:
                file_obj.close()

        if save:
            payload = {
                "started_at": self._started_at,
                "ended_at": ended_at,
                "fps": self._fps,
                "trajectory_hz": self._trajectory_hz,
                "codec": self._codec,
                "video_frame_count": self._video_frame_count,
                "trajectory_sample_count": self._trajectory_sample_count,
                "control_input_count": self._control_input_count,
                "pause_count": len(self._pause_intervals),
                "paused_duration_sec": sum(
                    float(interval["duration_sec"]) for interval in self._pause_intervals
                ),
                "pause_intervals": self._pause_intervals,
                "record_time_origin_ns": self._first_video_record_time_ns,
                "timing_statistics": self._timing_statistics(),
                "recording_state": "complete",
                "stereo_left_video": "stereo_left.mp4",
                "stereo_right_video": "stereo_right.mp4",
                "third_person_video": "stereo_left.mp4",
                "third_person_stereo_left_video": "stereo_left.mp4",
                "third_person_stereo_right_video": "stereo_right.mp4",
                "wrist_stereo_left_video": "wrist_stereo_left.mp4",
                "wrist_stereo_right_video": "wrist_stereo_right.mp4",
                "wrist_video": "wrist.mp4",
                "wrist_mono_video": "wrist.mp4",
                "combined_video": "combined.mp4" if self._combined_writer is not None else None,
                "video_frames_csv": "video_frames.csv",
                "trajectory_csv": "trajectory.csv",
                "control_input_csv": "control_inputs.csv",
                **self._metadata,
            }
            metadata_path = self._session_dir / "metadata.json"
            temporary_metadata_path = self._session_dir / "metadata.json.tmp"
            with temporary_metadata_path.open("w", encoding="utf-8") as f:
                json.dump(payload, f, ensure_ascii=False, indent=2)
                f.write("\n")
            temporary_metadata_path.replace(metadata_path)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Record third-person stereo, wrist stereo, and wrist mono ROS2 image streams to MP4, "
            "and sample FR3 joint/pose trajectory data with timestamps."
        )
    )
    parser.add_argument("--stereo-left-topic", default="/stereo/left/image_raw")
    parser.add_argument("--stereo-right-topic", default="/stereo/right/image_raw")
    parser.add_argument("--wrist-stereo-left-topic", default="/wrist/stereo/left/image_raw")
    parser.add_argument("--wrist-stereo-right-topic", default="/wrist/stereo/right/image_raw")
    parser.add_argument(
        "--third-person-topic",
        dest="stereo_left_topic",
        default=argparse.SUPPRESS,
        help=argparse.SUPPRESS,
    )
    parser.add_argument("--wrist-topic", default="/mono/image_raw")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--control-input-topic", default="/fr3_web_teleop/input")
    parser.add_argument("--base-frame", default="fr3_link0")
    parser.add_argument("--end-effector-frame", default="fr3_hand_tcp")
    parser.add_argument("--output-root", default=str(Path.home() / "data" / "camera_recordings"))
    parser.add_argument("--session-prefix", default="")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--trajectory-hz", type=float, default=100.0)
    parser.add_argument("--gripper-max-width", type=float, default=0.08)
    parser.add_argument(
        "--require-trajectory",
        action=argparse.BooleanOptionalAction,
        default=False,
        help=(
            "Wait for FR3 arm joint states and end-effector TF before recording. "
            "Use --no-require-trajectory for camera-only sessions."
        ),
    )
    parser.add_argument("--codec", default="mp4v")
    parser.add_argument(
        "--image-qos-reliability",
        choices=("best_effort", "reliable"),
        default="best_effort",
        help="Reliability requested for the five image subscriptions.",
    )
    parser.add_argument(
        "--image-transport",
        choices=("raw", "compressed"),
        default="compressed",
        help="Subscribe to raw images or standard image_transport JPEG topics.",
    )
    parser.add_argument("--write-combined", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument(
        "--auto-start",
        action="store_true",
        help="Start recording as soon as all five camera streams have frames.",
    )
    parser.add_argument(
        "--record-seconds",
        type=float,
        default=0.0,
        help=(
            "Record for this many seconds and exit. This implies --auto-start and disables "
            "stdin keyboard handling, making the recorder safe for scripts and services."
        ),
    )
    return parser.parse_args()


def control_input_sample(payload: object, record_time_ns: int) -> ControlInputSample:
    if not isinstance(payload, dict):
        raise ValueError("payload must be an object")
    sequence = int(payload.get("sequence", -1))
    if sequence < 0:
        raise ValueError("sequence must be non-negative")
    active = payload.get("active", False)
    pointer_locked = payload.get("pointer_locked", False)
    if not isinstance(active, bool) or not isinstance(pointer_locked, bool):
        raise ValueError("active and pointer_locked must be booleans")
    keys = payload.get("keys", [])
    if (
        not isinstance(keys, list)
        or len(keys) > len(CONTROL_KEY_CODES)
        or any(not isinstance(key, str) or key not in CONTROL_KEY_CODES for key in keys)
    ):
        raise ValueError("keys contains an unsupported code")
    client_id = str(payload.get("client_id", ""))[:80]
    gripper = str(payload.get("gripper", "stop"))
    if gripper not in ("stop", "close", "open"):
        raise ValueError("unsupported gripper intent")
    numeric_fields = (
        "mouse_delta_x", "mouse_delta_y", "forward", "right", "up", "roll", "pitch",
        "yaw", "look_yaw", "look_pitch", "speed_scale",
    )
    values = {name: float(payload.get(name, 0.0)) for name in numeric_fields}
    if not all(np.isfinite(value) for value in values.values()):
        raise ValueError("control values must be finite")
    return ControlInputSample(
        record_time_sec=record_time_ns / 1e9,
        record_time_ns=record_time_ns,
        sequence=sequence,
        client_id=client_id,
        active=active,
        pointer_locked=pointer_locked,
        keys=sorted(set(keys)),
        gripper=gripper,
        **values,
    )


def copy_frame_slot(slot: FrameSlot) -> FrameSnapshot | None:
    if slot.image is None:
        return None
    # Callbacks replace the complete slot and never mutate an existing image. Keeping
    # this reference also keeps the ROS message buffer alive, while avoiding three
    # large copies under the shared lock on every recording tick.
    return FrameSnapshot(
        image=slot.image,
        stamp_sec=slot.stamp_sec,
        stamp_ns=slot.stamp_ns,
        received_count=slot.received_count,
    )


def make_image_qos(reliability: str) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=2,
        reliability=(
            ReliabilityPolicy.RELIABLE
            if reliability == "reliable"
            else ReliabilityPolicy.BEST_EFFORT
        ),
        durability=DurabilityPolicy.VOLATILE,
    )


def stamp_to_sec(msg: Image | CompressedImage | JointState) -> float:
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def stamp_to_ns(msg: Image | CompressedImage | JointState) -> int:
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


def image_transport_topic(base_topic: str, transport: str) -> str:
    return base_topic if transport == "raw" else f"{base_topic.rstrip('/')}/{transport}"


def ros_image_to_bgr(
    msg: Image | CompressedImage, warn: Callable[[str], None]
) -> np.ndarray | None:
    if isinstance(msg, CompressedImage):
        image = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if image is None:
            warn("Malformed compressed image buffer; skipping frame.")
        return image

    if msg.encoding not in ("bgr8", "rgb8", "mono8"):
        warn(f"Unsupported image encoding '{msg.encoding}'. Expected bgr8, rgb8, or mono8.")
        return None

    channels = 1 if msg.encoding == "mono8" else 3
    expected_row_bytes = int(msg.width) * channels
    if msg.step < expected_row_bytes:
        warn(f"Malformed image step={msg.step}, expected at least {expected_row_bytes}.")
        return None

    try:
        data = np.frombuffer(msg.data, dtype=np.uint8)
        image = data.reshape(int(msg.height), int(msg.step))
        image = image[:, :expected_row_bytes].reshape(int(msg.height), int(msg.width), channels)
    except ValueError:
        warn("Malformed image buffer; skipping frame.")
        return None

    if msg.encoding == "rgb8":
        image = image[..., ::-1]
    elif msg.encoding == "mono8":
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)

    # bgr8 frames without row padding can stay as a view of msg.data. NumPy keeps
    # that buffer alive after the callback, avoiding a full copy for every stream.
    return image if image.flags.c_contiguous else np.ascontiguousarray(image)


def make_session_dir(output_root: Path, session_prefix: str) -> Path:
    now = datetime.now()
    date_dir = output_root.expanduser().resolve() / now.strftime("%Y-%m-%d")
    base_name = now.strftime("%H-%M-%S")
    if session_prefix:
        base_name = f"{base_name}_{safe_path_part(session_prefix)}"

    session_dir = date_dir / base_name
    suffix = 1
    while session_dir.exists():
        session_dir = date_dir / f"{base_name}_{suffix:02d}"
        suffix += 1
    return session_dir


def safe_path_part(value: str) -> str:
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in value).strip("_")


def frame_size(frame: np.ndarray) -> tuple[int, int]:
    return int(frame.shape[1]), int(frame.shape[0])


def ensure_size(frame: np.ndarray, size: tuple[int, int] | None) -> np.ndarray:
    if size is None or frame_size(frame) == size:
        return frame
    return cv2.resize(frame, size, interpolation=cv2.INTER_AREA)


def make_combined_frame(
    stereo_left: np.ndarray,
    stereo_right: np.ndarray,
    wrist_stereo_left: np.ndarray,
    wrist_stereo_right: np.ndarray,
    wrist: np.ndarray,
) -> np.ndarray:
    cell_size = min(
        stereo_left.shape[0],
        stereo_right.shape[0],
        wrist_stereo_left.shape[0],
        wrist_stereo_right.shape[0],
        wrist.shape[0],
    )
    wrist_column = cv2.resize(wrist, (cell_size * 2, cell_size * 2), interpolation=cv2.INTER_AREA)
    wrist_stereo_column = np.concatenate(
        (
            cv2.resize(wrist_stereo_left, (cell_size, cell_size), interpolation=cv2.INTER_AREA),
            cv2.resize(wrist_stereo_right, (cell_size, cell_size), interpolation=cv2.INTER_AREA),
        ),
        axis=0,
    )
    third_person_column = np.concatenate(
        (
            cv2.resize(stereo_left, (cell_size, cell_size), interpolation=cv2.INTER_AREA),
            cv2.resize(stereo_right, (cell_size, cell_size), interpolation=cv2.INTER_AREA),
        ),
        axis=0,
    )
    return np.ascontiguousarray(
        np.concatenate((wrist_column, wrist_stereo_column, third_person_column), axis=1)
    )


def resize_to_height(frame: np.ndarray, target_height: int) -> np.ndarray:
    if frame.shape[0] == target_height:
        return frame
    scale = target_height / frame.shape[0]
    target_width = max(1, int(round(frame.shape[1] * scale)))
    return cv2.resize(frame, (target_width, target_height), interpolation=cv2.INTER_AREA)


def remove_session_dir(session_dir: Path) -> None:
    for path in sorted(session_dir.glob("*"), reverse=True):
        if path.is_file():
            path.unlink()
    session_dir.rmdir()


def keyboard_thread(command_queue: "queue.Queue[str]", stop_event: threading.Event) -> None:
    if not sys.stdin.isatty():
        line_input_thread(command_queue, stop_event)
        return

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while not stop_event.is_set():
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            char = sys.stdin.read(1).lower()
            if char in ("r", "s", "p", "d", "q"):
                command_queue.put(char)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def line_input_thread(command_queue: "queue.Queue[str]", stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        line = sys.stdin.readline()
        if not line:
            time.sleep(0.1)
            continue
        command = line.strip().lower()[:1]
        if command in ("r", "s", "p", "d", "q"):
            command_queue.put(command)


def print_controls() -> None:
    print()
    print("Controls:")
    print("  r  start recording")
    print("  p  pause or resume the current recording")
    print("  s  stop and save")
    print("  d  discard current recording")
    print("  q  quit (saves current recording if active)")
    print()
    sys.stdout.flush()


def wait_for_initial_frames(node: StereoWristRecorder, *, timeout_sec: float = 30.0) -> bool:
    start = time.monotonic()
    while rclpy.ok() and time.monotonic() - start < timeout_sec:
        counts = node.latest_counts()
        if all(count > 0 for count in counts):
            return True
        time.sleep(0.1)
    return False


def main() -> int:
    args = parse_args()
    if args.fps <= 0:
        print("--fps must be greater than 0.", file=sys.stderr)
        return 2
    if args.trajectory_hz <= 0:
        print("--trajectory-hz must be greater than 0.", file=sys.stderr)
        return 2
    if args.record_seconds < 0:
        print("--record-seconds must be greater than or equal to 0.", file=sys.stderr)
        return 2

    rclpy.init()
    node = StereoWristRecorder(args)
    # Image decode, robot state, TF, and control input callbacks are independent.
    # Ten workers prevent one large image stream from starving the other four.
    executor = MultiThreadedExecutor(num_threads=10)
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    command_queue: "queue.Queue[str]" = queue.Queue()
    stop_event = threading.Event()
    input_thread = None
    if args.record_seconds <= 0:
        input_thread = threading.Thread(
            target=keyboard_thread,
            args=(command_queue, stop_event),
            daemon=True,
        )
        input_thread.start()

    print_controls()
    print("Waiting for third-person stereo, wrist stereo, and wrist mono camera streams...")
    if wait_for_initial_frames(node):
        print("All five camera streams are ready.")
    else:
        print("Still waiting for camera frames. You can press 'r' after cameras are up.")

    auto_start_pending = args.auto_start or args.record_seconds > 0
    recording_started_at: float | None = None
    next_auto_start_attempt = 0.0
    if auto_start_pending:
        if node.start_recording() is not None:
            recording_started_at = time.monotonic()
            auto_start_pending = False

    video_period = 1.0 / args.fps
    trajectory_period = 1.0 / args.trajectory_hz
    next_video_tick = time.monotonic()
    next_trajectory_tick = time.monotonic()

    try:
        while rclpy.ok():
            try:
                command = command_queue.get_nowait()
            except queue.Empty:
                command = ""

            if command == "r":
                node.start_recording()
            elif command == "p":
                node.toggle_pause()
            elif command == "s":
                node.stop_and_save()
            elif command == "d":
                node.discard()
            elif command == "q":
                break

            now = time.monotonic()
            did_work = False

            if auto_start_pending and now >= next_auto_start_attempt:
                if node.start_recording() is not None:
                    recording_started_at = now
                    auto_start_pending = False
                next_auto_start_attempt = now + 1.0

            if (
                args.record_seconds > 0
                and recording_started_at is not None
                and now - recording_started_at >= args.record_seconds
            ):
                node.stop_and_save()
                break

            if now >= next_video_tick:
                node.tick_video()
                next_video_tick += video_period
                if now - next_video_tick > video_period:
                    next_video_tick = now + video_period
                did_work = True

            if now >= next_trajectory_tick:
                node.tick_trajectory()
                next_trajectory_tick += trajectory_period
                if now - next_trajectory_tick > trajectory_period:
                    next_trajectory_tick = now + trajectory_period
                did_work = True

            if not did_work:
                sleep_until = min(next_video_tick, next_trajectory_tick)
                time.sleep(min(0.02, max(0.0, sleep_until - now)))
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        node.shutdown()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        rclpy.try_shutdown()
        spin_thread.join(timeout=2.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
