#!/usr/bin/env python3
"""Publish deterministic camera, FR3 joint, gripper, and TF inputs for recorder smoke tests."""

from __future__ import annotations

import argparse
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState
from tf2_ros import TransformBroadcaster


ARM_JOINT_NAMES = [f"fr3_joint{index}" for index in range(1, 8)]
GRIPPER_JOINT_NAMES = ["fr3_finger_joint1", "fr3_finger_joint2"]


class SyntheticRecordingInputs(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("synthetic_recording_inputs")
        self._args = args
        self._sequence = 0
        self._left_publisher = self.create_publisher(
            Image, args.stereo_left_topic, qos_profile_sensor_data
        )
        self._right_publisher = self.create_publisher(
            Image, args.stereo_right_topic, qos_profile_sensor_data
        )
        self._wrist_publisher = self.create_publisher(
            Image, args.wrist_topic, qos_profile_sensor_data
        )
        self._joint_publisher = self.create_publisher(
            JointState, args.joint_states_topic, qos_profile_sensor_data
        )
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / args.fps, self._publish_sample)

    def _publish_image(self, publisher, stamp, seed: int) -> None:
        width = self._args.width
        height = self._args.height
        sequence_value = (self._sequence + seed) % 256
        pixel = bytes((sequence_value, (seed * 3) % 256, (255 - sequence_value) % 256))
        message = Image()
        message.header.stamp = stamp
        message.height = height
        message.width = width
        message.encoding = "bgr8"
        message.is_bigendian = 0
        message.step = width * 3
        message.data = pixel * (width * height)
        publisher.publish(message)

    def _publish_sample(self) -> None:
        stamp = self.get_clock().now().to_msg()
        phase = self._sequence / self._args.fps

        self._publish_image(self._left_publisher, stamp, 17)
        self._publish_image(self._right_publisher, stamp, 83)
        self._publish_image(self._wrist_publisher, stamp, 149)

        if self._args.camera_only:
            self._sequence += 1
            return

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = ARM_JOINT_NAMES + GRIPPER_JOINT_NAMES
        joint_state.position = [
            0.1 * joint_index + 0.01 * math.sin(phase)
            for joint_index in range(1, 8)
        ] + [0.02, 0.02]
        self._joint_publisher.publish(joint_state)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self._args.base_frame
        transform.child_frame_id = self._args.end_effector_frame
        transform.transform.translation.x = 0.4 + 0.01 * math.sin(phase)
        transform.transform.translation.y = 0.1
        transform.transform.translation.z = 0.5
        transform.transform.rotation.w = 1.0
        self._tf_broadcaster.sendTransform(transform)
        self._sequence += 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stereo-left-topic", default="/stereo/left/image_raw")
    parser.add_argument("--stereo-right-topic", default="/stereo/right/image_raw")
    parser.add_argument("--wrist-topic", default="/mono/image_raw")
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--base-frame", default="fr3_link0")
    parser.add_argument("--end-effector-frame", default="fr3_hand_tcp")
    parser.add_argument("--fps", type=float, default=30.0)
    parser.add_argument("--width", type=int, default=48)
    parser.add_argument("--height", type=int, default=32)
    parser.add_argument(
        "--camera-only",
        action="store_true",
        help="Omit joint states and TF to verify that trajectory-required recording stays blocked.",
    )
    args = parser.parse_args()
    if args.fps <= 0:
        parser.error("--fps must be greater than zero")
    if args.width <= 0 or args.height <= 0:
        parser.error("--width and --height must be greater than zero")
    return args


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = SyntheticRecordingInputs(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
