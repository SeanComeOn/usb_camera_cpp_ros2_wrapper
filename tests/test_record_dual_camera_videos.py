from __future__ import annotations

import csv
import importlib.util
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np
from rclpy.qos import ReliabilityPolicy


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "record_dual_camera_videos.py"
SPEC = importlib.util.spec_from_file_location("record_dual_camera_videos", MODULE_PATH)
assert SPEC and SPEC.loader
recorder = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = recorder
SPEC.loader.exec_module(recorder)

VALIDATOR_PATH = Path(__file__).resolve().parents[1] / "scripts" / "validate_recording_dataset.py"
VALIDATOR_SPEC = importlib.util.spec_from_file_location("validate_recording_dataset", VALIDATOR_PATH)
assert VALIDATOR_SPEC and VALIDATOR_SPEC.loader
validator = importlib.util.module_from_spec(VALIDATOR_SPEC)
sys.modules[VALIDATOR_SPEC.name] = validator
VALIDATOR_SPEC.loader.exec_module(validator)


class FakeVideoWriter:
    def __init__(self, path: str, _fourcc: int, _fps: float, size: tuple[int, int]) -> None:
        self.path = Path(path)
        self.size = size
        self.frames: list[np.ndarray] = []
        self.released = False

    def isOpened(self) -> bool:  # noqa: N802
        return True

    def write(self, frame: np.ndarray) -> None:
        self.frames.append(frame.copy())

    def release(self) -> None:
        self.released = True


class RecordingSessionWriterTest(unittest.TestCase):
    def test_freshness_gate_allows_one_percent_transients(self) -> None:
        rows = [
            {
                "record_time_ns": str(index * 1_000_000_000),
                "image_stamp_ns": str(index * 1_000_000_000 - (120_000_000 if index == 1 else 20_000_000)),
            }
            for index in range(1, 101)
        ]
        result = validator.ValidationResult(Path("session"))

        validator._check_source_freshness(
            rows, "image", "image_stamp_ns", 100.0, 0.01, result
        )

        self.assertTrue(result.valid)
        self.assertEqual(len(result.warnings), 1)

        rows[1]["image_stamp_ns"] = str(2_000_000_000 - 120_000_000)
        result = validator.ValidationResult(Path("session"))
        validator._check_source_freshness(
            rows, "image", "image_stamp_ns", 100.0, 0.01, result
        )
        self.assertFalse(result.valid)

    def test_default_trajectory_rate_is_100_hz(self) -> None:
        with mock.patch.object(sys, "argv", [str(MODULE_PATH)]):
            args = recorder.parse_args()

        self.assertEqual(args.trajectory_hz, 100.0)
        self.assertEqual(args.image_transport, "compressed")

    def test_compressed_image_transport_decodes_jpeg(self) -> None:
        source = np.full((4, 6, 3), (10, 80, 160), dtype=np.uint8)
        ok, encoded = recorder.cv2.imencode(".jpg", source)
        self.assertTrue(ok)
        message = recorder.CompressedImage()
        message.format = "jpeg"
        message.data = encoded.tobytes()

        image = recorder.ros_image_to_bgr(message, self.fail)

        self.assertIsNotNone(image)
        self.assertEqual(image.shape, source.shape)
        self.assertEqual(
            recorder.image_transport_topic("/wrist/stereo/left/image_raw", "compressed"),
            "/wrist/stereo/left/image_raw/compressed",
        )

    def test_reliable_image_qos_uses_small_latest_frame_queue(self) -> None:
        qos = recorder.make_image_qos("reliable")

        self.assertEqual(qos.depth, 2)
        self.assertEqual(qos.reliability, ReliabilityPolicy.RELIABLE)

    def test_frame_snapshot_reuses_immutable_slot_image(self) -> None:
        image = np.zeros((4, 6, 3), dtype=np.uint8)
        slot = recorder.FrameSlot(image=image, stamp_sec=1.0, stamp_ns=1, received_count=2)

        snapshot = recorder.copy_frame_slot(slot)

        self.assertIsNotNone(snapshot)
        self.assertIs(snapshot.image, image)

    def test_contiguous_bgr_ros_image_uses_message_buffer_without_copy(self) -> None:
        message = recorder.Image()
        message.width = 6
        message.height = 4
        message.step = 18
        message.encoding = "bgr8"
        message.data = bytes(range(72))

        image = recorder.ros_image_to_bgr(message, self.fail)

        self.assertIsNotNone(image)
        self.assertTrue(image.flags.c_contiguous)
        self.assertTrue(np.shares_memory(image, np.frombuffer(message.data, dtype=np.uint8)))

    def test_control_input_payload_is_normalized_for_recording(self) -> None:
        sample = recorder.control_input_sample(
            {
                "sequence": 4,
                "client_id": "browser_a_123",
                "active": True,
                "pointer_locked": True,
                "keys": ["KeyW", "KeyC", "KeyW"],
                "mouse_delta_x": 3,
                "mouse_delta_y": -2,
                "forward": 1,
                "speed_scale": 0.4,
                "gripper": "close",
            },
            2_000_000_000,
        )

        self.assertEqual(sample.keys, ["KeyC", "KeyW"])
        self.assertEqual(sample.record_time_sec, 2.0)
        self.assertEqual(sample.forward, 1.0)
        with self.assertRaisesRegex(ValueError, "unsupported code"):
            recorder.control_input_sample({"sequence": 5, "keys": ["Escape"]}, 3)

    def test_three_stream_session_contract(self) -> None:
        left = np.full((4, 6, 3), 10, dtype=np.uint8)
        right = np.full((4, 6, 3), 20, dtype=np.uint8)
        wrist = np.full((6, 6, 3), 30, dtype=np.uint8)

        with tempfile.TemporaryDirectory() as temporary:
            session_dir = Path(temporary) / "session"
            created_writers: list[FakeVideoWriter] = []

            def make_writer(*args):
                writer = FakeVideoWriter(*args)
                created_writers.append(writer)
                return writer

            with mock.patch.object(recorder.cv2, "VideoWriter", side_effect=make_writer):
                writer = recorder.RecordingSessionWriter(
                    session_dir=session_dir,
                    fps=30.0,
                    trajectory_hz=10.0,
                    codec="mp4v",
                    write_combined=True,
                    stereo_left_topic="/stereo/left/image_raw",
                    stereo_right_topic="/stereo/right/image_raw",
                    wrist_stereo_left_topic="/wrist/stereo/left/image_raw",
                    wrist_stereo_right_topic="/wrist/stereo/right/image_raw",
                    wrist_topic="/mono/image_raw",
                    joint_states_topic="/joint_states",
                    base_frame="fr3_link0",
                    end_effector_frame="fr3_hand_tcp",
                    gripper_max_width=0.08,
                    trajectory_required=True,
                )
                writer.open(left, right, left, right, wrist)
                writer.write_video_frame(
                    recorder.FrameSnapshot(left, 1.98, 1_980_000_000, 1),
                    recorder.FrameSnapshot(right, 1.985, 1_985_000_000, 2),
                    recorder.FrameSnapshot(left, 1.982, 1_982_000_000, 3),
                    recorder.FrameSnapshot(right, 1.987, 1_987_000_000, 4),
                    recorder.FrameSnapshot(wrist, 1.99, 1_990_000_000, 3),
                    record_time_ns=2_000_000_000,
                )
                writer.write_trajectory_sample(
                    recorder.TrajectorySample(
                        record_time_sec=2.0,
                        record_time_ns=2_000_000_000,
                        latest_video_frame_index=0,
                        joint_state_received_count=4,
                        joint_state_stamp_sec=1.99,
                        joint_state_stamp_ns=1_990_000_000,
                        ee_transform_stamp_sec=1.995,
                        ee_transform_stamp_ns=1_995_000_000,
                        stereo_left_image_stamp_sec=1.98,
                        stereo_left_image_stamp_ns=1_980_000_000,
                        stereo_right_image_stamp_sec=1.985,
                        stereo_right_image_stamp_ns=1_985_000_000,
                        wrist_stereo_left_image_stamp_sec=1.982,
                        wrist_stereo_left_image_stamp_ns=1_982_000_000,
                        wrist_stereo_right_image_stamp_sec=1.987,
                        wrist_stereo_right_image_stamp_ns=1_987_000_000,
                        wrist_image_stamp_sec=1.99,
                        wrist_image_stamp_ns=1_990_000_000,
                        joint_positions=np.arange(1.0, 8.0),
                        joint_velocities=np.arange(0.1, 0.8, 0.1),
                        joint_efforts=np.arange(10.0, 17.0),
                        ee_position=np.asarray([0.4, 0.1, 0.5]),
                        ee_quaternion_xyzw=np.asarray([0.0, 0.0, 0.0, 1.0]),
                        gripper_positions=np.asarray([0.02, 0.02]),
                        gripper_width=0.04,
                        gripper_open_ratio=0.5,
                    )
                )
                writer.write_control_input(
                    recorder.ControlInputSample(
                        record_time_sec=2.005,
                        record_time_ns=2_005_000_000,
                        sequence=9,
                        client_id="browser_a_123",
                        active=True,
                        pointer_locked=True,
                        keys=["KeyD", "KeyW"],
                        mouse_delta_x=4.0,
                        mouse_delta_y=-2.0,
                        forward=1.0,
                        right=1.0,
                        up=0.0,
                        roll=0.0,
                        pitch=0.0,
                        yaw=0.0,
                        look_yaw=0.008,
                        look_pitch=0.004,
                        speed_scale=0.4,
                        gripper="stop",
                    )
                )
                writer.close(save=True)

            self.assertEqual(len(created_writers), 6)
            self.assertEqual([item.path.name for item in created_writers], [
                "stereo_left.mp4",
                "stereo_right.mp4",
                "wrist_stereo_left.mp4",
                "wrist_stereo_right.mp4",
                "wrist.mp4",
                "combined.mp4",
            ])
            self.assertTrue(all(len(item.frames) == 1 for item in created_writers))
            self.assertTrue(all(item.released for item in created_writers))
            self.assertEqual(created_writers[-1].frames[0].shape, (8, 16, 3))

            metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["recording_schema"], "fr3-multimodal-recording")
            self.assertEqual(metadata["recording_format_version"], 4)
            self.assertEqual(metadata["minimum_reader_version"], 2)
            self.assertEqual(metadata["recording_state"], "complete")
            self.assertEqual(metadata["stereo_left_video"], "stereo_left.mp4")
            self.assertEqual(metadata["stereo_right_video"], "stereo_right.mp4")
            self.assertEqual(metadata["wrist_stereo_left_video"], "wrist_stereo_left.mp4")
            self.assertEqual(metadata["wrist_stereo_right_video"], "wrist_stereo_right.mp4")
            self.assertEqual(metadata["wrist_video"], "wrist.mp4")
            self.assertEqual(metadata["third_person_video"], "stereo_left.mp4")
            self.assertEqual(metadata["video_frame_count"], 1)
            self.assertEqual(metadata["trajectory_sample_count"], 1)
            self.assertEqual(metadata["control_input_count"], 1)
            self.assertEqual(metadata["control_input_csv"], "control_inputs.csv")
            self.assertEqual(metadata["joint_state_fields"], ["position", "velocity", "effort"])
            self.assertTrue(metadata["trajectory_required"])
            self.assertEqual(metadata["timing_contract"]["trajectory_requested_hz"], 10.0)
            self.assertEqual(metadata["timing_statistics"]["trajectory"]["sample_count"], 1)
            self.assertEqual(
                metadata["timing_statistics"]["sources"]["joint_state"]["unique_timestamp_count"],
                1,
            )
            self.assertEqual(
                metadata["timing_statistics"]["sources"]["joint_state"]["age_ms"]["mean"],
                10.0,
            )

            with (session_dir / "video_frames.csv").open(newline="", encoding="utf-8") as file_obj:
                rows = list(csv.DictReader(file_obj))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["stereo_left_received_count"], "1")
            self.assertEqual(rows[0]["stereo_right_received_count"], "2")
            self.assertEqual(rows[0]["wrist_received_count"], "3")
            self.assertEqual(rows[0]["third_person_received_count"], "1")

            with (session_dir / "trajectory.csv").open(newline="", encoding="utf-8") as file_obj:
                trajectory_rows = list(csv.DictReader(file_obj))
            self.assertEqual(len(trajectory_rows), 1)
            self.assertEqual(trajectory_rows[0]["latest_video_frame_index"], "0")
            self.assertEqual(trajectory_rows[0]["video_time_sec"], "0.0")
            self.assertEqual(trajectory_rows[0]["record_elapsed_sec"], "0.0")
            self.assertEqual(trajectory_rows[0]["joint_state_received_count"], "4")
            self.assertEqual(trajectory_rows[0]["joint_state_age_ns"], "10000000")
            self.assertEqual(trajectory_rows[0]["ee_transform_age_ns"], "5000000")
            self.assertEqual(trajectory_rows[0]["fr3_joint7"], "7.0")
            self.assertAlmostEqual(float(trajectory_rows[0]["fr3_joint7_velocity"]), 0.7)
            self.assertEqual(trajectory_rows[0]["fr3_joint7_effort"], "16.0")
            self.assertEqual(trajectory_rows[0]["ee_x"], "0.4")
            self.assertEqual(trajectory_rows[0]["gripper_width"], "0.04")
            self.assertEqual(trajectory_rows[0]["gripper_open_ratio"], "0.5")

            with (session_dir / "control_inputs.csv").open(newline="", encoding="utf-8") as file_obj:
                control_rows = list(csv.DictReader(file_obj))
            self.assertEqual(len(control_rows), 1)
            self.assertEqual(json.loads(control_rows[0]["keys"]), ["KeyD", "KeyW"])
            self.assertEqual(control_rows[0]["sequence"], "9")
            self.assertEqual(control_rows[0]["latest_video_frame_index"], "0")

            validation = validator.validate_session(session_dir)
            self.assertTrue(validation.valid, validation.errors)
            self.assertEqual(validation.format_version, 4)

            control_path = session_dir / "control_inputs.csv"
            control_bytes = control_path.read_bytes()
            control_path.unlink()
            missing_control = validator.validate_session(session_dir)
            self.assertFalse(missing_control.valid)
            self.assertTrue(any("control_inputs.csv" in error for error in missing_control.errors))
            control_path.write_bytes(control_bytes)

            metadata["timing_statistics"]["sources"]["joint_state"]["observed_hz"] = 3.0
            (session_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
            held_source = validator.validate_session(session_dir)
            self.assertTrue(held_source.valid, held_source.errors)

            metadata["timing_statistics"]["sources"]["stereo_right_image"]["observed_hz"] = 3.0
            (session_dir / "metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
            rejected = validator.validate_session(session_dir)
            self.assertFalse(rejected.valid)
            self.assertTrue(any("stereo_right_image source" in error for error in rejected.errors))

    def test_pause_stitches_one_session_and_excludes_paused_time(self) -> None:
        frame = np.full((4, 6, 3), 10, dtype=np.uint8)
        with tempfile.TemporaryDirectory() as temporary:
            session_dir = Path(temporary) / "paused-session"
            with mock.patch.object(recorder.cv2, "VideoWriter", side_effect=FakeVideoWriter):
                writer = recorder.RecordingSessionWriter(
                    session_dir=session_dir,
                    fps=20.0,
                    trajectory_hz=100.0,
                    codec="mp4v",
                    write_combined=False,
                    stereo_left_topic="/stereo/left/image_raw",
                    stereo_right_topic="/stereo/right/image_raw",
                    wrist_stereo_left_topic="/wrist/stereo/left/image_raw",
                    wrist_stereo_right_topic="/wrist/stereo/right/image_raw",
                    wrist_topic="/mono/image_raw",
                    joint_states_topic="/joint_states",
                    base_frame="fr3_link0",
                    end_effector_frame="fr3_hand_tcp",
                    gripper_max_width=0.08,
                    trajectory_required=False,
                )
                writer.open(frame, frame, frame, frame, frame)
                writer.write_video_frame(
                    recorder.FrameSnapshot(frame, 2.0, 2_000_000_000, 1),
                    recorder.FrameSnapshot(frame, 2.0, 2_000_000_000, 1),
                    recorder.FrameSnapshot(frame, 2.0, 2_000_000_000, 1),
                    recorder.FrameSnapshot(frame, 2.0, 2_000_000_000, 1),
                    recorder.FrameSnapshot(frame, 2.0, 2_000_000_000, 1),
                    record_time_ns=2_000_000_000,
                )
                writer.pause(2_010_000_000)
                writer.resume(4_010_000_000)
                writer.write_video_frame(
                    recorder.FrameSnapshot(frame, 4.05, 4_050_000_000, 2),
                    recorder.FrameSnapshot(frame, 4.05, 4_050_000_000, 2),
                    recorder.FrameSnapshot(frame, 4.05, 4_050_000_000, 2),
                    recorder.FrameSnapshot(frame, 4.05, 4_050_000_000, 2),
                    recorder.FrameSnapshot(frame, 4.05, 4_050_000_000, 2),
                    record_time_ns=4_050_000_000,
                )
                writer.close(save=True)

            metadata = json.loads((session_dir / "metadata.json").read_text(encoding="utf-8"))
            self.assertEqual(metadata["pause_count"], 1)
            self.assertAlmostEqual(metadata["paused_duration_sec"], 2.0)
            self.assertAlmostEqual(metadata["timing_statistics"]["video"]["duration_sec"], 0.05)
            self.assertAlmostEqual(metadata["timing_statistics"]["video"]["observed_hz"], 20.0)
            with (session_dir / "video_frames.csv").open(newline="", encoding="utf-8") as file_obj:
                rows = list(csv.DictReader(file_obj))
            self.assertEqual(len(rows), 2)
            self.assertAlmostEqual(float(rows[1]["record_elapsed_sec"]), 0.05)


if __name__ == "__main__":
    unittest.main()
