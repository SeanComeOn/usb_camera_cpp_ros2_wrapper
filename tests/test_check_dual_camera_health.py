from __future__ import annotations

import argparse
import importlib.util
import sys
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "check_dual_camera_health.py"
SPEC = importlib.util.spec_from_file_location("check_dual_camera_health", MODULE_PATH)
assert SPEC and SPEC.loader
health = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = health
SPEC.loader.exec_module(health)


class CameraHealthTest(unittest.TestCase):
    def test_build_streams_includes_both_stereo_eyes_and_wrist(self) -> None:
        args = argparse.Namespace(
            stereo_left_topic="/left",
            stereo_right_topic="/right",
            mono_topic="/wrist",
            stereo_fps_topic="/stereo/fps",
            mono_fps_topic="/mono/fps",
            stereo_width=720,
            stereo_height=720,
            wrist_width=1080,
            wrist_height=1080,
            include_wrist_stereo=True,
            wrist_stereo_left_topic="/wrist/left",
            wrist_stereo_right_topic="/wrist/right",
            wrist_stereo_fps_topic="/wrist/stereo/fps",
            wrist_stereo_width=960,
            wrist_stereo_height=960,
        )

        streams = health.build_streams(args)

        self.assertEqual(
            set(streams),
            {"stereo_left", "stereo_right", "wrist", "wrist_stereo_left", "wrist_stereo_right"},
        )
        self.assertEqual(streams["stereo_right"].image_topic, "/right")
        self.assertEqual(streams["wrist"].expected_width, 1080)
        self.assertEqual(streams["wrist_stereo_left"].image_topic, "/wrist/left")
        self.assertEqual(streams["wrist_stereo_right"].expected_height, 960)

    def test_stream_health_requires_sustained_images_and_expected_size(self) -> None:
        stats = health.StreamStats(
            image_received=True,
            image_valid=True,
            width=720,
            height=720,
            fps_samples=[30.0] * 10,
            last_fps_received=10.0,
            image_count=31,
            first_image_received=9.0,
            last_image_received=10.0,
        )
        config = health.StreamConfig("/left", "/fps", 720, 720)

        self.assertTrue(
            health.stream_is_healthy(
                stats,
                config,
                finished_at=10.1,
                min_fps=27.0,
                min_fps_samples=10,
                min_image_samples=10,
                max_stall=2.0,
            )
        )

        stats.width = 640
        self.assertFalse(
            health.stream_is_healthy(
                stats,
                config,
                finished_at=10.1,
                min_fps=27.0,
                min_fps_samples=10,
                min_image_samples=10,
                max_stall=2.0,
            )
        )

    def test_source_heartbeat_not_monitor_decode_rate_controls_fps_gate(self) -> None:
        stats = health.StreamStats(
            image_received=True,
            image_valid=True,
            width=540,
            height=540,
            fps_samples=[30.0] * 10,
            last_fps_received=10.0,
            image_count=12,
            first_image_received=1.0,
            last_image_received=10.0,
        )
        config = health.StreamConfig("/right", "/fps", 540, 540)

        self.assertLess(stats.received_image_fps(), 2.0)
        self.assertTrue(
            health.stream_is_healthy(
                stats,
                config,
                finished_at=10.1,
                min_fps=27.0,
                min_fps_samples=10,
                min_image_samples=10,
                max_stall=2.0,
            )
        )

        stats.fps_samples = [20.0] * 10
        self.assertFalse(
            health.stream_is_healthy(
                stats,
                config,
                finished_at=10.1,
                min_fps=27.0,
                min_fps_samples=10,
                min_image_samples=10,
                max_stall=2.0,
            )
        )

    def test_build_streams_can_skip_wrist_camera(self) -> None:
        args = argparse.Namespace(
            stereo_left_topic="/left",
            stereo_right_topic="/right",
            mono_topic="/wrist",
            stereo_fps_topic="/stereo/fps",
            mono_fps_topic="/mono/fps",
            stereo_width=720,
            stereo_height=720,
            wrist_width=1080,
            wrist_height=1080,
            skip_mono=True,
            include_wrist_stereo=False,
        )

        streams = health.build_streams(args)

        self.assertEqual(set(streams), {"stereo_left", "stereo_right"})


if __name__ == "__main__":
    unittest.main()
