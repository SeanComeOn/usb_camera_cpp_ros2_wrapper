from __future__ import annotations

import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = (
    Path(__file__).resolve().parents[1]
    / "src"
    / "camera_bringup_ros2"
    / "scripts"
    / "camera_stream_watchdog.py"
)
SPEC = importlib.util.spec_from_file_location("camera_stream_watchdog", MODULE_PATH)
assert SPEC and SPEC.loader
watchdog = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = watchdog
SPEC.loader.exec_module(watchdog)


class CameraStreamWatchdogTest(unittest.TestCase):
    def test_device_path_distinguishes_identical_usb_cameras(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            camera = Path(temporary) / "wrist-stereo"
            camera.touch()
            self.assertTrue(watchdog.usb_device_present(str(camera), "0bda:5883"))
            self.assertFalse(
                watchdog.usb_device_present(str(camera.with_name("third-person")), "0bda:5883")
            )

    def test_process_lookup_selects_one_ros_node_name(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            proc_root = Path(temporary)
            for pid, node_name in (
                ("101", "third_person_stereo_camera_publisher"),
                ("102", "wrist_stereo_camera_publisher"),
            ):
                process = proc_root / pid
                process.mkdir()
                (process / "exe").symlink_to("/opt/ros/lib/stereo_camera_node")
                (process / "cmdline").write_bytes(
                    f"stereo_camera_node\0--ros-args\0-r\0__node:={node_name}\0".encode()
                )
            original = watchdog.PROC_ROOT
            watchdog.PROC_ROOT = proc_root
            try:
                self.assertEqual(
                    watchdog.process_ids("stereo_camera_node", "wrist_stereo_camera_publisher"),
                    [102],
                )
            finally:
                watchdog.PROC_ROOT = original


if __name__ == "__main__":
    unittest.main()
