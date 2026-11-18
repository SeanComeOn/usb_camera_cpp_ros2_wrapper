from pathlib import Path
import unittest

import yaml


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BRINGUP_DIR = PROJECT_ROOT / "src" / "camera_bringup_ros2"
RVIZ_DIR = BRINGUP_DIR / "rviz"


class FpvRvizConfigTests(unittest.TestCase):
    def test_all_five_camera_streams_are_enabled_together(self) -> None:
        expected = {
            "Wrist Mono FPV": "/mono/image_raw/compressed",
            "Wrist Stereo FPV Left": "/wrist/stereo/left/image_raw/compressed",
            "Wrist Stereo FPV Right": "/wrist/stereo/right/image_raw/compressed",
            "Third-person Stereo Left": "/stereo/left/image_raw/compressed",
            "Third-person Stereo Right": "/stereo/right/image_raw/compressed",
        }
        for filename in (
            "dual_usb_cameras_stable.rviz",
            "dual_usb_cameras_with_franka.rviz",
        ):
            with self.subTest(filename=filename):
                config = yaml.safe_load((RVIZ_DIR / filename).read_text(encoding="utf-8"))
                displays = config["Visualization Manager"]["Displays"]
                images = {
                    display["Name"]: display["Topic"]["Value"]
                    for display in displays
                    if display.get("Class") == "rviz_default_plugins/Image"
                    and display.get("Enabled") is True
                }
                self.assertEqual(images, expected)
                self.assertNotIn("/stereo/image_raw", images.values())

    def test_identical_stereo_cameras_use_distinct_physical_paths(self) -> None:
        config_dir = BRINGUP_DIR / "config"
        third_person = yaml.safe_load(
            (config_dir / "beside_third_person_stereo_5883.yaml").read_text(encoding="utf-8")
        )
        wrist_stereo = yaml.safe_load(
            (config_dir / "beside_wrist_stereo_5883.yaml").read_text(encoding="utf-8")
        )
        third_person_path = third_person["camera"]["preferred_device"]
        wrist_stereo_path = wrist_stereo["camera"]["preferred_device"]

        self.assertEqual(
            third_person_path,
            "/dev/v4l/by-path/pci-0000:00:14.0-usb-0:8:1.0-video-index0",
        )
        self.assertNotEqual(third_person_path, wrist_stereo_path)

    def test_launch_device_paths_override_yaml_and_feed_watchdog(self) -> None:
        source = (BRINGUP_DIR / "launch" / "dual_usb_cameras_stable.launch.py").read_text(
            encoding="utf-8"
        )
        for device in ("wrist_usb_device", "stereo_usb_device", "wrist_stereo_usb_device"):
            with self.subTest(device=device):
                self.assertGreaterEqual(source.count(f'"device_path": {device}'), 1)
                self.assertGreaterEqual(source.count(f'"{device}": {device}'), 1)

    def test_nonempty_launch_device_path_overrides_yaml(self) -> None:
        for package in ("mono_camera_cpp_ros2", "bi_camera_cpp_ros2"):
            with self.subTest(package=package):
                node_name = "mono_camera_node.cpp" if package.startswith("mono") else "stereo_camera_node.cpp"
                source = (
                    PROJECT_ROOT / "src" / package / "src" / node_name
                ).read_text(encoding="utf-8")
                override_block = source.split("auto maybe_override_string", 1)[1].split("};", 1)[0]
                self.assertIn('get_parameter(name).as_string()', override_block)
                self.assertNotIn("override_names_.count(name)", override_block)


if __name__ == "__main__":
    unittest.main()
