from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch


MODULE_PATH = Path(__file__).resolve().parents[1] / "scripts" / "orin_usb_camera_guard.py"
SPEC = importlib.util.spec_from_file_location("orin_usb_camera_guard", MODULE_PATH)
assert SPEC and SPEC.loader
guard = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = guard
SPEC.loader.exec_module(guard)


class UsbCameraGuardTest(unittest.TestCase):
    def test_root_hub_is_a_power_policy_target(self) -> None:
        self.assertEqual(guard.topology_names("usb2"), ["usb2"])

    def test_device_topology_walks_from_device_to_upstream_hubs(self) -> None:
        self.assertEqual(guard.topology_names("2-1.3.4"), ["2-1.3.4", "2-1.3", "2-1"])

    def test_root_hub_recovery_target_has_no_port(self) -> None:
        target = guard.parse_recovery_target("0bda:5883,usb2")

        self.assertEqual(target.identity, "0bda:5883")
        self.assertEqual(target.hub, "usb2")
        self.assertIsNone(target.port)

    def test_beside_does_not_reset_shared_usb2_root_hub(self) -> None:
        default_file = MODULE_PATH.parents[1] / "config" / "default" / "beside-usb-camera-guard"
        config = default_file.read_text(encoding="utf-8")

        self.assertIn("--expected 0bda:5883", config)
        self.assertNotIn("--recovery-target", config)
        self.assertNotIn(",usb2", config)


class StalledCameraTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        self.status = self.root / 'wrist.json'
        self.target = guard.parse_stalled_camera(f'1e45:0209,serial,{self.status},/mono/image_raw')
        self.device_path = self.root / '1-1'
        self.device_path.mkdir()
        for key, value in {'serial':'serial','busnum':'1','devnum':'31'}.items():
            (self.device_path/key).write_text(value)
        self.device = guard.UsbDevice('1-1', self.device_path, '1e45', '0209')
        self.recovery = guard.StalledCameraRecovery(self.target)
        self.heartbeat(900)

    def tearDown(self):
        self.tmp.cleanup()

    def heartbeat(self, stamp, **extra):
        self.status.write_text(json.dumps({'topic':'/mono/image_raw','source_stamp_ns':int(stamp*1e9), **extra}))

    def test_uncommissioned_device_or_relative_path_is_rejected(self):
        for value in ('0bda:5883,serial,/tmp/status,/mono/image_raw',
                      '1e45:0209,serial,status,/mono/image_raw',
                      '1e45:0209,,/tmp/status,/mono/image_raw',
                      '1e45:0209,serial,/tmp/status,/stereo/image_raw'):
            with self.assertRaises(ValueError):
                guard.parse_stalled_camera(value)

    def test_identity_serial_and_ambiguity_are_enforced(self):
        self.assertEqual(guard.matching_camera(self.target, [self.device]), self.device)
        self.assertIsNone(guard.matching_camera(self.target, [self.device, self.device]))
        (self.device_path/'serial').write_text('wrong')
        self.assertIsNone(guard.matching_camera(self.target, [self.device]))

    def test_heartbeat_rejects_wrong_topic_invalid_data_and_symlink(self):
        self.assertEqual(guard.camera_status_stamp(self.target), 900)
        for data in ({'topic':'/wrong','source_stamp_ns':900_000_000_000},
                     {'topic':'/mono/image_raw','source_stamp_ns':True}, [], {}):
            self.status.write_text(json.dumps(data))
            self.assertIsNone(guard.camera_status_stamp(self.target))
        self.status.write_text(' ' * 17000)
        self.assertIsNone(guard.camera_status_stamp(self.target))
        self.status.unlink(); self.status.symlink_to(self.root/'missing')
        self.assertIsNone(guard.camera_status_stamp(self.target))

    def test_heartbeat_fifo_is_nonblocking_and_rejected(self):
        self.status.unlink();os.mkfifo(self.status)
        self.assertIsNone(guard.camera_status_stamp(self.target))

    def test_fresh_stream_missing_device_or_short_gap_never_reset(self):
        with patch.object(guard, 'subprocess') as process, patch.object(guard, 'mono_publisher_running',return_value=True):
            self.recovery.check([], 1000, 1)
            self.heartbeat(999.9);self.recovery.check([self.device],1000,2)
            self.heartbeat(990);self.recovery.check([self.device],1000,3)
            process.run.assert_not_called()

    def test_stale_preview_without_usb_evidence_is_not_reset(self):
        with patch.object(guard, 'recent_usb_timeout',return_value=False), \
             patch.object(guard, 'mono_publisher_running',return_value=True), \
             patch.object(guard.subprocess,'run') as run:
            self.recovery.check([self.device],1000,1)
            run.assert_not_called()
            self.assertIn('without recent',self.recovery.last_state)

    def test_stopped_publisher_does_not_reset(self):
        with patch.object(guard, 'mono_publisher_running',return_value=False), \
             patch.object(guard, 'recent_usb_timeout') as evidence:
            self.recovery.check([self.device],1000,1)
            evidence.assert_not_called()

    def test_timeout_evidence_must_match_exact_port(self):
        for line,expected in [('uvcvideo 1-1:1.1: Failed to set UVC probe control : -110 (exp. 26).', True),
                              ('usb 1-1: Failed to query control: -110 (exp. 1).',True),
                              ('uvcvideo 1-10:1.1: probe control : -110',False),
                              ('usb 1-1: USB disconnect',False)]:
            with patch.object(guard.subprocess,'run',return_value=Mock(returncode=0,stdout=line)):
                self.assertEqual(guard.recent_usb_timeout(self.device,900),expected,line)

    def test_two_resets_with_cooldown_and_only_fresh_recovery_rearms(self):
        with patch.object(guard, 'recent_usb_timeout',return_value=True), \
             patch.object(guard, 'mono_publisher_running',return_value=True), \
             patch.object(guard.subprocess,'run',return_value=Mock(returncode=0)) as run:
            for tick in (1,2,120,121,242,1000):
                self.recovery.check([self.device],1000,tick)
            self.assertEqual(run.call_count,2)
            self.assertEqual(run.call_args.args[0][-2],'--reset-single-camera')
            self.assertEqual(run.call_args.kwargs['timeout'],45)
            self.assertIn('exhausted',self.recovery.last_state)
            self.heartbeat(999.9)
            self.recovery.check([self.device],1000,1001)
            self.recovery.check([self.device],1000,1061)
            self.assertEqual(self.recovery.attempts,0)

    def test_reset_timeout_is_bounded_and_not_reported_as_recovered(self):
        with patch.object(guard, 'recent_usb_timeout',return_value=True), \
             patch.object(guard, 'mono_publisher_running',return_value=True), \
             patch.object(guard.subprocess,'run',side_effect=subprocess.TimeoutExpired('reset',45)):
            self.recovery.check([self.device],1000,1)
            self.assertEqual(self.recovery.attempts,1)
            self.assertIn('timed out',self.recovery.last_state)

    def test_ioctl_checks_opened_descriptor_before_reset(self):
        bus=self.root/'bus';(bus/'001').mkdir(parents=True)
        node=bus/'001/031'
        descriptor=bytearray(18);descriptor[1]=1;descriptor[8:10]=(0x1e45).to_bytes(2,'little');descriptor[10:12]=(0x0209).to_bytes(2,'little')
        node.write_bytes(descriptor)
        with patch.object(guard,'USB_BUS',bus), \
             patch.object(guard,'discover_usb_devices',return_value=[self.device]), \
             patch.object(guard.fcntl,'ioctl') as ioctl:
            self.assertTrue(guard.reset_single_camera(self.target))
            ioctl.assert_called_once()
            ioctl.reset_mock();descriptor[8]=0;node.write_bytes(descriptor)
            self.assertFalse(guard.reset_single_camera(self.target));ioctl.assert_not_called()


if __name__ == "__main__":
    unittest.main()
