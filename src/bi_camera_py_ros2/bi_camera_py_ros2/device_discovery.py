from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from pathlib import Path

from .v4l2_caps import enumerate_formats, query_capabilities


@dataclass
class DeviceSummary:
    video_path: str
    name: str
    vendor_id: str
    product_id: str
    serial: str
    manufacturer: str
    product_name: str
    usb_path: str
    supports_capture: bool
    supports_streaming: bool

    def to_dict(self) -> dict[str, object]:
        return {
            "video_path": self.video_path,
            "name": self.name,
            "vendor_id": self.vendor_id,
            "product_id": self.product_id,
            "serial": self.serial,
            "manufacturer": self.manufacturer,
            "product_name": self.product_name,
            "usb_path": self.usb_path,
            "supports_capture": self.supports_capture,
            "supports_streaming": self.supports_streaming,
        }


def _read_sysfs_file(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8").strip()
    except OSError:
        return ""


def _find_usb_root(device_path: Path) -> Path | None:
    current = device_path.resolve()
    while current != current.parent:
        if (current / "idVendor").exists() and (current / "idProduct").exists():
            return current
        current = current.parent
    return None


def discover_video_devices() -> list[DeviceSummary]:
    devices: list[DeviceSummary] = []
    for video_sysfs in sorted(glob.glob("/sys/class/video4linux/video*")):
        video_dir = Path(video_sysfs)
        video_name = video_dir.name
        video_path = f"/dev/{video_name}"

        if not Path(video_path).exists():
            continue

        try:
            caps = query_capabilities(video_path)
        except OSError:
            continue

        usb_root = _find_usb_root(video_dir / "device")
        devices.append(
            DeviceSummary(
                video_path=video_path,
                name=_read_sysfs_file(video_dir / "name"),
                vendor_id=_read_sysfs_file(usb_root / "idVendor") if usb_root else "",
                product_id=_read_sysfs_file(usb_root / "idProduct") if usb_root else "",
                serial=_read_sysfs_file(usb_root / "serial") if usb_root else "",
                manufacturer=_read_sysfs_file(usb_root / "manufacturer") if usb_root else "",
                product_name=_read_sysfs_file(usb_root / "product") if usb_root else "",
                usb_path=str(usb_root) if usb_root else "",
                supports_capture=bool(caps["supports_capture"]),
                supports_streaming=bool(caps["supports_streaming"]),
            )
        )
    return devices


def find_matching_device(
    preferred_device: str = "",
    vendor_id: str = "",
    product_id: str = "",
    serial: str = "",
) -> DeviceSummary | None:
    devices = discover_video_devices()

    if preferred_device:
        for device in devices:
            if device.video_path == preferred_device and device.supports_capture:
                return device

    for device in devices:
        if not device.supports_capture:
            continue
        if vendor_id and device.vendor_id.lower() != vendor_id.lower():
            continue
        if product_id and device.product_id.lower() != product_id.lower():
            continue
        if serial and device.serial and device.serial != serial:
            continue
        if vendor_id or product_id or serial:
            return device

    for device in devices:
        if device.supports_capture:
            return device
    return None


def enumerate_capture_modes(device_path: str) -> list[dict[str, object]]:
    modes: list[dict[str, object]] = []
    for fmt in enumerate_formats(device_path):
        for frame_size in fmt.frame_sizes:
            modes.append(
                {
                    "pixel_format": fmt.pixel_format,
                    "description": fmt.description,
                    "width": frame_size.width,
                    "height": frame_size.height,
                    "intervals": [interval.to_dict() for interval in frame_size.intervals],
                }
            )
    modes.sort(
        key=lambda item: (
            0 if item["pixel_format"] == "MJPG" else 1,
            -int(item["width"]),
            -int(item["height"]),
        )
    )
    return modes
