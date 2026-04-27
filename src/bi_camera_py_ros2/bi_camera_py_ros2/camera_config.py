from __future__ import annotations

from copy import deepcopy
from pathlib import Path
from typing import Any

import yaml


DEFAULT_CONFIG: dict[str, Any] = {
    "camera": {
        "preferred_device": "/dev/video0",
        "vendor_id": "",
        "product_id": "",
        "usb_serial": "",
        "manufacturer": "",
        "product_name": "",
        "pixel_format": "MJPG",
        "width": 1280,
        "height": 480,
        "fps": 30.0,
        "split_stereo": True,
        "stereo_layout": "horizontal",
        "publish_combined": False,
        "left_frame_id": "stereo_left_optical_frame",
        "right_frame_id": "stereo_right_optical_frame",
        "combined_frame_id": "stereo_combined_optical_frame",
    },
    "ros": {
        "node_name": "bi_camera_publisher",
        "combined_topic": "image_raw",
        "left_topic": "left/image_raw",
        "right_topic": "right/image_raw",
        "fps_topic": "actual_fps",
    },
}


def deep_merge(base: dict[str, Any], override: dict[str, Any]) -> dict[str, Any]:
    merged = deepcopy(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def load_config_file(path: str | Path | None) -> dict[str, Any]:
    if not path:
        return deepcopy(DEFAULT_CONFIG)

    config_path = Path(path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config file not found: {config_path}")

    with config_path.open("r", encoding="utf-8") as handle:
        loaded = yaml.safe_load(handle) or {}

    if not isinstance(loaded, dict):
        raise ValueError(f"Config file must contain a YAML mapping: {config_path}")

    return deep_merge(DEFAULT_CONFIG, loaded)


def save_config_file(path: str | Path, config: dict[str, Any]) -> Path:
    config_path = Path(path)
    config_path.parent.mkdir(parents=True, exist_ok=True)
    with config_path.open("w", encoding="utf-8") as handle:
        yaml.safe_dump(config, handle, sort_keys=False, allow_unicode=False)
    return config_path
