from __future__ import annotations

import argparse
from pathlib import Path
from typing import Any

from .camera_config import DEFAULT_CONFIG, deep_merge, save_config_file
from .device_discovery import DeviceSummary, discover_video_devices, enumerate_capture_modes


def _prompt_choice(prompt: str, upper_bound: int, default_index: int = 0) -> int:
    default_display = default_index + 1
    while True:
        raw = input(f"{prompt} [{default_display}]: ").strip()
        if not raw:
            return default_index
        if raw.isdigit():
            value = int(raw) - 1
            if 0 <= value < upper_bound:
                return value
        print(f"请输入 1 到 {upper_bound} 之间的编号。")


def _prompt_text(prompt: str, default: str) -> str:
    raw = input(f"{prompt} [{default}]: ").strip()
    return raw or default


def _default_mode_index(modes: list[dict[str, Any]]) -> int:
    preferred_sizes = [
        (1280, 480),
        (2560, 720),
        (3840, 1080),
        (640, 480),
    ]
    for width, height in preferred_sizes:
        for index, mode in enumerate(modes):
            if mode["pixel_format"] != "MJPG":
                continue
            if int(mode["width"]) != width or int(mode["height"]) != height:
                continue
            if any(item.get("fps") == 30.0 for item in mode["intervals"]):
                return index

    for index, mode in enumerate(modes):
        if mode["pixel_format"] != "MJPG":
            continue
        intervals = mode["intervals"]
        if any(item.get("fps") == 30.0 for item in intervals):
            return index
    for index, mode in enumerate(modes):
        if mode["pixel_format"] == "MJPG":
            return index
    return 0


def _default_fps(intervals: list[dict[str, Any]]) -> float:
    fps_values = [item.get("fps") for item in intervals if item.get("fps") is not None]
    if 30.0 in fps_values:
        return 30.0
    if fps_values:
        return max(fps_values)

    ranged = intervals[0] if intervals else {}
    if ranged.get("fps_max"):
        return float(ranged["fps_max"])
    return 30.0


def _format_interval(interval: dict[str, Any]) -> str:
    if "fps" in interval:
        return f'{interval["fps"]:.3f} fps'
    return (
        f'range {interval.get("fps_min", 0.0):.3f}'
        f' - {interval.get("fps_max", 0.0):.3f} fps'
        f' step {interval.get("fps_step", 0.0):.3f}'
    )


def _select_device(devices: list[DeviceSummary], requested_path: str = "") -> DeviceSummary:
    capture_devices = [device for device in devices if device.supports_capture]
    if not capture_devices:
        raise RuntimeError("没有发现可用于采集的 /dev/video* 设备。")

    if requested_path:
        for device in capture_devices:
            if device.video_path == requested_path:
                return device
        raise RuntimeError(f"指定设备不存在或不可采集: {requested_path}")

    print("发现以下可采集视频设备：")
    for index, device in enumerate(capture_devices, start=1):
        print(
            f"  {index}. {device.video_path} | {device.name} | "
            f"VID:PID={device.vendor_id}:{device.product_id} | "
            f"{device.manufacturer} {device.product_name}".strip()
        )

    selection = _prompt_choice("请选择相机设备编号", len(capture_devices))
    return capture_devices[selection]


def _select_mode(device: DeviceSummary, requested_pixel_format: str = "") -> tuple[dict[str, Any], float]:
    modes = enumerate_capture_modes(device.video_path)
    if not modes:
        raise RuntimeError(f"无法读取设备模式: {device.video_path}")

    filtered = modes
    if requested_pixel_format:
        filtered = [item for item in modes if item["pixel_format"] == requested_pixel_format]
        if not filtered:
            raise RuntimeError(f"{device.video_path} 不支持像素格式 {requested_pixel_format}")

    print("")
    print(f"{device.video_path} 支持的模式如下：")
    for index, mode in enumerate(filtered, start=1):
        fps_text = ", ".join(_format_interval(interval) for interval in mode["intervals"]) or "未枚举到帧率"
        print(
            f"  {index}. {mode['pixel_format']} {mode['width']}x{mode['height']} "
            f"| {mode['description']} | {fps_text}"
        )

    default_index = _default_mode_index(filtered)
    selection = _prompt_choice("请选择分辨率/格式编号", len(filtered), default_index=default_index)
    selected_mode = filtered[selection]
    default_fps = _default_fps(selected_mode["intervals"])
    fps = float(_prompt_text("请输入目标 FPS", f"{default_fps:.3f}"))
    return selected_mode, fps


def _build_config(device: DeviceSummary, mode: dict[str, Any], fps: float, layout: str) -> dict[str, Any]:
    return deep_merge(
        DEFAULT_CONFIG,
        {
            "camera": {
                "preferred_device": device.video_path,
                "vendor_id": device.vendor_id,
                "product_id": device.product_id,
                "usb_serial": device.serial,
                "manufacturer": device.manufacturer,
                "product_name": device.product_name,
                "pixel_format": mode["pixel_format"],
                "width": int(mode["width"]),
                "height": int(mode["height"]),
                "fps": float(fps),
                "split_stereo": True,
                "stereo_layout": layout,
                "publish_combined": False,
            }
        },
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Enumerate USB cameras and save a YAML config for ROS 2 stereo publishing.")
    parser.add_argument("--output", default="config/bi_camera.yaml", help="Output YAML path.")
    parser.add_argument("--device", default="", help="Explicit /dev/videoX path.")
    parser.add_argument("--pixel-format", default="MJPG", help="Pixel format to prefer, default MJPG.")
    parser.add_argument(
        "--layout",
        default="horizontal",
        choices=["horizontal", "vertical", "mono"],
        help="Stereo split layout.",
    )
    args = parser.parse_args()

    devices = discover_video_devices()
    device = _select_device(devices, requested_path=args.device)
    mode, fps = _select_mode(device, requested_pixel_format=args.pixel_format)
    config = _build_config(device, mode, fps, args.layout)
    output_path = save_config_file(args.output, config)

    print("")
    print(f"已写入配置: {output_path.resolve()}")
    print(f"选择设备: {device.video_path} | VID:PID={device.vendor_id}:{device.product_id}")
    print(
        f"选择模式: {mode['pixel_format']} {mode['width']}x{mode['height']} @ {fps:.3f} fps | "
        f"layout={args.layout}"
    )


if __name__ == "__main__":
    main()
