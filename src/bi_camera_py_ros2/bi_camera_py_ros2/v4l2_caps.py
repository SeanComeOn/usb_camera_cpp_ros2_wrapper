from __future__ import annotations

import ctypes
import errno
import fcntl
import os
from dataclasses import dataclass


V4L2_BUF_TYPE_VIDEO_CAPTURE = 1
V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE = 9

V4L2_CAP_VIDEO_CAPTURE = 0x00000001
V4L2_CAP_VIDEO_CAPTURE_MPLANE = 0x00001000
V4L2_CAP_STREAMING = 0x04000000
V4L2_CAP_DEVICE_CAPS = 0x80000000

V4L2_FRMSIZE_TYPE_DISCRETE = 1
V4L2_FRMSIZE_TYPE_CONTINUOUS = 2
V4L2_FRMSIZE_TYPE_STEPWISE = 3

V4L2_FRMIVAL_TYPE_DISCRETE = 1
V4L2_FRMIVAL_TYPE_CONTINUOUS = 2
V4L2_FRMIVAL_TYPE_STEPWISE = 3


def _ioc(direction: int, type_char: str, number: int, size: int) -> int:
    ioc_nrbits = 8
    ioc_typebits = 8
    ioc_sizebits = 14
    ioc_nrshift = 0
    ioc_typeshift = ioc_nrshift + ioc_nrbits
    ioc_sizeshift = ioc_typeshift + ioc_typebits
    ioc_dirshift = ioc_sizeshift + ioc_sizebits
    return (
        (direction << ioc_dirshift)
        | (ord(type_char) << ioc_typeshift)
        | (number << ioc_nrshift)
        | (size << ioc_sizeshift)
    )


def _ior(type_char: str, number: int, struct_type: type[ctypes.Structure]) -> int:
    return _ioc(2, type_char, number, ctypes.sizeof(struct_type))


def _iowr(type_char: str, number: int, struct_type: type[ctypes.Structure]) -> int:
    return _ioc(3, type_char, number, ctypes.sizeof(struct_type))


class V4L2Capability(ctypes.Structure):
    _fields_ = [
        ("driver", ctypes.c_uint8 * 16),
        ("card", ctypes.c_uint8 * 32),
        ("bus_info", ctypes.c_uint8 * 32),
        ("version", ctypes.c_uint32),
        ("capabilities", ctypes.c_uint32),
        ("device_caps", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class V4L2Fmtdesc(ctypes.Structure):
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
        ("description", ctypes.c_uint8 * 32),
        ("pixelformat", ctypes.c_uint32),
        ("mbus_code", ctypes.c_uint32),
        ("reserved", ctypes.c_uint32 * 3),
    ]


class V4L2Fract(ctypes.Structure):
    _fields_ = [
        ("numerator", ctypes.c_uint32),
        ("denominator", ctypes.c_uint32),
    ]


class V4L2FrmsizeDiscrete(ctypes.Structure):
    _fields_ = [
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
    ]


class V4L2FrmsizeStepwise(ctypes.Structure):
    _fields_ = [
        ("min_width", ctypes.c_uint32),
        ("max_width", ctypes.c_uint32),
        ("step_width", ctypes.c_uint32),
        ("min_height", ctypes.c_uint32),
        ("max_height", ctypes.c_uint32),
        ("step_height", ctypes.c_uint32),
    ]


class V4L2FrmsizeUnion(ctypes.Union):
    _fields_ = [
        ("discrete", V4L2FrmsizeDiscrete),
        ("stepwise", V4L2FrmsizeStepwise),
    ]


class V4L2Frmsizeenum(ctypes.Structure):
    _anonymous_ = ("size",)
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("pixel_format", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("size", V4L2FrmsizeUnion),
        ("reserved", ctypes.c_uint32 * 2),
    ]


class V4L2FrmivalStepwise(ctypes.Structure):
    _fields_ = [
        ("min", V4L2Fract),
        ("max", V4L2Fract),
        ("step", V4L2Fract),
    ]


class V4L2FrmivalUnion(ctypes.Union):
    _fields_ = [
        ("discrete", V4L2Fract),
        ("stepwise", V4L2FrmivalStepwise),
    ]


class V4L2Frmivalenum(ctypes.Structure):
    _anonymous_ = ("interval",)
    _fields_ = [
        ("index", ctypes.c_uint32),
        ("pixel_format", ctypes.c_uint32),
        ("width", ctypes.c_uint32),
        ("height", ctypes.c_uint32),
        ("type", ctypes.c_uint32),
        ("interval", V4L2FrmivalUnion),
        ("reserved", ctypes.c_uint32 * 2),
    ]


VIDIOC_QUERYCAP = _ior("V", 0, V4L2Capability)
VIDIOC_ENUM_FMT = _iowr("V", 2, V4L2Fmtdesc)
VIDIOC_ENUM_FRAMESIZES = _iowr("V", 74, V4L2Frmsizeenum)
VIDIOC_ENUM_FRAMEINTERVALS = _iowr("V", 75, V4L2Frmivalenum)


@dataclass
class FrameInterval:
    fps: float | None
    fps_min: float | None = None
    fps_max: float | None = None
    fps_step: float | None = None

    def to_dict(self) -> dict[str, float]:
        if self.fps is not None:
            return {"fps": round(self.fps, 3)}
        return {
            "fps_min": round(self.fps_min or 0.0, 3),
            "fps_max": round(self.fps_max or 0.0, 3),
            "fps_step": round(self.fps_step or 0.0, 3),
        }


@dataclass
class FrameSize:
    width: int
    height: int
    intervals: list[FrameInterval]

    def to_dict(self) -> dict[str, object]:
        return {
            "width": self.width,
            "height": self.height,
            "intervals": [item.to_dict() for item in self.intervals],
        }


@dataclass
class PixelFormat:
    pixel_format: str
    description: str
    frame_sizes: list[FrameSize]

    def to_dict(self) -> dict[str, object]:
        return {
            "pixel_format": self.pixel_format,
            "description": self.description,
            "frame_sizes": [item.to_dict() for item in self.frame_sizes],
        }


def _cstring(value: ctypes.Array[ctypes.c_uint8]) -> str:
    return bytes(value).split(b"\x00", 1)[0].decode("utf-8", errors="ignore")


def _ioctl_struct(fd: int, request: int, struct_obj: ctypes.Structure) -> None:
    size = ctypes.sizeof(struct_obj)
    buffer = bytearray(ctypes.string_at(ctypes.addressof(struct_obj), size))
    fcntl.ioctl(fd, request, buffer, True)
    ctypes.memmove(ctypes.addressof(struct_obj), bytes(buffer), size)


def fourcc_to_string(code: int) -> str:
    return "".join(chr((code >> (8 * offset)) & 0xFF) for offset in range(4))


def _fract_to_fps(fract: V4L2Fract) -> float | None:
    if fract.numerator == 0:
        return None
    return fract.denominator / fract.numerator


def query_capabilities(device_path: str) -> dict[str, object]:
    fd = os.open(device_path, os.O_RDWR | os.O_NONBLOCK)
    try:
        capability = V4L2Capability()
        _ioctl_struct(fd, VIDIOC_QUERYCAP, capability)
    finally:
        os.close(fd)

    caps = capability.device_caps if capability.capabilities & V4L2_CAP_DEVICE_CAPS else capability.capabilities
    return {
        "driver": _cstring(capability.driver),
        "card": _cstring(capability.card),
        "bus_info": _cstring(capability.bus_info),
        "version": capability.version,
        "capabilities": caps,
        "supports_capture": bool(caps & (V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_VIDEO_CAPTURE_MPLANE)),
        "supports_streaming": bool(caps & V4L2_CAP_STREAMING),
    }


def enumerate_formats(device_path: str) -> list[PixelFormat]:
    fd = os.open(device_path, os.O_RDWR | os.O_NONBLOCK)
    try:
        formats: list[PixelFormat] = []
        index = 0
        while True:
            format_desc = V4L2Fmtdesc()
            format_desc.index = index
            format_desc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
            try:
                _ioctl_struct(fd, VIDIOC_ENUM_FMT, format_desc)
            except OSError as exc:
                if exc.errno == errno.EINVAL:
                    break
                raise

            frame_sizes = _enumerate_frame_sizes(fd, format_desc.pixelformat)
            formats.append(
                PixelFormat(
                    pixel_format=fourcc_to_string(format_desc.pixelformat),
                    description=_cstring(format_desc.description),
                    frame_sizes=frame_sizes,
                )
            )
            index += 1
        return formats
    finally:
        os.close(fd)


def _enumerate_frame_sizes(fd: int, pixel_format: int) -> list[FrameSize]:
    frame_sizes: list[FrameSize] = []
    index = 0
    while True:
        size_desc = V4L2Frmsizeenum()
        size_desc.index = index
        size_desc.pixel_format = pixel_format
        try:
            _ioctl_struct(fd, VIDIOC_ENUM_FRAMESIZES, size_desc)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                break
            raise

        if size_desc.type == V4L2_FRMSIZE_TYPE_DISCRETE:
            width = int(size_desc.discrete.width)
            height = int(size_desc.discrete.height)
            intervals = _enumerate_frame_intervals(fd, pixel_format, width, height)
            frame_sizes.append(FrameSize(width=width, height=height, intervals=intervals))
        else:
            stepwise = size_desc.stepwise
            frame_sizes.extend(
                _expand_stepwise_sizes(
                    fd,
                    pixel_format,
                    int(stepwise.min_width),
                    int(stepwise.max_width),
                    max(int(stepwise.step_width), 1),
                    int(stepwise.min_height),
                    int(stepwise.max_height),
                    max(int(stepwise.step_height), 1),
                )
            )
        index += 1
    return frame_sizes


def _expand_stepwise_sizes(
    fd: int,
    pixel_format: int,
    min_width: int,
    max_width: int,
    step_width: int,
    min_height: int,
    max_height: int,
    step_height: int,
) -> list[FrameSize]:
    frame_sizes: list[FrameSize] = []
    widths = range(min_width, max_width + 1, step_width)
    heights = range(min_height, max_height + 1, step_height)
    for width in widths:
        for height in heights:
            intervals = _enumerate_frame_intervals(fd, pixel_format, width, height)
            if intervals:
                frame_sizes.append(FrameSize(width=width, height=height, intervals=intervals))
    return frame_sizes


def _enumerate_frame_intervals(fd: int, pixel_format: int, width: int, height: int) -> list[FrameInterval]:
    intervals: list[FrameInterval] = []
    index = 0
    while True:
        interval_desc = V4L2Frmivalenum()
        interval_desc.index = index
        interval_desc.pixel_format = pixel_format
        interval_desc.width = width
        interval_desc.height = height
        try:
            _ioctl_struct(fd, VIDIOC_ENUM_FRAMEINTERVALS, interval_desc)
        except OSError as exc:
            if exc.errno == errno.EINVAL:
                break
            raise

        if interval_desc.type == V4L2_FRMIVAL_TYPE_DISCRETE:
            fps = _fract_to_fps(interval_desc.discrete)
            if fps is not None:
                intervals.append(FrameInterval(fps=fps))
        else:
            stepwise = interval_desc.stepwise
            fps_min = _fract_to_fps(stepwise.max)
            fps_max = _fract_to_fps(stepwise.min)
            fps_step = _fract_to_fps(stepwise.step)
            intervals.append(FrameInterval(fps=None, fps_min=fps_min, fps_max=fps_max, fps_step=fps_step))
        index += 1

    intervals.sort(key=lambda item: item.fps if item.fps is not None else item.fps_max or 0.0, reverse=True)
    return intervals
