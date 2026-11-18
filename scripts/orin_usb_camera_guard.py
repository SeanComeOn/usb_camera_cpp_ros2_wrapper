#!/usr/bin/env python3
"""Keep Orin USB camera paths awake and recover disconnected cameras."""

from __future__ import annotations

import argparse
import fcntl
import json
import math
import os
import re
import signal
import stat
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path


USB_SYSFS = Path("/sys/bus/usb/devices")
USB_BUS = Path("/dev/bus/usb")
USB_DRIVER = Path("/sys/bus/usb/drivers/usb")
USBDEVFS_RESET = 0x5514


@dataclass(frozen=True)
class UsbDevice:
    name: str
    path: Path
    vendor_id: str
    product_id: str

    @property
    def identity(self) -> str:
        return f"{self.vendor_id}:{self.product_id}"


@dataclass(frozen=True)
class RecoveryTarget:
    identity: str
    hub: str
    port: int | None = None
    fallback_hub: str | None = None
    fallback_port: int | None = None


@dataclass
class RecoveryState:
    missing_since: float | None = None
    next_attempt: float = 0.0
    attempts: int = 0


@dataclass(frozen=True)
class StalledCamera:
    identity: str
    serial: str
    status_path: Path
    topic: str


def parse_stalled_camera(value: str) -> StalledCamera:
    parts = value.split(",")
    if (len(parts) != 4 or not re.fullmatch(r"[0-9a-fA-F]{4}:[0-9a-fA-F]{4}", parts[0])
            or not parts[1] or not Path(parts[2]).is_absolute() or not parts[3].startswith("/")):
        raise ValueError("stalled camera requires vendor:product,serial,absolute-status-path,/topic")
    # This recovery mode is deliberately limited to the commissioned mono model.
    if parts[0].lower() != "1e45:0209" or parts[3] != "/mono/image_raw":
        raise ValueError("device reset is only commissioned for 1e45:0209 /mono/image_raw")
    return StalledCamera(parts[0].lower(), parts[1], Path(parts[2]), parts[3])


def matching_camera(target: StalledCamera, devices: list[UsbDevice]) -> UsbDevice | None:
    matches = [d for d in devices if d.identity == target.identity
               and read_text(d.path / "serial") == target.serial
               and re.fullmatch(r"\d+-\d+(?:\.\d+)*", d.name)]
    return matches[0] if len(matches) == 1 else None


def camera_status_stamp(target: StalledCamera) -> float | None:
    try:
        # The user-owned heartbeat is untrusted. Never read unlimited data or
        # echo its contents from this privileged service.
        fd = os.open(target.status_path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
        with os.fdopen(fd, "rb") as handle:
            info = os.fstat(handle.fileno())
            if not stat.S_ISREG(info.st_mode) or info.st_size > 16384:
                return None
            data = json.loads(handle.read(16385))
        if not isinstance(data, dict) or data.get("topic") != target.topic:
            return None
        stamp = data.get("source_stamp_ns")
        if isinstance(stamp, bool) or not isinstance(stamp, int) or stamp <= 0:
            return None
        value = stamp / 1e9
        return value if math.isfinite(value) else None
    except (OSError, ValueError, TypeError, OverflowError):
        return None


def mono_publisher_running() -> bool:
    for process in Path("/proc").glob("[0-9]*"):
        try:
            if (process / "exe").resolve().name == "mono_camera_node":
                return True
        except OSError:
            pass
    return False


def recent_usb_timeout(device: UsbDevice, since: float) -> bool:
    try:
        result = subprocess.run(
            ["journalctl", "-k", "--since", "@" + str(int(since)), "--no-pager", "-o", "cat"],
            capture_output=True, text=True, timeout=5, check=False)
    except (OSError, subprocess.TimeoutExpired):
        return False
    pattern = re.compile(r"(?:usb |uvcvideo )" + re.escape(device.name) + r"(?:[: ]).* -110\b")
    return result.returncode == 0 and any(pattern.search(line) for line in result.stdout.splitlines())


def reset_single_camera(target: StalledCamera) -> bool:
    device = matching_camera(target, discover_usb_devices())
    if device is None:
        return False
    try:
        bus = int(read_text(device.path / "busnum"))
        number = int(read_text(device.path / "devnum"))
        with (USB_BUS / f"{bus:03d}" / f"{number:03d}").open("r+b", buffering=0) as handle:
            # Confirm the opened device descriptor as well as the current
            # sysfs serial/number before ioctl; never reset a parent hub.
            descriptor = handle.read(18)
            if (len(descriptor) != 18 or descriptor[1] != 1
                    or f"{int.from_bytes(descriptor[8:10], 'little'):04x}:"
                       f"{int.from_bytes(descriptor[10:12], 'little'):04x}" != target.identity
                    or read_text(device.path / "serial") != target.serial
                    or int(read_text(device.path / "devnum")) != number):
                return False
            fcntl.ioctl(handle.fileno(), USBDEVFS_RESET, 0)
        return True
    except (OSError, ValueError):
        return False


class StalledCameraRecovery:
    """Corroborated, bounded single-camera recovery; no hub power operations."""

    def __init__(self, target: StalledCamera) -> None:
        self.target = target
        self.attempts = 0
        self.next_attempt = 0.0
        self.healthy_since: float | None = None
        self.last_state = ""

    def state(self, message: str) -> None:
        if message != self.last_state:
            log(f"Mono USB {self.target.serial}: {message}")
            self.last_state = message

    def check(self, devices: list[UsbDevice], now: float, monotonic: float) -> None:
        device = matching_camera(self.target, devices)
        if device is None:
            self.healthy_since = None
            self.state("not uniquely enumerated; physical reconnect required, no hub reset attempted")
            return
        stamp = camera_status_stamp(self.target)
        age = now - stamp if stamp is not None else None
        if age is not None and -1 <= age < 2:
            if self.healthy_since is None:
                self.healthy_since = monotonic
            if monotonic - self.healthy_since >= 60:
                self.attempts = 0
            self.state("fresh image stream")
            return
        self.healthy_since = None
        if age is None or age < 20 or not mono_publisher_running():
            self.state("awaiting valid active-stream evidence; no device reset")
            return
        if self.attempts >= 2:
            self.state("two device-reset attempts exhausted; inspect cable/port/camera")
            return
        if monotonic < self.next_attempt:
            return
        # Stale ROS/preview data alone cannot justify a USB reset.
        if not recent_usb_timeout(device, max(stamp, now - 60)):
            self.state("stale stream without recent device USB timeout; no device reset")
            return
        self.attempts += 1
        self.next_attempt = monotonic + 120
        self.state(f"USB -110 and stale stream; device-only reset attempt {self.attempts}")
        spec = f"{self.target.identity},{self.target.serial},{self.target.status_path},{self.target.topic}"
        try:
            completed = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), "--reset-single-camera", spec],
                capture_output=True, timeout=45, check=False)
            outcome = "reset issued; waiting for fresh frames" if completed.returncode == 0 else "device reset failed"
        except (OSError, subprocess.TimeoutExpired):
            outcome = "device reset failed or timed out"
        self.state(outcome)


def log(message: str) -> None:
    print(time.strftime("%Y-%m-%d %H:%M:%S"), message, flush=True)


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="ascii").strip()
    except (FileNotFoundError, OSError):
        return ""


def discover_usb_devices() -> list[UsbDevice]:
    devices: list[UsbDevice] = []
    try:
        entries = list(USB_SYSFS.iterdir())
    except OSError as error:
        log(f"Cannot enumerate {USB_SYSFS}: {error}")
        return devices

    for entry in entries:
        vendor_id = read_text(entry / "idVendor").lower()
        product_id = read_text(entry / "idProduct").lower()
        if not vendor_id or not product_id:
            continue
        devices.append(UsbDevice(entry.name, entry, vendor_id, product_id))
    return devices


def topology_names(device_name: str) -> list[str]:
    if re.fullmatch(r"usb\d+", device_name):
        return [device_name]
    if "-" not in device_name or ":" in device_name:
        return []
    bus, ports = device_name.split("-", 1)
    port_parts = ports.split(".")
    return [f"{bus}-" + ".".join(port_parts[:count]) for count in range(len(port_parts), 0, -1)]


def force_power_on(device_name: str) -> None:
    for topology_name in topology_names(device_name):
        control_path = USB_SYSFS / topology_name / "power" / "control"
        if not control_path.exists() or read_text(control_path) == "on":
            continue
        try:
            control_path.write_text("on\n", encoding="ascii")
            log(f"Set {topology_name} power/control=on")
        except OSError as error:
            log(f"Failed to set {control_path}: {error}")


def normalize_identity(value: str) -> str:
    normalized = value.strip().lower()
    parts = normalized.split(":", 1)
    if len(parts) != 2 or not all(parts):
        raise ValueError(f"Invalid USB identity {value!r}; expected vendor:product")
    return normalized


def expected_identities(values: list[str]) -> set[str]:
    return {normalize_identity(value) for value in values}


def parse_recovery_target(value: str) -> RecoveryTarget:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) not in (2, 3, 4, 5) or not parts[1]:
        raise ValueError(
            f"Invalid recovery target {value!r}; expected "
            "vendor:product,hub[,port[,fallback_hub[,fallback_port]]]"
        )
    port = None
    if len(parts) >= 3:
        try:
            port = int(parts[2])
        except ValueError as error:
            raise ValueError(f"Invalid USB hub port in recovery target {value!r}") from error
        if port <= 0:
            raise ValueError(f"USB hub port must be positive in recovery target {value!r}")
    fallback_hub = parts[3] if len(parts) == 4 and parts[3] else None
    if len(parts) == 5:
        fallback_hub = parts[3] or None
    if len(parts) >= 4 and fallback_hub is None:
        raise ValueError(f"Fallback hub may not be empty in recovery target {value!r}")
    fallback_port = None
    if len(parts) == 5:
        try:
            fallback_port = int(parts[4])
        except ValueError as error:
            raise ValueError(f"Invalid fallback hub port in recovery target {value!r}") from error
        if fallback_port <= 0:
            raise ValueError(f"Fallback hub port must be positive in recovery target {value!r}")
    return RecoveryTarget(normalize_identity(parts[0]), parts[1], port, fallback_hub, fallback_port)


def reset_hub(hub_name: str) -> bool:
    hub_path = USB_SYSFS / hub_name
    busnum = read_text(hub_path / "busnum")
    devnum = read_text(hub_path / "devnum")
    if not busnum or not devnum:
        log(f"Recovery hub {hub_name} is not enumerated")
        return False

    device_file = USB_BUS / f"{int(busnum):03d}" / f"{int(devnum):03d}"
    try:
        descriptor = os.open(device_file, os.O_WRONLY)
        try:
            fcntl.ioctl(descriptor, USBDEVFS_RESET, 0)
        finally:
            os.close(descriptor)
    except OSError as error:
        log(f"USBDEVFS_RESET failed for {hub_name} via {device_file}: {error}")
        return False

    log(f"Reset recovery hub {hub_name} via USBDEVFS_RESET")
    return True


def rebind_hub(hub_name: str) -> bool:
    unbind_path = USB_DRIVER / "unbind"
    bind_path = USB_DRIVER / "bind"
    if not (USB_SYSFS / hub_name).exists():
        log(f"Cannot rebind missing recovery hub {hub_name}")
        return False

    try:
        unbind_path.write_text(f"{hub_name}\n", encoding="ascii")
        time.sleep(2.0)
        bind_path.write_text(f"{hub_name}\n", encoding="ascii")
    except OSError as error:
        log(f"Driver rebind failed for recovery hub {hub_name}: {error}")
        return False

    log(f"Rebound USB driver for recovery hub {hub_name}")
    return True


def cycle_hub_port(hub_name: str, port: int, off_seconds: float) -> bool:
    command = [
        "/usr/sbin/uhubctl",
        "-l",
        hub_name,
        "-p",
        str(port),
        "-a",
        "cycle",
        "-d",
        str(max(1, int(off_seconds))),
    ]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=max(20.0, off_seconds + 15.0),
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        log(f"Port power cycle failed for {hub_name} port {port}: {error}")
        return False
    if result.returncode != 0:
        detail = (result.stderr or result.stdout).strip().splitlines()
        log(
            f"Port power cycle failed for {hub_name} port {port}: "
            + (detail[-1] if detail else f"exit {result.returncode}")
        )
        return False
    log(f"Power-cycled recovery hub {hub_name} port {port}")
    return True


def hub_port_powered(hub_name: str, port: int) -> bool | None:
    """Return the physical port power state without opening the camera device."""
    command = ["/usr/sbin/uhubctl", "-l", hub_name, "-p", str(port)]
    try:
        result = subprocess.run(
            command,
            check=False,
            capture_output=True,
            text=True,
            timeout=10.0,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None
    if result.returncode != 0:
        return None

    match = re.search(
        rf"^\s*Port\s+{port}:\s+[0-9a-fA-F]+\s+(.*)$",
        result.stdout,
        flags=re.MULTILINE,
    )
    if match is None:
        return None
    status = match.group(1).lower().split()
    if "off" in status:
        return False
    if "power" in status:
        return True
    return None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected", action="append", default=[], help="Expected vendor:product USB identity")
    parser.add_argument("--stalled-camera", action="append", default=[],
                        help="Device-only recovery: vendor:product,serial,status-json,/topic")
    parser.add_argument("--reset-single-camera", help=argparse.SUPPRESS)
    parser.add_argument(
        "--recovery-target",
        action="append",
        default=[],
        help="Recovery mapping: vendor:product,hub[,port[,fallback_hub[,fallback_port]]]",
    )
    parser.add_argument(
        "--recover-for",
        action="append",
        default=[],
        help="Legacy identity whose disappearance should recover --recovery-hub",
    )
    parser.add_argument("--recovery-hub", default="", help="Legacy downstream recovery hub")
    parser.add_argument("--interval", type=float, default=5.0)
    parser.add_argument("--missing-grace", type=float, default=15.0)
    parser.add_argument("--recovery-wait", type=float, default=10.0)
    parser.add_argument("--recovery-cooldown", type=float, default=120.0)
    parser.add_argument("--port-off-seconds", type=float, default=5.0)
    parser.add_argument("--max-recovery-attempts", type=int, default=2)
    parser.add_argument("--once", action="store_true", help="Apply power policy once without recovery")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if os.geteuid() != 0:
        print("orin_usb_camera_guard.py must run as root", file=sys.stderr)
        return 2

    if args.reset_single_camera:
        try:
            return 0 if reset_single_camera(parse_stalled_camera(args.reset_single_camera)) else 1
        except ValueError:
            return 2

    try:
        expected = expected_identities(args.expected)
        stalled = [StalledCameraRecovery(parse_stalled_camera(value)) for value in args.stalled_camera]
        targets = [parse_recovery_target(value) for value in args.recovery_target]
        if args.recover_for:
            if not args.recovery_hub:
                raise ValueError("--recover-for requires --recovery-hub")
            targets.extend(
                RecoveryTarget(identity, args.recovery_hub)
                for identity in expected_identities(args.recover_for)
            )
    except ValueError as error:
        print(error, file=sys.stderr)
        return 2
    if not expected:
        print("At least one --expected vendor:product identity is required", file=sys.stderr)
        return 2
    if any(item.target.identity not in expected for item in stalled):
        print("Every stalled camera must be listed with --expected", file=sys.stderr)
        return 2
    if len({item.target.serial for item in stalled}) != len(stalled):
        print("Each camera serial may have only one stalled-camera mapping", file=sys.stderr)
        return 2
    if any(target.identity not in expected for target in targets):
        print("Every recovery target identity must also be listed with --expected", file=sys.stderr)
        return 2
    if len({target.identity for target in targets}) != len(targets):
        print("Each USB identity may have only one recovery target", file=sys.stderr)
        return 2

    stop_requested = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)

    states = {target.identity: RecoveryState() for target in targets}
    previous_missing: set[str] = set()
    target_summary = ",".join(
        f"{target.identity}->{target.hub}"
        + (f":port{target.port}" if target.port else "")
        + (
            f"->fallback:{target.fallback_hub}"
            + (f":port{target.fallback_port}" if target.fallback_port else "")
            if target.fallback_hub
            else ""
        )
        for target in targets
    )
    log(
        "USB camera guard started; expected="
        + ",".join(sorted(expected))
        + (f" recovery={target_summary}" if target_summary else " hub recovery disabled")
        + (" device-only recovery=" + ",".join(item.target.serial for item in stalled) if stalled else "")
    )

    while not stop_requested:
        devices = discover_usb_devices()
        present = {device.identity for device in devices}
        recovery_hubs = {target.hub for target in targets}
        for device in devices:
            if device.identity in expected or device.name in recovery_hubs:
                force_power_on(device.name)

        # A powered-off hub port can leave stale USB and V4L2 entries behind.
        # Treat its mapped camera as absent so the normal recovery path cycles it.
        for target in targets:
            if target.port is not None and hub_port_powered(target.hub, target.port) is False:
                present.discard(target.identity)

        missing = expected - present
        if args.once:
            if missing:
                log("Missing expected USB device(s): " + ",".join(sorted(missing)))
                return 1
            log("All expected USB devices are present")
            return 0

        for recovery in stalled:
            recovery.check(devices, time.time(), time.monotonic())

        if missing != previous_missing:
            if missing:
                log("Missing expected USB device(s): " + ",".join(sorted(missing)))
            if previous_missing - missing:
                log("USB camera(s) recovered: " + ",".join(sorted(previous_missing - missing)))

        now = time.monotonic()
        for target in targets:
            state = states[target.identity]
            if target.identity in present:
                state.missing_since = None
                state.next_attempt = 0.0
                state.attempts = 0
                continue

            if state.missing_since is None:
                state.missing_since = now
            if now - state.missing_since < max(0.0, args.missing_grace) or now < state.next_attempt:
                continue

            state.attempts += 1
            maximum_attempts = max(1, args.max_recovery_attempts)
            escalating = target.fallback_hub is not None and state.attempts > maximum_attempts
            if escalating:
                log(
                    f"Escalating recovery for {target.identity} to shared upstream hub "
                    f"{target.fallback_hub}"
                )
                if target.fallback_port is not None:
                    cycle_hub_port(target.fallback_hub, target.fallback_port, args.port_off_seconds)
                else:
                    rebind_hub(target.fallback_hub)
            elif target.port is not None:
                cycle_hub_port(target.hub, target.port, args.port_off_seconds)
            elif state.attempts % 2 == 1:
                reset_hub(target.hub)
            else:
                rebind_hub(target.hub)

            if escalating or (state.attempts >= maximum_attempts and target.fallback_hub is None):
                log(
                    f"Recovery cycle exhausted for {target.identity}; retrying automatically after "
                    f"{max(0.0, args.recovery_cooldown):.0f}s if it remains disconnected"
                )
                state.attempts = 0
                state.next_attempt = time.monotonic() + max(0.0, args.recovery_cooldown)
            else:
                state.next_attempt = time.monotonic() + max(0.0, args.recovery_wait)

        previous_missing = missing
        deadline = time.monotonic() + max(0.2, args.interval)
        while not stop_requested and time.monotonic() < deadline:
            time.sleep(max(0.0, min(0.2, deadline - time.monotonic())))

    log("USB camera guard stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())
