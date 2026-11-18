#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from dataclasses import dataclass, field
import json
import math
from pathlib import Path
from typing import Any


RECORDING_SCHEMA = "fr3-multimodal-recording"
SUPPORTED_FORMAT_VERSIONS = frozenset((1, 2, 3, 4))
V3_VIDEO_COLUMNS = frozenset(
    (
        "video_frame_index",
        "record_time_ns",
        "record_time_sec",
        "record_elapsed_sec",
        "stereo_left_image_stamp_ns",
        "stereo_right_image_stamp_ns",
        "wrist_image_stamp_ns",
    )
)
V3_TRAJECTORY_COLUMNS = frozenset(
    (
        "sample_index",
        "record_time_ns",
        "record_time_sec",
        "record_elapsed_sec",
        "latest_video_frame_index",
        "video_time_sec",
        "joint_state_received_count",
        "joint_state_stamp_ns",
        "joint_state_age_ns",
        "ee_transform_stamp_ns",
        "ee_transform_age_ns",
        "stereo_left_image_stamp_ns",
        "stereo_left_image_age_ns",
        "stereo_right_image_stamp_ns",
        "stereo_right_image_age_ns",
        "wrist_image_stamp_ns",
        "wrist_image_age_ns",
        *(f"fr3_joint{index}" for index in range(1, 8)),
        "ee_x",
        "ee_y",
        "ee_z",
        "ee_qx",
        "ee_qy",
        "ee_qz",
        "ee_qw",
    )
)
V4_VIDEO_COLUMNS = V3_VIDEO_COLUMNS | frozenset(
    (
        "wrist_stereo_left_image_stamp_ns",
        "wrist_stereo_right_image_stamp_ns",
    )
)
V4_TRAJECTORY_COLUMNS = V3_TRAJECTORY_COLUMNS | frozenset(
    (
        "wrist_stereo_left_image_stamp_ns",
        "wrist_stereo_left_image_age_ns",
        "wrist_stereo_right_image_stamp_ns",
        "wrist_stereo_right_image_age_ns",
    )
)
V3_JOINT_PARAMETER_COLUMNS = frozenset(
    (
        *(f"fr3_joint{index}_velocity" for index in range(1, 8)),
        *(f"fr3_joint{index}_effort" for index in range(1, 8)),
    )
)
V3_CONTROL_INPUT_COLUMNS = frozenset(
    (
        "input_index",
        "record_time_ns",
        "record_time_sec",
        "record_elapsed_sec",
        "latest_video_frame_index",
        "video_time_sec",
        "sequence",
        "client_id",
        "active",
        "pointer_locked",
        "keys",
        "mouse_delta_x",
        "mouse_delta_y",
        "forward",
        "right",
        "up",
        "roll",
        "pitch",
        "yaw",
        "look_yaw",
        "look_pitch",
        "speed_scale",
        "gripper",
    )
)


@dataclass
class ValidationResult:
    session_dir: Path
    format_version: int = 1
    errors: list[str] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors

    def as_dict(self) -> dict[str, Any]:
        return {
            "session": str(self.session_dir),
            "format_version": self.format_version,
            "valid": self.valid,
            "errors": self.errors,
            "warnings": self.warnings,
        }


def _integer(value: Any) -> int | None:
    try:
        return int(value)
    except (TypeError, ValueError):
        return None


def _number(value: Any) -> float | None:
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return None
    return parsed if math.isfinite(parsed) else None


def _read_csv(path: Path, result: ValidationResult) -> tuple[list[str], list[dict[str, str]]]:
    if not path.is_file():
        result.errors.append(f"missing required file: {path.name}")
        return [], []
    try:
        with path.open("r", encoding="utf-8", newline="") as file_obj:
            reader = csv.DictReader(file_obj)
            return list(reader.fieldnames or []), list(reader)
    except (OSError, csv.Error) as exc:
        result.errors.append(f"could not read {path.name}: {exc}")
        return [], []


def _require_columns(
    filename: str,
    columns: list[str],
    required: frozenset[str],
    result: ValidationResult,
) -> None:
    missing = sorted(required.difference(columns))
    if missing:
        result.errors.append(f"{filename} is missing columns: {', '.join(missing)}")


def _check_monotonic(
    rows: list[dict[str, str]],
    column: str,
    result: ValidationResult,
    *,
    strictly: bool,
) -> None:
    previous = None
    for row_number, row in enumerate(rows, start=2):
        value = _number(row.get(column))
        if value is None:
            result.errors.append(f"{column} is invalid at CSV row {row_number}")
            return
        if previous is not None and (value <= previous if strictly else value < previous):
            relation = "strictly increasing" if strictly else "nondecreasing"
            result.errors.append(f"{column} is not {relation} at CSV row {row_number}")
            return
        previous = value


def _check_age_columns(rows: list[dict[str, str]], result: ValidationResult) -> None:
    pairs = (
        ("joint_state_stamp_ns", "joint_state_age_ns"),
        ("ee_transform_stamp_ns", "ee_transform_age_ns"),
        ("stereo_left_image_stamp_ns", "stereo_left_image_age_ns"),
        ("stereo_right_image_stamp_ns", "stereo_right_image_age_ns"),
        ("wrist_stereo_left_image_stamp_ns", "wrist_stereo_left_image_age_ns"),
        ("wrist_stereo_right_image_stamp_ns", "wrist_stereo_right_image_age_ns"),
        ("wrist_image_stamp_ns", "wrist_image_age_ns"),
    )
    for row_number, row in enumerate(rows, start=2):
        record_time_ns = _integer(row.get("record_time_ns"))
        if record_time_ns is None:
            continue
        for stamp_column, age_column in pairs:
            if stamp_column not in row and age_column not in row:
                continue
            stamp_ns = _integer(row.get(stamp_column))
            age_ns = _integer(row.get(age_column))
            if stamp_ns is None or age_ns is None or record_time_ns - stamp_ns != age_ns:
                result.errors.append(
                    f"{age_column} does not equal record_time_ns - {stamp_column} at CSV row {row_number}"
                )
                return


def _check_source_freshness(
    rows: list[dict[str, str]],
    source_name: str,
    stamp_column: str,
    maximum_source_age_ms: float,
    maximum_stale_ratio: float,
    result: ValidationResult,
) -> None:
    ages_ms: list[float] = []
    for row_number, row in enumerate(rows, start=2):
        record_time_ns = _integer(row.get("record_time_ns"))
        stamp_ns = _integer(row.get(stamp_column))
        if record_time_ns is None or stamp_ns is None:
            result.errors.append(
                f"{source_name} has an invalid timestamp at CSV row {row_number}"
            )
            return
        ages_ms.append((record_time_ns - stamp_ns) / 1e6)

    if not ages_ms:
        return
    minimum_age_ms = min(ages_ms)
    if minimum_age_ms < -maximum_source_age_ms:
        result.errors.append(
            f"{source_name} timestamp is {-minimum_age_ms:.3f} ms in the future; clocks are not aligned"
        )
    stale_count = sum(age_ms > maximum_source_age_ms for age_ms in ages_ms)
    if not stale_count:
        return
    stale_ratio = stale_count / len(ages_ms)
    message = (
        f"{source_name} has {stale_count}/{len(ages_ms)} timestamps "
        f"({stale_ratio:.2%}) older than {maximum_source_age_ms:.3f} ms; "
        f"maximum age is {max(ages_ms):.3f} ms"
    )
    if stale_ratio > maximum_stale_ratio + 1e-12:
        result.errors.append(message)
    else:
        result.warnings.append(message)


def _check_rate(
    metadata: dict[str, Any],
    stream: str,
    minimum_rate_ratio: float,
    result: ValidationResult,
) -> None:
    statistics = metadata.get("timing_statistics")
    values = statistics.get(stream) if isinstance(statistics, dict) else None
    if not isinstance(values, dict):
        result.errors.append(f"timing_statistics.{stream} is missing")
        return
    requested_hz = _number(values.get("requested_hz"))
    observed_hz = _number(values.get("observed_hz"))
    sample_count = _integer(values.get("sample_count")) or 0
    if sample_count >= 2 and requested_hz and observed_hz is None:
        result.errors.append(f"timing_statistics.{stream}.observed_hz is missing")
    elif requested_hz and observed_hz is not None and observed_hz < requested_hz * minimum_rate_ratio:
        result.errors.append(
            f"{stream} observed rate {observed_hz:.3f} Hz is below "
            f"{minimum_rate_ratio:.0%} of requested {requested_hz:.3f} Hz"
        )


def validate_session(
    session_dir: Path,
    minimum_rate_ratio: float = 0.9,
    maximum_source_age_ms: float = 100.0,
    maximum_stale_ratio: float = 0.01,
) -> ValidationResult:
    session_dir = session_dir.expanduser().resolve()
    result = ValidationResult(session_dir=session_dir)
    metadata_path = session_dir / "metadata.json"
    if not metadata_path.is_file():
        result.errors.append("missing required file: metadata.json")
        return result
    try:
        metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        result.errors.append(f"could not read metadata.json: {exc}")
        return result
    if not isinstance(metadata, dict):
        result.errors.append("metadata.json must contain an object")
        return result

    version = _integer(metadata.get("recording_format_version")) or 1
    result.format_version = version
    if version not in SUPPORTED_FORMAT_VERSIONS:
        result.errors.append(
            f"unsupported recording_format_version={version}; supported={sorted(SUPPORTED_FORMAT_VERSIONS)}"
        )
        return result

    video_path = session_dir / str(metadata.get("video_frames_csv") or "video_frames.csv")
    trajectory_path = session_dir / str(metadata.get("trajectory_csv") or "trajectory.csv")
    control_input_name = metadata.get("control_input_csv")
    control_input_path = session_dir / str(control_input_name or "control_inputs.csv")
    video_columns, video_rows = _read_csv(video_path, result)
    trajectory_columns, trajectory_rows = _read_csv(trajectory_path, result)
    control_input_columns: list[str] = []
    control_input_rows: list[dict[str, str]] = []
    if control_input_name is not None:
        control_input_columns, control_input_rows = _read_csv(control_input_path, result)

    expected_video_count = _integer(metadata.get("video_frame_count"))
    expected_trajectory_count = _integer(metadata.get("trajectory_sample_count"))
    expected_control_input_count = _integer(metadata.get("control_input_count"))
    if expected_video_count is not None and expected_video_count != len(video_rows):
        result.errors.append(
            f"video_frame_count={expected_video_count} but {video_path.name} has {len(video_rows)} rows"
        )
    if expected_trajectory_count is not None and expected_trajectory_count != len(trajectory_rows):
        result.errors.append(
            f"trajectory_sample_count={expected_trajectory_count} but {trajectory_path.name} has {len(trajectory_rows)} rows"
        )
    if expected_control_input_count is not None and expected_control_input_count != len(control_input_rows):
        result.errors.append(
            f"control_input_count={expected_control_input_count} but {control_input_path.name} "
            f"has {len(control_input_rows)} rows"
        )

    if version < 3:
        result.warnings.append(
            f"legacy format v{version}: v3 timing contract and source freshness cannot be verified"
        )
        return result

    if metadata.get("recording_schema") != RECORDING_SCHEMA:
        result.errors.append(f"recording_schema must be {RECORDING_SCHEMA!r}")
    if metadata.get("recording_state") != "complete":
        result.errors.append("recording_state must be 'complete'")
    if _integer(metadata.get("minimum_reader_version")) != 2:
        result.errors.append("minimum_reader_version must be 2 for additive v2 compatibility")
    contract = metadata.get("timing_contract")
    if not isinstance(contract, dict):
        result.errors.append("timing_contract is missing")
    else:
        expected_contract = {
            "scheduler_clock": "steady_monotonic",
            "record_time_clock": "ros_node_clock",
            "source_timestamp_clock": "ros_message_header_or_tf_header",
            "timestamp_unit": "nanoseconds",
            "trajectory_alignment": "latest_observation_hold_at_recorder_tick",
            "video_alignment": "latest_encoded_video_frame",
        }
        for key, expected in expected_contract.items():
            if contract.get(key) != expected:
                result.errors.append(f"timing_contract.{key} must be {expected!r}")

    _require_columns(
        video_path.name,
        video_columns,
        V4_VIDEO_COLUMNS if version >= 4 else V3_VIDEO_COLUMNS,
        result,
    )
    _require_columns(
        trajectory_path.name,
        trajectory_columns,
        V4_TRAJECTORY_COLUMNS if version >= 4 else V3_TRAJECTORY_COLUMNS,
        result,
    )
    joint_state_fields = metadata.get("joint_state_fields")
    if isinstance(joint_state_fields, list) and {"velocity", "effort"}.issubset(joint_state_fields):
        _require_columns(
            trajectory_path.name,
            trajectory_columns,
            V3_JOINT_PARAMETER_COLUMNS,
            result,
        )
    if control_input_name is not None:
        _require_columns(
            control_input_path.name,
            control_input_columns,
            V3_CONTROL_INPUT_COLUMNS,
            result,
        )
    if video_rows:
        _check_monotonic(video_rows, "video_frame_index", result, strictly=True)
        _check_monotonic(video_rows, "record_time_ns", result, strictly=True)
        _check_monotonic(video_rows, "record_elapsed_sec", result, strictly=False)
    if trajectory_rows:
        _check_monotonic(trajectory_rows, "sample_index", result, strictly=True)
        _check_monotonic(trajectory_rows, "record_time_ns", result, strictly=True)
        _check_monotonic(trajectory_rows, "record_elapsed_sec", result, strictly=False)
        _check_monotonic(trajectory_rows, "joint_state_received_count", result, strictly=False)
        _check_age_columns(trajectory_rows, result)
    if control_input_rows:
        _check_monotonic(control_input_rows, "input_index", result, strictly=True)
        _check_monotonic(control_input_rows, "record_time_ns", result, strictly=True)
        _check_monotonic(control_input_rows, "record_elapsed_sec", result, strictly=False)
        _check_monotonic(control_input_rows, "sequence", result, strictly=True)
    _check_rate(metadata, "video", minimum_rate_ratio, result)
    _check_rate(metadata, "trajectory", minimum_rate_ratio, result)

    sources = (metadata.get("timing_statistics") or {}).get("sources")
    trajectory_hz = _number(metadata.get("trajectory_hz"))
    video_fps = _number(metadata.get("fps"))
    trajectory_count = _integer(metadata.get("trajectory_sample_count")) or 0
    video_count = _integer(metadata.get("video_frame_count")) or 0
    expected_sources = {
        "stereo_left_image": (video_fps, video_count, True),
        "stereo_right_image": (video_fps, video_count, True),
        "wrist_image": (video_fps, video_count, True),
    }
    if version >= 4:
        expected_sources.update(
            {
                "wrist_stereo_left_image": (video_fps, video_count, True),
                "wrist_stereo_right_image": (video_fps, video_count, True),
            }
        )
    if metadata.get("trajectory_required"):
        expected_sources.update(
            {
                # Trajectory rows intentionally use latest-observation hold. The
                # source must keep advancing, but it need not publish at the
                # recorder's trajectory output rate.
                "joint_state": (trajectory_hz, trajectory_count, False),
                "end_effector_transform": (trajectory_hz, trajectory_count, False),
            }
        )
    if not isinstance(sources, dict):
        result.errors.append("timing_statistics.sources is missing")
    else:
        for source_name, (expected_hz, output_count, enforce_output_rate) in expected_sources.items():
            source_stats = sources.get(source_name)
            observed_hz = (
                _number(source_stats.get("observed_hz"))
                if isinstance(source_stats, dict)
                else None
            )
            if output_count >= 2 and observed_hz is None:
                result.errors.append(
                    f"{source_name} observed_hz is missing for a multi-sample output"
                )
            elif (
                observed_hz is not None
                and enforce_output_rate
                and expected_hz
                and observed_hz < expected_hz * minimum_rate_ratio
            ):
                result.errors.append(
                    f"{source_name} source is only {observed_hz:.3f} Hz for a "
                    f"{expected_hz:.3f} Hz output"
                )
    freshness_sources = [
        (video_rows, "stereo_left_image", "stereo_left_image_stamp_ns"),
        (video_rows, "stereo_right_image", "stereo_right_image_stamp_ns"),
        (video_rows, "wrist_image", "wrist_image_stamp_ns"),
        (trajectory_rows, "joint_state", "joint_state_stamp_ns"),
        (trajectory_rows, "end_effector_transform", "ee_transform_stamp_ns"),
    ]
    if version >= 4:
        freshness_sources.extend(
            (
                (video_rows, "wrist_stereo_left_image", "wrist_stereo_left_image_stamp_ns"),
                (video_rows, "wrist_stereo_right_image", "wrist_stereo_right_image_stamp_ns"),
            )
        )
    for rows, source_name, stamp_column in freshness_sources:
        if source_name in expected_sources:
            _check_source_freshness(
                rows,
                source_name,
                stamp_column,
                maximum_source_age_ms,
                maximum_stale_ratio,
                result,
            )
    return result


def find_sessions(path: Path) -> list[Path]:
    resolved = path.expanduser().resolve()
    if (resolved / "metadata.json").is_file():
        return [resolved]
    if not resolved.is_dir():
        return []
    return sorted(metadata_path.parent for metadata_path in resolved.rglob("metadata.json"))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Validate FR3 recording sessions before dataset use")
    parser.add_argument("path", type=Path, help="A recording session or dataset root")
    parser.add_argument(
        "--minimum-rate-ratio",
        type=float,
        default=0.9,
        help="Warn when measured rate is below this fraction of requested rate (default: 0.9)",
    )
    parser.add_argument(
        "--maximum-source-age-ms",
        type=float,
        default=100.0,
        help="Freshness threshold for v3 source timestamps in milliseconds (default: 100)",
    )
    parser.add_argument(
        "--maximum-stale-ratio",
        type=float,
        default=0.01,
        help="Fail when more than this fraction of timestamps exceed the age limit (default: 0.01)",
    )
    parser.add_argument("--json", action="store_true", help="Emit a machine-readable JSON report")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not 0.0 < args.minimum_rate_ratio <= 1.0:
        raise SystemExit("--minimum-rate-ratio must be in (0, 1]")
    if args.maximum_source_age_ms <= 0.0:
        raise SystemExit("--maximum-source-age-ms must be greater than 0")
    if not 0.0 <= args.maximum_stale_ratio <= 1.0:
        raise SystemExit("--maximum-stale-ratio must be in [0, 1]")
    sessions = find_sessions(args.path)
    if not sessions:
        raise SystemExit(f"no recording sessions found under {args.path}")
    results = [
        validate_session(
            path,
            args.minimum_rate_ratio,
            args.maximum_source_age_ms,
            args.maximum_stale_ratio,
        )
        for path in sessions
    ]
    if args.json:
        print(
            json.dumps(
                {
                    "session_count": len(results),
                    "valid": all(result.valid for result in results),
                    "results": [result.as_dict() for result in results],
                },
                ensure_ascii=False,
                indent=2,
            )
        )
    else:
        for result in results:
            print(f"{'PASS' if result.valid else 'FAIL'} v{result.format_version} {result.session_dir}")
            for message in result.errors:
                print(f"  error: {message}")
            for message in result.warnings:
                print(f"  warning: {message}")
    return 0 if all(result.valid for result in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
