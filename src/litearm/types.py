"""Thread-safe arm state, cancellation, trajectory types.

Mirrors pylitearm types but is fully independent — no cross-imports, no numpy.
"""
from __future__ import annotations

import json
import math
import os
import tempfile
from dataclasses import dataclass, field
from datetime import datetime, timezone
from enum import Enum
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple, Union

import threading

try:
    from typing import TypedDict
except ImportError:  # Python 3.7
    from typing_extensions import TypedDict


# ── Arm state enum ────────────────────────────────────────────────────────────

class ArmState(str, Enum):
    DISCONNECTED = "disconnected"
    CONNECTING = "connecting"
    READY = "ready"
    MOVING = "moving"
    HOLDING = "holding"
    ZERO_GRAVITY = "zero_gravity"
    IMPEDANCE = "impedance"
    FOLLOWING = "following"
    STOPPING = "stopping"
    FAULT = "fault"


# ── Cancellation token ───────────────────────────────────────────────────────

class CancellationToken:
    """Cooperative, thread-safe cancellation token for long-running commands."""

    def __init__(self) -> None:
        self._event = threading.Event()

    def cancel(self) -> None:
        self._event.set()

    def clear(self) -> None:
        self._event.clear()

    @property
    def cancelled(self) -> bool:
        return self._event.is_set()

    def is_cancelled(self) -> bool:
        return self._event.is_set()


# ── TypedDicts for state ─────────────────────────────────────────────────────

class JointFeedbackState(TypedDict):
    joint: int
    received: int
    age_s: Optional[float]
    fresh: bool


class FeedbackState(TypedDict):
    max_age_s: float
    joints: List[JointFeedbackState]
    stale_joints: List[int]


class WatchdogState(TypedDict):
    enabled: bool
    timeout_s: float
    mode: str
    tripped: bool
    last_kick_age_s: float


class RobotState(TypedDict):
    q: List[float]
    dq: List[float]
    tau: List[float]
    fault: List[Tuple[int, int]]
    errs: List[int]
    temps: List[Tuple[int, int]]
    state: str
    feedback: FeedbackState
    watchdog: WatchdogState
    robot_serial: str
    config_checksum_sha256: str


# ── Trajectory types ─────────────────────────────────────────────────────────

SCHEMA = "pylitearm.joint_trajectory.v1"
PathLike = Union[str, os.PathLike]


def _vector7(value: Iterable[Any], label: str) -> List[float]:
    result = [float(item) for item in value]
    if len(result) != 7 or any(not math.isfinite(item) for item in result):
        raise ValueError(f"{label} 必须包含 7 个有限数值")
    return result


@dataclass
class TrajectoryFrame:
    """One timestamped measured robot sample."""

    t: float
    q: List[float]
    dq: Optional[List[float]] = None
    tau: Optional[List[float]] = None

    def __post_init__(self) -> None:
        self.t = float(self.t)
        if not math.isfinite(self.t) or self.t < 0.0:
            raise ValueError("轨迹时间戳必须是非负有限数值")
        self.q = _vector7(self.q, "q")
        if self.dq is not None:
            self.dq = _vector7(self.dq, "dq")
        if self.tau is not None:
            self.tau = _vector7(self.tau, "tau")

    def to_dict(self) -> Dict[str, Any]:
        data: Dict[str, Any] = {"t": self.t, "q": self.q}
        if self.dq is not None:
            data["dq"] = self.dq
        if self.tau is not None:
            data["tau"] = self.tau
        return data


@dataclass
class JointTrajectory:
    """Validated, portable recording of a seven-axis joint trajectory."""

    frames: List[TrajectoryFrame]
    name: str = "trajectory"
    created_at: str = field(
        default_factory=lambda: datetime.now(timezone.utc).isoformat())
    sample_rate_hz: Optional[float] = None
    filter_alpha: Optional[float] = None
    robot_serial: Optional[str] = None
    config_checksum_sha256: Optional[str] = None
    source_path: Optional[str] = field(default=None, repr=False, compare=False)

    def __post_init__(self) -> None:
        self.name = str(self.name).strip() or "trajectory"
        self.frames = [
            frame if isinstance(frame, TrajectoryFrame) else TrajectoryFrame(**frame)
            for frame in self.frames
        ]
        if len(self.frames) < 2:
            raise ValueError("轨迹至少需要 2 帧")
        last_t = -1.0
        for index, frame in enumerate(self.frames):
            if frame.t <= last_t:
                raise ValueError(f"第 {index} 帧时间戳未严格递增")
            last_t = frame.t
        origin = self.frames[0].t
        if origin != 0.0:
            for frame in self.frames:
                frame.t -= origin
        if self.sample_rate_hz is not None:
            self.sample_rate_hz = float(self.sample_rate_hz)
            if not math.isfinite(self.sample_rate_hz) or self.sample_rate_hz <= 0.0:
                raise ValueError("sample_rate_hz 必须为正有限数值")
        if self.filter_alpha is not None:
            self.filter_alpha = float(self.filter_alpha)
            if not 0.0 < self.filter_alpha <= 1.0:
                raise ValueError("filter_alpha 必须在 (0, 1] 范围")

    @property
    def duration_s(self) -> float:
        return self.frames[-1].t

    @property
    def q(self) -> List[List[float]]:
        return [list(frame.q) for frame in self.frames]

    @property
    def t(self) -> List[float]:
        return [frame.t for frame in self.frames]

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": SCHEMA,
            "name": self.name,
            "created_at": self.created_at,
            "duration_s": self.duration_s,
            "sample_rate_hz": self.sample_rate_hz,
            "filter_alpha": self.filter_alpha,
            "robot_serial": self.robot_serial,
            "config_checksum_sha256": self.config_checksum_sha256,
            "frames": [frame.to_dict() for frame in self.frames],
        }

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "JointTrajectory":
        schema = data.get("schema")
        if schema not in (None, SCHEMA):
            raise ValueError(f"不支持的轨迹 schema: {schema!r}")

        raw_frames = data.get("frames")
        if raw_frames is None:
            raw_frames = data.get("trajectory")
        if not isinstance(raw_frames, list):
            raise ValueError("轨迹文件缺少 frames/trajectory 数组")
        frames = []
        for item in raw_frames:
            if not isinstance(item, Mapping):
                raise ValueError("轨迹帧必须是对象")
            frames.append(TrajectoryFrame(
                t=item["t"],
                q=item.get("q", item.get("joints")),
                dq=item.get("dq"),
                tau=item.get("tau"),
            ))
        return cls(
            frames=frames,
            name=data.get("name", "trajectory"),
            created_at=data.get("created_at", datetime.now(timezone.utc).isoformat()),
            sample_rate_hz=data.get("sample_rate_hz", data.get("sample_rate")),
            filter_alpha=data.get("filter_alpha"),
            robot_serial=data.get("robot_serial"),
            config_checksum_sha256=data.get("config_checksum_sha256"),
        )

    @classmethod
    def load(cls, path: PathLike) -> "JointTrajectory":
        source = Path(path).expanduser()
        with source.open("r", encoding="utf-8") as stream:
            data = json.load(stream)
        trajectory = cls.from_dict(data)
        trajectory.source_path = str(source.resolve())
        return trajectory

    def save(self, path: PathLike, overwrite: bool = False) -> Path:
        """Atomically save JSON. Existing recordings are protected by default."""
        target = Path(path).expanduser()
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() and not overwrite:
            raise FileExistsError(f"轨迹文件已存在: {target}")
        fd, temporary_name = tempfile.mkstemp(
            prefix=f".{target.name}.", suffix=".tmp", dir=str(target.parent))
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as stream:
                json.dump(self.to_dict(), stream, ensure_ascii=False, indent=2,
                          allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            if overwrite:
                os.replace(temporary_name, target)
            else:
                try:
                    os.link(temporary_name, target)
                except FileExistsError:
                    raise FileExistsError(f"轨迹文件已存在: {target}") from None
                os.unlink(temporary_name)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)
        self.source_path = str(target.resolve())
        return target
