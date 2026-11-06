"""Shared Pinocchio model used by local kinematics and self-collision checks."""
from __future__ import annotations

import importlib
import math
from pathlib import Path
import threading
from typing import Iterable, Optional, Sequence, Union


_ASSET_ROOT = Path(__file__).resolve().parent / "assets"
_DEFAULT_URDF = _ASSET_ROOT / "litearm" / "urdf" / "litearm.urdf"


def robot_urdf_path() -> str:
    return str(_DEFAULT_URDF)


class RobotModel:
    """One mutable Pinocchio model/data/geometry bundle.

    Pinocchio updates ``Data`` and ``GeometryData`` in-place, so every bundle
    owns its data and serializes access.  Kinematics and collision queries on
    the same bundle therefore always use the same URDF and frame selection.
    """

    def __init__(self, urdf_path: Optional[Union[str, Path]] = None,
                 package_dirs: Optional[Union[
                     str, Path, Sequence[Union[str, Path]]]] = None,
                 *, dependency_hint: str = "litearm-python[robotics]",
                 require_ee: bool = True):
        try:
            self.np = importlib.import_module("numpy")
            self.pin = importlib.import_module("pinocchio")
        except (ImportError, OSError) as exc:
            raise ImportError(
                "SDK 本地运动学/碰撞检测需要 NumPy 与 Pinocchio/hpp-fcl；"
                f"请安装 {dependency_hint}。若当前 shell source 过 ROS，请用 "
                "`env -u PYTHONPATH -u LD_LIBRARY_PATH python ...` 运行"
            ) from exc

        path = Path(urdf_path) if urdf_path is not None else _DEFAULT_URDF
        path = path.expanduser().resolve()
        if not path.is_file():
            raise FileNotFoundError(f"LiteArm URDF 不存在: {path}")
        if package_dirs is None:
            packages = [str(_ASSET_ROOT)]
        elif isinstance(package_dirs, (str, Path)):
            packages = [str(Path(package_dirs).expanduser().resolve())]
        else:
            packages = [str(Path(p).expanduser().resolve()) for p in package_dirs]

        self.urdf_path = str(path)
        self.package_dirs = tuple(packages)
        self.model = self.pin.buildModelFromUrdf(self.urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = None
        if require_ee:
            matches = [i for i, frame in enumerate(self.model.frames)
                       if frame.name == "ee_link" and frame.type == self.pin.BODY]
            if len(matches) != 1:
                raise ValueError(f"URDF 应恰有一个 BODY ee_link，实得 {len(matches)}")
            self.ee_frame_id = matches[0]

        self.geom = self.pin.buildGeomFromUrdf(
            self.model, self.urdf_path, self.pin.COLLISION,
            package_dirs=list(self.package_dirs))
        self.geom.addAllCollisionPairs()
        self.n_all_pairs = len(self.geom.collisionPairs)
        self._remove_adjacent_pairs()
        self.n_active_pairs = len(self.geom.collisionPairs)
        self.geom_data = self.geom.createData()
        self.lock = threading.RLock()

    @property
    def nq(self) -> int:
        return int(self.model.nq)

    def q(self, values: Iterable[float]):
        try:
            vals = tuple(float(v) for v in values)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"q 必须包含 {self.nq} 个有限数值") from exc
        if len(vals) != self.nq or not all(math.isfinite(v) for v in vals):
            raise ValueError(f"q 必须包含 {self.nq} 个有限数值")
        return self.np.asarray(vals, dtype=float)

    def _remove_adjacent_pairs(self) -> None:
        remove = []
        for pair in self.geom.collisionPairs:
            first, second = int(pair.first), int(pair.second)
            j1 = int(self.geom.geometryObjects[first].parentJoint)
            j2 = int(self.geom.geometryObjects[second].parentJoint)
            if (j1 == j2 or int(self.model.parents[j1]) == j2
                    or int(self.model.parents[j2]) == j1):
                remove.append((first, second))
        for first, second in remove:
            self.geom.removeCollisionPair(self.pin.CollisionPair(first, second))

    def placement(self, q):
        if self.ee_frame_id is None:
            raise RuntimeError("该 RobotModel 未启用 ee_link 运动学")
        self.pin.forwardKinematics(self.model, self.data, q)
        self.pin.updateFramePlacements(self.model, self.data)
        return self.data.oMf[self.ee_frame_id]

    def jacobian(self, q):
        if self.ee_frame_id is None:
            raise RuntimeError("该 RobotModel 未启用 ee_link 运动学")
        return self.np.asarray(self.pin.computeFrameJacobian(
            self.model, self.data, q, self.ee_frame_id,
            self.pin.LOCAL_WORLD_ALIGNED), dtype=float)

    def sigma_min(self, q) -> float:
        singular = self.np.linalg.svd(self.jacobian(q), compute_uv=False)
        value = float(singular[-1]) if singular.size else 0.0
        return value if math.isfinite(value) else 0.0

    def in_collision(self, q) -> bool:
        return bool(self.pin.computeCollisions(
            self.model, self.data, self.geom, self.geom_data, q, True))

    def colliding_pairs(self, q) -> list[tuple[str, str]]:
        self.pin.computeCollisions(
            self.model, self.data, self.geom, self.geom_data, q, False)
        result = []
        for index, collision in enumerate(self.geom_data.collisionResults):
            if collision.isCollision():
                pair = self.geom.collisionPairs[index]
                result.append((str(self.geom.geometryObjects[int(pair.first)].name),
                               str(self.geom.geometryObjects[int(pair.second)].name)))
        return result

    def minimum_distance(self, q) -> float:
        if not self.geom.collisionPairs:
            return math.inf
        self.pin.computeDistances(
            self.model, self.data, self.geom, self.geom_data, q)
        values = [float(result.min_distance)
                  for result in self.geom_data.distanceResults]
        if not values or not all(math.isfinite(v) for v in values):
            raise ValueError("碰撞距离计算返回非有限值")
        return min(values)


__all__ = ["RobotModel", "robot_urdf_path"]
