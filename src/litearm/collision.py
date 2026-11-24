"""Optional host-side self-collision and clearance checks."""
from __future__ import annotations

import importlib  # compatibility injection point used by dependency tests
from pathlib import Path
from typing import Iterable, Optional, Sequence, Union

from litearm._robot_model import RobotModel, robot_urdf_path


def collision_urdf_path() -> str:
    return robot_urdf_path()


class CollisionModel:
    """Pinocchio/hpp-fcl checker backed by the SDK's packaged URDF.

    Same-link and directly adjacent-link pairs are excluded because their
    meshes normally touch by design.  Each checker owns mutable Pinocchio data
    and serializes calls on that instance.
    """

    def __init__(self, urdf_path: Optional[Union[str, Path]] = None,
                 package_dirs: Optional[Union[
                     str, Path, Sequence[Union[str, Path]]]] = None,
                 *, _bundle: Optional[RobotModel] = None):
        self._bundle = _bundle or RobotModel(
            urdf_path, package_dirs, dependency_hint="litearm-python[collision]",
            require_ee=False)
        # Preserve the established diagnostic attributes.
        self._np = self._bundle.np
        self._pin = self._bundle.pin
        self.urdf_path = self._bundle.urdf_path
        self.package_dirs = self._bundle.package_dirs
        self.model = self._bundle.model
        self.data = self._bundle.data
        self.geom = self._bundle.geom
        self.geom_data = self._bundle.geom_data
        self.n_all_pairs = self._bundle.n_all_pairs
        self.n_active_pairs = self._bundle.n_active_pairs
        self._lock = self._bundle.lock

    @property
    def nq(self) -> int:
        return self._bundle.nq

    def _q(self, q: Iterable[float]):
        return self._bundle.q(q)

    def in_collision(self, q: Iterable[float]) -> bool:
        values = self._q(q)
        with self._lock:
            return self._bundle.in_collision(values)

    def colliding_pairs(self, q: Iterable[float]) -> list[tuple[str, str]]:
        values = self._q(q)
        with self._lock:
            return self._bundle.colliding_pairs(values)

    def minimum_distance(self, q: Iterable[float]) -> float:
        """Minimum separation in metres over active non-adjacent pairs."""
        values = self._q(q)
        with self._lock:
            return self._bundle.minimum_distance(values)


__all__ = ["CollisionModel", "collision_urdf_path"]
