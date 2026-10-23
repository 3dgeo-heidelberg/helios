from collections.abc import Sequence
from typing import Literal

import _helios
import numpy as np
from pydantic import validate_call

from helios.validation import Angle, Model, Quaternion, R3Vector


class Rotation(Model, cpp_class=_helios.Rotation):
    """A three-dimensional rotation backed by ``_helios.Rotation``."""

    q0: float = 1.0
    q1: float = 0.0
    q2: float = 0.0
    q3: float = 0.0

    @classmethod
    @validate_call
    def from_quaternion(cls, quaternion: Quaternion):
        if np.linalg.norm(quaternion) == 0:
            raise ValueError("A rotation quaternion must be non-zero.")
        return cls._from_cpp(_helios.Rotation(*quaternion, True))

    @classmethod
    @validate_call
    def from_axis_angle(cls, *, axis: R3Vector, angle: Angle):
        if np.linalg.norm(axis) == 0:
            raise ValueError("A rotation axis must be non-zero.")
        return cls._from_cpp(_helios.Rotation(axis, angle))

    @classmethod
    @validate_call
    def from_directions(cls, *, source: R3Vector, target: R3Vector):
        if np.linalg.norm(source) == 0 or np.linalg.norm(target) == 0:
            raise ValueError("Rotation source and target directions must be non-zero.")
        return cls._from_cpp(_helios.Rotation(source, target))

    @classmethod
    @validate_call
    def from_axis_angles(
        cls,
        *,
        rotations: Sequence[tuple[Literal["x", "y", "z"], Angle]],
        frame: Literal["global", "local"] = "global",
    ):
        result = _helios.Rotation()
        axes = {
            "x": np.array((1.0, 0.0, 0.0)),
            "y": np.array((0.0, 1.0, 0.0)),
            "z": np.array((0.0, 0.0, 1.0)),
        }
        for axis, angle in rotations:
            current = _helios.Rotation(axes[axis], angle)
            if frame == "global":
                result = current.apply_to(result)
            else:
                result = result.apply_to(current)
        return cls._from_cpp(result)
