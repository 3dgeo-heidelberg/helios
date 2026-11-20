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
        """Construct a rotation from quaternion coordinates.

        The scalar component comes first. The supplied quaternion is normalized
        before it is used, so it does not need to have unit length.

        Parameters
        ----------
        quaternion : Quaternion
            Quaternion ``(q0, q1, q2, q3)``, where ``q0`` is the scalar
            component and the remaining coordinates form the vector component.
            The quaternion must be non-zero.

        Returns
        -------
        Rotation
            The rotation represented by the normalized quaternion.

        Raises
        ------
        ValueError
            If ``quaternion`` has zero norm.
        """
        if np.linalg.norm(quaternion) == 0:
            raise ValueError("A rotation quaternion must be non-zero.")
        return cls._from_cpp(_helios.Rotation(*quaternion, True))

    @classmethod
    @validate_call
    def from_axis_angle(cls, *, axis: R3Vector, angle: Angle):
        """Construct a rotation from an axis and an angle.

        Angles describe the effect on vectors in a fixed frame. For example, a
        positive quarter-turn of ``pi / 2`` about the positive z-axis maps the
        positive x-axis to the positive y-axis. The axis is normalized
        internally.

        Parameters
        ----------
        axis : R3Vector
            Axis around which to rotate. It must be non-zero.
        angle : Angle
            Rotation angle in radians or as an angle quantity.

        Returns
        -------
        Rotation
            The rotation about ``axis`` by ``angle``.

        Raises
        ------
        ValueError
            If ``axis`` has zero norm.
        """
        if np.linalg.norm(axis) == 0:
            raise ValueError("A rotation axis must be non-zero.")
        return cls._from_cpp(_helios.Rotation(axis, angle))

    @classmethod
    @validate_call
    def from_directions(cls, *, source: R3Vector, target: R3Vector):
        """Construct the smallest-angle rotation between two directions.

        Applying the result to ``source`` produces a vector in the direction of
        ``target``; differences in vector magnitude are ignored. Of the
        infinitely many possible rotations, this method chooses the one with
        the smallest angle, whose axis is orthogonal to the plane of the two
        vectors. If the vectors are antiparallel, an arbitrary orthogonal axis
        is chosen.

        Parameters
        ----------
        source : R3Vector
            Direction before the rotation. It must be non-zero.
        target : R3Vector
            Desired direction after the rotation. It must be non-zero.

        Returns
        -------
        Rotation
            The smallest-angle rotation from ``source`` to ``target``.

        Raises
        ------
        ValueError
            If either direction has zero norm.
        """
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
        """Construct a rotation from successive elementary axis rotations.

        The rotations are applied in the order given. With ``frame="global"``,
        every axis refers to the fixed global coordinate system. With
        ``frame="local"``, every axis refers to the local coordinate system as
        transformed by the preceding rotations.

        Parameters
        ----------
        rotations : Sequence[tuple[Literal["x", "y", "z"], Angle]]
            Ordered ``(axis, angle)`` pairs defining the elementary rotations.
            Angles may be given in radians or as angle quantities.
        frame : {"global", "local"}, optional
            Coordinate frame in which the axes are interpreted. The default is
            ``"global"``.

        Returns
        -------
        Rotation
            The composition of the elementary rotations.
        """
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
