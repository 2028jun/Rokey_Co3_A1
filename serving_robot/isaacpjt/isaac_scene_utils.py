"""Small USD scene helpers shared by Isaac Sim demo modules."""

import numpy as np
from pxr import UsdGeom


def find_serving_robot_prim(stage, name):
    """Return one named prim below the serving robot hierarchy."""
    matches = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == name
        and (str(prim.GetPath()).startswith("/World/ServingRobot") or str(prim.GetPath()).startswith("/World/NavRobot"))
    ]
    if len(matches) != 1:
        raise RuntimeError(f"expected one serving-robot {name}, got {matches}")
    return matches[0]


def prim_world_pose(prim):
    """Return a prim's world translation, scalar-first quaternion, and matrix."""
    transform = UsdGeom.XformCache().GetLocalToWorldTransform(prim)
    translation = transform.ExtractTranslation()
    rotation = transform.ExtractRotationQuat()
    imaginary = rotation.GetImaginary()
    return (
        np.array(translation, dtype=float),
        np.array(
            [rotation.GetReal(), imaginary[0], imaginary[1], imaginary[2]],
            dtype=float,
        ),
        transform,
    )
