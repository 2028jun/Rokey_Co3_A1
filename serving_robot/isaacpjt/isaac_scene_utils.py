"""Small USD scene helpers shared by Isaac Sim demo modules."""

import numpy as np
from pxr import UsdGeom


def find_serving_robot_prim(stage, name, robot_root=None):
    """Return one named prim below the selected serving robot hierarchy."""
    if robot_root:
        prefixes = (f"{str(robot_root).rstrip('/')}/",)
    else:
        prefixes = ("/World/ServingRobot", "/World/NavRobot")
    matches = [
        prim
        for prim in stage.Traverse()
        if prim.GetName() == name
        and str(prim.GetPath()).startswith(prefixes)
    ]
    if len(matches) != 1:
        scope = robot_root or "legacy serving-robot roots"
        raise RuntimeError(
            f"expected one serving-robot {name} below {scope}, got {matches}"
        )
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
