"""Isaac Sim parking-brake helper (FixedJoint base <-> world).

Mirrors the idea in serving_robot's add_parking_brake(), but supports runtime
engage/release so a docked manipulator can lock the base against arm reaction
forces, then unlock for navigation.

This module is importable only inside Isaac (pxr / UsdPhysics). The ROS hold
node does not import it — Isaac demos/bridges call engage/release when they
see /base/hold_state change.
"""

from __future__ import annotations

from typing import Sequence

DEFAULT_BRAKE_PATH = "/World/BaseHold/ParkingBrake"


def engage(
    stage,
    articulation_path: str,
    world_xyz: Sequence[float],
    *,
    brake_path: str = DEFAULT_BRAKE_PATH,
    world_quat_wxyz: Sequence[float] = (1.0, 0.0, 0.0, 0.0),
) -> str:
    """Create (or replace) a FixedJoint pinning the articulation base to world.

    Parameters
    ----------
    stage :
        pxr Usd.Stage
    articulation_path :
        Prim path of the floating-base articulation root (body1).
    world_xyz :
        World-frame position where the base should be locked (usually current pose).
    brake_path :
        Prim path for the FixedJoint.
    world_quat_wxyz :
        World orientation as (w, x, y, z). Default identity.

    Returns
    -------
    str
        Path of the created joint.
    """
    from pxr import Gf, Sdf, UsdPhysics

    release(stage, brake_path=brake_path)

    joint = UsdPhysics.FixedJoint.Define(stage, brake_path)
    joint.CreateBody1Rel().SetTargets([Sdf.Path(articulation_path)])
    joint.CreateLocalPos0Attr().Set(
        Gf.Vec3f(float(world_xyz[0]), float(world_xyz[1]), float(world_xyz[2]))
    )
    w, x, y, z = world_quat_wxyz
    # UsdPhysics FixedJoint localRot0 uses Gf.Quatf(real, imag_i, imag_j, imag_k)
    joint.CreateLocalRot0Attr().Set(Gf.Quatf(float(w), float(x), float(y), float(z)))
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
    joint.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
    return brake_path


def release(stage, *, brake_path: str = DEFAULT_BRAKE_PATH) -> bool:
    """Remove the parking-brake FixedJoint if it exists.

    Returns
    -------
    bool
        True if a prim was removed, False if nothing to remove.
    """
    prim = stage.GetPrimAtPath(brake_path)
    if not prim.IsValid():
        return False
    stage.RemovePrim(brake_path)
    return True


def is_engaged(stage, *, brake_path: str = DEFAULT_BRAKE_PATH) -> bool:
    """Return True if the parking-brake prim exists."""
    return stage.GetPrimAtPath(brake_path).IsValid()
