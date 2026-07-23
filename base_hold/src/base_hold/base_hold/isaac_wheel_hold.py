"""Isaac-safe wheel lock: high-stiffness joint position hold (no FixedJoint).

Problem
-------
cmd_vel=0 only stops *commanding* motion. Arm reaction torques can still spin
velocity-driven wheels (stiffness≈0). World FixedJoint / ground pins have
crashed Isaac in this project — do **not** use them.

Solution
--------
On engage: snapshot wheel angles, raise PhysX DOF gains (Kp/Kd) via the
articulation view, and hold with ``ArticulationAction(joint_positions=…)``.
Call ``tick()`` every sim step while held (re-asserts position targets).
On release: restore previous gains and return to velocity drive.

Important
---------
While held, the Isaac cmd_vel bridge must **not** apply wheel joint
velocities — ``apply_action(joint_velocities=…)`` overrides position mode.
``tick()`` alone cannot win if a later velocity action in the same step
zeros stiffness again; skip wheel velocity commands when ``held``.

Works for differential (2) or mecanum (4) wheel name lists.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Sequence

# Conservative vs arm drives (200k): enough to resist dock arm wrench, soft
# enough to avoid PhysX explosions.
DEFAULT_HOLD_STIFFNESS = 8000.0
DEFAULT_HOLD_DAMPING = 800.0
DEFAULT_HOLD_MAX_FORCE = 2000.0

# Typical nav restore (2-wheel map_generate / nav_robot6)
DEFAULT_RELEASE_STIFFNESS = 0.0
DEFAULT_RELEASE_DAMPING = 140.0
DEFAULT_RELEASE_MAX_FORCE = 350.0


@dataclass
class _SavedDrive:
    prim_path: str
    stiffness: float
    damping: float
    max_force: float
    target_position: float | None
    target_velocity: float | None


def _drive_api(prim):
    from pxr import UsdPhysics

    return UsdPhysics.DriveAPI.Apply(prim, "angular")


def _find_joint_prims(stage, joint_names: Iterable[str]) -> dict[str, object]:
    want = set(joint_names)
    found: dict[str, object] = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        if name in want:
            found[name] = prim
    missing = want - set(found)
    if missing:
        raise KeyError(f"wheel joint prims not found: {sorted(missing)}")
    return found


def _attr_get(attr, default: float) -> float:
    if attr and attr.HasAuthoredValue():
        return float(attr.Get())
    if attr:
        val = attr.Get()
        if val is not None:
            return float(val)
    return float(default)


class IsaacWheelHold:
    """Toggle wheel joints between velocity drive (nav) and position hold (arm)."""

    def __init__(
        self,
        stage,
        wheel_joint_names: Sequence[str],
        *,
        hold_stiffness: float = DEFAULT_HOLD_STIFFNESS,
        hold_damping: float = DEFAULT_HOLD_DAMPING,
        hold_max_force: float = DEFAULT_HOLD_MAX_FORCE,
        release_stiffness: float = DEFAULT_RELEASE_STIFFNESS,
        release_damping: float = DEFAULT_RELEASE_DAMPING,
        release_max_force: float = DEFAULT_RELEASE_MAX_FORCE,
    ) -> None:
        self._stage = stage
        self._names = list(wheel_joint_names)
        self._prims = _find_joint_prims(stage, self._names)
        self._hold_stiffness = hold_stiffness
        self._hold_damping = hold_damping
        self._hold_max_force = hold_max_force
        self._release_stiffness = release_stiffness
        self._release_damping = release_damping
        self._release_max_force = release_max_force
        self._saved: dict[str, _SavedDrive] = {}
        self._held = False
        self._locked_angles: dict[str, float] = {}
        self._articulation = None
        self._dof_names: list[str] | None = None
        self._wheel_indices: list[int] | None = None
        self._saved_kps = None
        self._saved_kds = None

    @property
    def held(self) -> bool:
        return self._held

    def engage(self, articulation=None, dof_names: Sequence[str] | None = None) -> None:
        """Lock wheels at current angles (position hold). Idempotent."""
        if self._held:
            return

        if articulation is None or dof_names is None:
            raise ValueError("engage requires articulation + dof_names for PhysX gains")

        self._articulation = articulation
        self._dof_names = list(dof_names)
        self._wheel_indices = [self._dof_names.index(n) for n in self._names]
        self._locked_angles = self._read_angles(articulation, self._dof_names)

        for name, prim in self._prims.items():
            drive = _drive_api(prim)
            path = str(prim.GetPath())
            self._saved[name] = _SavedDrive(
                prim_path=path,
                stiffness=_attr_get(drive.GetStiffnessAttr(), self._release_stiffness),
                damping=_attr_get(drive.GetDampingAttr(), self._release_damping),
                max_force=_attr_get(drive.GetMaxForceAttr(), self._release_max_force),
                target_position=(
                    float(drive.GetTargetPositionAttr().Get())
                    if drive.GetTargetPositionAttr()
                    else None
                ),
                target_velocity=(
                    float(drive.GetTargetVelocityAttr().Get())
                    if drive.GetTargetVelocityAttr()
                    else None
                ),
            )
            angle = self._locked_angles[name]
            drive.CreateStiffnessAttr(self._hold_stiffness)
            drive.CreateDampingAttr(self._hold_damping)
            drive.CreateMaxForceAttr(self._hold_max_force)
            drive.CreateTargetPositionAttr(angle)
            drive.CreateTargetVelocityAttr(0.0)

        self._apply_physx_gains(
            articulation,
            kps=self._hold_stiffness,
            kds=self._hold_damping,
            save=True,
        )
        self._zero_wheel_velocities(articulation, self._dof_names)
        self._held = True
        self.tick()

    def tick(self) -> None:
        """Re-assert wheel position targets. Call every sim step while held."""
        if not self._held or self._articulation is None or self._wheel_indices is None:
            return

        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction

        # Velocity actions zero Kp; restore gains every tick in case a stray
        # cmd_vel path ran earlier in the frame (still prefer skipping it).
        self._apply_physx_gains(
            self._articulation,
            kps=self._hold_stiffness,
            kds=self._hold_damping,
            save=False,
        )
        targets = np.array(
            [self._locked_angles[n] for n in self._names], dtype=float
        )
        indices = np.array(self._wheel_indices, dtype=np.int32)
        self._articulation.apply_action(
            ArticulationAction(joint_positions=targets, joint_indices=indices)
        )

    def release(self) -> None:
        """Restore velocity drive for navigation. Idempotent."""
        if not self._held:
            return

        for name, prim in self._prims.items():
            drive = _drive_api(prim)
            saved = self._saved.get(name)
            if saved is None:
                drive.CreateStiffnessAttr(self._release_stiffness)
                drive.CreateDampingAttr(self._release_damping)
                drive.CreateMaxForceAttr(self._release_max_force)
                drive.CreateTargetVelocityAttr(0.0)
            else:
                drive.CreateStiffnessAttr(saved.stiffness)
                drive.CreateDampingAttr(saved.damping)
                drive.CreateMaxForceAttr(saved.max_force)
                drive.CreateTargetVelocityAttr(
                    0.0 if saved.target_velocity is None else saved.target_velocity
                )
                if saved.target_position is not None:
                    drive.CreateTargetPositionAttr(saved.target_position)

        if self._articulation is not None and self._wheel_indices is not None:
            self._restore_physx_gains(self._articulation)
            self._zero_wheel_velocities(self._articulation, self._dof_names or [])

        self._saved.clear()
        self._locked_angles.clear()
        self._held = False
        self._articulation = None
        self._dof_names = None
        self._wheel_indices = None
        self._saved_kps = None
        self._saved_kds = None

    def _apply_physx_gains(self, articulation, *, kps: float, kds: float, save: bool) -> None:
        import numpy as np

        view = getattr(articulation, "_articulation_view", None)
        if view is None or not hasattr(view, "set_gains"):
            return
        n = len(self._wheel_indices or [])
        if n == 0:
            return
        if save:
            try:
                cur_kps, cur_kds = view.get_gains(joint_indices=np.array(self._wheel_indices))
                # shapes (1, K) for SingleArticulation
                self._saved_kps = np.array(cur_kps, dtype=float).reshape(-1)[:n].copy()
                self._saved_kds = np.array(cur_kds, dtype=float).reshape(-1)[:n].copy()
            except Exception:
                self._saved_kps = np.full(n, self._release_stiffness, dtype=float)
                self._saved_kds = np.full(n, self._release_damping, dtype=float)
        kp = np.full((1, n), float(kps), dtype=float)
        kd = np.full((1, n), float(kds), dtype=float)
        view.set_gains(
            kps=kp,
            kds=kd,
            joint_indices=np.array(self._wheel_indices, dtype=np.int32),
        )
        if hasattr(view, "set_max_efforts"):
            try:
                view.set_max_efforts(
                    np.full((1, n), float(self._hold_max_force), dtype=float),
                    joint_indices=np.array(self._wheel_indices, dtype=np.int32),
                )
            except Exception:
                pass

    def _restore_physx_gains(self, articulation) -> None:
        import numpy as np

        view = getattr(articulation, "_articulation_view", None)
        if view is None or self._wheel_indices is None:
            return
        n = len(self._wheel_indices)
        if self._saved_kps is not None and self._saved_kds is not None:
            kp = np.array(self._saved_kps, dtype=float).reshape(1, n)
            kd = np.array(self._saved_kds, dtype=float).reshape(1, n)
        else:
            kp = np.full((1, n), self._release_stiffness, dtype=float)
            kd = np.full((1, n), self._release_damping, dtype=float)
        view.set_gains(
            kps=kp,
            kds=kd,
            joint_indices=np.array(self._wheel_indices, dtype=np.int32),
        )

    def _read_angles(
        self, articulation, dof_names: Sequence[str] | None
    ) -> dict[str, float]:
        if articulation is None or dof_names is None:
            return {name: 0.0 for name in self._names}
        names = list(dof_names)
        positions = articulation.get_joint_positions()
        out: dict[str, float] = {}
        for name in self._names:
            if name not in names:
                raise KeyError(f"{name} not in articulation dof_names")
            out[name] = float(positions[names.index(name)])
        return out

    def _zero_wheel_velocities(self, articulation, dof_names: Sequence[str]) -> None:
        import numpy as np

        names = list(dof_names)
        vel = np.array(articulation.get_joint_velocities(), dtype=float)
        for name in self._names:
            vel[names.index(name)] = 0.0
        articulation.set_joint_velocities(vel)


# Backward-compatible names for call sites that imported parking brake APIs.
# These intentionally do NOT create FixedJoints.
def engage(stage, wheel_joint_names: Sequence[str], articulation=None, dof_names=None, **kwargs):
    hold = IsaacWheelHold(stage, wheel_joint_names, **kwargs)
    hold.engage(articulation=articulation, dof_names=dof_names)
    return hold


def release_hold(hold: IsaacWheelHold) -> None:
    hold.release()
