#!/usr/bin/env python3
"""Isaac test: IsaacWheelHold resists external wheel torque.

Headless:
  MAP_GEN_HEADLESS=1 python.sh examples/test_isaac_wheel_hold.py

GUI (watch wheels):
  MAP_GEN_HEADLESS=0 python.sh examples/test_isaac_wheel_hold.py

Phases (on-screen print):
  1) WITHOUT hold — wheels spin under velocity cmds
  2) WITH hold — effort disturbance, wheels stay put
  3) AFTER release — wheels spin again
"""

from __future__ import annotations

import math
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parent / "map_generate"
# Prefer source tree over stale colcon install.
sys.path.insert(0, str(ROOT / "install" / "base_hold" / "lib" / "python3.10" / "site-packages"))
sys.path.insert(0, str(ROOT / "src" / "base_hold"))

os.environ.setdefault("MAP_GEN_WS", str(WS))
os.environ.setdefault("ROS_DOMAIN_ID", "119")

HEADLESS = os.environ.get("MAP_GEN_HEADLESS", "0").strip() not in ("0", "false", "False", "")

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": HEADLESS})

import numpy as np
import omni.timeline
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import get_current_stage
from pxr import Gf, PhysxSchema, Sdf, UsdPhysics
import omni.usd

RESTAURANT = WS / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
ROBOT = WS / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
ROBOT_ASSET_ROOT = "/two_wheel_ridgeback_serving_robot"
ROBOT_ROOT = "/World/NavRobot/Robot"
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
SPAWN = Gf.Vec3d(0.0, 5.25, 0.002)
ARTICULATION_CANDIDATES = [
    f"{ROBOT_ROOT}/ridgeback_base_link",
    ROBOT_ROOT,
]

# Longer loops in GUI so you can see each phase.
STEPS_FREE = 90 if HEADLESS else 240
STEPS_HOLD = 120 if HEADLESS else 300
STEPS_AFTER = 60 if HEADLESS else 180
PHASE_PAUSE_S = 0.0 if HEADLESS else 2.0
# GUI: keep Isaac open until the user closes the window (0 = forever).
KEEP_OPEN_S = 0.0 if HEADLESS else float(os.environ.get("HOLD_GUI_KEEP_S", "0") or 0)
# GUI: repeat spin/hold/release cycles so you have time to watch.
GUI_LOOPS = 1 if HEADLESS else int(os.environ.get("HOLD_GUI_LOOPS", "3") or 3)


def _banner(msg: str) -> None:
    line = "=" * 60
    print(f"\n{line}\n[test] {msg}\n{line}", flush=True)


def _open_scene():
    from pxr import UsdGeom

    if not RESTAURANT.is_file():
        raise FileNotFoundError(RESTAURANT)
    if not ROBOT.is_file():
        raise FileNotFoundError(ROBOT)
    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT)):
        raise RuntimeError("open restaurant failed")
    for _ in range(30):
        simulation_app.update()
    stage = get_current_stage()
    spawn = UsdGeom.Xform.Define(stage, ROBOT_ROOT)
    spawn.AddTranslateOp().Set(SPAWN)
    spawn.AddOrientOp().Set(
        Gf.Quatf(float(math.cos(-math.pi / 4)), 0.0, 0.0, float(math.sin(-math.pi / 4)))
    )
    robot = UsdGeom.Xform.Define(stage, f"{ROBOT_ROOT}/Robot")
    robot.GetPrim().GetReferences().AddReference(str(ROBOT), Sdf.Path(ROBOT_ASSET_ROOT))
    return stage


def _configure_physics(stage, art_path: str):
    scene_prim = stage.GetPrimAtPath("/World/PhysicsScene")
    if scene_prim.IsValid():
        physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene_prim)
        physx_scene.CreateEnableStabilizationAttr(True)
        physx_scene.CreateEnableGPUDynamicsAttr(False)
    articulation_api = PhysxSchema.PhysxArticulationAPI.Apply(
        stage.GetPrimAtPath(art_path)
    )
    articulation_api.CreateSolverPositionIterationCountAttr(32)
    articulation_api.CreateSolverVelocityIterationCountAttr(4)


def _configure_wheels(stage):
    for prim in stage.Traverse():
        if prim.GetName() in WHEEL_JOINTS:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(0.0)
            drive.CreateDampingAttr(140.0)
            drive.CreateMaxForceAttr(350.0)
            drive.CreateTargetVelocityAttr(0.0)


def _find_articulation(stage) -> str:
    for path in ARTICULATION_CANDIDATES:
        if stage.GetPrimAtPath(path).IsValid():
            return path
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if path.startswith(ROBOT_ROOT) and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return path
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
            return str(prim.GetPath())
    raise RuntimeError("no articulation")


def _aim_camera() -> None:
    """Point viewport at the spawned robot (GUI only)."""
    if HEADLESS:
        return
    try:
        import omni.kit.viewport.utility as vp_util

        viewport = vp_util.get_active_viewport()
        if viewport is None:
            return
        cam_path = viewport.get_active_camera()
        stage = get_current_stage()
        cam = stage.GetPrimAtPath(cam_path)
        if not cam.IsValid():
            return
        from pxr import UsdGeom

        xform = UsdGeom.Xformable(cam)
        # Eye near kitchen spawn looking at robot
        eye = Gf.Vec3d(SPAWN[0] + 2.5, SPAWN[1] - 2.0, SPAWN[2] + 1.6)
        # Clear existing ops then set translate (best-effort)
        for op in xform.GetOrderedXformOps():
            if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                op.Set(eye)
                break
        else:
            xform.AddTranslateOp().Set(eye)
        print(f"[test] camera near robot @ {tuple(eye)}", flush=True)
    except Exception as exc:
        print(f"[test] camera aim skipped: {exc}", flush=True)


def _init_robot(path: str):
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(10):
        simulation_app.update()
    art = SingleArticulation(prim_path=path, name="hold_test")
    art.initialize()
    for _ in range(30):
        if art.handles_initialized:
            break
        simulation_app.update()
        art.initialize()
    if not art.handles_initialized:
        raise RuntimeError("articulation init failed")
    dof = list(art.dof_names)
    pos = art.get_joint_positions()
    for w in WHEEL_JOINTS:
        if w in dof:
            pos[dof.index(w)] = 0.0
    art.set_joint_positions(pos)
    art.set_joint_velocities(np.zeros(len(dof)))
    return art, dof


def _wheel_angles(art, dof):
    pos = art.get_joint_positions()
    return {w: float(pos[dof.index(w)]) for w in WHEEL_JOINTS if w in dof}


def _apply_wheel_vel(art, dof, omega: float):
    from isaacsim.core.utils.types import ArticulationAction

    vel = np.zeros(len(dof), dtype=float)
    for w in WHEEL_JOINTS:
        if w in dof:
            vel[dof.index(w)] = omega
    art.apply_action(ArticulationAction(joint_velocities=vel))


def _apply_wheel_effort(art, dof, tau: float):
    efforts = np.zeros(len(dof), dtype=float)
    for w in WHEEL_JOINTS:
        if w in dof:
            efforts[dof.index(w)] = tau
    art.set_joint_efforts(efforts)


def _pause(seconds: float) -> None:
    if seconds <= 0:
        return
    t_end = time.time() + seconds
    while time.time() < t_end and simulation_app.is_running():
        simulation_app.update()


def main() -> int:
    from base_hold.isaac_wheel_hold import IsaacWheelHold

    mode = "headless" if HEADLESS else "GUI"
    _banner(f"opening restaurant stage ({mode})")
    stage = _open_scene()
    _configure_wheels(stage)
    art_path = _find_articulation(stage)
    _configure_physics(stage, art_path)
    print(f"[test] articulation={art_path}", flush=True)
    art, dof = _init_robot(art_path)
    print(f"[test] dofs={len(dof)} wheels={[w for w in WHEEL_JOINTS if w in dof]}", flush=True)
    _aim_camera()

    for _ in range(60):
        simulation_app.update()
    _pause(PHASE_PAUSE_S)

    drift_free = drift_hold = drift_after = 0.0
    hold = IsaacWheelHold(stage, [w for w in WHEEL_JOINTS if w in dof])

    for loop in range(max(1, GUI_LOOPS)):
        if not simulation_app.is_running():
            break
        _banner(f"LOOP {loop + 1}/{GUI_LOOPS}")

        # --- 1) baseline: no hold ---
        _banner("PHASE 1/3  WITHOUT hold — wheels should SPIN")
        a0 = _wheel_angles(art, dof)
        for i in range(STEPS_FREE):
            if not simulation_app.is_running():
                break
            _apply_wheel_vel(art, dof, 8.0)
            simulation_app.update()
            if not HEADLESS and i % 60 == 0:
                ang = _wheel_angles(art, dof)
                print(f"[test] spinning… θ={ {k: round(v, 2) for k, v in ang.items()} }", flush=True)
        a1 = _wheel_angles(art, dof)
        drift_free = max(abs(a1[w] - a0[w]) for w in a0)
        print(f"[test] WITHOUT hold: max |Δθ|={drift_free:.3f} rad", flush=True)
        _pause(PHASE_PAUSE_S)

        for _ in range(30):
            _apply_wheel_vel(art, dof, 0.0)
            simulation_app.update()

        # --- 2) with hold ---
        _banner("PHASE 2/3  WITH hold — wheels should STAY (effort disturbance)")
        if hold.held:
            hold.release()
        hold.engage(articulation=art, dof_names=dof)
        a2 = _wheel_angles(art, dof)
        for i in range(STEPS_HOLD):
            if not simulation_app.is_running():
                break
            _apply_wheel_effort(art, dof, 80.0)
            hold.tick()
            simulation_app.update()
            if not HEADLESS and i % 60 == 0:
                ang = _wheel_angles(art, dof)
                print(f"[test] holding… θ={ {k: round(v, 2) for k, v in ang.items()} }", flush=True)
        a3 = _wheel_angles(art, dof)
        drift_hold = max(abs(a3[w] - a2[w]) for w in a2)
        print(f"[test] WITH hold+effort: max |Δθ|={drift_hold:.3f} rad", flush=True)
        _pause(PHASE_PAUSE_S)

        # --- 3) after release ---
        _banner("PHASE 3/3  AFTER release — wheels should SPIN again")
        hold.release()
        for i in range(STEPS_AFTER):
            if not simulation_app.is_running():
                break
            _apply_wheel_vel(art, dof, 8.0)
            simulation_app.update()
            if not HEADLESS and i % 60 == 0:
                ang = _wheel_angles(art, dof)
                print(f"[test] free again… θ={ {k: round(v, 2) for k, v in ang.items()} }", flush=True)
        a4 = _wheel_angles(art, dof)
        drift_after = max(abs(a4[w] - a3[w]) for w in a3)
        print(f"[test] AFTER release: max |Δθ|={drift_after:.3f} rad", flush=True)

    ok = drift_hold < 0.5 and drift_free > 1.0 and drift_hold < 0.25 * drift_free
    _banner(
        f"RESULT drift_free={drift_free:.3f} drift_hold={drift_hold:.3f} "
        f"drift_after={drift_after:.3f} => {'PASS' if ok else 'FAIL'}"
    )

    if not HEADLESS:
        if KEEP_OPEN_S > 0:
            print(f"[test] keeping window open ~{KEEP_OPEN_S:.0f}s", flush=True)
            _pause(KEEP_OPEN_S)
        else:
            print("[test] idle — close the Isaac window when done watching", flush=True)
            while simulation_app.is_running():
                simulation_app.update()

    simulation_app.close()
    return 0 if ok else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[test] ERROR: {exc}", flush=True)
        try:
            simulation_app.close()
        except Exception:
            pass
        raise
