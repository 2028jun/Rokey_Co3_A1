"""Spawn the stock Clearpath Jackal J100 base and run a wheel-driven test route.

The test deliberately excludes ROS, Nav2, sensors, payloads, and direct body
velocity control.  All planar movement comes from the four wheel joints.
"""

from __future__ import annotations

import math
import os
from pathlib import Path

import numpy as np

from isaacsim import SimulationApp


WORKSPACE = Path(__file__).resolve().parents[1]
os.environ["ROS_PACKAGE_PATH"] = ":".join(
    value
    for value in (
        str(WORKSPACE / "src"),
        os.environ.get("ROS_PACKAGE_PATH", ""),
    )
    if value
)
HEADLESS = os.environ.get("NAV_ROBOT_HEADLESS", "0") == "1"
simulation_app = SimulationApp({"headless": HEADLESS})

import omni.kit.app
import omni.kit.commands
import omni.timeline
import omni.usd
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, UsdGeom, UsdPhysics, UsdShade


_jackal_urdf_path = os.environ.get("JACKAL_URDF_PATH")
URDF_PATH = (
    Path(_jackal_urdf_path).expanduser().resolve()
    if _jackal_urdf_path
    else None
)
ROBOT_USD = WORKSPACE / "assets/diagnostics/jackal_j100_stock_v1.usd"
RESTAURANT_USD = WORKSPACE / "assets/lightweight_restaurant/lightweight_pizza_restaurant.usda"
ROBOT_ROOT = "/World/JackalDiagnostic"
ASSET_ROOT = "/j100_base"
ROBOT_LABEL = "stock J100 base"
MERGE_FIXED_JOINTS = True
WHEEL_DRIVE_DAMPING = 35.0
WHEEL_DRIVE_MAX_FORCE = 60.0
POST_INITIALIZE_HOOK = None
SPAWN_POSITION = Gf.Vec3d(0.0, 5.25, 0.12)
SPAWN_YAW = -math.pi / 2.0

WHEEL_NAMES = (
    "front_left_wheel_joint",
    "front_right_wheel_joint",
    "rear_left_wheel_joint",
    "rear_right_wheel_joint",
)
WHEEL_LINKS = {name.replace("_joint", "_link") for name in WHEEL_NAMES}


def yaw_quaternion(yaw):
    return Gf.Quatf(math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0))


def quaternion_to_yaw(orientation):
    if hasattr(orientation, "GetReal"):
        w = float(orientation.GetReal())
        x, y, z = [float(value) for value in orientation.GetImaginary()]
    else:
        w, x, y, z = [float(value) for value in orientation]
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def import_robot():
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
    ROBOT_USD.parent.mkdir(parents=True, exist_ok=True)
    if ROBOT_USD.is_file() and os.environ.get("NAV_ROBOT_REIMPORT", "0") != "1":
        return
    status, config = omni.kit.commands.execute("URDFCreateImportConfig")
    if not status:
        raise RuntimeError("URDFCreateImportConfig failed")
    config.merge_fixed_joints = MERGE_FIXED_JOINTS
    config.convex_decomp = False
    config.import_inertia_tensor = True
    config.fix_base = False
    config.collision_from_visuals = False
    config.distance_scale = 1.0
    status, _ = omni.kit.commands.execute(
        "URDFParseAndImportFile",
        urdf_path=str(URDF_PATH),
        import_config=config,
        dest_path=str(ROBOT_USD),
        get_articulation_root=True,
    )
    if not status or not ROBOT_USD.is_file():
        raise RuntimeError(f"failed to import {URDF_PATH}")


def configure_physics(stage):
    scene = stage.GetPrimAtPath("/World/PhysicsScene")
    if not scene.IsValid():
        raise RuntimeError("restaurant PhysicsScene is missing")
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(scene)
    physx_scene.CreateEnableStabilizationAttr(True)
    physx_scene.CreateEnableGPUDynamicsAttr(False)
    physx_scene.CreateBroadphaseTypeAttr("MBP")
    physx_scene.CreateTimeStepsPerSecondAttr(120)

    articulation_path = None
    for prim in stage.Traverse():
        if str(prim.GetPath()).startswith(ROBOT_ROOT) and prim.HasAPI(
            UsdPhysics.ArticulationRootAPI
        ):
            articulation_path = str(prim.GetPath())
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateSolverPositionIterationCountAttr(32)
            api.CreateSolverVelocityIterationCountAttr(4)
            api.CreateStabilizationThresholdAttr(0.01)
            api.CreateSleepThresholdAttr(0.05)
            break
    if articulation_path is None:
        raise RuntimeError("Jackal articulation root not found")

    material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/JackalTire")
    physics_material = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    physics_material.CreateStaticFrictionAttr(0.5)
    physics_material.CreateDynamicFrictionAttr(0.5)
    physics_material.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr(
        "average"
    )
    colliders = []
    for prim in stage.Traverse():
        if (
            str(prim.GetPath()).startswith(ROBOT_ROOT)
            and prim.GetName() == "collisions"
            and prim.GetParent().GetName() in WHEEL_LINKS
        ):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
            colliders.append(str(prim.GetPath()))
    if len(colliders) != len(WHEEL_NAMES):
        raise RuntimeError(
            f"expected {len(WHEEL_NAMES)} drive-wheel colliders, got {colliders}"
        )
    return articulation_path


def configure_wheel_drives(stage):
    found = set()
    for prim in stage.Traverse():
        if prim.GetName() not in WHEEL_NAMES or not str(prim.GetPath()).startswith(
            ROBOT_ROOT
        ):
            continue
        drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateStiffnessAttr(0.0)
        drive.CreateDampingAttr(WHEEL_DRIVE_DAMPING)
        drive.CreateMaxForceAttr(WHEEL_DRIVE_MAX_FORCE)
        drive.CreateTargetVelocityAttr(0.0)
        found.add(prim.GetName())
    if found != set(WHEEL_NAMES):
        raise RuntimeError(f"missing Jackal wheel joints: {sorted(set(WHEEL_NAMES)-found)}")


class Route:
    WHEEL_RADIUS = 0.098
    HALF_TRACK = 0.37559 / 2.0

    def __init__(self, articulation):
        self.robot = articulation
        names = list(articulation.dof_names)
        self.indices = np.asarray([names.index(name) for name in WHEEL_NAMES], dtype=np.int32)
        self.phase = "straight"
        self.v = 0.0
        self.w = 0.0
        self.stopped_at = None
        print("[wheel-route] straight to y=-2.20, rotate left 90deg, then stop", flush=True)

    @staticmethod
    def slew(value, target, rate, dt):
        delta = target - value
        return target if abs(delta) <= rate * dt else value + math.copysign(rate * dt, delta)

    def step(self, dt, sim_time):
        position, orientation = self.robot.get_world_pose()
        y = float(position[1])
        yaw = quaternion_to_yaw(orientation)
        target_v = 0.0
        target_w = 0.0
        if self.phase == "straight":
            remaining = y - (-2.20)
            if remaining <= 0.02:
                self.phase = "turn"
                print(f"[jackal-route] turn start y={y:.3f}", flush=True)
            else:
                target_v = min(0.35, max(0.05, 0.55 * remaining))
                target_w = np.clip(1.5 * wrap(-math.pi / 2.0 - yaw), -0.15, 0.15)
        if self.phase == "turn":
            error = wrap(0.0 - yaw)
            if abs(error) <= math.radians(2.0):
                self.phase = "stopped"
                self.stopped_at = sim_time
                print(f"[jackal-route] stopped yaw={math.degrees(yaw):.2f}deg", flush=True)
            else:
                target_w = float(np.clip(1.0 * error, 0.10, 0.40))
        self.v = self.slew(self.v, target_v, 0.20, dt)
        self.w = self.slew(self.w, target_w, 0.35, dt)
        if self.phase == "stopped" and abs(self.v) < 0.002 and abs(self.w) < 0.002:
            self.v = self.w = 0.0
        turn = self.HALF_TRACK * self.w
        velocity = np.asarray(
            [
                (self.v - turn) / self.WHEEL_RADIUS,
                (self.v + turn) / self.WHEEL_RADIUS,
                (self.v - turn) / self.WHEEL_RADIUS,
                (self.v + turn) / self.WHEEL_RADIUS,
            ]
        )
        self.robot.apply_action(
            ArticulationAction(joint_velocities=velocity, joint_indices=self.indices)
        )


def main():
    if URDF_PATH is None:
        raise RuntimeError(
            "JACKAL_URDF_PATH가 설정되어 있지 않습니다. "
            "Jackal J100 URDF의 절대경로를 지정하세요."
        )
    for path in (URDF_PATH, RESTAURANT_USD):
        if not path.is_file():
            raise FileNotFoundError(path)
    import_robot()
    context = omni.usd.get_context()
    if not context.open_stage(str(RESTAURANT_USD)):
        raise RuntimeError(f"failed to open {RESTAURANT_USD}")
    for _ in range(30):
        simulation_app.update()
    stage = context.get_stage()
    spawn = UsdGeom.Xform.Define(stage, ROBOT_ROOT)
    spawn.AddTranslateOp().Set(SPAWN_POSITION)
    spawn.AddOrientOp().Set(yaw_quaternion(SPAWN_YAW))
    robot = UsdGeom.Xform.Define(stage, f"{ROBOT_ROOT}/Robot")
    robot.GetPrim().GetReferences().AddReference(str(ROBOT_USD), Sdf.Path(ASSET_ROOT))
    for _ in range(5):
        simulation_app.update()
    configure_wheel_drives(stage)
    articulation_path = configure_physics(stage)
    timeline = omni.timeline.get_timeline_interface()
    timeline.play()
    for _ in range(3):
        simulation_app.update()
    articulation = SingleArticulation(
        prim_path=articulation_path, name="jackal_j100_diagnostic"
    )
    articulation.initialize()
    articulation.set_enabled_self_collisions(False)
    if POST_INITIALIZE_HOOK is not None:
        POST_INITIALIZE_HOOK(stage)
    route = Route(articulation)
    start = last = timeline.get_current_time()
    duration = float(os.environ.get("NAV_DIAGNOSTIC_SECONDS", "0"))
    print(
        f"[wheel-test] {ROBOT_LABEL}; wheel-drive only; articulation={articulation_path}",
        flush=True,
    )
    try:
        while simulation_app.is_running():
            simulation_app.update()
            now = timeline.get_current_time()
            route.step(min(max(now - last, 1.0 / 240.0), 0.05), now)
            last = now
            if duration > 0.0 and now - start >= duration:
                break
    finally:
        simulation_app.close()


if __name__ == "__main__":
    main()
