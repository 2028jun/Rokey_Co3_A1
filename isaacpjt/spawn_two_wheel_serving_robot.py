"""Ridgeback-sized serving robot on the stable two-wheel/caster base."""

from __future__ import annotations

import math
import os
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

import spawn_jackal_fixed_route as diagnostic
from kitchen_return_module import build_kitchen_route
from table_route_module import build_table_route
import omni.kit.app
import omni.kit.commands
import omni.usd
import omni.graph.core as og
import usdrt
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade


WORKSPACE = diagnostic.WORKSPACE
# The consolidated workspace keeps the canonical serving robot URDF here.
SOURCE_URDF = (
    WORKSPACE
    / "src/ridgeback_m0609_description/urdf/ridgeback_m0609.urdf"
)
GENERATED_URDF = Path("/tmp/two_wheel_ridgeback_serving_robot.urdf")
D455_USD = (
    WORKSPACE
    / "assets/isaac/Assets/Isaac/5.1/Isaac/Sensors/Intel/RealSense/rsd455.usd"
)
ROBOT_SENSOR_LAYER = (
    WORKSPACE
    / "assets/diagnostics/configuration/two_wheel_serving_robot_v2_sensor.usd"
)
_lidar_render_product = None
_lidar_writer = None

os.environ["ROS_PACKAGE_PATH"] = ":".join(
    value
    for value in (
        str(WORKSPACE / "src"),
        str(WORKSPACE / "isaacpjt/M0609"),
        str(WORKSPACE / "install/m0609_isaac_description/share"),
        os.environ.get("ROS_PACKAGE_PATH", ""),
    )
    if value
)


def make_link(name, visual_geometry, collision_geometry, mass, inertia):
    link = ET.Element("link", {"name": name})
    visual = ET.SubElement(link, "visual")
    ET.SubElement(visual, "origin", {"rpy": "1.5707963267948966 0 0"})
    ET.SubElement(visual, "geometry").append(visual_geometry)
    ET.SubElement(visual, "material", {"name": "ridgeback_black"})
    collision = ET.SubElement(link, "collision")
    ET.SubElement(collision, "origin", {"rpy": "1.5707963267948966 0 0"})
    ET.SubElement(collision, "geometry").append(collision_geometry)
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "mass", {"value": str(mass)})
    ET.SubElement(inertial, "inertia", inertia)
    return link


def prepare_urdf():
    tree = ET.parse(SOURCE_URDF)
    root = tree.getroot()
    old_wheels = {
        f"{corner}_{kind}"
        for corner in ("front", "rear")
        for kind in ("left_wheel_link", "right_wheel_link", "left_wheel_joint", "right_wheel_joint")
    }
    for child in list(root):
        if child.tag in {"link", "joint"} and child.get("name") in old_wheels:
            root.remove(child)

    wheel_inertia = {
        "ixx": "0.003125", "ixy": "0", "ixz": "0",
        "iyy": "0.005", "iyz": "0", "izz": "0.003125",
    }
    for side, y in (("left", "0.315"), ("right", "-0.315")):
        mesh = ET.Element(
            "mesh",
            {
                "filename": "package://ridgeback_m0609_description/meshes/r100/wheel.stl",
                "scale": "1.317523 1.317523 1.317523",
            },
        )
        cylinder = ET.Element("cylinder", {"radius": "0.10", "length": "0.079"})
        root.append(
            make_link(
                f"{side}_wheel_link", mesh, cylinder, "1.0", wheel_inertia
            )
        )
        joint = ET.Element("joint", {"name": f"{side}_wheel_joint", "type": "continuous"})
        ET.SubElement(joint, "parent", {"link": "ridgeback_base_link"})
        ET.SubElement(joint, "child", {"link": f"{side}_wheel_link"})
        ET.SubElement(joint, "origin", {"xyz": f"0 {y} 0.10"})
        ET.SubElement(joint, "axis", {"xyz": "0 1 0"})
        ET.SubElement(joint, "limit", {"effort": "400", "velocity": "20"})
        root.append(joint)

    caster_inertia = {
        "ixx": "0.000128", "ixy": "0", "ixz": "0",
        "iyy": "0.000128", "iyz": "0", "izz": "0.000128",
    }
    for end, x in (("front", "0.31"), ("rear", "-0.31")):
        sphere_visual = ET.Element("sphere", {"radius": "0.04"})
        sphere_collision = ET.Element("sphere", {"radius": "0.04"})
        link = make_link(
            f"{end}_caster_link",
            sphere_visual,
            sphere_collision,
            "0.2",
            caster_inertia,
        )
        # Spheres do not need the wheel cylinder's 90-degree visual transform.
        link.find("visual/origin").set("rpy", "0 0 0")
        link.find("collision/origin").set("rpy", "0 0 0")
        root.append(link)
        joint = ET.Element("joint", {"name": f"{end}_caster_joint", "type": "fixed"})
        ET.SubElement(joint, "parent", {"link": "ridgeback_base_link"})
        ET.SubElement(joint, "child", {"link": f"{end}_caster_link"})
        ET.SubElement(joint, "origin", {"xyz": f"{x} 0 0.04"})
        root.append(joint)

    root.set("name", "two_wheel_ridgeback_serving_robot")
    tree.write(GENERATED_URDF, encoding="utf-8", xml_declaration=True)


prepare_urdf()
diagnostic.URDF_PATH = GENERATED_URDF
diagnostic.ROBOT_USD = WORKSPACE / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
diagnostic.ROBOT_ROOT = "/World/TwoWheelServingRobot"
diagnostic.ASSET_ROOT = "/two_wheel_ridgeback_serving_robot"
diagnostic.ROBOT_LABEL = "Ridgeback body + trays + M0609/RG2 on two-wheel base"
diagnostic.MERGE_FIXED_JOINTS = False
diagnostic.SPAWN_POSITION = diagnostic.Gf.Vec3d(0.0, 5.25, 0.01)
diagnostic.WHEEL_NAMES = ("left_wheel_joint", "right_wheel_joint")
diagnostic.WHEEL_LINKS = {"left_wheel_link", "right_wheel_link"}
diagnostic.WHEEL_DRIVE_DAMPING = 140.0
diagnostic.WHEEL_DRIVE_MAX_FORCE = 350.0


class ServingRoute(diagnostic.Route):
    WHEEL_RADIUS = 0.10
    HALF_TRACK = 0.315

    def __init__(self, articulation):
        self.robot = articulation
        names = list(articulation.dof_names)
        self.indices = np.asarray(
            [names.index(name) for name in diagnostic.WHEEL_NAMES], dtype=np.int32
        )
        self.phase = "idle"
        self.v = self.w = 0.0
        self.mission = None
        self.pending_command = None
        arm_names = [f"joint_{index}" for index in range(1, 7)]
        arm_indices = np.asarray([names.index(name) for name in arm_names], dtype=np.int32)
        stow = np.deg2rad([90.0, 0.0, -90.0, 0.0, -60.0, 90.0])
        positions = articulation.get_joint_positions()
        positions[arm_indices] = stow
        tray_names = [
            name
            for name in (
                "upper_tray_left_slide_joint",
                "upper_tray_right_slide_joint",
            )
            if name in names
        ]
        tray_indices = np.asarray(
            [names.index(name) for name in tray_names], dtype=np.int32
        )
        if len(tray_indices):
            positions[tray_indices] = 0.0
        articulation.set_joint_positions(positions)
        articulation.apply_action(
            ArticulationAction(joint_positions=stow, joint_indices=arm_indices)
        )
        if len(tray_indices):
            articulation.apply_action(
                ArticulationAction(
                    joint_positions=np.zeros(len(tray_indices)),
                    joint_indices=tray_indices,
                )
            )
        print(
            f"[serving-robot] v2 sliding tray DOFs={tray_names}",
            flush=True,
        )

        # The command interface is deliberately independent of the route
        # modules: 0..3 select a table and 4 requests the kitchen.
        import rclpy
        from std_msgs.msg import Int32

        self.rclpy = rclpy
        if not rclpy.ok():
            rclpy.init()
        self.command_node = rclpy.create_node("two_wheel_serving_route")
        self.status_pub = self.command_node.create_publisher(
            Int32, "/navigation/status", 10
        )
        self.location_pub = self.command_node.create_publisher(
            Int32, "/navigation/current_location", 10
        )
        self.command_node.create_subscription(
            Int32, "/navigation/trigger", self._on_command, 10
        )
        self.Int32 = Int32
        self.location_pub.publish(Int32(data=4))
        self.status_pub.publish(Int32(data=2))
        print(
            "[serving-route] waiting for /navigation/trigger: "
            "0=table0, 1=table1, 2=table2, 3=table3, 4=kitchen",
            flush=True,
        )

    def _on_command(self, msg):
        target = int(msg.data)
        if target not in (0, 1, 2, 3, 4):
            print(f"[serving-route] ignored invalid target={target}", flush=True)
            return
        if self.mission is not None and target != 4:
            print("[serving-route] route already active; command ignored", flush=True)
            return
        self.pending_command = target
        self.status_pub.publish(self.Int32(data=1))

    def _start_command(self, target, x, y):
        stages = (
            build_kitchen_route(x, y)
            if target == 4
            else build_table_route(target, x, y)
        )
        self.mission = {"target": target, "stages": stages, "index": 0}
        self.phase = "route"
        self.v = self.w = 0.0
        label = "kitchen" if target == 4 else f"table{target}"
        print(f"[serving-route] command accepted: {label}", flush=True)

    def _finish_mission(self, position, yaw):
        target = self.mission["target"]
        self.mission = None
        self.phase = "idle"
        self.v = self.w = 0.0
        self.location_pub.publish(self.Int32(data=target))
        self.status_pub.publish(self.Int32(data=2))
        print(
            f"[serving-route] arrived target={target} "
            f"pose=({float(position[0]):.3f},{float(position[1]):.3f},"
            f"{math.degrees(yaw):.1f}deg); waiting for next command",
            flush=True,
        )

    def step(self, dt, sim_time):
        self.rclpy.spin_once(self.command_node, timeout_sec=0.0)
        position, orientation = self.robot.get_world_pose()
        x = float(position[0])
        y = float(position[1])
        yaw = diagnostic.quaternion_to_yaw(orientation)

        if self.pending_command is not None:
            target = self.pending_command
            self.pending_command = None
            self._start_command(target, x, y)

        target_v = target_w = 0.0
        if self.mission is not None:
            stage = self.mission["stages"][self.mission["index"]]
            if stage["kind"] == "pivot":
                error = diagnostic.wrap(stage["yaw"] - yaw)
                done = abs(error) <= math.radians(3.0)
                if not done:
                    target_w = math.copysign(
                        min(0.48, max(0.18, 1.4 * abs(error))), error
                    )
            else:
                axis = x if stage["kind"] == "axis_x" else y
                error = stage["value"] - axis
                done = abs(error) <= 0.025
                if not done:
                    target_v = math.copysign(
                        min(abs(stage["speed"]), max(0.04, 0.55 * abs(error))),
                        stage["speed"],
                    )
                    target_w = float(
                        np.clip(
                            1.2 * diagnostic.wrap(stage["yaw"] - yaw),
                            -0.08,
                            0.08,
                        )
                    )
            if done:
                self.mission["index"] += 1
                self.v = self.w = 0.0
                if self.mission["index"] >= len(self.mission["stages"]):
                    self._finish_mission(position, yaw)
                else:
                    next_kind = self.mission["stages"][self.mission["index"]]["kind"]
                    print(f"[serving-route] stage complete; next={next_kind}", flush=True)

        self.v = self.slew(self.v, target_v, 0.25, dt)
        angular_acceleration = 1.20 if target_w else 0.55
        self.w = self.slew(self.w, target_w, angular_acceleration, dt)
        if self.phase == "idle" and abs(self.v) < 0.002 and abs(self.w) < 0.002:
            self.v = self.w = 0.0
        turn = self.HALF_TRACK * self.w
        velocity = np.asarray(
            [(self.v-turn)/self.WHEEL_RADIUS, (self.v+turn)/self.WHEEL_RADIUS]
        )
        self.robot.apply_action(
            ArticulationAction(joint_velocities=velocity, joint_indices=self.indices)
        )


_configure_wheel_drives = diagnostic.configure_wheel_drives
_configure_physics = diagnostic.configure_physics


def find_base_path(stage):
    matches = [
        str(prim.GetPath())
        for prim in stage.Traverse()
        if prim.GetName() == "ridgeback_base_link"
        and str(prim.GetPath()).startswith(diagnostic.ROBOT_ROOT)
    ]
    if not matches:
        raise RuntimeError("ridgeback_base_link missing for sensor mounts")
    return max(matches, key=lambda path: path.count("/"))


def create_d455_ros_graph(stage, color_camera_path, depth_camera_path):
    """Render and publish the actual RGB and depth cameras inside the D455."""
    for camera_path in (color_camera_path, depth_camera_path):
        if not stage.GetPrimAtPath(camera_path).IsValid():
            raise RuntimeError(f"D455 camera prim is missing: {camera_path}")
    keys = og.Controller.Keys
    og.Controller.edit(
        {
            "graph_path": f"{diagnostic.ROBOT_ROOT}/Robot/D455ROS2",
            "evaluator_name": "execution",
        },
        {
            keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("ColorPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPub", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                # Match the integrated test: one D455 pseudo-depth render
                # product is shared by the RGB and depth annotators.
                ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(depth_camera_path)]),
                ("RenderProduct.inputs:width", 320),
                ("RenderProduct.inputs:height", 240),
                ("ColorPub.inputs:nodeNamespace", "camera/color"),
                ("ColorPub.inputs:topicName", "image_raw"),
                ("ColorPub.inputs:frameId", "d455_color_optical_frame"),
                ("ColorPub.inputs:type", "rgb"),
                ("ColorPub.inputs:frameSkipCount", 3),
                ("DepthPub.inputs:nodeNamespace", "camera/depth"),
                ("DepthPub.inputs:topicName", "image_raw"),
                ("DepthPub.inputs:frameId", "d455_depth_optical_frame"),
                ("DepthPub.inputs:type", "depth"),
                ("DepthPub.inputs:frameSkipCount", 3),
            ],
            keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "RenderProduct.inputs:execIn"),
                ("Context.outputs:context", "ColorPub.inputs:context"),
                ("Context.outputs:context", "DepthPub.inputs:context"),
                ("RenderProduct.outputs:execOut", "ColorPub.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "ColorPub.inputs:renderProductPath"),
                ("RenderProduct.outputs:execOut", "DepthPub.inputs:execIn"),
                ("RenderProduct.outputs:renderProductPath", "DepthPub.inputs:renderProductPath"),
            ],
        },
    )


def attach_all_sensors(stage):
    """Attach the sensor set used by the restaurant navigation demo."""
    global _lidar_render_product, _lidar_writer
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for _ in range(3):
        diagnostic.simulation_app.update()
    base_path = find_base_path(stage)

    # Reproduce the complete fixed camera assembly from the original serving
    # robot: tall mast, forward boom, and the vendor D455 at its calibrated
    # camera-to-base transform.
    d455_mount_path = f"{base_path}/fixed_table_depth_camera"
    UsdGeom.Xform.Define(stage, d455_mount_path)

    mast = UsdGeom.Cylinder.Define(stage, f"{d455_mount_path}/mast")
    mast.CreateRadiusAttr(0.018)
    mast.CreateHeightAttr(0.935)
    mast.CreateAxisAttr(UsdGeom.Tokens.z)
    mast.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.285, 1.3225))
    mast.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])

    boom = UsdGeom.Cylinder.Define(stage, f"{d455_mount_path}/boom")
    boom.CreateRadiusAttr(0.018)
    boom.CreateHeightAttr(0.215)
    boom.CreateAxisAttr(UsdGeom.Tokens.z)
    boom.AddTranslateOp().Set(Gf.Vec3f(-0.25, 0.3925, 1.79))
    boom.AddRotateXOp().Set(90.0)
    boom.CreateDisplayColorAttr([Gf.Vec3f(0.12, 0.15, 0.18)])

    camera_position = Gf.Vec3d(-0.25, 0.50, 1.85)
    table_target = Gf.Vec3d(1.00, 0.15, 0.74)
    desired_camera_to_base = Gf.Matrix4d().SetLookAt(
        camera_position, table_target, Gf.Vec3d(0.0, 0.0, 1.0)
    ).GetInverse()
    d455_source_stage = Usd.Stage.Open(str(D455_USD))
    source_color_camera = d455_source_stage.GetPrimAtPath(
        "/Root/RSD455/Camera_OmniVision_OV9782_Color"
    )
    source_rsd = d455_source_stage.GetPrimAtPath("/Root/RSD455")
    if not source_color_camera.IsValid() or not source_rsd.IsValid():
        raise RuntimeError("D455 source camera hierarchy is missing")
    camera_to_sensor = UsdGeom.Xformable(
        source_color_camera
    ).GetLocalTransformation()
    rsd_to_sensor = UsdGeom.Xformable(source_rsd).GetLocalTransformation()
    camera_to_outer_mount = camera_to_sensor * rsd_to_sensor
    sensor_to_base = camera_to_outer_mount.GetInverse() * desired_camera_to_base

    sensor_mount_path = f"{d455_mount_path}/realsense_d455"
    sensor_mount = UsdGeom.Xform.Define(stage, sensor_mount_path)
    sensor_mount.MakeMatrixXform().Set(sensor_to_base)
    d455_prim = stage.OverridePrim(f"{sensor_mount_path}/RSD455")
    d455_prim.GetReferences().SetReferences(
        [Sdf.Reference(str(D455_USD), Sdf.Path("/Root/RSD455"))]
    )
    # The vendor sensor asset contains standalone rigid bodies.  When mounted
    # below the mobile-base rigid body they must be visual/sensor-only, or
    # PhysX creates a nested rigid-body hierarchy that can reintroduce jitter.
    for _ in range(2):
        diagnostic.simulation_app.update()
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(d455_mount_path):
            continue
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            prim.RemoveAPI(UsdPhysics.CollisionAPI)
        if prim.HasAPI(UsdPhysics.MassAPI):
            prim.RemoveAPI(UsdPhysics.MassAPI)

    color_camera_path = (
        f"{sensor_mount_path}/RSD455/Camera_OmniVision_OV9782_Color"
    )
    depth_camera_path = f"{sensor_mount_path}/RSD455/Camera_Pseudo_Depth"
    create_d455_ros_graph(stage, color_camera_path, depth_camera_path)

    lidar_mount_path = f"{base_path}/base_scan"
    lidar_mount = UsdGeom.Xform.Define(stage, lidar_mount_path)
    lidar_mount.AddTranslateOp().Set(Gf.Vec3d(0.40, 0.0, 0.33))
    status, lidar = omni.kit.commands.execute(
        "IsaacSensorCreateRtxLidar",
        path="/RPLIDAR_S2E",
        parent=lidar_mount_path,
        config="RPLIDAR_S2E",
        translation=Gf.Vec3d(0.0, 0.0, 0.0),
        orientation=Gf.Quatd(1.0, 0.0, 0.0, 0.0),
        visibility=True,
    )
    if not status or lidar is None:
        raise RuntimeError("failed to attach front RPLIDAR S2E")
    try:
        import omni.replicator.core as rep

        _lidar_render_product = rep.create.render_product(
            lidar.GetPath(), [1, 1], name="TwoWheelServingRPLidar"
        )
        _lidar_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
        _lidar_writer.initialize(topicName="/scan", frameId="base_scan")
        _lidar_writer.attach([_lidar_render_product])
        lidar_status = "ROS2 /scan connected"
    except Exception as exc:
        lidar_status = f"mounted; ROS2 writer unavailable: {exc}"
    print(
        f"[serving-sensors] D455 stand={d455_mount_path} "
        "RGB=/camera/color/image_raw depth=/camera/depth/image_raw; "
        f"RPLIDAR={lidar_mount_path} ({lidar_status})",
        flush=True,
    )
    if os.environ.get("NAV_ROBOT_EXPORT_SENSOR_USD", "0") == "1":
        export_sensorized_robot_usd(stage)


def export_sensorized_robot_usd(stage):
    """Persist camera, lidar and ROS graph in the importer's sensor layer."""
    target_path = Sdf.Path("/two_wheel_ridgeback_serving_robot")
    output_layer = Sdf.Layer.FindOrOpen(str(ROBOT_SENSOR_LAYER))
    if output_layer is None:
        raise RuntimeError(f"robot sensor layer is missing: {ROBOT_SENSOR_LAYER}")
    output_layer.Clear()
    root_spec = Sdf.CreatePrimInLayer(output_layer, target_path)
    root_spec.specifier = Sdf.SpecifierDef
    root_spec.typeName = "Xform"
    output_layer.defaultPrim = target_path.name
    source_layer = stage.GetRootLayer()
    source_base = Sdf.Path(
        f"{diagnostic.ROBOT_ROOT}/Robot/ridgeback_base_link/ridgeback_base_link"
    )
    # The main asset references this layer at its outer ridgeback_base_link;
    # one relative link level here becomes the inner physical base link.
    target_base = target_path.AppendChild("ridgeback_base_link")
    Sdf.CreatePrimInLayer(output_layer, target_base)
    for child in ("fixed_table_depth_camera", "base_scan"):
        Sdf.CopySpec(
            source_layer,
            source_base.AppendChild(child),
            output_layer,
            target_base.AppendChild(child),
        )
    output_layer.Save()
    print(
        f"[serving-sensors] embedded sensors in USD layer={ROBOT_SENSOR_LAYER}",
        flush=True,
    )


def connect_embedded_sensors(stage):
    """Connect ROS render/writer nodes to sensors already stored in the USD."""
    global _lidar_render_product, _lidar_writer
    manager = omni.kit.app.get_app().get_extension_manager()
    manager.set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
    for _ in range(3):
        diagnostic.simulation_app.update()
    base_path = find_base_path(stage)
    sensor_mount_path = f"{base_path}/fixed_table_depth_camera/realsense_d455"
    color_camera_path = (
        f"{sensor_mount_path}/RSD455/Camera_OmniVision_OV9782_Color"
    )
    depth_camera_path = f"{sensor_mount_path}/RSD455/Camera_Pseudo_Depth"
    lidar_path = f"{base_path}/base_scan/RPLIDAR_S2E"
    for required in (color_camera_path, depth_camera_path, lidar_path):
        if not stage.GetPrimAtPath(required).IsValid():
            raise RuntimeError(f"embedded sensor prim is missing: {required}")
    create_d455_ros_graph(stage, color_camera_path, depth_camera_path)
    import omni.replicator.core as rep

    _lidar_render_product = rep.create.render_product(
        lidar_path, [1, 1], name="TwoWheelServingRPLidar"
    )
    _lidar_writer = rep.writers.get("RtxLidarROS2PublishLaserScan")
    _lidar_writer.initialize(topicName="/scan", frameId="base_scan")
    _lidar_writer.attach([_lidar_render_product])
    print(
        f"[serving-sensors] connected embedded D455={sensor_mount_path}; "
        f"RPLIDAR={lidar_path} (ROS2 /scan connected)",
        flush=True,
    )


def initialize_sensors(stage):
    if os.environ.get("NAV_ROBOT_EXPORT_SENSOR_USD", "0") == "1":
        attach_all_sensors(stage)
    else:
        connect_embedded_sensors(stage)


def configure_all_drives(stage):
    _configure_wheel_drives(stage)
    tray_names = {"upper_tray_left_slide_joint", "upper_tray_right_slide_joint"}
    arm_names = {f"joint_{index}" for index in range(1, 7)}
    found_trays = set()
    found_arm = set()
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(diagnostic.ROBOT_ROOT):
            continue
        if prim.GetName() in tray_names:
            drive = UsdPhysics.DriveAPI.Apply(prim, "linear")
            drive.CreateStiffnessAttr(4000.0)
            drive.CreateDampingAttr(500.0)
            drive.CreateMaxForceAttr(500.0)
            drive.CreateTargetPositionAttr(0.0)
            found_trays.add(prim.GetName())
        elif prim.GetName() in arm_names:
            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(200000.0)
            drive.CreateDampingAttr(20000.0)
            drive.CreateMaxForceAttr(10000.0)
            found_arm.add(prim.GetName())
    if found_arm != arm_names:
        raise RuntimeError(
            f"payload arm joints missing: {arm_names-found_arm}"
        )
    if found_trays != tray_names:
        print(
            "[serving-robot] importer fixed the two tray slides at their URDF zero pose",
            flush=True,
        )


def configure_serving_physics(stage):
    articulation_path = _configure_physics(stage)
    material = UsdShade.Material.Define(stage, "/World/PhysicsMaterials/ServingCaster")
    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api.CreateStaticFrictionAttr(0.03)
    api.CreateDynamicFrictionAttr(0.03)
    api.CreateRestitutionAttr(0.0)
    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr("min")
    caster_links = {"front_caster_link", "rear_caster_link"}
    found = []
    for prim in stage.Traverse():
        if (
            str(prim.GetPath()).startswith(diagnostic.ROBOT_ROOT)
            and prim.GetName() == "collisions"
            and prim.GetParent().GetName() in caster_links
        ):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, UsdShade.Tokens.weakerThanDescendants, "physics"
            )
            found.append(str(prim.GetPath()))
    if len(found) != 2:
        raise RuntimeError(f"expected two caster colliders, got {found}")
    return articulation_path


diagnostic.Route = ServingRoute
diagnostic.configure_wheel_drives = configure_all_drives
diagnostic.configure_physics = configure_serving_physics
diagnostic.POST_INITIALIZE_HOOK = initialize_sensors


if __name__ == "__main__":
    diagnostic.main()
