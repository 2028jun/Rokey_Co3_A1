"""Presentation showcase for the serving robot's five main components.

This is a static Isaac Sim scene.  It uses the same composed robot USD as the
restaurant simulation, but shows only the selected visual subtrees in each
display slot:

1. custom differential-drive mobile base
2. split sliding serving tray
3. Doosan M0609 manipulator
4. RPLIDAR S2E
5. Intel RealSense D455

Run without sourcing ROS in the same terminal::

    export ISAAC_SIM_ROOT="$HOME/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release"
    "$ISAAC_SIM_ROOT/python.sh" isaacpjt/robot_components_showcase.py

For a short non-interactive validation, set ``ROBOT_SHOWCASE_HEADLESS=1`` and
``ROBOT_SHOWCASE_EXIT_AFTER_FRAMES`` to a positive number.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from isaacsim import SimulationApp


HEADLESS = os.environ.get("ROBOT_SHOWCASE_HEADLESS", "0") == "1"
simulation_app = SimulationApp(
    {
        "headless": HEADLESS,
        "width": 1600,
        "height": 900,
    }
)

import omni.kit.app
import omni.usd
from isaacsim.core.utils.viewports import set_camera_view
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux


WORKSPACE = Path(
    os.environ.get("PROJECT_WS", Path(__file__).resolve().parents[1])
).resolve()
ROBOT_USD = WORKSPACE / "assets/diagnostics/two_wheel_serving_robot_v2.usd"
ROBOT_ASSET_ROOT = Sdf.Path("/two_wheel_ridgeback_serving_robot")
M0609_VISUAL_USD = (
    WORKSPACE / "isaacpjt/M0609/Collected_m0609_camera2/m0609_gripper.usd"
)
M0609_VISUAL_ROOT = Sdf.Path("/World/m0609")

SHOWCASE_ROOT = Sdf.Path("/World/RobotComponentsShowcase")
ROBOT_LINK_ROOT = "ridgeback_base_link"


@dataclass(frozen=True)
class ComponentDisplay:
    key: str
    title: str
    subtitle: str
    position: tuple[float, float, float]
    scale: float
    visible_prefixes: tuple[str, ...]


COMPONENTS = (
    ComponentDisplay(
        key="CustomMobileBase",
        title="1  CUSTOM MOBILE BASE",
        subtitle="Differential drive / in-house design",
        position=(-4.8, 0.0, 0.42),
        scale=1.25,
        visible_prefixes=(
            f"{ROBOT_LINK_ROOT}/ridgeback_base_link/visuals",
            f"{ROBOT_LINK_ROOT}/front_caster_link/visuals",
            f"{ROBOT_LINK_ROOT}/left_wheel_link/visuals",
            f"{ROBOT_LINK_ROOT}/rear_caster_link/visuals",
            f"{ROBOT_LINK_ROOT}/right_wheel_link/visuals",
        ),
    ),
    ComponentDisplay(
        key="SlidingTray",
        title="2  SLIDING TRAY",
        subtitle="Two prismatic joints / 0.25 m stroke",
        position=(-2.4, 0.0, 0.18),
        scale=1.18,
        visible_prefixes=(
            f"{ROBOT_LINK_ROOT}/serving_shelf_link/visuals",
            f"{ROBOT_LINK_ROOT}/upper_tray_left_link/visuals",
            f"{ROBOT_LINK_ROOT}/upper_tray_right_link/visuals",
        ),
    ),
    ComponentDisplay(
        key="M0609",
        title="3  DOOSAN M0609",
        subtitle="6-axis serving manipulator",
        position=(0.0, 0.0, 0.18),
        scale=0.92,
        visible_prefixes=tuple(
            f"{ROBOT_LINK_ROOT}/{link_name}/visuals"
            for link_name in (
                "base_link",
                "base",
                "link_1",
                "link_2",
                "link_3",
                "link_4",
                "link_5",
                "link_6",
                "tool0",
            )
        ),
    ),
    ComponentDisplay(
        key="Lidar",
        title="4  2D LiDAR",
        subtitle="RPLIDAR S2E / 3.0x display scale",
        position=(2.4, 0.0, 0.28),
        scale=3.0,
        visible_prefixes=(
            f"{ROBOT_LINK_ROOT}/ridgeback_base_link/base_scan/RPLIDAR_S2E",
        ),
    ),
    ComponentDisplay(
        key="D455",
        title="5  REALSENSE D455",
        subtitle="RGB-D camera / 3.0x display scale",
        position=(4.8, 0.0, 0.40),
        scale=3.0,
        visible_prefixes=(
            f"{ROBOT_LINK_ROOT}/ridgeback_base_link/fixed_table_depth_camera/"
            "realsense_d455/RSD455",
        ),
    ),
)


def _validate_assets() -> None:
    missing = [path for path in (ROBOT_USD, M0609_VISUAL_USD) if not path.is_file()]
    if missing:
        raise FileNotFoundError(
            "Showcase asset missing: " + ", ".join(str(path) for path in missing)
        )


def _set_transform(
    prim: Usd.Prim,
    position: tuple[float, float, float],
    scale: float = 1.0,
) -> None:
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    xform.AddTranslateOp().Set(Gf.Vec3d(*position))
    xform.AddScaleOp().Set(Gf.Vec3f(scale, scale, scale))


def _add_reference(stage: Usd.Stage, component: ComponentDisplay) -> Usd.Prim:
    component_path = SHOWCASE_ROOT.AppendChild(component.key)
    component_prim = UsdGeom.Xform.Define(stage, component_path).GetPrim()
    component_prim.GetReferences().SetReferences(
        [Sdf.Reference(str(ROBOT_USD), ROBOT_ASSET_ROOT)]
    )
    _set_transform(component_prim, component.position, component.scale)
    return component_prim


def _relative_path(root: Usd.Prim, prim: Usd.Prim) -> str:
    root_text = str(root.GetPath()).rstrip("/")
    return str(prim.GetPath())[len(root_text) + 1 :]


def _keep_component_geometry(
    component_prim: Usd.Prim,
    visible_prefixes: tuple[str, ...],
) -> int:
    """Hide every renderable prim except geometry under selected subtrees."""
    selected_count = 0
    stage = component_prim.GetStage()
    for prefix in visible_prefixes:
        selected = stage.GetPrimAtPath(component_prim.GetPath().AppendPath(prefix))
        if not selected.IsValid():
            raise RuntimeError(
                f"Missing component subtree: {component_prim.GetPath()}/{prefix}"
            )
        selected_count += 1

    for prim in Usd.PrimRange(component_prim):
        if prim == component_prim or not prim.IsA(UsdGeom.Imageable):
            continue
        relative = _relative_path(component_prim, prim)
        keep = any(
            relative == prefix or relative.startswith(prefix + "/")
            or prefix.startswith(relative + "/")
            for prefix in visible_prefixes
        )
        imageable = UsdGeom.Imageable(prim)
        imageable.MakeVisible() if keep else imageable.MakeInvisible()
    return selected_count


def _center_on_pedestal(
    stage: Usd.Stage,
    component_prim: Usd.Prim,
    component: ComponentDisplay,
) -> None:
    """Center composed geometry and place its lowest point on the pedestal."""
    bounds = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=False,
    ).ComputeWorldBound(component_prim).ComputeAlignedRange()
    minimum = bounds.GetMin()
    maximum = bounds.GetMax()
    if bounds.IsEmpty():
        raise RuntimeError(f"Empty world bound for {component.key}")

    center_x = 0.5 * (float(minimum[0]) + float(maximum[0]))
    center_y = 0.5 * (float(minimum[1]) + float(maximum[1]))
    pedestal_top = 0.16
    correction = Gf.Vec3d(
        component.position[0] - center_x,
        component.position[1] - center_y,
        pedestal_top - float(minimum[2]),
    )
    xform = UsdGeom.Xformable(component_prim)
    translate_op = next(
        op
        for op in xform.GetOrderedXformOps()
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate
    )
    translate_op.Set(translate_op.Get() + correction)


def _attach_m0609_visuals(stage: Usd.Stage, component_prim: Usd.Prim) -> None:
    """Use the same detailed M0609 meshes attached by nav_restaurant_demo."""
    link_root = component_prim.GetPath().AppendChild(ROBOT_LINK_ROOT)
    for link_name in ("base_link", *(f"link_{index}" for index in range(1, 7))):
        visual_path = link_root.AppendChild(link_name).AppendChild("visuals")
        visual_prim = stage.OverridePrim(visual_path)
        if visual_prim.IsInstanceable():
            visual_prim.SetInstanceable(False)
        visual_prim.GetReferences().ClearReferences()
        visual_prim.GetReferences().SetReferences(
            [
                Sdf.Reference(
                    str(M0609_VISUAL_USD),
                    M0609_VISUAL_ROOT.AppendChild(link_name).AppendChild("visuals"),
                )
            ]
        )


def _create_pedestal(
    stage: Usd.Stage,
    component: ComponentDisplay,
    index: int,
) -> None:
    pedestal_path = SHOWCASE_ROOT.AppendChild("Pedestals").AppendChild(
        f"Pedestal{index}"
    )
    pedestal = UsdGeom.Cylinder.Define(stage, pedestal_path)
    pedestal.CreateRadiusAttr(0.86)
    pedestal.CreateHeightAttr(0.16)
    pedestal.CreateAxisAttr(UsdGeom.Tokens.z)
    pedestal.CreateDisplayColorAttr([Gf.Vec3f(0.10, 0.13, 0.17)])
    pedestal.CreateDisplayOpacityAttr([1.0])
    pedestal.AddTranslateOp().Set(
        Gf.Vec3d(component.position[0], component.position[1], 0.08)
    )

    accent = UsdGeom.Cylinder.Define(
        stage, pedestal_path.AppendChild("AccentRing")
    )
    accent.CreateRadiusAttr(0.88)
    accent.CreateHeightAttr(0.025)
    accent.CreateAxisAttr(UsdGeom.Tokens.z)
    accent.CreateDisplayColorAttr([Gf.Vec3f(0.05, 0.72, 0.82)])
    accent.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.065))


def _create_floor_and_lights(stage: Usd.Stage) -> None:
    floor = UsdGeom.Cube.Define(stage, "/World/Floor")
    floor.CreateSizeAttr(1.0)
    floor.CreateDisplayColorAttr([Gf.Vec3f(0.035, 0.045, 0.060)])
    floor.AddScaleOp().Set(Gf.Vec3f(13.0, 7.0, 0.08))
    floor.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, -0.08))

    backdrop = UsdGeom.Cube.Define(stage, "/World/Backdrop")
    backdrop.CreateSizeAttr(1.0)
    backdrop.CreateDisplayColorAttr([Gf.Vec3f(0.025, 0.035, 0.055)])
    backdrop.AddScaleOp().Set(Gf.Vec3f(13.0, 0.08, 5.0))
    backdrop.AddTranslateOp().Set(Gf.Vec3d(0.0, 2.4, 2.5))

    key = UsdLux.RectLight.Define(stage, "/World/Lights/Key")
    key.CreateIntensityAttr(4200.0)
    key.CreateExposureAttr(0.0)
    key.CreateColorAttr(Gf.Vec3f(0.82, 0.90, 1.0))
    key.CreateWidthAttr(8.0)
    key.CreateHeightAttr(4.0)
    key.AddTranslateOp().Set(Gf.Vec3d(-2.0, -3.5, 6.5))
    key.AddRotateXYZOp().Set(Gf.Vec3f(35.0, 0.0, 0.0))

    fill = UsdLux.DomeLight.Define(stage, "/World/Lights/Fill")
    fill.CreateIntensityAttr(700.0)
    fill.CreateColorAttr(Gf.Vec3f(0.30, 0.38, 0.52))

    rim = UsdLux.DistantLight.Define(stage, "/World/Lights/Rim")
    rim.CreateIntensityAttr(1200.0)
    rim.CreateAngleAttr(1.5)
    rim.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 20.0, 150.0))


def _create_overlay():
    if HEADLESS:
        return None
    import omni.ui as ui

    window = ui.Window(
        "Serving Robot Components",
        width=1500,
        height=170,
        flags=(
            ui.WINDOW_FLAGS_NO_SCROLLBAR
            | ui.WINDOW_FLAGS_NO_RESIZE
            | ui.WINDOW_FLAGS_NO_COLLAPSE
        ),
    )
    window.position_x = 45
    window.position_y = 45
    with window.frame:
        with ui.VStack(spacing=8, style={"background_color": 0xD9182230}):
            ui.Label(
                "SERVING ROBOT  |  COMPONENT BREAKDOWN",
                height=38,
                alignment=ui.Alignment.CENTER,
                style={"font_size": 26, "color": 0xFFFFFFFF},
            )
            with ui.HStack(spacing=12):
                for component in COMPONENTS:
                    with ui.VStack(width=0):
                        ui.Label(
                            component.title,
                            alignment=ui.Alignment.CENTER,
                            style={"font_size": 18, "color": 0xFF43D8EE},
                        )
                        ui.Label(
                            component.subtitle,
                            alignment=ui.Alignment.CENTER,
                            style={"font_size": 13, "color": 0xFFD8DEE9},
                        )
    return window


def build_showcase() -> tuple[Usd.Stage, object | None]:
    _validate_assets()
    omni.usd.get_context().new_stage()
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.Xform.Define(stage, "/World")
    UsdGeom.Xform.Define(stage, SHOWCASE_ROOT)
    UsdGeom.Xform.Define(stage, SHOWCASE_ROOT.AppendChild("Pedestals"))

    _create_floor_and_lights(stage)
    for index, component in enumerate(COMPONENTS, start=1):
        component_prim = _add_reference(stage, component)
        if component.key == "M0609":
            _attach_m0609_visuals(stage, component_prim)
        visible_count = _keep_component_geometry(
            component_prim, component.visible_prefixes
        )
        if visible_count == 0:
            raise RuntimeError(f"No visible geometry selected for {component.key}")
        _center_on_pedestal(stage, component_prim, component)
        _create_pedestal(stage, component, index)
        print(
            f"[showcase] {component.key}: selected subtrees={visible_count}",
            flush=True,
        )

    for _ in range(8):
        simulation_app.update()
    set_camera_view(
        eye=[0.0, -12.8, 5.2],
        target=[0.0, 0.15, 1.15],
        camera_prim_path="/OmniverseKit_Persp",
    )
    overlay = _create_overlay()

    save_path = os.environ.get("ROBOT_SHOWCASE_SAVE_STAGE", "").strip()
    if save_path:
        resolved = Path(save_path).expanduser().resolve()
        resolved.parent.mkdir(parents=True, exist_ok=True)
        stage.GetRootLayer().Export(str(resolved))
        print(f"[showcase] stage exported to {resolved}", flush=True)

    return stage, overlay


def main() -> None:
    _stage, overlay = build_showcase()
    exit_after = int(os.environ.get("ROBOT_SHOWCASE_EXIT_AFTER_FRAMES", "0"))
    rendered_frames = 0
    print(
        "[showcase] ready - close the Isaac Sim window to exit",
        flush=True,
    )
    while simulation_app.is_running():
        simulation_app.update()
        rendered_frames += 1
        if exit_after > 0 and rendered_frames >= exit_after:
            break
    # Keep the UI object alive until the application loop finishes.
    del overlay
    simulation_app.close()


if __name__ == "__main__":
    main()
