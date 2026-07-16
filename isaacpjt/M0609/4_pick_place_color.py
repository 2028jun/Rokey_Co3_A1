
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

from pathlib import Path
import sys
import time

import numpy as np
import omni.usd
import rclpy
from pxr import Usd, UsdGeom, UsdPhysics
from rclpy.node import Node
from std_msgs.msg import Int32

from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.api.materials.physics_material import PhysicsMaterial
from isaacsim.core.prims import SingleGeometryPrim
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator

_THIS_DIR = Path(__file__).resolve().parent

# rmpflow 인프라 폴더 경로 등록 (인프라 파일 내부 import가 그대로 동작)
RMPFLOW_DIR = str(_THIS_DIR / "rmpflow")
if RMPFLOW_DIR not in sys.path:
    sys.path.insert(0, RMPFLOW_DIR)
 
from m0609_pick_place_controller import PickPlaceController

# ╔══════════════════════════════════════════════════════════════╗
# ║  A. Task 파라미터 (이전 장과 동일)                              ║
# ╚══════════════════════════════════════════════════════════════╝
USD_PATH        = str(_THIS_DIR / "Collected_m0609_camera2/m0609_gripper.usd")
ROBOT_PRIM_PATH = "/World/m0609"
EE_LINK_NAME    = "link_6"
GRIPPER_JOINTS  = ["finger_joint", "right_inner_knuckle_joint"]

DRIVE_STIFFNESS = 1e8
DRIVE_DAMPING   = 1e4
DRIVE_MAX_FORCE = 1e8

GRIPPER_OPEN    = [0.0, 0.0]
GRIPPER_CLOSE   = [0.5, 0.5]
GRIPPER_DELTA   = [-0.5, -0.5]

FINGER_STATIC   = 1.8
FINGER_DYNAMIC  = 1.4
CUBE_STATIC     = 1.2
CUBE_DYNAMIC    = 1.0


# ╔══════════════════════════════════════════════════════════════╗
# ║  B. Controller 파라미터 (★ 이번 장에서 새로 추가)               ║
# ╚══════════════════════════════════════════════════════════════╝

# ── B-1. 인프라 파일 경로 (RMPFlow가 참조) ────────────────────
M0609_URDF_PATH           = str(_THIS_DIR / "doosan-robot2/urdf/m0609_isaac_sim.urdf")
M0609_DESCRIPTION_PATH    = str(_THIS_DIR / "rmpflow/m0609_description.yaml")
M0609_RMPFLOW_CONFIG_PATH = str(_THIS_DIR / "rmpflow/m0609_rmpflow_common.yaml")

# ── B-2. Pick & Place 동작 파라미터 ───────────────────────────
PICK_POS = np.array([0.30, 0.40, 0.02575])

BLUE_WAIT_POS = np.array([0.10, 0.60, 0.40])
GREEN_WAIT_POS = np.array([0.10, 0.75, 0.40])

BLUE_GOAL_POS = np.array([0.55, -0.35, 0.01])
GREEN_GOAL_POS = np.array([0.55, 0.35, 0.01])

BLUE = 1
GREEN = 2

COLOR_TOPIC = "/cube_color"

EE_OFFSET     = np.array([0.0, 0.0, 0.2])               # 접근 높이

# ── B-3. 10단계 타이밍 (작을수록 해당 단계가 오래 지속됨) ─────
EVENTS_DT = [
    0.008,   # 0. 접근 이동
    0.005,   # 1. 하강
    0.02,    # 2. 그리퍼 닫기 대기
    0.1,     # 3. 그리퍼 닫힘 유지
    0.0025,  # 4. 들어올리기
    0.01,    # 5. Place 위치로 이동
    0.0025,  # 6. 하강
    1,   # 7. 그리퍼 열기 대기
    0.0025,   # 8. 상승
    0.08,    # 9. 복귀
]


# ============================================================
# 유틸 (이전 장과 동일)
# ============================================================
def find_prim_path_by_name(root_path: str, name: str):
    stage = omni.usd.get_context().get_stage()
    root_prim = stage.GetPrimAtPath(root_path)
    if not root_prim.IsValid():
        return None
    for prim in Usd.PrimRange(root_prim):
        if prim.GetName() == name:
            return str(prim.GetPath())
    return None


def initialize_robot(robot, world):
    robot.initialize()
    robot.gripper.initialize(
        physics_sim_view=world.physics_sim_view,
        articulation_apply_action_func=robot.apply_action,
        get_joint_positions_func=robot.get_joint_positions,
        set_joint_positions_func=robot.set_joint_positions,
        dof_names=robot.dof_names,
    )
    robot.set_joint_positions(np.zeros(robot.num_dof))


class ColorResultSubscriber(Node):
    """PC B가 판별한 큐브 색상(1=파랑, 2=초록)을 수신한다."""

    def __init__(self):
        super().__init__("isaac_color_result_subscriber")
        self.detected_color = None
        self.create_subscription(Int32, COLOR_TOPIC, self._color_callback, 10)

    def _color_callback(self, msg):
        if msg.data not in (BLUE, GREEN):
            self.get_logger().warning(f"지원하지 않는 색상 값: {msg.data}")
            return
        self.detected_color = int(msg.data)

    def clear(self):
        """새 라운드가 시작될 때 이전 판별 결과를 폐기한다."""
        self.detected_color = None


# ============================================================
# Task — 이전 장에서 완성한 M0609Task (변경 없음)
# ============================================================
class M0609Task(BaseTask):

    def __init__(self, name):
        super().__init__(name=name, offset=None)
        self._task_achieved = False
        self._selected_color = None
        self._selected_cube = None
        self._selected_goal = None

    def set_up_scene(self, scene):
        super().set_up_scene(scene)
        self._load_usd()
        self._discover_links()
        self._setup_physics()
        self._register_robot(scene)
        self._create_scene(scene)
        print("\n  [완료] 씬 구성 성공!\n")

    def _load_usd(self):
        print("\n" + "=" * 60)
        print("[1.LOAD] USD 로드")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()
        world_prim = stage.GetPrimAtPath("/World")
        if not world_prim.IsValid():
            world_prim = UsdGeom.Xform.Define(stage, "/World").GetPrim()
        world_prim.GetReferences().AddReference(USD_PATH)
        for _ in range(15):
            simulation_app.update()
        print(f"  [OK] {USD_PATH}")

    def _discover_links(self):
        print("\n" + "=" * 60)
        print("[2.DISCOVER] 링크 경로 탐색")
        print("=" * 60)
        self._ee_path = find_prim_path_by_name(ROBOT_PRIM_PATH, EE_LINK_NAME)
        if self._ee_path is None:
            raise RuntimeError(f"'{EE_LINK_NAME}' not found")
        print(f"  EE ({EE_LINK_NAME}) = {self._ee_path}")
        for jn in GRIPPER_JOINTS:
            print(f"  {jn:<35} = {find_prim_path_by_name(ROBOT_PRIM_PATH, jn)}")


    def _setup_physics(self):
        print("\n" + "=" * 60)
        print("[3.PHYSICS] 물리 설정")
        print("=" * 60)
        stage = omni.usd.get_context().get_stage()

        drive_count = 0
        for prim in Usd.PrimRange(stage.GetPrimAtPath(ROBOT_PRIM_PATH)):
            for dt in ["angular", "linear"]:
                drive = UsdPhysics.DriveAPI.Get(prim, dt)
                if drive:
                    drive.GetStiffnessAttr().Set(DRIVE_STIFFNESS)
                    drive.GetDampingAttr().Set(DRIVE_DAMPING)
                    drive.GetMaxForceAttr().Set(DRIVE_MAX_FORCE)
                    drive_count += 1
        print(f"  [OK] drive updated: {drive_count}")

    def _register_robot(self, scene):
        print("\n" + "=" * 60)
        print("[4.REGISTER] 로봇 등록")
        print("=" * 60)
        gripper = ParallelGripper(
            end_effector_prim_path=self._ee_path,
            joint_prim_names=GRIPPER_JOINTS,
            joint_opened_positions=np.array(GRIPPER_OPEN),
            joint_closed_positions=np.array(GRIPPER_CLOSE),
            action_deltas=np.array(GRIPPER_DELTA),
        )
        self._robot = scene.add(
            SingleManipulator(
                prim_path=ROBOT_PRIM_PATH,
                name="m0609_robot",
                end_effector_prim_path=self._ee_path,
                gripper=gripper,
            )
        )
        print(f"  [OK] SingleManipulator: {ROBOT_PRIM_PATH}")

    def _create_scene(self, scene):
        print("\n" + "=" * 60)
        print("[5.SCENE] 작업 환경 구성")
        print("=" * 60)
        cube_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/cube_material",
            static_friction=CUBE_STATIC,
            dynamic_friction=CUBE_DYNAMIC,
            restitution=0.0,
        )
        # 두 큐브는 공중 대기 위치에서 생성한다. Reset할 때 하나만
        # 무작위로 PICK_POS로 이동하고 나머지는 공중에 고정한다.
        self._blue_cube = scene.add(
            DynamicCuboid(
                prim_path="/World/blue_cube",
                name="blue_cube",
                position=BLUE_WAIT_POS,
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 0.0, 1.0]),
                mass=0.05,
                physics_material=cube_material,
            )
        )

        self._green_cube = scene.add(
            DynamicCuboid(
                prim_path="/World/green_cube",
                name="green_cube",
                position=GREEN_WAIT_POS,
                scale=np.array([0.05, 0.05, 0.05]),
                color=np.array([0.0, 1.0, 0.0]),
                mass=0.05,
                physics_material=cube_material,
            )
        )
        print(f"  [OK] blue cube wait  @ {BLUE_WAIT_POS}")
        print(f"  [OK] green cube wait @ {GREEN_WAIT_POS}")

        # 색상별 Place 위치를 눈으로 확인할 수 있는 마커 두 개를 만든다.
        scene.add(
            VisualCuboid(
                prim_path="/World/blue_goal_marker",
                name="blue_goal_marker",
                position=BLUE_GOAL_POS,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 0.0, 1.0]),
            )
        )

        scene.add(
            VisualCuboid(
                prim_path="/World/green_goal_marker",
                name="green_goal_marker",
                position=GREEN_GOAL_POS,
                scale=np.array([0.06, 0.06, 0.001]),
                color=np.array([0.0, 1.0, 0.0]),
            )
        )
        print(f"  [OK] blue goal  @ {BLUE_GOAL_POS}")
        print(f"  [OK] green goal @ {GREEN_GOAL_POS}")
        finger_material = PhysicsMaterial(
            prim_path="/World/Physics_Materials/finger_material",
            static_friction=FINGER_STATIC,
            dynamic_friction=FINGER_DYNAMIC,
            restitution=0.0,
        )
        for link_name in ["left_inner_finger", "right_inner_finger"]:
            link_path = find_prim_path_by_name(ROBOT_PRIM_PATH, link_name)
            if link_path:
                SingleGeometryPrim(
                    prim_path=link_path,
                    name=f"{link_name}_geom",
                ).apply_physics_material(finger_material)
                print(f"  [OK] friction: {link_path}")

    def _set_cube_kinematic(self, cube, enabled):
        """공중 대기 큐브가 중력으로 떨어지지 않도록 고정 상태를 설정한다."""
        stage = omni.usd.get_context().get_stage()
        # 이 USD 버전의 Get()은 Prim 하나가 아니라 (Stage, Prim 경로)를 받는다.
        rigid_body = UsdPhysics.RigidBodyAPI.Get(stage, cube.prim_path)
        if not rigid_body:
            raise RuntimeError(f"RigidBodyAPI를 찾을 수 없습니다: {cube.prim_path}")
        rigid_body.GetKinematicEnabledAttr().Set(enabled)

    def prepare_round(self):
        """Reset마다 큐브 하나를 무작위로 골라 Pick 영역에 배치한다."""
        # 두 큐브를 먼저 원래 공중 대기 위치로 되돌리고 고정한다.
        self._set_cube_kinematic(self._blue_cube, True)
        self._set_cube_kinematic(self._green_cube, True)
        self._blue_cube.set_world_pose(position=BLUE_WAIT_POS)
        self._green_cube.set_world_pose(position=GREEN_WAIT_POS)

        # 이번 라운드에 사용할 큐브와 동일 색상의 목표 위치를 선택한다.
        self._selected_color = int(np.random.choice([BLUE, GREEN]))
        if self._selected_color == BLUE:
            self._selected_cube = self._blue_cube
            self._selected_goal = BLUE_GOAL_POS
            color_name = "BLUE"
        else:
            self._selected_cube = self._green_cube
            self._selected_goal = GREEN_GOAL_POS
            color_name = "GREEN"

        # 선택된 큐브만 Pick 영역으로 옮기고 물리 영향을 받도록 해제한다.
        self._selected_cube.set_world_pose(position=PICK_POS)
        self._set_cube_kinematic(self._selected_cube, False)
        self._task_achieved = False

        print(f"[ROUND] selected color = {color_name} ({self._selected_color})")
        print(f"[ROUND] pick position  = {PICK_POS}")
        print(f"[ROUND] goal position  = {self._selected_goal}")

    def get_observations(self):
        """선택된 큐브 위치와 로봇 관절 상태를 Controller에 제공한다."""
        cube_pos, _ = self._selected_cube.get_world_pose()
        return {
            self._robot.name: {
                "joint_positions": self._robot.get_joint_positions(),
            },
            "selected_cube": {
                "position": cube_pos,
                "color": self._selected_color,
                "goal_position": self._selected_goal,
            },
        }

    def pre_step(self, control_index, simulation_time):
        """큐브 색상은 유지하고 해당 색상 목표에 도착했는지만 검사한다."""
        cube_pos, _ = self._selected_cube.get_world_pose()
        if not self._task_achieved and np.mean(np.abs(self._selected_goal - cube_pos)) < 0.02:
            self._task_achieved = True

    def post_reset(self):
        """World Reset 후 그리퍼를 열고 새 랜덤 라운드를 준비한다."""
        self._robot.gripper.set_joint_positions(
            self._robot.gripper.joint_opened_positions
        )
        self.prepare_round()


# ╔══════════════════════════════════════════════════════════════╗
# ║  C. 메인 — Controller 생성 및 실행 (★ 이번 장 핵심)           ║
# ╚══════════════════════════════════════════════════════════════╝

def main():
    # ── C-1. World + Task (이전 장과 동일) ────────────────────
    # rclpy.spin()은 시뮬레이션을 막으므로 아래 루프에서 spin_once()만 호출한다.
    if not rclpy.ok():
        rclpy.init()
    color_subscriber = ColorResultSubscriber()

    my_world = World(stage_units_in_meters=1.0)
    task = M0609Task(name="m0609_task")
    my_world.add_task(task)
    my_world.reset()

    robot = my_world.scene.get_object("m0609_robot")
    initialize_robot(robot, my_world)

    # 홈 포지션 안정화 대기
    for _ in range(30):
        my_world.step(render=True)

    # ── C-2. Controller 생성 (initialize 이후에만 가능) ───────
    print("\n" + "=" * 60)
    print("[C-2] PickPlaceController 생성")
    print("=" * 60)
    print(f"  URDF        = {M0609_URDF_PATH}")
    print(f"  description = {M0609_DESCRIPTION_PATH}")
    print(f"  rmpflow     = {M0609_RMPFLOW_CONFIG_PATH}")
    print(f"  events_dt   = {EVENTS_DT}")
    print(f"  EE frame    = {EE_LINK_NAME}")

    controller = PickPlaceController(
        name="m0609_pick_place_controller",
        gripper=robot.gripper,
        robot_articulation=robot,
        end_effector_initial_height=0.30,
        events_dt=EVENTS_DT,
        urdf_path=M0609_URDF_PATH,
        robot_description_path=M0609_DESCRIPTION_PATH,
        rmpflow_config_path=M0609_RMPFLOW_CONFIG_PATH,
        end_effector_frame_name=EE_LINK_NAME,
    )
    print("  [OK] Controller 생성 완료")

    # ── C-3. 초기 상태 진단 ───────────────────────────────────
    ee_pos, _ = robot.end_effector.get_world_pose()
    print(f"\n  EE 초기 위치 = {ee_pos}")
    print(f"  Pick 위치    = {PICK_POS}")
    print(f"  파란 목표    = {BLUE_GOAL_POS}")
    print(f"  초록 목표    = {GREEN_GOAL_POS}")

    # ── C-4. Controller 실행 루프 ─────────────────────────────
    print("\n[Pick & Place 시작]\n")
    was_playing = False
    task_done = False
    inspection_ready = False
    detected_color = None
    waiting_for_color_logged = False

    while simulation_app.is_running():
        my_world.step(render=True)
        # PC B의 /cube_color 콜백을 Isaac Sim 렌더 루프를 막지 않고 처리한다.
        rclpy.spin_once(color_subscriber, timeout_sec=0.0)
        time.sleep(0.01)
        is_playing = my_world.is_playing()

        # Play 시작 감지 → 리셋
        if is_playing and not was_playing:
            my_world.reset()
            initialize_robot(robot, my_world)
            controller.reset()
            task_done = False
            inspection_ready = False
            detected_color = None
            waiting_for_color_logged = False
            color_subscriber.clear()

        # 매 스텝 제어
        if is_playing and not task_done:
            # 선택된 큐브의 실제 위치와 로봇 관절 상태를 읽는다.
            obs = task.get_observations()
            cube_position = obs["selected_cube"]["position"]
            current_joints = obs["m0609_robot"]["joint_positions"]

            if not inspection_ready:
                # PickPlaceController의 event 0만 먼저 실행한다.
                # 이 단계는 그리퍼와 Wrist Camera를 큐브 위 접근 높이로 이동시킨다.
                actions = controller.forward(
                    picking_position=cube_position,
                    # event 0에서는 Place 위치를 사용하지 않으므로 임시값을 넣는다.
                    placing_position=BLUE_GOAL_POS,
                    current_joint_positions=current_joints,
                    end_effector_offset=EE_OFFSET,
                )
                robot.apply_action(actions)

                # event 1(하강)로 넘어가기 직전에 제어 호출을 멈추고 색상을 기다린다.
                if controller.get_current_event() >= 1:
                    inspection_ready = True
                    detected_color = None
                    color_subscriber.clear()
                    print("[검사 위치 도착] Wrist Camera 색상 판별을 시작합니다.")
            else:
                # 검사 위치에 도착한 이후에 받은 첫 정상 색상값만 고정해서 사용한다.
                if detected_color is None and color_subscriber.detected_color is not None:
                    detected_color = color_subscriber.detected_color
                    color_name = "BLUE" if detected_color == BLUE else "GREEN"
                    print(f"[ROS] {COLOR_TOPIC} 수신: {color_name} ({detected_color})")

                if detected_color is None:
                    if not waiting_for_color_logged:
                        print(f"[대기] PC B의 {COLOR_TOPIC} 판별 결과를 기다립니다.")
                        waiting_for_color_logged = True
                else:
                    # PC B가 보낸 색상에 따라 Place 목표를 선택한다.
                    placing_position = BLUE_GOAL_POS if detected_color == BLUE else GREEN_GOAL_POS

                    # event 1부터 Pick & Place 시퀀스를 이어서 실행한다.
                    actions = controller.forward(
                        picking_position=cube_position,
                        placing_position=placing_position,
                        current_joint_positions=current_joints,
                        end_effector_offset=EE_OFFSET,
                    )
                    robot.apply_action(actions)

                    if controller.is_done():
                        print("[완료] Pick & Place 시퀀스 종료")
                        task_done = True
                        my_world.pause()

                    event = controller.get_current_event()
                    ee_pos, _ = robot.end_effector.get_world_pose()
                    print(f"  [event={event}] cube_z={cube_position[2]:.4f}  ee_z={ee_pos[2]:.4f}")

        was_playing = is_playing

    color_subscriber.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    simulation_app.close()


if __name__ == "__main__":
    main()
