"""Actor SDG customers for typing and kitchen-exit path crossing."""

from __future__ import annotations

import os
import time
from pathlib import Path

import carb
import carb.settings
import omni.kit.app
from pxr import Gf


PERSON_NAME = "CrossingPedestrian"
ENABLED = os.environ.get("NAV_CROSSING_PEDESTRIAN", "1") == "1"
TYPING_ENABLED = os.environ.get("NAV_TYPING_CUSTOMER", "1") == "1"

# Restaurant walls are centered at x=+/-6.0.  These endpoints leave 0.60 m
# of body clearance.  y=3.0 is the open aisle directly in front of the
# kitchen exit, clear of the upper tables/chairs and rear-corner plants.
LEFT_X = float(os.environ.get("NAV_CROSSING_LEFT_X", "-5.40"))
RIGHT_X = float(os.environ.get("NAV_CROSSING_RIGHT_X", "5.40"))
LANE_Y = float(os.environ.get("NAV_CROSSING_Y", "3.00"))
FLOOR_Z = float(os.environ.get("NAV_CROSSING_Z", "0.0"))
SPAWN_YAW_DEGREES = float(
    os.environ.get("NAV_CROSSING_SPAWN_YAW", "90.0")
)
TURN_DISTANCE = float(os.environ.get("NAV_CROSSING_TURN_DISTANCE", "0.20"))
LANE_TOLERANCE = float(
    os.environ.get("NAV_CROSSING_LANE_TOLERANCE", "0.05")
)
PROGRESS_LOG_SECONDS = float(
    os.environ.get("NAV_CROSSING_PROGRESS_LOG_SECONDS", "5.0")
)

# The typing customer stays at the far chair of TableSet_00, facing the
# tabletop.  The controller holds this exact pose before, during, and after
# the animation so root motion cannot sink or rotate the character.
TYPING_PERSON_NAME = "TypingCustomer"
TYPING_X = float(os.environ.get("HAND_TEST_STAND_X", "-3.70"))
TYPING_Y = float(os.environ.get("HAND_TEST_STAND_Y", "-3.10"))
TYPING_Z = float(os.environ.get("HAND_TEST_STAND_Z", "0.0"))
TYPING_YAW_DEGREES = float(
    os.environ.get("HAND_TEST_STAND_YAW", "180.0")
)
TYPING_TRIGGER_TOPIC = os.environ.get(
    "HAND_TEST_TYPING_TOPIC", "/hand_test/type_keyboard"
)
TYPING_DURATION_SECONDS = float(
    os.environ.get("HAND_TEST_TYPING_SECONDS", "10.0")
)
TYPING_CLIP_SECONDS = float(
    os.environ.get("HAND_TEST_TYPING_CLIP_SECONDS", "3.0")
)
_ANIMATION_DIR = (
    Path(__file__).resolve().parents[1] / "assets" / "actor_animations"
)
TYPE_KEYBOARD_ANIM = os.environ.get(
    "HAND_TEST_TYPE_KEYBOARD_ANIM",
    str(_ANIMATION_DIR / "type_keyboard.skelanim.usd"),
)


def enable_extensions() -> None:
    """Enable character extensions before the restaurant stage is opened."""
    extension_manager = omni.kit.app.get_app().get_extension_manager()
    for extension in (
        "omni.anim.people",
        "isaacsim.replicator.agent.core",
    ):
        enabled = extension_manager.set_extension_enabled_immediate(
            extension, True
        )
        print(
            f"[crossing_pedestrian] enable {extension} -> {enabled}",
            flush=True,
        )


def _load_character(position, yaw_degrees: float, name: str):
    from isaacsim.replicator.agent.core.settings import AssetPaths
    from isaacsim.replicator.agent.core.stage_util import CharacterUtil

    app = omni.kit.app.get_app()
    biped_prim = CharacterUtil.load_default_biped_to_stage()
    for _ in range(10):
        app.update()

    animation_graph = CharacterUtil.get_anim_graph_from_character(biped_prim)
    if animation_graph is None or not animation_graph.IsValid():
        raise RuntimeError("default biped animation graph is unavailable")

    person_prim = CharacterUtil.load_character_usd_to_stage(
        AssetPaths.default_biped_asset_path(),
        position,
        yaw_degrees,
        name,
    )
    for _ in range(20):
        app.update()

    skelroot = CharacterUtil.get_character_skelroot_by_root(person_prim)
    if skelroot is None:
        raise RuntimeError(
            f"no SkelRoot found under {person_prim.GetPath()}"
        )
    CharacterUtil.setup_animation_graph_to_character(
        [skelroot], animation_graph
    )
    for _ in range(10):
        app.update()
    return person_prim


def _register_typing_action() -> None:
    animation_path = Path(TYPE_KEYBOARD_ANIM)
    if "://" not in TYPE_KEYBOARD_ANIM and not animation_path.is_file():
        raise FileNotFoundError(
            f"typing animation is missing: {TYPE_KEYBOARD_ANIM}"
        )

    from omni.anim.people.python_ext import (
        get_instance as get_people_instance,
    )
    from omni.anim.people.scripts.custom_command.defines import (
        CustomCommand,
        CustomCommandTemplate,
    )
    from omni.anim.people.settings import PeopleSettings

    settings = carb.settings.get_settings()
    settings.set(PeopleSettings.NAVMESH_ENABLED, False)
    settings.set(PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED, False)

    command_manager = (
        get_people_instance().get_custom_command_manager()
    )
    existing = set(command_manager.get_all_custom_command_names())
    if "type_keyboard" not in existing:
        command_manager._commands.append(
            CustomCommand(
                anim_path=TYPE_KEYBOARD_ANIM,
                name="type_keyboard",
                template=CustomCommandTemplate.TIMING,
                min_random_time=TYPING_CLIP_SECONDS,
                max_random_time=TYPING_CLIP_SECONDS,
            )
        )
    print(
        f"[typing_topic] registered type_keyboard={TYPE_KEYBOARD_ANIM}",
        flush=True,
    )


def spawn_typing_customer(stage):
    """Spawn the stationary customer controlled by the typing topic."""
    del stage
    _register_typing_action()
    person_prim = _load_character(
        Gf.Vec3d(TYPING_X, TYPING_Y, TYPING_Z),
        TYPING_YAW_DEGREES,
        TYPING_PERSON_NAME,
    )
    print(
        f"[typing_topic] spawned={person_prim.GetPath()} "
        f"home=({TYPING_X:.2f},{TYPING_Y:.2f},{TYPING_Z:.2f}) "
        f"yaw={TYPING_YAW_DEGREES:.1f}",
        flush=True,
    )
    return person_prim


def spawn(stage):
    """Spawn a character with the default Actor SDG walking graph."""
    del stage  # CharacterUtil uses omni.usd's active stage.
    person_prim = _load_character(
        Gf.Vec3d(LEFT_X, LANE_Y, FLOOR_Z),
        SPAWN_YAW_DEGREES,
        PERSON_NAME,
    )

    print(
        f"[crossing_pedestrian] spawned={person_prim.GetPath()} "
        f"path=({LEFT_X:.2f}, {LANE_Y:.2f}) <-> "
        f"({RIGHT_X:.2f}, {LANE_Y:.2f})",
        flush=True,
    )
    return person_prim


class CrossingPedestrianController:
    """Keep one character walking wall-to-wall along the world X axis."""

    def __init__(self, person_prim):
        if LEFT_X >= RIGHT_X:
            raise ValueError(
                "NAV_CROSSING_LEFT_X must be smaller than "
                "NAV_CROSSING_RIGHT_X"
            )
        if TURN_DISTANCE <= 0.0:
            raise ValueError(
                "NAV_CROSSING_TURN_DISTANCE must be greater than zero"
            )

        from pxr import Usd

        self._skelroot_path = None
        for prim in Usd.PrimRange(person_prim):
            if prim.GetTypeName() == "SkelRoot":
                self._skelroot_path = str(prim.GetPath())
                break
        if self._skelroot_path is None:
            raise RuntimeError(
                f"no SkelRoot found under {person_prim.GetPath()}"
            )

        self._character = None
        self._target_x = RIGHT_X
        self._path_points = None
        self._initialized = False
        self._turn_count = 0
        self._last_correction_log = 0.0
        self._last_progress_log = 0.0

    def _get_character(self):
        if self._character is None:
            import omni.anim.graph.core as ag

            self._character = ag.get_character(self._skelroot_path)
        return self._character

    def _set_target(self, current_position, target_x: float) -> None:
        self._target_x = target_x
        self._path_points = [
            carb.Float3(current_position[0], LANE_Y, FLOOR_Z),
            carb.Float3(target_x, LANE_Y, FLOOR_Z),
        ]
        self._turn_count += 1
        direction = "+X" if target_x == RIGHT_X else "-X"
        print(
            f"[crossing_pedestrian] walking {direction} toward "
            f"x={target_x:.2f} (turn={self._turn_count})",
            flush=True,
        )

    def update(self) -> None:
        character = self._get_character()
        if character is None:
            return

        from omni.anim.people.scripts.utils import Utils

        current_position, current_rotation = (
            Utils.get_character_transform(character)
        )
        if not self._initialized:
            current_position = carb.Float3(LEFT_X, LANE_Y, FLOOR_Z)
            character.set_world_transform(
                current_position, current_rotation
            )
            self._set_target(current_position, RIGHT_X)
            self._initialized = True

        if (
            abs(current_position[1] - LANE_Y) > LANE_TOLERANCE
            or abs(current_position[2] - FLOOR_Z) > LANE_TOLERANCE
        ):
            current_position = carb.Float3(
                min(max(current_position[0], LEFT_X), RIGHT_X),
                LANE_Y,
                FLOOR_Z,
            )
            character.set_world_transform(
                current_position, current_rotation
            )
            self._path_points[0] = current_position
            now = time.monotonic()
            if now - self._last_correction_log >= 1.0:
                print(
                    "[crossing_pedestrian] corrected lane drift to "
                    f"{tuple(round(v, 3) for v in current_position)}",
                    flush=True,
                )
                self._last_correction_log = now

        if abs(current_position[0] - self._target_x) <= TURN_DISTANCE:
            next_x = LEFT_X if self._target_x == RIGHT_X else RIGHT_X
            self._set_target(current_position, next_x)

        # Never transition through idle at an endpoint.  Retargeting while
        # Walk remains active avoids the root-pose sinking seen with a
        # command-file Walk->None loop.
        character.set_variable("Action", "Walk")
        character.set_variable("Walk", 1.0)
        character.set_variable("PathPoints", self._path_points)

        now = time.monotonic()
        if (
            PROGRESS_LOG_SECONDS > 0.0
            and now - self._last_progress_log >= PROGRESS_LOG_SECONDS
        ):
            print(
                "[crossing_pedestrian] position="
                f"({current_position[0]:.2f},"
                f"{current_position[1]:.2f},"
                f"{current_position[2]:.2f}) "
                f"target_x={self._target_x:.2f}",
                flush=True,
            )
            self._last_progress_log = now

    def shutdown(self) -> None:
        character = self._get_character()
        if character is not None:
            character.set_variable("Walk", 0.0)
            character.set_variable("Action", "None")


class TypingTopicController:
    """Run one fixed-pose typing action for each ROS Empty trigger."""

    def __init__(self, person_prim):
        if TYPING_DURATION_SECONDS <= 0.0:
            raise ValueError(
                "HAND_TEST_TYPING_SECONDS must be greater than zero"
            )

        import rclpy
        from std_msgs.msg import Empty

        self._rclpy = rclpy
        self._owns_rclpy_context = not rclpy.ok()
        if self._owns_rclpy_context:
            rclpy.init(args=[])

        self._node = rclpy.create_node("hand_test_typing_controller")
        self._subscription = self._node.create_subscription(
            Empty, TYPING_TRIGGER_TOPIC, self._on_trigger, 10
        )
        self._pending = False
        self._active = False
        self._end_time = 0.0
        self._anchor_position = None
        self._anchor_rotation = None
        self._character = None
        self._home_ready = False
        self._shutdown = False

        from pxr import Usd

        self._skelroot_path = None
        for prim in Usd.PrimRange(person_prim):
            if prim.GetTypeName() == "SkelRoot":
                self._skelroot_path = str(prim.GetPath())
                break
        if self._skelroot_path is None:
            raise RuntimeError(
                f"no SkelRoot found under {person_prim.GetPath()}"
            )

        print(
            f"[typing_topic] waiting on {TYPING_TRIGGER_TOPIC} "
            f"(std_msgs/msg/Empty, duration={TYPING_DURATION_SECONDS:.1f}s)",
            flush=True,
        )

    def _on_trigger(self, _message) -> None:
        if self._active or self._pending:
            print(
                "[typing_topic] trigger ignored: typing already active",
                flush=True,
            )
            return
        self._pending = True
        print("[typing_topic] trigger received", flush=True)

    def _get_character(self):
        if self._character is None:
            import omni.anim.graph.core as ag

            self._character = ag.get_character(self._skelroot_path)
        return self._character

    def _start_typing(self, now: float) -> bool:
        character = self._get_character()
        if character is None or not self._home_ready:
            return False
        character.set_world_transform(
            self._anchor_position, self._anchor_rotation
        )
        character.set_variable("Action", "type_keyboard")
        self._active = True
        self._pending = False
        self._end_time = now + TYPING_DURATION_SECONDS
        print(
            f"[typing_topic] typing started for "
            f"{TYPING_DURATION_SECONDS:.1f}s",
            flush=True,
        )
        return True

    def _stop_typing(self) -> None:
        character = self._get_character()
        if character is not None:
            character.set_variable("Action", "None")
            character.set_world_transform(
                self._anchor_position, self._anchor_rotation
            )
        self._active = False
        print(
            "[typing_topic] typing finished; returned to idle",
            flush=True,
        )

    def update(self) -> None:
        if self._shutdown:
            return
        self._rclpy.spin_once(self._node, timeout_sec=0.0)
        character = self._get_character()
        if character is None:
            return

        if not self._home_ready:
            from omni.anim.people.scripts.utils import Utils

            _, current_rotation = Utils.get_character_transform(character)
            self._anchor_position = carb.Float3(
                TYPING_X, TYPING_Y, TYPING_Z
            )
            self._anchor_rotation = current_rotation
            character.set_variable("Action", "None")
            character.set_world_transform(
                self._anchor_position, self._anchor_rotation
            )
            self._home_ready = True
            print(
                "[typing_topic] idle home pose locked at "
                f"{tuple(self._anchor_position)}",
                flush=True,
            )

        now = time.monotonic()
        if self._pending and not self._active:
            self._start_typing(now)
        if not self._active:
            character.set_world_transform(
                self._anchor_position, self._anchor_rotation
            )
            return
        if now >= self._end_time:
            self._stop_typing()
            return
        character.set_world_transform(
            self._anchor_position, self._anchor_rotation
        )

    def shutdown(self) -> None:
        if self._shutdown:
            return
        if self._active:
            self._stop_typing()
        self._node.destroy_node()
        if self._owns_rclpy_context and self._rclpy.ok():
            self._rclpy.shutdown()
        self._shutdown = True
