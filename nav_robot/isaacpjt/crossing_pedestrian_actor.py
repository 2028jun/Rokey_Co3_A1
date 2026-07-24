"""Actor SDG pedestrian that crosses the kitchen-exit driving path."""

from __future__ import annotations

import os
import time

import carb
import omni.kit.app
from pxr import Gf


PERSON_NAME = "CrossingPedestrian"
ENABLED = os.environ.get("NAV_CROSSING_PEDESTRIAN", "1") == "1"

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


def spawn(stage):
    """Spawn a character with the default Actor SDG walking graph."""
    del stage  # CharacterUtil uses omni.usd's active stage.

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
        Gf.Vec3d(LEFT_X, LANE_Y, FLOOR_Z),
        SPAWN_YAW_DEGREES,
        PERSON_NAME,
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
