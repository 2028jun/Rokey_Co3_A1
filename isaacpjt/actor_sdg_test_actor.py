"""Actor SDG / omni.anim.people replacement for the hand-authored
rigid-arm two-bone-IK mechanism in hand_intrusion_test_actor.py.

STATUS (pass 9, see GPU_RUN_LOG.txt): NOT currently shipped as the default
(HAND_TEST_RIG_MODE stays "rigid_arm" in mobile_manipulator_demo.py).
This module works correctly when driven in isolation on a bare restaurant
stage (confirmed by rendering: the character walks to and sits at a real
TableSet_00 chair, stands, and plays push_button/type_keyboard, looping).
It does NOT currently work when wired into mobile_manipulator_demo.py's
full restaurant+robot pipeline: `ag.get_character()` (which
CharacterBehavior.init_character() depends on to do anything at all) never
registers the character there, for reasons isolated but not resolved this
pass -- see enable_extensions()'s docstring for the full account. Kept and
gated behind HAND_TEST_RIG_MODE=actor_sdg as a real, working starting point
for whoever picks up that investigation next, not as dead code.

Passes 5-8 built a custom static two-bone IK rig to make a hand reach a
fixed table target. The reviewer's explicit call (see GPU_RUN_LOG.txt pass
8 and VISION_TEST_GPU_PROMPT.txt) was to discard that mechanism entirely --
not worth further cosmetic iteration -- in favor of triggering real,
professionally-authored skelanim clips through omni.anim.people's own
command system, exactly as isaacpjt/anim_spike_test.py proved works
headless with no crash (see ANIM_SPIKE_RESULTS.txt). That spike ran on an
empty stage, though, and did not exercise the failure mode described above.

This module drives a real character through a repeating
typing -> sit -> push_button -> sit cycle at one of TableSet_00's chairs:

- `Sit` and `GoTo` are omni.anim.people BUILT-IN commands (not the
  CustomCommand mechanism) -- see
  omni/anim/people/scripts/commands/{sit,goto}.py in the extension source.
  `Sit`'s own setup() falls back to the seat prim's own transform when it
  has no `walk_to_offset`/`interact_offset` child prims (see
  interactable_object_helper.py's get_interact_prim_offsets), so it works
  directly against TableSet_00's plain Chair_00/01_Visual meshes with no
  extra authoring needed -- confirmed by rendering on a bare restaurant
  stage (see status note above), not assumed.
- `push_button`/`type_keyboard` are registered via the same CustomCommand
  mechanism the spike used, since the source anim USDs
  (assets/actor_animations/*.skelanim.usd, copied from the GPU machine's
  ~/Downloads where they were provided) are bare SkelAnimation prims with
  none of the CustomCommandName/CustomCommandTemplate attributes
  CustomCommandManager.add_custom_command() requires -- append CustomCommand
  objects directly instead, exactly as the spike did.
- Navigation (`Sit`'s walk-to-seat, `GoTo`) uses omni.anim.people's
  navmesh_enabled=False straight-line path mode -- confirmed from
  navigation_manager.py's generate_path(): with navmesh disabled it just
  walks point-to-point with no baked NavMesh required, and
  lightweight_restaurant/ has none.

Usage (opt-in from mobile_manipulator_demo.py via HAND_TEST_RIG_MODE=
actor_sdg, mirrors hand_intrusion_test_actor.py's spawn_seated_person
shape):

    import actor_sdg_test_actor as actor_sdg
    ...
    if os.environ.get("MOBILE_DEMO_HAND_TEST", "1") == "1":
        person_prim = actor_sdg.spawn_and_configure_actor(stage)
    ...
    while simulation_app.is_running():
        simulation_app.update()
        # No per-frame driver call needed here (unlike the old
        # ReachAnimator): omni.anim.people's own BehaviorScript.on_update()
        # self-drives the character once the timeline is playing -- when
        # ag.get_character() actually registers it (see status note).
        time.sleep(0.010)
"""

from __future__ import annotations

import os
from pathlib import Path

import carb
import carb.settings
import omni.kit.app
from pxr import Gf

PERSON_PRIM_PATH = "/World/Characters/Customer"

# TableSet_00 (assets/lightweight_restaurant/lightweight_pizza_restaurant.usda)
# has Chair_00_Visual/Chair_01_Visual under this path; Chair_01 sits on the
# table's +X (east) side, closer to the fixed table camera's mast (see
# mobile_manipulator_demo.py's attach_fixed_table_depth_camera).
CHAIR_PRIM_PATH = os.environ.get(
    "HAND_TEST_CHAIR_PATH", "/World/Dining/TableSet_00/Chair_01_Visual"
)
SIT_DURATION_SECONDS = float(os.environ.get("HAND_TEST_SIT_SECONDS", "3.0"))

# Standing position the actor returns to between activities. Sit's own exit
# logic walks back to wherever the character was standing when that
# particular Sit command started, so this position anchors the whole cycle
# (typing/push_button always play here unless HAND_TEST_WALK_IN_BEFORE
# below relocates one of them).
STAND_XY = (
    float(os.environ.get("HAND_TEST_STAND_X", "-2.70")),
    float(os.environ.get("HAND_TEST_STAND_Y", "-3.60")),
)
STAND_YAW_DEGREES = float(os.environ.get("HAND_TEST_STAND_YAW", "0.0"))

# Adaptive walk-in (goal 3): if measurement (projecting the animated hand
# through the real table camera, the same technique pass 8 used) shows
# "push_button" or "type_keyboard" does not bring the hand into
# TABLE_ROI_NORMALIZED from STAND_XY, set HAND_TEST_WALK_IN_BEFORE to that
# behavior's name so a GoTo is inserted right before it, and the character
# returns to STAND_XY again afterward. Sourced from an env var (not
# hardcoded) so a future pass can re-measure and retune without a code
# change -- see GPU_RUN_LOG.txt for the measurement that set the default.
WALK_IN_BEFORE = os.environ.get("HAND_TEST_WALK_IN_BEFORE", "")  # "push_button" / "type_keyboard" / ""
WALK_IN_XY = (
    float(os.environ.get("HAND_TEST_WALK_IN_X", str(STAND_XY[0]))),
    float(os.environ.get("HAND_TEST_WALK_IN_Y", str(STAND_XY[1]))),
)

_ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets" / "actor_animations"
PUSH_BUTTON_ANIM = os.environ.get(
    "HAND_TEST_PUSH_BUTTON_ANIM", str(_ASSETS_DIR / "push_button.skelanim.usd")
)
TYPE_KEYBOARD_ANIM = os.environ.get(
    "HAND_TEST_TYPE_KEYBOARD_ANIM", str(_ASSETS_DIR / "type_keyboard.skelanim.usd")
)

COMMAND_FILE_PATH = os.environ.get("HAND_TEST_COMMAND_FILE", "/tmp/actor_sdg_commands.txt")


def enable_extensions():
    """Enable omni.anim.people/isaacsim.replicator.agent.core.

    Must be called after the target stage is already open, not before --
    confirmed by repeated testing in mobile_manipulator_demo.py's
    restaurant+robot pipeline specifically: enabling any anim.graph.core-
    derived extension (tried omni.anim.people, isaacsim.replicator.agent.
    core, and omni.anim.graph.core alone, in every ordering relative to
    enable_urdf_importer()) BEFORE that stage's first open reproducibly
    segfaults during the open -- observed right after an
    "omni.anim.graph.core.plugin: CharacterManager::Shutdown() called
    without a prior successful call to CharacterManager::Initialize()"
    warning, immediately followed by the crash while loading the
    restaurant's Lightwheel_Kitchen sublayer (a pre-existing, unrelated
    "Could not load sublayer ... metricsAssembler" warning that appears in
    every pass's log regardless of this module -- worth investigating
    directly as a possible root cause, not yet done).

    Enabling after the stage is open avoids that crash, but does not fully
    fix the underlying problem: `ag.get_character()` (what
    CharacterBehavior.init_character() depends on to do anything) still
    never registers the character in THIS SPECIFIC integrated pipeline --
    confirmed stuck at None even after 60+ real seconds of playback, after
    toggling the omni.anim.graph.core extension off/on, and after
    re-applying AnimationGraphAPI post-toggle. None of these symptoms
    reproduce on a bare restaurant stage with no robot: there, the
    identical spawn_and_configure_actor() call resolves ag.get_character()
    within 1 frame of the first play(). The actual differentiator between
    "bare restaurant stage" (works) and "mobile_manipulator_demo.py's full
    pipeline" (crashes if enabled early, silently never registers if
    enabled late) was not isolated this pass -- ruled out so far: extension
    enable ordering relative to enable_urdf_importer()/import_robot_usd(),
    the robot's own USD reference being present, and timeline stop()/play()
    transition timing. Suspects not yet tried: the specific extra Python
    imports mobile_manipulator_demo.py has at module load time (numpy,
    omni.graph.core, isaacsim.core.utils.viewports, etc.), and the
    Lightwheel_Kitchen metricsAssembler warning noted above.
    """
    ext_manager = omni.kit.app.get_app().get_extension_manager()
    for ext in ("omni.anim.people", "isaacsim.replicator.agent.core"):
        ok = ext_manager.set_extension_enabled_immediate(ext, True)
        print(f"[actor_sdg] enable {ext} -> {ok}", flush=True)


def _register_custom_commands(ccm):
    from omni.anim.people.scripts.custom_command.defines import CustomCommand, CustomCommandTemplate

    existing = set(ccm.get_all_custom_command_names())
    if "push_button" not in existing:
        ccm._commands.append(
            CustomCommand(
                anim_path=PUSH_BUTTON_ANIM,
                name="push_button",
                template=CustomCommandTemplate.TIMING,
                min_random_time=SIT_DURATION_SECONDS,
                max_random_time=SIT_DURATION_SECONDS,
            )
        )
    if "type_keyboard" not in existing:
        ccm._commands.append(
            CustomCommand(
                anim_path=TYPE_KEYBOARD_ANIM,
                name="type_keyboard",
                template=CustomCommandTemplate.TIMING,
                min_random_time=SIT_DURATION_SECONDS,
                max_random_time=SIT_DURATION_SECONDS,
            )
        )


def _build_command_lines(character_name: str) -> list[str]:
    def sit():
        return f"{character_name} Sit {CHAIR_PRIM_PATH} {SIT_DURATION_SECONDS}"

    def goto_walk_in():
        return f"{character_name} GoTo {WALK_IN_XY[0]} {WALK_IN_XY[1]} 0 _"

    def goto_stand():
        return f"{character_name} GoTo {STAND_XY[0]} {STAND_XY[1]} {STAND_YAW_DEGREES}"

    lines = []
    if WALK_IN_BEFORE == "type_keyboard":
        lines.append(goto_walk_in())
    lines.append(f"{character_name} type_keyboard")
    if WALK_IN_BEFORE == "type_keyboard":
        lines.append(goto_stand())
    lines.append(sit())
    if WALK_IN_BEFORE == "push_button":
        lines.append(goto_walk_in())
    lines.append(f"{character_name} push_button")
    if WALK_IN_BEFORE == "push_button":
        lines.append(goto_stand())
    lines.append(sit())
    return lines


def _write_command_file(character_name: str) -> str:
    lines = _build_command_lines(character_name)
    with open(COMMAND_FILE_PATH, "w") as f:
        f.write("\n".join(lines) + "\n")
    print(f"[actor_sdg] wrote command file {COMMAND_FILE_PATH}:\n  " + "\n  ".join(lines), flush=True)
    return COMMAND_FILE_PATH


def spawn_and_configure_actor(stage):
    """Spawn a visible character, wire it to the sit/type/push cycle, and
    return its prim. Call once; the character then drives itself via
    omni.anim.people's BehaviorScript once the timeline is playing.
    """
    enable_extensions()
    app = omni.kit.app.get_app()
    for _ in range(10):
        app.update()

    from omni.anim.people.python_ext import get_instance as get_people_instance
    from omni.anim.people.settings import PeopleSettings
    from isaacsim.replicator.agent.core.stage_util import CharacterUtil
    from isaacsim.replicator.agent.core.settings import AssetPaths, BehaviorScriptPaths

    settings = carb.settings.get_settings()
    settings.set(PeopleSettings.NAVMESH_ENABLED, False)
    settings.set(PeopleSettings.DYNAMIC_AVOIDANCE_ENABLED, False)
    settings.set_string(PeopleSettings.NUMBER_OF_LOOP, "inf")

    people_instance = get_people_instance()
    ccm = people_instance.get_custom_command_manager()
    _register_custom_commands(ccm)
    print(f"[actor_sdg] registered custom commands: {ccm.get_all_custom_command_names()}", flush=True)

    biped_prim = CharacterUtil.load_default_biped_to_stage()
    for _ in range(20):
        app.update()
    anim_graph_prim = CharacterUtil.get_anim_graph_from_character(biped_prim)
    if anim_graph_prim is None or not anim_graph_prim.IsValid():
        raise RuntimeError("default biped animation graph missing after load_default_biped_to_stage()")

    char_asset_path = AssetPaths.default_biped_asset_path()
    stand_position = Gf.Vec3d(STAND_XY[0], STAND_XY[1], 0.0)
    char_prim = CharacterUtil.load_character_usd_to_stage(
        char_asset_path, stand_position, STAND_YAW_DEGREES, "Customer"
    )
    for _ in range(20):
        app.update()

    skelroot = CharacterUtil.get_character_skelroot_by_root(char_prim)
    if skelroot is None:
        raise RuntimeError(f"no SkelRoot found under spawned character {char_prim.GetPath()}")

    # CharacterBehavior.on_init() (which reads these settings via
    # renew_character_state()) fires at attach time regardless of the
    # timeline's play state, and on_play() only fires on a later
    # stopped->playing TRANSITION -- if the timeline happens to already be
    # playing when this character attaches (true in
    # mobile_manipulator_demo.py, where initialize_robot() plays earlier
    # for the robot's PhysX handles), on_play() never fires again for this
    # instance. So the command-file settings MUST already be correct
    # before setup_python_scripts_to_character() attaches the script below
    # -- setting them afterward is a real, confirmed-by-testing bug (the
    # character finds an empty command_path and self.character stays None
    # forever, since init_character() never gets a chance to re-run with
    # the right settings).
    character_name = char_prim.GetName()
    cmd_file = _write_command_file(character_name)
    settings.set_string(PeopleSettings.COMMAND_FILE_PATH, cmd_file)

    CharacterUtil.setup_animation_graph_to_character([skelroot], anim_graph_prim)
    CharacterUtil.setup_python_scripts_to_character([skelroot], BehaviorScriptPaths.behavior_script_path())
    for _ in range(10):
        app.update()

    print(
        f"[actor_sdg] spawned character={char_prim.GetPath()} name={character_name} "
        f"stand_xy={STAND_XY} chair={CHAIR_PRIM_PATH} walk_in_before={WALK_IN_BEFORE!r}",
        flush=True,
    )
    return char_prim
