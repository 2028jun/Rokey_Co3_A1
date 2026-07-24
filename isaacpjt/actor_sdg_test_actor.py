"""Actor SDG / omni.anim.people replacement for the hand-authored
rigid-arm two-bone-IK mechanism in hand_intrusion_test_actor.py.

STATUS (pass 10, see GPU_RUN_LOG.txt): mechanically working, NOT yet the
default (HAND_TEST_RIG_MODE stays "rigid_arm" in mobile_manipulator_demo.py).
Pass 9 found `ag.get_character()` (which CharacterBehavior.init_character()
depends on to do anything at all) never registered the character inside
mobile_manipulator_demo.py's full restaurant+robot pipeline, though the
identical setup worked on a bare restaurant stage. Pass 10 root-caused and
fixed that -- see enable_extensions()'s docstring -- and confirmed via real
command-queue logs that the full type_keyboard -> Sit -> push_button -> Sit
cycle now registers, executes, and loops correctly (NUMBER_OF_LOOP=inf)
inside the real pipeline, across 40+ second runs with zero crashes and zero
registration failures. What pass 10 could NOT yet confirm is visual pose
quality there: its own ad hoc QA cameras were unreliable (wall clipping,
chair occlusion), and it found a real ~90 degree yaw convention mismatch
between the character's spawned orientation and what
`Utils.convert_to_angle()` reads back for the automatic loop-closing `GoTo`
(see `STAND_YAW_DEGREES`). Until pose quality is confirmed by eye with a
working camera, `HAND_TEST_RIG_MODE` stays "rigid_arm"; this module is a
real, mechanically-working opt-in (`HAND_TEST_RIG_MODE=actor_sdg`) for
whoever finishes that confirmation next, not dead code.

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
import sys
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

_patched_sit_class_ids: set[int] = set()


def _build_patched_sit_update():
    """Build the replacement for omni.anim.people's built-in Sit.update().

    Confirmed by direct testing (rendered frames + logged world transforms,
    see GPU_RUN_LOG.txt pass 11): during Sit's "stand" sub-phase (standing
    back up after sitting), the character's world ROTATION visibly and
    numerically tips away from a pure Z-axis yaw within about a second --
    quaternion x/y components that should stay 0 for a Z-only rotation
    drift to values like x=-0.19 -- and the rendered character ends up
    lying flat on the floor. Root cause: `Sit.update()`'s "stand" branch
    reads the character's CURRENT world rotation fresh every frame via
    `Utils.get_character_transform()` and immediately re-applies it via
    `set_world_transform()`; that current rotation includes whatever
    transient root-motion the currently-blending stand-up animation
    contributes, so re-applying it every frame accumulates the drift
    instead of leaving rotation alone. The "sit" phase does not have this
    bug because it reuses a frozen `self._char_start_rot` captured once,
    never a live re-read -- this patch makes "stand" do the same.
    """
    from omni.metropolis.utils.carb_util import CarbUtil
    from omni.anim.people.scripts.utils import Utils
    from omni.anim.people.settings import MetadataTag, TaskStatus
    from omni.anim.people.scripts.interactable_object_helper import InteractableObjectHelper

    def _patched_update(self, dt):
        if self.current_action == "walk" or self.current_action is None:
            if self.walk(dt):
                if not InteractableObjectHelper.is_object_interactable(self.seat_prim):
                    carb.log_warn("Fail to sit... Object is not interactable now")
                    self.command_status = TaskStatus.failed
                    self.force_quit_command()
                InteractableObjectHelper.add_owner(target_prim=self.seat_prim, agent_name=self.character_name)
                self.current_action = "sit"
                # FIX (vs. upstream): don't use a live read here as the
                # frozen rotation -- confirmed by testing (GPU_RUN_LOG.txt
                # pass 11) that whichever frame `walk(dt)` happens to
                # return True on can catch the character mid-turn, still
                # blending its turn-to-face-chair animation, so the "live"
                # rotation snapshotted at this exact instant is sometimes a
                # transient, not-yet-settled value -- freezing THAT for the
                # whole sit+stand duration reproduces the same lying-on-
                # floor drift on some (not all) sit cycles. `self.interact_
                # rot` is geometry-derived from the chair prim itself (set
                # in setup(), stable, not a live character-pose read), so
                # use it instead for rotation; only translation needs the
                # live arrival position.
                self._char_start_pos, _ = Utils.get_character_transform(self.character)
                self._char_start_rot = self.interact_rot
                self.update_metadata_callback(
                    agent_name=self.character_name, data_name=MetadataTag.AgentActionTag, data_value="Sitting"
                )
        elif self.current_action == "sit":
            self._char_lerp_t = min(self._char_lerp_t + dt, 1.0)
            lerp_pos = CarbUtil.lerp3(self._char_start_pos, self.interact_pos, self._char_lerp_t)
            self.character.set_world_transform(lerp_pos, self._char_start_rot)
            self.character.set_variable("Action", "Sit")
            self.sit_time += dt
            if self.sit_time > self.duration:
                self.character.set_variable("Action", "None")
                self._char_lerp_t = 0.0
                self.current_action = "stand"
                self.update_metadata_callback(
                    agent_name=self.character_name, data_name=MetadataTag.AgentActionTag, data_value="StndingUp"
                )
                InteractableObjectHelper.remove_owner(target_prim=self.seat_prim, agent_name=self.character_name)
        elif self.current_action == "stand":
            # FIX (vs. upstream): reuse the frozen self._char_start_rot
            # captured when sitting began, instead of live-reading and
            # re-applying the character's current (drifting) rotation
            # every frame.
            if self.stand_animation_time < 1.5:
                self._char_lerp_t = min(self._char_lerp_t + dt, 1.0)
                lerp_pos = CarbUtil.lerp3(self.interact_pos, self._char_start_pos, self._char_lerp_t)
                self.character.set_world_transform(lerp_pos, self._char_start_rot)
                if os.environ.get("ACTOR_SDG_DEBUG_STAND_ROT"):
                    readback_pos, readback_rot = Utils.get_character_transform(self.character)
                    print(
                        f"[actor_sdg][stand-debug] t={self.stand_animation_time:.2f} "
                        f"set_rot={tuple(self._char_start_rot)} readback_rot={tuple(readback_rot)}",
                        flush=True,
                    )
                self.stand_animation_time += dt
            if self.stand_animation_time > 1.5:
                self.character.set_world_transform(self._char_start_pos, self._char_start_rot)
                return self.exit_command()

    return _patched_update


_patched_sit_update_fn = None


def _patch_sit_command_stand_rotation():
    """Patch every live `Sit` class currently loaded in sys.modules.

    Kit's extension loader imports `omni.anim.people.scripts.commands.sit`
    under a mangled, install-path-specific module name (observed at
    runtime as `..._omni_anim_people_scripts_.commands.sit`), which is a
    DIFFERENT module/class object than what a normal top-level
    `from omni.anim.people.scripts.commands.sit import Sit` import
    resolves to. Patching only the "logical" import's `Sit` class is a
    silent no-op: the print fires, but the class actually driving
    `CharacterBehavior`'s commands is untouched (confirmed pass 11 --
    identical rotation-drift numbers with and without that patch).

    So instead this scans `sys.modules` for every module whose name ends
    in `commands.sit` and patches each distinct `Sit` class object found
    (tracked by id() so re-scans are cheap and idempotent). Safe to call
    repeatedly and early: it does nothing until the real module has
    actually been imported by the extension, at which point the next
    call catches it.
    """
    global _patched_sit_update_fn
    if _patched_sit_update_fn is None:
        _patched_sit_update_fn = _build_patched_sit_update()

    patched_now = []
    for name, module in list(sys.modules.items()):
        if module is None or not name.endswith("commands.sit"):
            continue
        sit_cls = getattr(module, "Sit", None)
        if not isinstance(sit_cls, type):
            continue
        if id(sit_cls) in _patched_sit_class_ids:
            continue
        sit_cls.update = _patched_sit_update_fn
        _patched_sit_class_ids.add(id(sit_cls))
        patched_now.append(name)

    if patched_now:
        print(f"[actor_sdg] patched Sit.update() on module(s): {patched_now}", flush=True)


def enable_extensions():
    """Enable omni.anim.people/isaacsim.replicator.agent.core.

    Call this BEFORE the target stage is opened, and pump several
    simulation_app.update() calls before opening it (mobile_manipulator_
    demo.py's main() does ~30) -- both parts of this ordering matter, and
    getting either wrong breaks `ag.get_character()` (what
    CharacterBehavior.init_character() depends on to do anything at all)
    for the rest of the session:

    - Enabling these extensions and opening the stage back-to-back with no
      settle gap reproducibly segfaults during that open (pass 9's
      finding), observed right after an "omni.anim.graph.core.plugin:
      CharacterManager::Shutdown() called without a prior successful call
      to CharacterManager::Initialize()" warning -- the extensions'
      startup (which happens lazily, visible as "[ext: omni.anim.graph.
      core-...] startup" appearing several frames after
      set_extension_enabled_immediate() returns) was still in progress
      when the stage transition fired, and the stage-close notification
      reached a CharacterManager that hadn't finished initializing yet.
    - Enabling them AFTER the stage is already open avoids the crash but
      leaves `ag.get_character()` stuck at None forever, confirmed even
      after 60+ real seconds, toggling the extension off/on, and
      re-applying AnimationGraphAPI post-toggle (pass 9's other finding).
    - Pass 10 found the actual fix: enable before opening (like pass 9
      tried), but pump ~30 simulation_app.update() calls first so the
      extensions' own startup fully completes before the stage transition
      happens. With that gap, the same stage open is clean (no crash) and
      `ag.get_character()` registers the character normally once it's
      spawned -- confirmed across multiple 35-42 second runs with zero
      crashes and the full command queue (type_keyboard -> Sit ->
      push_button -> Sit -> GoTo, looping via NUMBER_OF_LOOP=inf) executing
      and cycling correctly, read directly from the live BehaviorScript
      instance.
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

    _patch_sit_command_stand_rotation()

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

    # The Sit command module (mangled name, see _patch_sit_command_stand_
    # rotation()'s docstring) is only importable by Kit's extension loader
    # once a Sit command is actually parsed/instantiated, which may not
    # have happened yet even after the script attaches -- this call is
    # cheap/idempotent, call it again from the capture loop to catch it.
    _patch_sit_command_stand_rotation()

    print(
        f"[actor_sdg] spawned character={char_prim.GetPath()} name={character_name} "
        f"stand_xy={STAND_XY} chair={CHAIR_PRIM_PATH} walk_in_before={WALK_IN_BEFORE!r}",
        flush=True,
    )
    return char_prim
