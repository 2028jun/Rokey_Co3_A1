"""Spike for ANIM_SPIKE_PROMPT.txt: can push_button/typing behavior
animations be triggered on a character via script (Actor SDG /
omni.anim.people custom-command mechanism), without crashing, headless?

Usage:
    export SPIKE_ANIM_USD=~/Downloads/push_button.skelanim.usd
    export SPIKE_COMMAND_NAME=push_button
    /home/rokey/dev_ws/isaac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
        isaacpjt/anim_spike_test.py [--gui]

Findings (2026-07-23, see ANIM_SPIKE_RESULTS.txt for full detail): works
headless, no crash, animation plays and looks natural. The provided
skelanim USDs do not carry the CustomCommandName/CustomCommandTemplate
attributes CustomCommandManager.add_custom_command() requires, so this
appends a CustomCommand object directly instead -- see the comment below.
"""
import sys
import os
import time

HEADLESS = "--gui" not in sys.argv
ANIM_USD = os.environ.get("SPIKE_ANIM_USD", os.path.expanduser("~/Downloads/push_button.skelanim.usd"))
COMMAND_NAME = os.environ.get("SPIKE_COMMAND_NAME", "push_button")
COMMAND_DURATION = float(os.environ.get("SPIKE_COMMAND_DURATION", "2.0"))
OUT_DIR = os.environ.get(
    "SPIKE_OUT_DIR",
    f"/tmp/anim_spike_frames_{COMMAND_NAME}",
)

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": HEADLESS})

import carb
import carb.settings
import omni.usd
import omni.kit.app
import omni.kit.commands
import omni.timeline
from pxr import Usd, UsdGeom, UsdLux, Gf

print(f"[spike] mode={'headless' if HEADLESS else 'gui'} command={COMMAND_NAME} anim={ANIM_USD}", flush=True)

ext_manager = omni.kit.app.get_app().get_extension_manager()
for ext in ["omni.anim.people", "isaacsim.replicator.agent.core"]:
    ok = ext_manager.set_extension_enabled_immediate(ext, True)
    print(f"[spike] enable {ext} -> {ok}", flush=True)

for _ in range(10):
    simulation_app.update()

from omni.anim.people.python_ext import get_instance as get_people_instance
from omni.anim.people.scripts.custom_command.defines import CustomCommand, CustomCommandTemplate
from isaacsim.replicator.agent.core.stage_util import CharacterUtil
from isaacsim.replicator.agent.core.settings import AssetPaths, BehaviorScriptPaths

omni.usd.get_context().new_stage()
for _ in range(5):
    simulation_app.update()
stage = omni.usd.get_context().get_stage()
print("[spike] new stage created", flush=True)

ccm = get_people_instance().get_custom_command_manager()

# The provided anim USDs (~/Downloads/*.skelanim.usd) do not have the
# CustomCommandName/CustomCommandTemplate attributes CustomCommandManager's
# own add_custom_command() requires (verified by opening them directly:
# their /Root prim is a bare SkelAnimation with no such attrs) -- appending
# a CustomCommand object directly reproduces what populate_anim_graph()
# actually consumes, without needing those attributes pre-authored.
ccm._commands.append(
    CustomCommand(
        anim_path=ANIM_USD,
        name=COMMAND_NAME,
        template=CustomCommandTemplate.TIMING,
        min_random_time=COMMAND_DURATION,
        max_random_time=COMMAND_DURATION,
    )
)
print("[spike] registered custom commands:", ccm.get_all_custom_command_names(), flush=True)

biped_prim = CharacterUtil.load_default_biped_to_stage()
for _ in range(20):
    simulation_app.update()
anim_graph_prim = CharacterUtil.get_anim_graph_from_character(biped_prim)

state_names = [p.GetName() for p in Usd.PrimRange(anim_graph_prim) if p.GetTypeName() == "State"] if anim_graph_prim else []
print("[spike] state machine states:", state_names, flush=True)

char_asset_path = AssetPaths.default_biped_asset_path()
char_prim = CharacterUtil.load_character_usd_to_stage(char_asset_path, Gf.Vec3d(0, 0, 0), 0.0, "TestChar")
for _ in range(20):
    simulation_app.update()

skelroot = CharacterUtil.get_character_skelroot_by_root(char_prim)
CharacterUtil.setup_animation_graph_to_character([skelroot], anim_graph_prim)
CharacterUtil.setup_python_scripts_to_character([skelroot], BehaviorScriptPaths.behavior_script_path())
print("[spike] anim graph + behavior script attached", flush=True)

cmd_file = "/tmp/anim_spike_commands.txt"
with open(cmd_file, "w") as f:
    f.write(f"{char_prim.GetName()} {COMMAND_NAME}\n")
carb.settings.get_settings().set("/exts/omni.anim.people/command_settings/command_file_path", cmd_file)
print(f"[spike] wrote command file {cmd_file}", flush=True)

from omni.kit.viewport.utility import create_viewport_window, capture_viewport_to_file
from omni.kit.scripting.scripts.script_manager import ScriptManager

light = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
light.CreateIntensityAttr(1000.0)

cam_path = "/World/SpikeCamera"
camera = UsdGeom.Camera.Define(stage, cam_path)
camera.CreateFocalLengthAttr(24.0)
eye = Gf.Vec3d(1.2, -1.2, 1.2)
target = Gf.Vec3d(0.0, 0.0, 0.9)
cam_to_world = Gf.Matrix4d().SetLookAt(eye, target, Gf.Vec3d(0, 0, 1)).GetInverse()
UsdGeom.Xformable(camera.GetPrim()).MakeMatrixXform().Set(cam_to_world)
viewport = create_viewport_window("Spike", width=960, height=720)
omni.kit.commands.execute("SetViewportCamera", camera_path=cam_path, viewport_api=viewport.viewport_api)
os.makedirs(OUT_DIR, exist_ok=True)

print("[spike] entering play...", flush=True)
omni.timeline.get_timeline_interface().play()
for i in range(180):
    simulation_app.update()
    time.sleep(1.0 / 30.0)
    if i % 30 == 0:
        print(f"[spike] update {i}", flush=True)
        capture_viewport_to_file(viewport.viewport_api, f"{OUT_DIR}/frame_{i:03d}.png")
        for scripts in ScriptManager.get_instance()._prim_to_scripts.values():
            for _, inst in scripts.items():
                if inst is not None and hasattr(inst, "commands"):
                    print(f"[spike]   behavior commands for {inst.prim_path}: {inst.commands}", flush=True)

for _ in range(10):
    simulation_app.update()
print("[spike] DONE, no crash", flush=True)
omni.timeline.get_timeline_interface().stop()
simulation_app.close()
