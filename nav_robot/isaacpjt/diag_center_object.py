#!/usr/bin/env python3
"""Headless Stage snapshots for NAV_ROBOT_COUNT=1 vs 2 center-object hunt.

Run with Isaac python:
  NAV_ROBOT_CENTER_DIAG=1 NAV_ROBOT_HEADLESS=1 NAV_ROBOT_COUNT=2 \\
    isaac_python isaacpjt/diag_center_object.py
"""

from __future__ import annotations

import json
import math
import os
import sys
from pathlib import Path

# Force early env before nav_restaurant_demo imports SimulationApp.
os.environ.setdefault("NAV_ROBOT_HEADLESS", "1")
os.environ.setdefault("NAV_CROSSING_PEDESTRIAN", "0")
os.environ.setdefault("NAV_TYPING_CUSTOMER", "0")
os.environ.setdefault("MOBILE_DEMO_HAND_TEST", "0")
os.environ.setdefault("MOBILE_DEMO_OBSTACLE_TEST", "0")

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).resolve().parent))

# Import demo after we may set COUNT via argv.
COUNT = int(os.environ.get("NAV_ROBOT_COUNT", "2"))
OUT_DIR = Path(os.environ.get("NAV_ROBOT_DIAG_DIR", "/tmp"))
OUT_DIR.mkdir(parents=True, exist_ok=True)
TAG = f"nav_robot_count{COUNT}"


def _mat_to_list(mat) -> list:
    return [[float(mat[r][c]) for c in range(4)] for r in range(4)]


def snapshot_prim(stage, prim, cache, bbox_cache) -> dict | None:
    from pxr import Usd, UsdGeom, UsdPhysics

    try:
        from pxr import PhysxSchema
    except Exception:
        PhysxSchema = None

    path = str(prim.GetPath())
    if path == "/":
        return None
    info = {
        "path": path,
        "name": prim.GetName(),
        "type": prim.GetTypeName(),
        "active": bool(prim.IsActive()),
        "parent": str(prim.GetParent().GetPath()) if prim.GetParent() else None,
        "has_ref": bool(prim.HasAuthoredReferences()),
        "has_payload": bool(prim.HasAuthoredPayloads()),
        "rigid_body": bool(prim.HasAPI(UsdPhysics.RigidBodyAPI)),
        "physx_rigid_body": bool(
            PhysxSchema is not None and prim.HasAPI(PhysxSchema.PhysxRigidBodyAPI)
        ),
        "collision": bool(prim.HasAPI(UsdPhysics.CollisionAPI)),
        "mass": bool(prim.HasAPI(UsdPhysics.MassAPI)),
        "articulation_root": bool(prim.HasAPI(UsdPhysics.ArticulationRootAPI)),
        "is_joint": bool(prim.IsA(UsdPhysics.Joint)),
        "instanceable": bool(prim.IsInstanceable()),
        "instance": bool(prim.IsInstance()),
        "instance_proxy": bool(prim.IsInstanceProxy()),
        "applied_schemas": [str(s) for s in prim.GetAppliedSchemas()],
    }
    enabled = prim.GetAttribute("physics:rigidBodyEnabled")
    if enabled and enabled.IsValid():
        info["rigid_body_enabled"] = bool(enabled.Get())
    if prim.IsA(UsdGeom.Imageable):
        try:
            info["visibility"] = str(UsdGeom.Imageable(prim).ComputeVisibility())
        except Exception:
            info["visibility"] = None
    if prim.IsA(UsdGeom.Xformable):
        try:
            local = UsdGeom.Xformable(prim).GetLocalTransformation()
            world = cache.GetLocalToWorldTransform(prim)
            info["local_matrix"] = _mat_to_list(local)
            info["world_matrix"] = _mat_to_list(world)
            t = world.ExtractTranslation()
            info["world_translation"] = [float(t[0]), float(t[1]), float(t[2])]
        except Exception as exc:
            info["xform_error"] = str(exc)
    if prim.IsA(UsdGeom.Boundable):
        try:
            bound = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mid = bound.GetMidpoint()
            size = bound.GetSize()
            info["world_bbox_center"] = [float(mid[0]), float(mid[1]), float(mid[2])]
            info["world_bbox_size"] = [float(size[0]), float(size[1]), float(size[2])]
        except Exception:
            pass
    if prim.HasAuthoredReferences():
        refs = []
        for ref in prim.GetPrimStack():
            ident = getattr(ref.layer, "identifier", None)
            if ident:
                refs.append(str(ident))
        info["prim_stack_layers"] = refs[:8]
    if prim.IsA(UsdPhysics.Joint):
        joint = UsdPhysics.Joint(prim)
        info["joint_body0"] = [str(t) for t in joint.GetBody0Rel().GetTargets()]
        info["joint_body1"] = [str(t) for t in joint.GetBody1Rel().GetTargets()]
    return info


def dump_stage(stage, label: str) -> Path:
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache()
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    rows = []
    # Include instance proxies so arm/sensor meshes under instanceable
    # visuals are not invisible to the dump.
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        row = snapshot_prim(stage, prim, cache, bbox_cache)
        if row:
            rows.append(row)
    rows.sort(key=lambda r: r["path"])
    out = OUT_DIR / f"{TAG}_{label}.json"
    out.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(f"[diag] wrote {out} prims={len(rows)}", flush=True)
    return out


def center_candidates(stage, label: str, xy=1.5, z_max=2.5):
    from pxr import Usd, UsdGeom

    cache = UsdGeom.XformCache()
    bbox_cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
    )
    hits = []
    for prim in stage.Traverse(Usd.TraverseInstanceProxies()):
        if not prim.IsA(UsdGeom.Boundable):
            continue
        try:
            bound = bbox_cache.ComputeWorldBound(prim).ComputeAlignedRange()
            mid = bound.GetMidpoint()
            cx, cy, cz = float(mid[0]), float(mid[1]), float(mid[2])
        except Exception:
            continue
        # Prefer world_translation when bbox is broken (0,0,0 / huge negative size)
        try:
            wt = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
            wx, wy, wz = float(wt[0]), float(wt[1]), float(wt[2])
            if abs(cx) < 1e-6 and abs(cy) < 1e-6 and (abs(wx) > 0.05 or abs(wy) > 0.05):
                cx, cy, cz = wx, wy, wz
        except Exception:
            pass
        if abs(cx) <= xy and abs(cy) <= xy and -0.2 <= cz <= z_max:
            # Skip giant floor / architecture if huge
            size = bound.GetSize()
            if float(size[0]) > 8 or float(size[1]) > 8:
                continue
            if float(size[0]) < 0:  # broken bbox
                continue
            path = str(prim.GetPath())
            # Skip dining floor itself
            if path.endswith("/Architecture/Floor"):
                continue
            row = snapshot_prim(stage, prim, cache, bbox_cache)
            hits.append(row)
            print(
                f"[CENTER CANDIDATE {label}] path={path} type={prim.GetTypeName()} "
                f"center=({cx:.3f},{cy:.3f},{cz:.3f}) "
                f"RB={prim.HasAPI(__import__('pxr', fromlist=['UsdPhysics']).UsdPhysics.RigidBodyAPI)}",
                flush=True,
            )
    out = OUT_DIR / f"{TAG}_{label}_center.json"
    out.write_text(json.dumps(hits, indent=2), encoding="utf-8")
    print(f"[diag] center candidates {label}: {len(hits)} -> {out}", flush=True)
    return hits


def main():
    # Import pulls up SimulationApp.
    import nav_restaurant_demo as demo
    from pxr import Usd, UsdPhysics

    global Usd  # used in dump_stage
    globals()["Usd"] = Usd

    print(f"[diag] COUNT={COUNT} OUT_DIR={OUT_DIR}", flush=True)
    stage = demo.open_restaurant_and_robot()
    dump_stage(stage, "after_spawn")
    center_candidates(stage, "after_spawn")

    demo.configure_joint_drives(stage)
    specs = demo._specs()
    arts = []
    for spec in specs:
        demo.configure_wheel_contact_material(stage, robot_root=spec.root)
        ap = demo.find_articulation_path(stage, robot_root=spec.root)
        demo.configure_physics_stability(stage, ap)
        art, dofs = demo.initialize_robot(ap)
        arts.append((spec, art, dofs))
        demo.sanitize_embedded_rsd455(stage, robot_root=spec.root)

    dump_stage(stage, "after_init")
    center_candidates(stage, "after_init")

    timeline = demo.omni.timeline.get_timeline_interface()
    if not timeline.is_playing():
        timeline.play()
    for _ in range(2):
        demo.simulation_app.update()
    dump_stage(stage, "after_play")
    center_candidates(stage, "after_play")

    for _ in range(8):
        demo.simulation_app.update()
    dump_stage(stage, "frame10")
    center_candidates(stage, "frame10")

    for _ in range(50):
        demo.simulation_app.update()
    dump_stage(stage, "frame60")
    center_candidates(stage, "frame60")

    for _ in range(60):
        demo.simulation_app.update()
    dump_stage(stage, "frame120")
    c120 = center_candidates(stage, "frame120")

    # Summarize robot-relative outliers near dining center.
    from pxr import UsdGeom

    cache = UsdGeom.XformCache()
    for spec, _a, _d in arts:
        root = spec.root
        outliers = []
        root_prim = stage.GetPrimAtPath(root)
        root_xf = cache.GetLocalToWorldTransform(root_prim)
        rx, ry = [float(v) for v in root_xf.ExtractTranslation()][:2]
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if not path.startswith(root + "/"):
                continue
            if not prim.IsA(UsdGeom.Boundable):
                continue
            try:
                t = cache.GetLocalToWorldTransform(prim).ExtractTranslation()
                wx, wy, wz = float(t[0]), float(t[1]), float(t[2])
            except Exception:
                continue
            dist = math.hypot(wx - rx, wy - ry)
            if dist >= 2.0 and abs(wx) <= 1.5 and abs(wy) <= 1.5:
                outliers.append(
                    (
                        path,
                        wx,
                        wy,
                        wz,
                        dist,
                        prim.HasAPI(UsdPhysics.RigidBodyAPI),
                    )
                )
        print(f"[diag] {spec.robot_id} center-outliers={len(outliers)}", flush=True)
        for row in outliers[:40]:
            print(f"  OUTLIER {row}", flush=True)

    print(f"[diag] done count={COUNT} center@120={len(c120)}", flush=True)
    demo.simulation_app.close()


if __name__ == "__main__":
    main()
