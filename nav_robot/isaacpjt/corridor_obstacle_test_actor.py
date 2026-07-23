"""Corridor obstacle person spawn controller for LiDAR obstacle safety test."""

from __future__ import annotations

import os
import threading
from pxr import Gf, UsdGeom, UsdPhysics

PERSON_USD = os.environ.get(
    "CORRIDOR_PERSON_USD",
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/People/Characters/F_Business_02/"
    "F_Business_02.usd",
)

CORRIDOR_PERSON_PRIM = "/World/CorridorObstacleTestPerson"
CORRIDOR_PERSON_POSITION = Gf.Vec3d(
    float(os.environ.get("CORRIDOR_PERSON_X", "0.0")),
    float(os.environ.get("CORRIDOR_PERSON_Y", "2.8")),
    float(os.environ.get("CORRIDOR_PERSON_Z", "0.0")),
)
CORRIDOR_PERSON_YAW = float(os.environ.get("CORRIDOR_PERSON_YAW", "90.0"))


class CorridorPersonSpawnController:
    """Service-controlled corridor person obstacle for LiDAR safety testing."""

    def __init__(self, stage):
        self.stage = stage
        self.active = False
        self._request_lock = threading.Lock()
        self._requested_visibility: bool | None = None

        prim = self.stage.GetPrimAtPath(CORRIDOR_PERSON_PRIM)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
        print(
            f"[obstacle_test] corridor person controller ready at {tuple(CORRIDOR_PERSON_POSITION)}",
            flush=True,
        )

    def request_visible(self, visible: bool) -> None:
        """Queue visibility change; USD mutation is deferred to sim thread."""
        with self._request_lock:
            self._requested_visibility = bool(visible)

    def _spawn(self) -> None:
        prim = self.stage.GetPrimAtPath(CORRIDOR_PERSON_PRIM)
        if not prim.IsValid():
            xform = UsdGeom.Xform.Define(self.stage, CORRIDOR_PERSON_PRIM)
            prim = xform.GetPrim()
            prim.GetReferences().AddReference(PERSON_USD)
            xformable = UsdGeom.Xformable(prim)
            xformable.ClearXformOpOrder()
            xformable.AddTranslateOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            ).Set(CORRIDOR_PERSON_POSITION)
            xformable.AddRotateZOp(
                precision=UsdGeom.XformOp.PrecisionDouble
            ).Set(CORRIDOR_PERSON_YAW)

            # Add invisible capsule collider so PhysX / RTX LiDAR rays hit the obstacle
            collider_path = f"{CORRIDOR_PERSON_PRIM}/CollisionBody"
            capsule = UsdGeom.Capsule.Define(self.stage, collider_path)
            capsule.CreateHeightAttr(1.7)
            capsule.CreateRadiusAttr(0.3)
            capsule.CreateAxisAttr("Z")
            UsdGeom.Xformable(capsule).AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.85))
            UsdPhysics.CollisionAPI.Apply(capsule.GetPrim())
            UsdGeom.Imageable(capsule.GetPrim()).MakeInvisible()

        UsdGeom.Imageable(prim).MakeVisible()
        self.active = True
        print(
            f"[obstacle_test] corridor person spawned with LiDAR collider at {tuple(CORRIDOR_PERSON_POSITION)}",
            flush=True,
        )

    def _remove(self) -> None:
        prim = self.stage.GetPrimAtPath(CORRIDOR_PERSON_PRIM)
        if prim.IsValid():
            UsdGeom.Imageable(prim).MakeInvisible()
        self.active = False
        print("[obstacle_test] corridor person made invisible", flush=True)

    def update(self) -> None:
        with self._request_lock:
            requested_visibility = self._requested_visibility
            self._requested_visibility = None
        if requested_visibility is None:
            return
        if requested_visibility:
            if not self.active:
                self._spawn()
        elif self.active:
            self._remove()
