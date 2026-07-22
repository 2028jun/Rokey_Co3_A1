#!/usr/bin/env python3
"""Extract a static right-hand mesh from NVIDIA's F_Business_02 USD."""

from __future__ import annotations

import argparse
from pathlib import Path

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade, Vt


HAND_JOINT_MIN = 83
HAND_JOINT_MAX = 98
WRIST_JOINT_MIN = 79
WRIST_JOINT_MAX = 82
JOINTS_PER_VERTEX = 5
MIN_HAND_WEIGHT = 0.01
NVIDIA_CHARACTER_USD = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com/"
    "Assets/Isaac/5.1/Isaac/People/Characters/F_Business_02/"
    "F_Business_02.usd"
)
SKIN_MATERIAL_PATH = Sdf.Path(
    "/Root/female_adult_business_02/Looks/trans__organic__skin"
)
HAND_CENTER = Gf.Vec3f(
    -0.6220545, 0.05085582, 1.3136680
)


def extract(source_path: Path, output_path: Path) -> None:
    source = Usd.Stage.Open(str(source_path))
    if source is None:
        raise RuntimeError(f"could not open source USD: {source_path}")
    source_prim = next(
        (
            prim
            for prim in source.Traverse()
            if prim.GetTypeName() == "Mesh"
            and prim.GetName() == "trans__organic__skin"
        ),
        None,
    )
    if source_prim is None:
        raise RuntimeError("trans__organic__skin mesh not found")

    source_mesh = UsdGeom.Mesh(source_prim)
    points = list(source_mesh.GetPointsAttr().Get() or [])
    counts = list(source_mesh.GetFaceVertexCountsAttr().Get() or [])
    indices = list(source_mesh.GetFaceVertexIndicesAttr().Get() or [])
    st_values = list(source_prim.GetAttribute("primvars:st0").Get() or [])
    st_indices = list(
        source_prim.GetAttribute("primvars:st0:indices").Get() or []
    )
    joint_indices = list(
        source_prim.GetAttribute("primvars:skel:jointIndices").Get() or []
    )
    joint_weights = list(
        source_prim.GetAttribute("primvars:skel:jointWeights").Get() or []
    )
    if len(joint_indices) != len(points) * JOINTS_PER_VERTEX:
        raise RuntimeError("unexpected skin joint-index layout")

    hand_vertices = set()
    for vertex_index in range(len(points)):
        offset = vertex_index * JOINTS_PER_VERTEX
        hand_weight = sum(
            float(joint_weights[offset + slot])
            for slot in range(JOINTS_PER_VERTEX)
            if HAND_JOINT_MIN
            <= int(joint_indices[offset + slot])
            <= HAND_JOINT_MAX
        )
        point = points[vertex_index]
        wrist_weight = sum(
            float(joint_weights[offset + slot])
            for slot in range(JOINTS_PER_VERTEX)
            if WRIST_JOINT_MIN
            <= int(joint_indices[offset + slot])
            <= WRIST_JOINT_MAX
        )
        # Keep the complete hand plus roughly 10 cm of the wrist/forearm.
        # Spatial limits prevent the forearm-joint weights from selecting the
        # entire exposed arm up to the elbow.
        in_wrist_region = (
            HAND_CENTER[0] - 0.18 <= point[0] <= HAND_CENTER[0] + 0.10
            and abs(point[1] - HAND_CENTER[1]) <= 0.09
            and abs(point[2] - HAND_CENTER[2]) <= 0.07
        )
        if hand_weight >= MIN_HAND_WEIGHT or (
            wrist_weight >= MIN_HAND_WEIGHT and in_wrist_region
        ):
            hand_vertices.add(vertex_index)

    selected_faces = []
    selected_st_indices = []
    corner = 0
    for count in counts:
        face = indices[corner : corner + count]
        if face and all(index in hand_vertices for index in face):
            selected_faces.append(face)
            selected_st_indices.extend(st_indices[corner : corner + count])
        corner += count
    if not selected_faces:
        raise RuntimeError("no right-hand faces selected")

    used_vertices = sorted({index for face in selected_faces for index in face})
    remap = {source_index: index for index, source_index in enumerate(used_vertices)}
    output_points = [points[index] - HAND_CENTER for index in used_vertices]
    output_counts = [len(face) for face in selected_faces]
    output_indices = [remap[index] for face in selected_faces for index in face]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    root = UsdGeom.Xform.Define(stage, "/HandSafetyTestHand")
    stage.SetDefaultPrim(root.GetPrim())
    mesh = UsdGeom.Mesh.Define(stage, "/HandSafetyTestHand/RightHand")
    mesh.CreatePointsAttr().Set(Vt.Vec3fArray(output_points))
    mesh.CreateFaceVertexCountsAttr().Set(Vt.IntArray(output_counts))
    mesh.CreateFaceVertexIndicesAttr().Set(Vt.IntArray(output_indices))
    mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.catmullClark)
    mesh.CreateDoubleSidedAttr().Set(True)
    if st_values and len(selected_st_indices) == len(output_indices):
        st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
            "st0",
            Sdf.ValueTypeNames.TexCoord2fArray,
            UsdGeom.Tokens.faceVarying,
        )
        st.Set(Vt.Vec2fArray(st_values))
        st.SetIndices(Vt.IntArray(selected_st_indices))
    mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant).Set(
        [Gf.Vec3f(0.38, 0.17, 0.10)]
    )

    material = UsdShade.Material.Define(stage, "/HandSafetyTestHand/Looks/Skin")
    material.GetPrim().GetReferences().AddReference(
        NVIDIA_CHARACTER_USD,
        SKIN_MATERIAL_PATH,
    )
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)
    stage.GetRootLayer().Save()
    print(
        f"wrote {output_path}: {len(output_points)} vertices, "
        f"{len(selected_faces)} faces"
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("source", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    extract(args.source, args.output)


if __name__ == "__main__":
    main()
