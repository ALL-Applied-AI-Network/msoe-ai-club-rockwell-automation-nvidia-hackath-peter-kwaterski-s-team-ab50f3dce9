"""Import the user-supplied Rev B tweezer/plunger concept into Isaac Sim.

The archive uses a root frame in millimetres: X across the jaw gap, Y across
finger width, and Z distal.  Current ER-4iA tooling uses local +X distal and
local +Y across the jaw gap, so points map as ``(Z, X, Y)`` after the relevant
parent-frame origin is removed.
"""

from __future__ import annotations

import json
import struct
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdPhysics


KIT_ROOT = Path(__file__).resolve().parent / "assets" / "gripper" / "tweezer_plunger_revC"
KINEMATICS = json.loads((KIT_ROOT / "kinematics.json").read_text(encoding="utf-8"))
AXIAL_LENGTH_M = 0.060
PLUNGER_FACE_RETRACTED_FROM_TIP_M = 0.008
PLUNGER_STROKE_M = 0.012
PLUNGER_MAX_FORCE_N = 20.0
PLUNGER_SIM_MASS_KG = 0.010  # explicit unverified simulation assumption
CLEARANCE_SLIDE_STROKE_M = 0.006
CLEARANCE_SLIDE_MAX_FORCE_N = 5.0
CLEARANCE_SLIDE_SIM_MASS_KG = 0.008  # replace with supplied Rev C mass


def _read_binary_stl(path: Path):
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"truncated STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    if len(data) != 84 + count * 50:
        raise ValueError(f"unexpected STL size: {path}")
    points, faces = [], []
    cursor = 84
    for _ in range(count):
        cursor += 12
        face = []
        for _vertex in range(3):
            points.append(struct.unpack_from("<fff", data, cursor))
            face.append(len(points) - 1)
            cursor += 12
        faces.append(face)
        cursor += 2
    return points, faces


def _read_obj(path: Path):
    points, faces = [], []
    for raw in path.read_text(encoding="utf-8").splitlines():
        fields = raw.split()
        if not fields:
            continue
        if fields[0] == "v" and len(fields) >= 4:
            points.append(tuple(float(value) for value in fields[1:4]))
        elif fields[0] == "f" and len(fields) >= 4:
            face = [int(value.split("/", 1)[0]) for value in fields[1:]]
            faces.append([index - 1 if index > 0 else len(points) + index for index in face])
    if not points or not faces:
        raise ValueError(f"empty OBJ: {path}")
    return points, faces


def _jaw_point(point_mm, axial_origin_m: float, closed_center_m: float):
    across_gap, across_width, axial = point_mm
    return Gf.Vec3f(
        axial_origin_m + axial * 0.001,
        across_gap * 0.001 - closed_center_m,
        across_width * 0.001,
    )


def _fixed_point(point_mm, axial_origin_m: float):
    across_gap, across_width, axial = point_mm
    return Gf.Vec3f(
        axial_origin_m + axial * 0.001,
        across_gap * 0.001,
        across_width * 0.001,
    )


def _moving_point(point_mm):
    across_gap, across_width, axial = point_mm
    return Gf.Vec3f(axial * 0.001, across_gap * 0.001, across_width * 0.001)


def _mesh(stage, path, source, mapper, *, collision, material, bind_visual, bind_physics):
    source = KIT_ROOT / source
    points, faces = (
        _read_binary_stl(source) if source.suffix.lower() == ".stl" else _read_obj(source)
    )
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([mapper(point) for point in points])
    mesh.CreateFaceVertexCountsAttr([len(face) for face in faces])
    mesh.CreateFaceVertexIndicesAttr([index for face in faces for index in face])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.GetPrim().CreateAttribute("kit:sourceAsset", Sdf.ValueTypeNames.Asset).Set(
        Sdf.AssetPath(str(source))
    )
    if collision:
        UsdGeom.Imageable(mesh.GetPrim()).CreateVisibilityAttr().Set("invisible")
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(
            "convexHull"
        )
        bind_physics(mesh.GetPrim(), material)
        from pxr import PhysxSchema

        contact = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        contact.CreateContactOffsetAttr().Set(0.00008)
        contact.CreateRestOffsetAttr().Set(0.0)
    else:
        bind_visual(mesh.GetPrim(), material)
    return mesh.GetPrim()


def _add_link(
    stage,
    parent_path: str,
    link_name: str,
    mapper,
    *,
    visual_material: str,
    physics_material: str,
    bind_visual,
    bind_physics,
):
    record = KINEMATICS["links"][link_name]
    root = UsdGeom.Xform.Define(stage, f"{parent_path}/{link_name}")
    _mesh(
        stage,
        f"{parent_path}/{link_name}/Visual",
        record["visual_mesh"],
        mapper,
        collision=False,
        material=visual_material,
        bind_visual=bind_visual,
        bind_physics=bind_physics,
    )
    for index, collision_source in enumerate(record["collision_meshes"]):
        _mesh(
            stage,
            f"{parent_path}/{link_name}/Collision{index:02d}",
            collision_source,
            mapper,
            collision=True,
            material=physics_material,
            bind_visual=bind_visual,
            bind_physics=bind_physics,
        )
    return root.GetPrim()


def _tag_root(root):
    root.CreateAttribute("kit:assetId", Sdf.ValueTypeNames.String).Set(
        "tweezer_plunger_revC"
    )
    root.CreateAttribute("kit:status", Sdf.ValueTypeNames.String).Set(
        "simulation_concept_unverified_physical_mount"
    )


def add_jaw(
    stage,
    jaw_path: str,
    closed_center_m: float,
    axial_origin_m: float,
    *,
    bind_visual,
    bind_physics,
    soft_material: str,
    object_material: str,
    link_suffixes=("carrier", "finger_insert", "finger_pad"),
):
    """Attach selected already-handed jaw links to one rigid motion group."""
    side = "right" if closed_center_m > 0 else "left"
    root = UsdGeom.Xform.Define(stage, jaw_path + "/TweezerPlungerRevC")
    _tag_root(root.GetPrim())
    mapper = lambda point: _jaw_point(point, axial_origin_m, closed_center_m)
    link_records = [
        ("carrier", "alu_anodised", object_material),
        ("finger_insert", "plastic_blue", object_material),
        ("finger_pad", "bin_yellow", soft_material),
    ]
    if f"{side}_retraction_guide" in KINEMATICS["links"]:
        link_records.append(("retraction_guide", "alu_anodised", object_material))
    selected = set(link_suffixes)
    unknown = selected.difference(record[0] for record in link_records)
    if unknown:
        raise ValueError(f"unknown Rev B jaw link suffixes: {sorted(unknown)}")
    for suffix, visual, physics in link_records:
        if suffix not in selected:
            continue
        _add_link(
            stage,
            str(root.GetPath()),
            f"{side}_{suffix}",
            mapper,
            visual_material=visual,
            physics_material=physics,
            bind_visual=bind_visual,
            bind_physics=bind_physics,
        )
    return root.GetPrim()


def add_fixed_housing(
    stage,
    flange_path: str,
    axial_origin_m: float,
    *,
    bind_visual,
    bind_physics,
    object_material: str,
):
    root = UsdGeom.Xform.Define(stage, flange_path + "/TweezerPlungerRevCFixed")
    _tag_root(root.GetPrim())
    _add_link(
        stage,
        str(root.GetPath()),
        "plunger_housing",
        lambda point: _fixed_point(point, axial_origin_m),
        visual_material="black_anodised",
        physics_material=object_material,
        bind_visual=bind_visual,
        bind_physics=bind_physics,
    )
    return root.GetPrim()


def add_moving_plunger(
    stage,
    plunger_path: str,
    *,
    tool_option: str = "flat",
    bind_visual,
    bind_physics,
    soft_material: str,
    object_material: str,
):
    root = UsdGeom.Xform.Define(stage, plunger_path + "/TweezerPlungerRevCMoving")
    _tag_root(root.GetPrim())
    links = ["plunger_rod", *KINEMATICS["tool_options"][tool_option]]
    for name in links:
        compliant = name.endswith("_pad")
        _add_link(
            stage,
            str(root.GetPath()),
            name,
            _moving_point,
            visual_material="bin_yellow" if compliant else "steel_dark",
            physics_material=soft_material if compliant else object_material,
            bind_visual=bind_visual,
            bind_physics=bind_physics,
        )
    root.GetPrim().CreateAttribute("kit:toolOption", Sdf.ValueTypeNames.String).Set(
        tool_option
    )
    return root.GetPrim()
