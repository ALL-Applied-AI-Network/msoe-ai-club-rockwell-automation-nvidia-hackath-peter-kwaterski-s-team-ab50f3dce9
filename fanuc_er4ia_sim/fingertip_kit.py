"""Load the user-supplied Rev A LEGO fingertip kit into a moving jaw body.

The source STLs are millimetres in the CAD frame documented by the kit:
``x`` inward, ``y`` across the finger, and ``z`` toward the part. This module
maps that frame onto the ER-4iA tool frame without rewriting the source files.
"""

from __future__ import annotations

import struct
from pathlib import Path

from pxr import Gf, Sdf, UsdGeom, UsdPhysics


KIT_ROOT = Path(__file__).resolve().parent / "assets" / "gripper" / "fingertip_kit_revA"
SOURCE_PARTS = (
    ("Carrier", "carrier.stl", "alu_anodised", False),
    ("EdgeHookInsert", "edge_hook_insert.stl", "plastic_blue", False),
    ("CompliantPad", "pad.stl", "bin_yellow", True),
)

# The CAD requires mount faces at +/-4.1 mm when closed. Existing jaw-body
# origins are +/-6.0 mm, so each kit datum is 1.9 mm inward in its jaw frame.
# Relative to a centred 6 mm carrier, this represents a 1.1 mm outward adapter.
MOUNT_FACE_FROM_JAW_ORIGIN_M = 0.0019
AXIAL_LENGTH_M = 0.060


def _read_binary_stl(path: Path):
    """Return triangle-soup points and indices from a binary STL."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"truncated STL: {path}")
    count = struct.unpack_from("<I", data, 80)[0]
    expected = 84 + count * 50
    if len(data) != expected:
        raise ValueError(f"unexpected STL size for {path}: {len(data)} != {expected}")
    points = []
    indices = []
    cursor = 84
    for _ in range(count):
        cursor += 12  # facet normal; USD recomputes normals
        for _vertex in range(3):
            points.append(struct.unpack_from("<fff", data, cursor))
            indices.append(len(indices))
            cursor += 12
        cursor += 2
    return points, indices


def _map_point(point_mm, sign: int, axial_origin_m: float):
    """Map kit CAD coordinates into one JawL/JawR local frame."""
    inward, across, axial = point_mm
    return Gf.Vec3f(
        axial_origin_m + axial * 0.001,
        -sign * (MOUNT_FACE_FROM_JAW_ORIGIN_M + inward * 0.001),
        -sign * across * 0.001,
    )


def add_rev_a(
    stage,
    jaw_path: str,
    sign: int,
    axial_origin_m: float,
    *,
    bind_visual_material,
    bind_physics_material,
    soft_tip_material_path: str,
    object_material_path: str,
):
    """Attach exact-source visuals and convex-decomposition collision meshes."""
    root = UsdGeom.Xform.Define(stage, jaw_path + "/FingertipRevA")
    root.GetPrim().CreateAttribute("kit:source", Sdf.ValueTypeNames.String).Set(
        str(KIT_ROOT / "README.md")
    )
    root.GetPrim().CreateAttribute("kit:units", Sdf.ValueTypeNames.String).Set("millimetres")
    root.GetPrim().CreateAttribute("kit:mountFaceSpacing", Sdf.ValueTypeNames.Float).Set(0.0082)
    root.GetPrim().CreateAttribute("kit:mountSpacingVerified", Sdf.ValueTypeNames.Bool).Set(False)
    root.GetPrim().CreateAttribute("kit:simulatedAdapterPerSide", Sdf.ValueTypeNames.Float).Set(0.0011)

    for name, filename, material, compliant in SOURCE_PARTS:
        source = KIT_ROOT / filename
        points_mm, indices = _read_binary_stl(source)
        mesh = UsdGeom.Mesh.Define(stage, f"{jaw_path}/FingertipRevA/{name}")
        mesh.CreatePointsAttr([_map_point(point, sign, axial_origin_m) for point in points_mm])
        mesh.CreateFaceVertexCountsAttr([3] * (len(indices) // 3))
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateSubdivisionSchemeAttr("none")
        mesh.GetPrim().CreateAttribute("kit:sourceAsset", Sdf.ValueTypeNames.Asset).Set(
            Sdf.AssetPath(str(source))
        )
        bind_visual_material(mesh.GetPrim(), material)
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
        UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr().Set(
            "convexDecomposition"
        )
        bind_physics_material(
            mesh.GetPrim(), soft_tip_material_path if compliant else object_material_path
        )
        from pxr import PhysxSchema

        contact = PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
        contact.CreateContactOffsetAttr().Set(0.00008)
        contact.CreateRestOffsetAttr().Set(0.0)

    return root.GetPrim()
