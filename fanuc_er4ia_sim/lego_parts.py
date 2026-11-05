"""LDraw-grounded LEGO visuals and nominal compound collision proxies."""

from __future__ import annotations

from functools import lru_cache
from pathlib import Path

import numpy as np

import lego_demo_config as D

LDRAW_UNIT_M = 0.0004


def _resolve_reference(current: Path, reference: str, root: Path) -> Path:
    reference = reference.replace("\\", "/")
    choices = []
    if reference.startswith("s/"):
        choices.append(root / "parts" / reference)
    elif reference.startswith(("48/", "8/")):
        choices.append(root / "p" / reference)
    else:
        choices.extend((current.parent / reference, root / "parts" / reference, root / "p" / reference))
    for choice in choices:
        if choice.is_file():
            return choice
        lower = choice.with_name(choice.name.lower())
        if lower.is_file():
            return lower
    raise FileNotFoundError(f"unresolved LDraw reference {reference!r} from {current}")


def _flatten(path: Path, root: Path, matrix: np.ndarray, offset: np.ndarray, out: list):
    for raw in path.read_text(encoding="latin-1").splitlines():
        fields = raw.strip().split()
        if not fields:
            continue
        kind = fields[0]
        if kind == "1" and len(fields) >= 15:
            child_offset = np.asarray([float(v) for v in fields[2:5]], float)
            child_matrix = np.asarray([float(v) for v in fields[5:14]], float).reshape(3, 3)
            child = _resolve_reference(path, fields[14], root)
            _flatten(child, root, matrix @ child_matrix, offset + matrix @ child_offset, out)
        elif kind in ("3", "4"):
            count = int(kind)
            values = np.asarray([float(v) for v in fields[2 : 2 + count * 3]], float).reshape(count, 3)
            points = [offset + matrix @ point for point in values]
            out.append(points)


@lru_cache(maxsize=32)
def ldraw_faces(design_id: str, root: str | Path = D.LDRAW_ROOT):
    """Return flattened faces in metres with Z up and the part bottom at Z=0."""
    root = Path(root)
    faces = []
    _flatten(root / "parts" / f"{design_id}.dat", root, np.eye(3), np.zeros(3), faces)
    converted = []
    # LDraw: X right, Y down, Z toward the viewer.  Demo: X right, Y forward, Z up.
    for face in faces:
        converted.append(
            [np.asarray((p[0], -p[2], -p[1]), float) * LDRAW_UNIT_M for p in face]
        )
    bottom = min(point[2] for face in converted for point in face)
    return tuple(tuple(tuple((point - (0, 0, bottom)).tolist()) for point in face) for face in converted)


def ldraw_bounds(design_id: str):
    points = np.asarray([p for face in ldraw_faces(design_id) for p in face], float)
    return points.min(axis=0), points.max(axis=0)


def _colour_name(colour: str) -> str:
    return {
        "black": "black_plastic",
        "bright_red": "plastic_red",
        "dark_stone_grey": "fixture_grey",
    }[colour]


def _visual_mesh(stage, path: str, design_id: str, material: str):
    from pxr import Gf, UsdGeom
    import cell

    faces = ldraw_faces(design_id)
    points = []
    counts = []
    indices = []
    for face in faces:
        first = len(points)
        points.extend(Gf.Vec3f(*point) for point in face)
        counts.append(len(face))
        indices.extend(range(first, first + len(face)))
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(points)
    mesh.CreateFaceVertexCountsAttr(counts)
    mesh.CreateFaceVertexIndicesAttr(indices)
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.CreateDoubleSidedAttr(True)
    cell.bind_mat(mesh.GetPrim(), material)
    return mesh.GetPrim()


def _box_collider(stage, path, size, centre):
    from pxr import Gf, UsdGeom, UsdPhysics
    import cell

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(1.0)
    xf = UsdGeom.Xformable(cube.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*centre))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))
    UsdGeom.Imageable(cube.GetPrim()).CreateVisibilityAttr().Set("invisible")
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    cell.small_contact_offsets(cube.GetPrim())
    cell.bind_physics_material(cube.GetPrim(), cell.PHYS_MAT_PATH)
    return cube.GetPrim()


def _cylinder_collider(stage, path, radius, height, centre, axis="Z"):
    from pxr import Gf, UsdGeom, UsdPhysics
    import cell

    cylinder = UsdGeom.Cylinder.Define(stage, path)
    cylinder.CreateAxisAttr(axis)
    cylinder.CreateRadiusAttr(radius)
    cylinder.CreateHeightAttr(height)
    UsdGeom.Xformable(cylinder.GetPrim()).AddTranslateOp().Set(Gf.Vec3d(*centre))
    UsdGeom.Imageable(cylinder.GetPrim()).CreateVisibilityAttr().Set("invisible")
    UsdPhysics.CollisionAPI.Apply(cylinder.GetPrim())
    cell.small_contact_offsets(cylinder.GetPrim())
    cell.bind_physics_material(cylinder.GetPrim(), cell.PHYS_MAT_PATH)
    return cylinder.GetPrim()


def _shell_boxes(width: float, depth: float, height: float):
    """Return a rigid perimeter shell with a hollow anti-stud volume."""
    allowance = 0.0003
    wall = 0.0008
    web = 0.0008
    outer_x = width - allowance
    outer_y = depth - allowance
    cavity_height = height - web
    return [
        ((outer_x, outer_y, web), (0.0, 0.0, height - web / 2)),
        ((wall, outer_y, cavity_height), (-(outer_x - wall) / 2, 0.0, cavity_height / 2)),
        ((wall, outer_y, cavity_height), ((outer_x - wall) / 2, 0.0, cavity_height / 2)),
        ((outer_x - 2 * wall, wall, cavity_height), (0.0, -(outer_y - wall) / 2, cavity_height / 2)),
        ((outer_x - 2 * wall, wall, cavity_height), (0.0, (outer_y - wall) / 2, cavity_height / 2)),
    ]


def _collision_boxes(design_id: str):
    p, h = D.LEGO_PITCH, D.PLATE_HEIGHT
    if design_id in {"3023", "15573", "99563"}:
        return _shell_boxes(2 * p, p, h)
    if design_id == "33909":
        return _shell_boxes(2 * p, 2 * p, h)
    if design_id == "99207":
        # The official LDraw wall is on local +Y.  The old proxy put it on
        # -Y, so visual and physical orientation differed by 180 degrees.
        return _shell_boxes(2 * p, p, h) + [
            ((2 * p - 0.0003, 0.0013, 0.0158), (0.0, 0.00475, 0.0079)),
        ]
    if design_id == "49132":
        # Official alternate LDraw ID 85943: a 16 x 8 mm main brick with two
        # separated liftarms extending local +Y. Preserve the central U gap.
        return _shell_boxes(2 * p, p, D.BRICK_HEIGHT) + [
            ((p / 2 - 0.0003, 0.0116, 0.0076), (-0.006, 0.0098, 0.0058)),
            ((p / 2 - 0.0003, 0.0116, 0.0076), (0.006, 0.0098, 0.0058)),
        ]
    raise KeyError(design_id)


def _collision_studs(design_id: str):
    """Return ``(axis, centre)`` for every exposed mating stud."""
    p = D.LEGO_PITCH
    top = {
        "3023": [("Z", (x, 0.0, D.PLATE_HEIGHT + D.STUD_HEIGHT / 2)) for x in (-p / 2, p / 2)],
        "15573": [("Z", (0.0, 0.0, D.PLATE_HEIGHT + D.STUD_HEIGHT / 2))],
        "33909": [("Z", (x, -p / 2, D.PLATE_HEIGHT + D.STUD_HEIGHT / 2)) for x in (-p / 2, p / 2)],
        "49132": [("Z", (x, 0.0, D.BRICK_HEIGHT + D.STUD_HEIGHT / 2)) for x in (-p / 2, p / 2)],
    }
    if design_id == "99207":
        studs = [("Z", (x, 0.0, D.PLATE_HEIGHT + D.STUD_HEIGHT / 2)) for x in (-p / 2, p / 2)]
        studs.extend(
            ("Y", (x, 0.0056 + D.STUD_HEIGHT / 2, z))
            for z in (0.004, 0.012)
            for x in (-p / 2, p / 2)
        )
        return studs
    return top.get(design_id, [])


def _socket_centres(design_id: str):
    """Nominal underside anti-stud centres in the part-local XY plane."""
    p = D.LEGO_PITCH
    if design_id in {"3023", "15573", "99563", "99207", "49132"}:
        return [(x, 0.0) for x in (-p / 2, p / 2)]
    if design_id == "33909":
        return [(x, y) for x in (-p / 2, p / 2) for y in (-p / 2, p / 2)]
    raise KeyError(design_id)


def _collision_sockets(design_id: str):
    """Return four rigid guide walls around every underside receiver."""
    wall = 0.00045
    inner_half = D.STUD_DIAMETER / 2 + 0.00005
    outer_span = 2 * (inner_half + wall)
    height = D.STUD_HEIGHT
    result = []
    for socket_number, (socket_x, socket_y) in enumerate(_socket_centres(design_id)):
        offset = inner_half + wall / 2
        result.extend(
            [
                (f"Socket{socket_number:02d}_WallMinX", (wall, outer_span, height), (socket_x - offset, socket_y, height / 2)),
                (f"Socket{socket_number:02d}_WallMaxX", (wall, outer_span, height), (socket_x + offset, socket_y, height / 2)),
                (f"Socket{socket_number:02d}_WallMinY", (outer_span, wall, height), (socket_x, socket_y - offset, height / 2)),
                (f"Socket{socket_number:02d}_WallMaxY", (outer_span, wall, height), (socket_x, socket_y + offset, height / 2)),
            ]
        )
    return result


def add_part(stage, instance: dict, asset: dict, spawn_xyz, spawn_rpy_deg):
    """Create one dynamic LEGO part and return its USD root path."""
    from pxr import Gf, UsdGeom, UsdPhysics
    import cell

    root_path = "/World/LegoParts/" + instance["id"]
    root = UsdGeom.Xform.Define(stage, root_path)
    xf = UsdGeom.Xformable(root.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(*spawn_xyz))
    xf.AddRotateXYZOp().Set(Gf.Vec3f(*spawn_rpy_deg))
    material = _colour_name(asset["colour"])
    if "ldraw" in asset:
        _visual_mesh(
            stage,
            root_path + "/Visual",
            asset.get("ldraw_design_id", instance["design_id"]),
            material,
        )
    else:
        raise ValueError(f"no visual builder for {instance['design_id']}")

    for number, (size, centre) in enumerate(_collision_boxes(instance["design_id"])):
        _box_collider(stage, root_path + f"/Collision/Part{number:02d}", size, centre)
    for number, (axis, centre) in enumerate(_collision_studs(instance["design_id"])):
        _cylinder_collider(
            stage,
            root_path + f"/Collision/Stud{number:02d}",
            D.STUD_DIAMETER / 2,
            D.STUD_HEIGHT,
            centre,
            axis=axis,
        )
    for name, size, centre in _collision_sockets(instance["design_id"]):
        _box_collider(
            stage,
            root_path + "/Collision/" + name,
            size,
            centre,
        )
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim()).CreateRigidBodyEnabledAttr().Set(True)
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(float(asset["mass_kg"]))
    return root_path

