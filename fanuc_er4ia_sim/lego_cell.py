"""Dual-arm workcell composition for the locked LEGO 77243 slice."""

from __future__ import annotations

import math

import lego_demo_config as D
from lego_plan import load_plan

OVERVIEW_CAM_PATH = "/World/LegoOverviewCam"
SIDE_CAM_PATH = "/World/LegoSideCam"


def _spawn_pose(index: int, part: dict):
    """Deterministic keyed tray pose; no random-bin perception is claimed."""
    column = index % D.TRAY_COLUMNS
    row = index // D.TRAY_COLUMNS
    x = D.TRAY_ORIGIN[0] + column * D.TRAY_PITCH[0]
    y = D.TRAY_ORIGIN[1] + row * D.TRAY_PITCH[1]
    feeder = part["feeder"]
    feeder_clearance = float(feeder["clearance_mm"]) / 1000
    z = D.SURFACE_Z + D.TRAY_POCKET[2] + feeder_clearance
    return (x, y, z), tuple(float(value) for value in feeder["rpy_deg"])


def _add_fixture(stage):
    from pxr import Gf, UsdGeom, UsdPhysics
    import cell

    top = D.FIXTURE_DATUM[2]
    centre = (D.FIXTURE_DATUM[0], D.FIXTURE_DATUM[1], top - D.FIXTURE_SIZE[2] / 2)
    base = cell.box(
        stage,
        "/World/LegoFixture/Base",
        D.FIXTURE_SIZE,
        centre,
        (0.12, 0.13, 0.15),
        mat="fixture_grey",
    )
    cell.bind_physics_material(base)
    # This removable support insert locates the seed and is retracted by the
    # executor before the negative-Z underside mate in step 76.
    # Three shallow walls form a deterministic, open-front nest.  They locate
    # the seed bracket while leaving the two arms and side insertions clear.
    wall_t, wall_h = 0.003, 0.014
    for name, size, offset in (
        ("Back", (0.052, wall_t, wall_h), (0.0, -0.026, wall_h / 2)),
        ("Left", (wall_t, 0.052, wall_h), (-0.026, 0.0, wall_h / 2)),
        ("Right", (wall_t, 0.052, wall_h), (0.026, 0.0, wall_h / 2)),
    ):
        cell.box(
            stage,
            "/World/LegoFixture/Wall" + name,
            size,
            (D.FIXTURE_DATUM[0] + offset[0], D.FIXTURE_DATUM[1] + offset[1], top + offset[2]),
            (0.16, 0.17, 0.19),
            mat="fixture_grey",
        )
    hx, hy = D.FIXTURE_HANDLE_XY
    hs = D.FIXTURE_HANDLE_SIZE
    cell.box(
        stage,
        "/World/LegoFixture/SupportHandle",
        hs,
        (hx, hy, D.SURFACE_Z + hs[2] / 2),
        (0.07, 0.08, 0.09),
        mat="black_plastic",
    )


def _add_trays(stage, parts):
    import cell

    for index, part in enumerate(parts):
        xyz, _ = _spawn_pose(index, part)
        sx, sy, sz = D.TRAY_POCKET
        cell.box(
            stage,
            f"/World/LegoTrays/Pocket{index:02d}",
            (sx, sy, sz),
            (xyz[0], xyz[1], D.SURFACE_Z + sz / 2),
            (0.10, 0.11, 0.13),
            mat="tray_dark",
        )
        feeder = part["feeder"]
        support = feeder["support"]
        if support.startswith(("raised_spread_", "inverted_spread_")):
            # Two keyed rails make the declared dynamic support height real
            # while leaving the selected Rev B pad approach faces open.  An
            # inverted part's root clearance describes its authored pose, not
            # the physical rail height, so those values remain independently
            # declarative in the feeder frame.
            rail_height_mm = (
                feeder["rail_height_mm"]
                if support.startswith("inverted_spread_")
                else feeder.get("rail_height_mm", feeder["clearance_mm"])
            )
            rail_h = float(rail_height_mm) / 1000
            rail_width = float(
                feeder.get("rail_width_mm", 1.0)
            ) / 1000
            rail_offset = float(
                feeder.get("rail_offset_mm", 2.5)
            ) / 1000
            spread_axis = support.rsplit("_", 1)[-1]
            rail_specs = (("Near", -rail_offset), ("Far", rail_offset))
            for side, offset in rail_specs:
                centre = (
                    (xyz[0] + offset, xyz[1])
                    if spread_axis == "y"
                    else (xyz[0], xyz[1] + offset)
                )
                rail_size = (
                    (0.004, rail_width, rail_h)
                    if spread_axis == "y"
                    else (rail_width, 0.004, rail_h)
                )
                cell.box(
                    stage,
                    f"/World/LegoTrays/Pocket{index:02d}_Riser{side}",
                    rail_size,
                    (
                        centre[0],
                        centre[1],
                        D.SURFACE_Z + sz + rail_h / 2,
                    ),
                    (0.13, 0.14, 0.16),
                    mat="tray_dark",
                )


def _add_connector_seats(stage, plan):
    """Pre-author disabled physical seating nests declared by AssemblyIR.

    The coarse LEGO body proxies cannot represent hollow anti-studs.  A thin
    seat and shallow footprint guide are therefore the explicit mechanical
    stud/anti-stud clearance abstraction. The executor enables the nest only
    at the measured plunger standoff; it never moves a part or supplies tensile
    connector retention, and remains engaged after the connector lock.
    """
    from pxr import Sdf, UsdGeom, UsdPhysics
    import cell
    from lego_operation_profiles import body_target_offset, handling_profile

    UsdGeom.Scope.Define(stage, "/World/LegoConnectorGuides")
    parts = {part["id"]: part for part in plan["parts"]}
    thickness = 0.0004
    for task in plan["tasks"]:
        seating = task.get("seating")
        if not seating:
            continue
        part = parts[task["part"]]
        target = task["target"]
        profile = handling_profile(part, task)
        body_offset = body_target_offset(part)
        gate_offset = profile.get("pose_gate_offset_m", (0.0, 0.0, 0.0))
        gate = [
            D.FIXTURE_DATUM[index]
            + float(target["xyz_mm"][index]) / 1000
            + float(body_offset[index])
            + float(gate_offset[index])
            for index in range(3)
        ]
        rx, ry, rz = (math.radians(float(value)) for value in target["rpy_deg"])
        cx, sx = math.cos(rx), math.sin(rx)
        cy, sy = math.cos(ry), math.sin(ry)
        cz, sz = math.cos(rz), math.sin(rz)
        # Columns of Rz @ Ry @ Rx are the desired part-local axes.
        axis_x = (cz * cy, sz * cy, -sy)
        axis_y = (
            cz * sy * sx - sz * cx,
            sz * sy * sx + cz * cx,
            cy * sx,
        )
        normal = (
            cz * sy * cx + sz * sx,
            sz * sy * cx - cz * sx,
            cy * cx,
        )
        centre = tuple(gate[index] - normal[index] * thickness / 2 for index in range(3))
        patch = seating["seat_patch_mm"]
        root_path = f"/World/LegoConnectorGuides/{task['id']}"
        UsdGeom.Scope.Define(stage, root_path)
        filtered_targets = [
            Sdf.Path("/World/LegoParts/" + part_id)
            for part_id in task.get("collision_filter_parts", [])
        ]

        def nest_box(name, size, box_centre, role):
            prim = cell.box(
                stage,
                root_path + "/" + name,
                size,
                box_centre,
                (0.10, 0.25, 0.12),
                rot=tuple(float(value) for value in target["rpy_deg"]),
                mat="fixture_grey",
            )
            UsdGeom.Imageable(prim).CreateVisibilityAttr().Set("invisible")
            UsdPhysics.CollisionAPI(prim).CreateCollisionEnabledAttr().Set(False)
            UsdPhysics.FilteredPairsAPI.Apply(prim).CreateFilteredPairsRel().SetTargets(
                filtered_targets
            )
            cell.small_contact_offsets(prim)
            cell.bind_physics_material(prim)
            prim.CreateAttribute("assembly:task", Sdf.ValueTypeNames.String).Set(task["id"])
            prim.CreateAttribute("assembly:role", Sdf.ValueTypeNames.String).Set(role)

        nest_box(
            "Seat",
            (float(patch[0]) / 1000, float(patch[1]) / 1000, thickness),
            centre,
            "connector_axial_stop",
        )

        min_x, max_x, min_y, max_y = (
            float(value) / 1000 for value in seating["guide_bounds_part_mm"]
        )
        wall = float(seating["guide_wall_mm"]) / 1000
        height = float(seating["guide_height_mm"]) / 1000
        centre_x = (min_x + max_x) / 2
        centre_y = (min_y + max_y) / 2
        span_x = max_x - min_x
        span_y = max_y - min_y

        def local_to_world(local_x, local_y, local_z):
            return tuple(
                gate[index]
                + axis_x[index] * local_x
                + axis_y[index] * local_y
                + normal[index] * local_z
                for index in range(3)
            )

        for name, size, local_centre in (
            (
                "WallMinX",
                (wall, span_y + 2 * wall, height),
                (min_x - wall / 2, centre_y, height / 2),
            ),
            (
                "WallMaxX",
                (wall, span_y + 2 * wall, height),
                (max_x + wall / 2, centre_y, height / 2),
            ),
            (
                "WallMinY",
                (span_x, wall, height),
                (centre_x, min_y - wall / 2, height / 2),
            ),
            (
                "WallMaxY",
                (span_x, wall, height),
                (centre_x, max_y + wall / 2, height / 2),
            ),
        ):
            nest_box(
                name,
                size,
                local_to_world(*local_centre),
                "connector_lateral_clearance",
            )


def build(stage):
    """Build two namespaced arms, fixture, keyed trays, parts, and review cameras."""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics
    import cell
    from lego_parts import add_part

    plan = load_plan()
    arm_a, arm_b = D.ARM_SPECS
    cell.build(
        stage,
        with_objects=False,
        robot_prim=arm_a["prim_path"],
        robot_position=arm_a["base_position"],
        finger_style=D.WORKER_FINGER_STYLE,
    )
    cell.add_robot(
        stage,
        arm_b["prim_path"],
        arm_b["base_position"],
        arm_b["base_yaw_deg"],
    )
    UsdGeom.Scope.Define(stage, "/World/LegoParts")
    UsdGeom.Scope.Define(stage, "/World/LegoTrays")
    UsdGeom.Scope.Define(stage, "/World/LegoFixture")
    _add_fixture(stage)
    _add_trays(stage, plan["parts"])

    assets = plan["asset_manifest"]["parts"]
    for index, instance in enumerate(plan["parts"]):
        xyz, rpy = _spawn_pose(index, instance)
        add_part(stage, instance, assets[instance["design_id"]], xyz, rpy)
    # Connector clearance belongs to each rigid LEGO compound collider.  Do
    # not author world-static seating guides: they can hide a wrong target or
    # hold a part after the real mate has missed.

    # Arm B can take ownership of the root bracket for a measured, no-jump
    # turnover handoff.  Pre-author both body relationships and all four local
    # frame attributes while the joint is disabled; the executor replaces the
    # identity frames with measured coincident frames before enabling it.
    turnover_joint = UsdPhysics.FixedJoint.Define(
        stage, "/World/LegoAssemblyTurnover"
    )
    turnover_joint.CreateBody0Rel().SetTargets(
        [Sdf.Path(arm_b["prim_path"] + "/JawL")]
    )
    turnover_joint.CreateBody1Rel().SetTargets(
        [Sdf.Path("/World/LegoParts/p74_bracket")]
    )
    turnover_joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
    turnover_joint.CreateLocalRot0Attr().Set(
        Gf.Quatf(1, Gf.Vec3f(0, 0, 0))
    )
    turnover_joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    turnover_joint.CreateLocalRot1Attr().Set(
        Gf.Quatf(1, Gf.Vec3f(0, 0, 0))
    )
    turnover_joint.CreateBreakForceAttr().Set(float("inf"))
    turnover_joint.CreateBreakTorqueAttr().Set(float("inf"))
    turnover_joint.CreateJointEnabledAttr().Set(False)

    # Pre-author disabled grasp joints so PhysX discovers both body relations
    # during reset.  The executor fills the no-jump local frame and enables a
    # joint only after a closed-jaw dwell, then disables it before release.
    UsdGeom.Scope.Define(stage, "/World/LegoGraspConstraints")
    for task in plan["tasks"]:
        joint = UsdPhysics.FixedJoint.Define(stage, "/World/LegoGraspConstraints/" + task["id"])
        joint.CreateBody0Rel().SetTargets([Sdf.Path(arm_a["prim_path"] + "/JawL")])
        joint.CreateBody1Rel().SetTargets([Sdf.Path("/World/LegoParts/" + task["part"])])
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        joint.CreateJointEnabledAttr().Set(False)

    # Connector locks are also discovered before the timeline starts. Creating
    # a joint prim live and then authoring its two body frames over several USD
    # notices can expose PhysX to a momentarily incomplete, enabled joint. The
    # executor fills these disabled joints from the measured passing pose and
    # enables each one only after every relation and local frame is complete.
    UsdGeom.Scope.Define(stage, "/World/LegoConnectorLocks")
    for task in plan["tasks"]:
        joint = UsdPhysics.FixedJoint.Define(
            stage, "/World/LegoConnectorLocks/" + task["id"]
        )
        parent_id = task.get("connector_parent")
        if parent_id is not None:
            joint.CreateBody0Rel().SetTargets(
                [Sdf.Path("/World/LegoParts/" + parent_id)]
            )
        joint.CreateBody1Rel().SetTargets(
            [Sdf.Path("/World/LegoParts/" + task["part"])]
        )
        joint.CreateLocalPos0Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot0Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
        joint.CreateLocalRot1Attr().Set(Gf.Quatf(1, Gf.Vec3f(0, 0, 0)))
        joint.CreateBreakForceAttr().Set(float("inf"))
        joint.CreateBreakTorqueAttr().Set(float("inf"))
        joint.CreateJointEnabledAttr().Set(False)

    # Fixed cameras are intentionally independent of either arm articulation.
    cell.camera(
        stage,
        OVERVIEW_CAM_PATH,
        {
            "eye": (0.78, -0.70, D.SURFACE_Z + 0.80),
            "look_at": (0.29, -0.02, D.SURFACE_Z + 0.03),
            "up": (0.0, 0.0, 1.0),
            "resolution": (1920, 1080),
            "focal_length": 15.0,
            "horiz_aperture": 20.955,
        },
    )
    cell.camera(
        stage,
        SIDE_CAM_PATH,
        {
            "eye": (0.39, 0.66, D.SURFACE_Z + 0.20),
            "look_at": (0.30, 0.0, D.SURFACE_Z + 0.04),
            "up": (0.0, 0.0, 1.0),
            "resolution": (1920, 1080),
            "focal_length": 14.0,
            "horiz_aperture": 20.955,
        },
    )
    return plan

