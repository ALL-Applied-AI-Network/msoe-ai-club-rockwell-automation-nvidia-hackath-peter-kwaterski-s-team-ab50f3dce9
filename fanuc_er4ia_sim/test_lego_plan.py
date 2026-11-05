import json
import unittest
from pathlib import Path

import numpy as np

import lego_demo_config as D
from lego_parts import (
    _collision_boxes,
    _collision_sockets,
    _collision_studs,
    _socket_centres,
    ldraw_bounds,
    ldraw_faces,
)
from lego_plan import load_plan, run_manifest
from lego_operation_profiles import GRASP_PROFILES, handling_profile


class LegoPlanTests(unittest.TestCase):
    def test_locked_slice_is_complete_and_source_indexed(self):
        plan = load_plan()
        self.assertEqual(plan["set_number"], "77243")
        self.assertEqual(len(plan["parts"]), 10)
        self.assertEqual(len(plan["tasks"]), 10)
        self.assertEqual(len(plan["booklet_index"]["steps"]), 106)
        self.assertEqual(
            [row["pdf_page"] for row in plan["booklet_index"]["steps"][73:79]],
            [77, 77, 77, 78, 78, 79],
        )
        self.assertEqual(
            [row["geometry_status"] for row in plan["booklet_index"]["steps"][73:79]],
            [
                "slice_executable",
                "appearance_only",
                "slice_executable",
                "slice_executable",
                "slice_executable",
                "slice_executable",
            ],
        )

    def test_plan_is_a_topologically_ordered_connection_graph(self):
        plan = load_plan()
        seen = set()
        for task in plan["tasks"]:
            self.assertTrue(set(task["prerequisites"]).issubset(seen))
            seen.add(task["id"])
        self.assertEqual({p["id"] for p in plan["parts"]}, {t["part"] for t in plan["tasks"]})
        self.assertEqual({t["operation"] for t in plan["tasks"]}, {"fixture_seed", "top_mate", "underside_mate", "side_mate"})
        part_ids = {part["id"] for part in plan["parts"]}
        for task in plan["tasks"]:
            self.assertTrue(
                task["connector_parent"] is None
                or task["connector_parent"] in part_ids
            )
            self.assertTrue(set(task["collision_filter_parts"]).issubset(part_ids))
            self.assertEqual(task["collision_filter_parts"], [])
            self.assertTrue(set(task["mates"]).issubset(part_ids))
            if task["connector_parent"] is not None:
                self.assertIn(task["connector_parent"], task["mates"])

    def test_executor_profiles_are_selected_without_task_ids(self):
        plan = load_plan()
        by_part = {part["id"]: part for part in plan["parts"]}
        self.assertTrue(
            {part["grasp_profile"] for part in plan["parts"]}.issubset(GRASP_PROFILES)
        )
        first_tile = handling_profile(by_part["p74_tile"], plan["tasks"][1])
        second_tile = handling_profile(by_part["p77_tile"], plan["tasks"][4])
        self.assertEqual(first_tile, second_tile)
        self.assertEqual(first_tile["name"], "pad_x_16_standard")
        executor = (Path(__file__).parent / "lego_scene.py").read_text(encoding="utf-8")
        for task in plan["tasks"]:
            self.assertNotIn(f'"{task["id"]}"', executor)
            self.assertNotIn(f"'{task['id']}'", executor)

    def test_feeder_orientation_and_grasp_axis_are_declarative(self):
        plan = load_plan()
        by_part = {part["id"]: part for part in plan["parts"]}
        tasks = {task["id"]: task for task in plan["tasks"]}
        half_beam = by_part["p76_half_beam"]
        profile = handling_profile(half_beam, plan["tasks"][3])
        self.assertEqual(half_beam["feeder"]["rpy_deg"], [0, 180, -90])
        self.assertEqual(profile["spread_part"], (1.0, 0.0, 0.0))
        self.assertEqual(profile["approach_axis_part"], (0.0, 0.0, -1.0))
        self.assertLessEqual(profile["carry_clearance_m"], 0.025)
        self.assertEqual(profile["preinsert_clearance_m"], 0.008)
        self.assertEqual(
            profile["closed_loop_approach_stages_m"],
            (0.018, 0.012, 0.008),
        )
        self.assertEqual(
            by_part["p76_red_plate"]["feeder"]["rpy_deg"],
            [0, 180, -90],
        )
        self.assertEqual(
            by_part["p77_bracket"]["feeder"]["rpy_deg"],
            [0, 180, 180],
        )
        for part_id in ("p76_red_plate", "p76_half_beam", "p77_bracket"):
            feeder = by_part[part_id]["feeder"]
            self.assertEqual(feeder["presentation"], "inverted_target")
            self.assertTrue(feeder["support"].startswith("inverted_spread_"))
            self.assertGreater(float(feeder["rail_height_mm"]), 0.0)
            self.assertIn("clearance_cradle", feeder["support_role"])
        self.assertGreaterEqual(by_part["p76_red_plate"]["feeder"]["clearance_mm"], 5.0)
        self.assertGreaterEqual(by_part["p76_half_beam"]["feeder"]["clearance_mm"], 11.2)
        self.assertEqual(
            by_part["p76_half_beam"]["feeder"]["pick_tip_from_part_root_z_mm"],
            3.0,
        )
        self.assertGreaterEqual(by_part["p77_bracket"]["feeder"]["clearance_mm"], 16.0)
        self.assertEqual(tasks["t02"]["before_actions"], ["present_underside"])
        self.assertNotIn("before_actions", tasks["t03"])
        self.assertEqual(tasks["t04"]["before_actions"], ["restore_upright_hold"])
        self.assertEqual(tasks["t05"]["before_actions"], ["present_underside"])
        self.assertEqual(
            tasks["t06"]["before_actions"], ["restore_upright_and_dock"]
        )
        self.assertTrue(
            all(
                "retract_fixture_base" not in task.get("before_actions", [])
                for task in plan["tasks"]
            )
        )
        self.assertNotIn("loaded_wrist_profile", profile)
        self.assertNotIn("carry_tracking", profile)

    def test_targets_match_the_booklet_body_frames(self):
        plan = load_plan()
        actual = {
            task["id"]: (task["target"]["xyz_mm"], task["target"]["rpy_deg"])
            for task in plan["tasks"]
        }
        expected = {
            "t00": ([0, 0, 0], [0, 0, 0]),
            "t01": ([0, -4, 3.2], [0, 0, 0]),
            "t02": ([4, -12, 0], [0, 0, 90]),
            "t03": ([-4, -12, -6.4], [0, 0, 90]),
            "t04": ([0, -20, 3.2], [0, 0, 180]),
            "t05": ([0, -24, 0], [0, 0, 180]),
            "t06": ([-4, -12, 6.4], [0, 0, 90]),
            "t07": ([4, -12, 6.4], [0, 0, 90]),
            "t08": ([4, -29.6, 8], [90, -90, 0]),
            "t09": ([-4, 5.6, 8], [90, -90, 180]),
        }
        self.assertEqual(actual, expected)
        self.assertEqual(set(plan["tasks"][4]["mates"]), {"p76_red_plate", "p76_half_beam"})
        self.assertEqual(set(plan["tasks"][6]["mates"]), {"p74_tile", "p77_tile"})

    def test_lego_colliders_use_object_not_soft_tip_material(self):
        source = (Path(__file__).parent / "lego_parts.py").read_text(encoding="utf-8")
        self.assertNotIn("SOFT_TIP_MAT_PATH", source)

    def test_rectangular_collision_axes_match_ldraw(self):
        for design_id in ("3023", "15573", "99563"):
            low, high = ldraw_bounds(design_id)
            collision_size = np.asarray(_collision_boxes(design_id)[0][0])
            self.assertAlmostEqual(collision_size[0], high[0] - low[0], delta=0.0004)
            self.assertAlmostEqual(collision_size[1], high[1] - low[1], delta=0.0004)

    def test_collision_geometry_has_hollow_receivers_and_real_studs(self):
        for design_id in ("3023", "33909", "15573", "99563", "99207", "49132"):
            boxes = _collision_boxes(design_id)
            self.assertGreaterEqual(len(boxes), 5)
            # A top web plus separate skirts replaces the forbidden solid slab.
            self.assertLess(boxes[0][0][2], D.PLATE_HEIGHT)
        self.assertEqual(len(_collision_studs("3023")), 2)
        self.assertEqual(len(_collision_studs("15573")), 1)
        self.assertEqual(len(_collision_studs("33909")), 2)
        bracket_studs = _collision_studs("99207")
        self.assertEqual(sum(axis == "Z" for axis, _ in bracket_studs), 2)
        self.assertEqual(sum(axis == "Y" for axis, _ in bracket_studs), 4)
        bracket_wall = _collision_boxes("99207")[5]
        self.assertGreater(bracket_wall[1][1], 0.0)
        self.assertEqual(len(_collision_sockets("33909")), 16)
        self.assertEqual(len(_collision_sockets("99563")), 8)
        executor = (Path(__file__).parent / "lego_scene.py").read_text(encoding="utf-8")
        self.assertNotIn("FilteredPairsAPI", executor)

    def test_final_compound_colliders_have_no_positive_volume_overlap(self):
        plan = load_plan()
        parts = {part["id"]: part for part in plan["parts"]}

        def rotation(rpy_deg):
            rx, ry, rz = np.deg2rad(rpy_deg)
            cx, cy, cz = np.cos((rx, ry, rz))
            sx, sy, sz = np.sin((rx, ry, rz))
            return np.array(
                [
                    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                    [-sy, cy * sx, cy * cx],
                ]
            )

        def aabb(size, centre, world_from_part, translation):
            half = np.asarray(size, float) / 2
            corners = np.asarray(
                [
                    [x, y, z]
                    for x in (-half[0], half[0])
                    for y in (-half[1], half[1])
                    for z in (-half[2], half[2])
                ]
            ) + np.asarray(centre, float)
            world = corners @ world_from_part.T + translation
            return world.min(axis=0), world.max(axis=0)

        colliders = []
        for task in plan["tasks"]:
            part = parts[task["part"]]
            transform = rotation(task["target"]["rpy_deg"])
            translation = np.asarray(task["target"]["xyz_mm"], float) / 1000
            for number, (size, centre) in enumerate(_collision_boxes(part["design_id"])):
                colliders.append(
                    (part["id"], f"Part{number:02d}", *aabb(size, centre, transform, translation))
                )
            for number, (axis, centre) in enumerate(_collision_studs(part["design_id"])):
                size = {
                    "X": (D.STUD_HEIGHT, D.STUD_DIAMETER, D.STUD_DIAMETER),
                    "Y": (D.STUD_DIAMETER, D.STUD_HEIGHT, D.STUD_DIAMETER),
                    "Z": (D.STUD_DIAMETER, D.STUD_DIAMETER, D.STUD_HEIGHT),
                }[axis]
                colliders.append(
                    (part["id"], f"Stud{number:02d}", *aabb(size, centre, transform, translation))
                )
            for name, size, centre in _collision_sockets(part["design_id"]):
                colliders.append(
                    (part["id"], name, *aabb(size, centre, transform, translation))
                )

        overlaps = []
        for index, left in enumerate(colliders):
            for right in colliders[index + 1 :]:
                if left[0] == right[0]:
                    continue
                depth = np.minimum(left[3], right[3]) - np.maximum(left[2], right[2])
                if np.all(depth > 1.0e-6):
                    overlaps.append((left[:2], right[:2], depth.tolist()))
        self.assertEqual(overlaps, [])

    def test_every_declared_mate_has_coaxial_stud_and_socket_geometry(self):
        plan = load_plan()
        parts = {part["id"]: part for part in plan["parts"]}
        tasks = {task["part"]: task for task in plan["tasks"]}

        def rotation(rpy_deg):
            rx, ry, rz = np.deg2rad(rpy_deg)
            cx, cy, cz = np.cos((rx, ry, rz))
            sx, sy, sz = np.sin((rx, ry, rz))
            return np.array(
                [
                    [cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx],
                    [sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx],
                    [-sy, cy * sx, cy * cx],
                ]
            )

        def connectors(part_id):
            task = tasks[part_id]
            design_id = parts[part_id]["design_id"]
            world_from_part = rotation(task["target"]["rpy_deg"])
            translation = np.asarray(task["target"]["xyz_mm"], float) / 1000
            axes = {
                "X": np.array((1.0, 0.0, 0.0)),
                "Y": np.array((0.0, 1.0, 0.0)),
                "Z": np.array((0.0, 0.0, 1.0)),
            }
            studs = [
                (
                    world_from_part @ np.asarray(centre) + translation,
                    world_from_part @ axes[axis],
                )
                for axis, centre in _collision_studs(design_id)
            ]
            sockets = [
                (
                    world_from_part
                    @ np.asarray((x, y, D.STUD_HEIGHT / 2))
                    + translation,
                    world_from_part @ axes["Z"],
                )
                for x, y in _socket_centres(design_id)
            ]
            return studs, sockets

        for task in plan["tasks"]:
            child_studs, child_sockets = connectors(task["part"])
            for mate_id in task["mates"]:
                mate_studs, mate_sockets = connectors(mate_id)
                candidate_pairs = [
                    *(
                        (stud, socket)
                        for stud in child_studs
                        for socket in mate_sockets
                    ),
                    *(
                        (stud, socket)
                        for stud in mate_studs
                        for socket in child_sockets
                    ),
                ]
                aligned = [
                    (np.linalg.norm(stud[0] - socket[0]), abs(np.dot(stud[1], socket[1])))
                    for stud, socket in candidate_pairs
                ]
                self.assertTrue(
                    any(distance < 1.0e-8 and axis_dot > 1 - 1.0e-8 for distance, axis_dot in aligned),
                    (task["id"], mate_id, aligned),
                )

    def test_late_connectors_and_side_axis_are_declarative(self):
        plan = load_plan()
        tasks = {task["id"]: task for task in plan["tasks"]}
        parts = {part["id"]: part for part in plan["parts"]}
        self.assertEqual(tasks["t06"]["connector_parent"], "p77_tile")
        self.assertEqual(tasks["t07"]["connector_parent"], "p77_tile")
        self.assertEqual(tasks["t06"]["collision_filter_parts"], [])
        side = handling_profile(parts[tasks["t08"]["part"]], tasks["t08"])
        self.assertEqual(side["approach_axis_part"], (0.0, 0.0, 1.0))

    def test_ldraw_assets_resolve_at_nominal_lego_scale(self):
        expected_xy = {
            "3023": (0.008, 0.016),
            "33909": (0.016, 0.016),
            "15573": (0.008, 0.016),
            "99563": (0.008, 0.016),
        }
        for design_id, nominal in expected_xy.items():
            faces = ldraw_faces(design_id)
            self.assertGreater(len(faces), 10)
            low, high = ldraw_bounds(design_id)
            extents = np.sort((high - low)[:2])
            self.assertTrue(np.allclose(extents, np.sort(nominal), atol=0.0005), (design_id, extents))
            self.assertGreater((high - low)[2], 0.003)
        halfbeam = load_plan()["asset_manifest"]["parts"]["49132"]
        self.assertEqual(halfbeam["ldraw_design_id"], "85943")
        faces = ldraw_faces(halfbeam["ldraw_design_id"])
        low, high = ldraw_bounds(halfbeam["ldraw_design_id"])
        self.assertGreater(len(faces), 700)
        self.assertTrue(
            np.allclose((high - low) * 1000, (16.0, 19.6, 11.2), atol=0.1)
        )

    def test_run_manifest_pins_all_authoritative_inputs(self):
        manifest = run_manifest(load_plan(), git_hash="abc123")
        self.assertEqual(manifest["manual_sha256"], D.MANUAL_SHA256)
        self.assertEqual(manifest["arm_count"], 2)
        for key in ("asset_manifest_sha256", "slice_sha256", "booklet_index_sha256"):
            self.assertRegex(manifest[key], r"^[0-9a-f]{64}$")
        tool = manifest["worker_fingertip"]
        self.assertFalse(tool["real_mount_verified"])
        self.assertEqual(set(tool["files_sha256"]), set(D.WORKER_FINGER_HASH_FILES))
        for digest in tool["files_sha256"].values():
            self.assertRegex(digest, r"^[0-9a-f]{64}$")


if __name__ == "__main__":
    unittest.main()

