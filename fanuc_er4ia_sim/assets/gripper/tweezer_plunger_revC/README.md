# Rev B-R — independently retracting JawL

Simulation concept derived from Rev B. This revision adds one prismatic DOF for the insert and its rubber pad. It does not assert a physically verified, actuator-ready attachment.

## Start here

- `assembly_pickup.step`: normal Rev B pinch position, plunger retracted, flat shoe.
- `assembly_jaw_retracted.step`: same carrier, opposite jaw and plunger poses; JawL displaced +6 mm in X.
- `assembly_seating_jaw_retracted.step`: JawL at +6 mm, plunger extended 12 mm.
- `configurations/`: all 8 endpoint combinations of base jaws open/closed, plunger retracted/extended, JawL pickup/retracted, for each original round/flat/bridge end (24 STEP assemblies).
- `components/`: each rigid component and rubber pad as STEP.
- `visual_meshes/`: detailed STL for each component.
- `collision_meshes/`: convex OBJ pieces. Use the JSON collision filters.
- `kinematics.json`: active joint definitions, moving-body names and mesh mapping.
- `joint_diagram.png`: root-frame comparison with the independent retraction axis.
- `build_retract.py`: rebuild script; requires CadQuery, NumPy, SciPy and matplotlib.
- `kinematics_revB_reference.json` and `right_finger_insert_revB_reference.step`: reference only; do not load these into the active assembly.

## Name mapping and axis

**JawL is the user's alias for the positive-X/right-hand insert of the CAD**, not the negative-X jaw. The active STEP solid names remain `right_finger_insert` and `right_finger_pad`. Both belong to the new moving group `JawL`. This explicit mapping avoids changing existing cell references.

| Item | Definition |
|---|---|
| Joint name | `JawL_retract` |
| Parent | `right_jaw` (existing carrier motion) |
| Moving body/group | `JawL` |
| Moving solids | `right_finger_insert`, `right_finger_pad` |
| Type / axis | Prismatic, +X = [1,0,0], away from the grasp center |
| Zero | Normal Rev B pickup/pinch position |
| Travel limit | 0..6 mm |
| Minimum intended post-lock release | 4 mm; use 6 mm where the cell allows |
| Root mesh coordinates | Already placed at zero; apply delta translations only |
| Total moving-insert translation | [q_right_jaw + q_JawL_retract, 0, 0] |
| Guide/carrier translation | [q_right_jaw, 0, 0] |
| Opposite jaw dependence on JawL | None |
| Plunger dependence on JawL | None |

The root origin, units and axes remain Rev B: mm; origin midway between closed inner carrier mounting faces at base reference Z=0; X across jaw opening, Y across fingertip width, +Z distal. In a meter-based simulator scale all mesh coordinates and translational joint values by 0.001.

The original left/right base jaw joints (0..6 outward) remain, and the plunger retains its 0..12 +Z joint. `JawL_retract` is **one additional DOF**. Hold the original base-jaw and plunger commands constant to retract only JawL. A hierarchical importer must apply the parent right-jaw translation once, not twice.

## Preserved interfaces and pickup geometry

The existing left carrier, left insert/pad, right carrier, plunger housing/rod and all three original plunger ends are reused from the saved Rev B STEP components. Their zero poses and geometry are retained. Mounting hole centers remain Z=10 and 23, with Rev B's provisional 3.4 mm diameter. The new fixed guide mounts into the same right carrier insert pocket at the same proposed M2 retention centers Z=28 and 33.

At pickup zero, the right tong occupies X=4.1..6.6 and retains its 3.6 mm distal width in Y. Its tip remains Z=60. The original 0.6 mm right rubber pad remains exactly at X=3.5..4.1, Y=-1.5..1.5, Z=43..59.5. The opposing pad remains X=-4.1..-3.5. Thus the normal closed pickup gap is still 7 mm.

After independent retraction, the right inner pad face is X=9.5 at 6 mm travel; the free gap is 13 mm with the original jaws still closed. At 4 mm travel the gap is 11 mm. In general, gap = 7 + q_left_jaw + q_right_jaw + q_JawL_retract. These are geometry gaps, not commanded grip-force setpoints.

The plunger interface, all original shoe positions, and contact face Z=52..64 are unchanged. No new robot bracket or J6 transform is invented. Rev B's real-world mounting assumptions still require measurement.

## Slider / guide geometry

Translating the original insert outward would intersect its carrier. This revision replaces its proximal attachment with a fixed guide and an independently sliding T-section carriage below the carrier:

| Feature | Root-zero dimensions, mm |
|---|---|
| Fixed guide mounting stem | X=4.1..6.6, Y=-3..3, Z=26..36 |
| Fixed guide housing | X=3.8..13.2, Y=-4..4, Z=36..42 |
| Carriage bar, zero | X=4.1..12.9, Y=-2.5..2.5, Z=38..40 |
| Tong below carriage | X=4.1..6.6, Y=-1.8..1.8, Z=40..60 |
| Guide carriage channel | Y=-2.6..2.6, Z=37.9..40.1, open along X |
| Distal slot | Y=-1.9..1.9, Z=40.1..42 |
| Guide/bar axial overlap at 6 mm | 3.1 mm |
| Maximum moving positive-X extent | 18.9 mm at 6 mm independent travel, base jaw closed |
| Maximum with base right jaw also open | 24.9 mm |

The guide captures the bar in Y/Z while allowing +X motion. The bar and tong are one rigid moving solid. The short 3.1 mm end-of-travel overlap is a simulation geometry allowance, not a verified bearing/load design. Check the increased guide and outward carriage envelope against the real cell.

Screw hardware, actuator, bearings, lubrication, preload, physical travel stops and sensors are not modeled. The JSON joint limits supply simulation stops only; an actual mechanism requires positive stops and a retention/assembly method. This is not a claim that the existing gripper can actuate this extra DOF without added hardware.

## Initial controller tuning targets

| Parameter | Provisional starting value |
|---|---|
| Initial retraction speed | 5 mm/s |
| Simulation speed cap | 10 mm/s |
| Initial acceleration | 20 mm/s^2 |
| Initial force cap | 2 N |
| Provisional upper force cap | 5 N |

These are suggested simulation/commissioning starting settings, **not validated hardware limits or a guarantee that the mechanism will move at those forces**. Friction, effective mass, actuator response, part retention and structural capacity have not been measured. Do not confuse these retraction values with Rev B's separate plunger seating-force target.

Sequence: retain normal pinch for pickup; align and seat; verify connector lock; relax grip force while maintaining the commanded base-jaw pose; command JawL_retract toward 4–6 mm while holding plunger and opposite jaw; stop on unexpected resistance; complete the release/withdrawal sequence. Do not use the retraction actuator to pry an unlatched part loose. Reset JawL to zero in free space before the next pickup.

## Collision and validation

Use the detailed STLs for visuals. Convex OBJ pieces are exported separately: two pieces for the moving bar/tong, six for the new fixed guide/stem, and six for the original plunger housing. Other bodies use convex hull proxies. Hulls fill holes and small recesses. JSON disables collisions within rigid groups and for the constrained guide/slider and rod/housing pairs. External workpiece and opposite-jaw collisions remain active.

All 14 component solids are valid single solids. All 24 endpoint STEP assemblies were re-imported and checked. Exact CAD interference checks also cover five intermediate retraction values for each original plunger end: 39 configurations total, with no positive-volume internal interference above 1e-6 mm^3. The tests do not include the real robot, gripper body, LEGO workpieces or nearby red plate and are not a continuous-motion proof.

The optional offset stud-cup pusher is **not included in this revision**. The three original Rev B plunger ends are retained unchanged so the independently retracting jaw can be integrated first.

## Rebuild

Run `python build_retract.py` inside the extracted kit. It uses the included component STEP files and Rev B reference JSON. On first authoring only, the script has a source-copy fallback for the original workspace; a delivered kit already contains every required input and does not need that fallback. Rebuilding regenerates geometry, assemblies, joint metadata, meshes, diagram and validation. Update this README if changing dimensions or controller targets.
