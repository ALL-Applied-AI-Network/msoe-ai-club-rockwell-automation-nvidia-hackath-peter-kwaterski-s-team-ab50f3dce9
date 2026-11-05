# Tweezer + center plunger — Rev B

**Simulation concept. Not a physically verified attachment.** Built from the Rev A mounting dimensions and the supplied Rev B requirements. No actual actuator, robot mounting bracket, gripper body, J6 transform, motor, pneumatic seals, sensors, screw hardware or real spring is represented.

## Files and quick start

- `assembly_retracted.step`: jaws closed, flat shoe, plunger at zero.
- `assembly_extended.step`: jaws closed, flat shoe, plunger at 12 mm.
- `assembly_closed_retracted.step`, `assembly_closed_extended.step`, `assembly_open_retracted.step`, `assembly_open_extended.step`: the four requested configurations with the flat shoe.
- `configurations/`: all four configurations for each of the round, flat and bridge ends (12 STEP assemblies total).
- `components/`: an individual STEP file for every rigid component and every compliant pad, including distinct left/right files.
- `visual_meshes/`: matching detailed STL files for all 13 components, tessellation tolerance 0.015 mm.
- `collision_meshes/`: convex OBJ proxies, separate from visual meshes. See collision notes below.
- `kinematics.json`: explicit axes, joint limits, zero positions, mesh grouping, interchangeable tool membership, force-control targets and configuration manifest.
- `validation.json`: geometry checks, not certification.
- `preview.png`: actual CAD tessellation, open jaws + extended bridge shoe.
- `build_revB.py`: reproducible CadQuery source, exports and validation.

STEP is static: joint semantics live in `kinematics.json`. Load exactly one interchangeable end at a time. Do not load all individual parts as a single assembly; the alternative shoes occupy the same space by design.

## Units and frame

All CAD and mesh coordinates are **millimeters**; force targets are in **newtons**. STL/OBJ do not encode units reliably. Set mesh scale to 0.001 in a meter-based simulator. STEP uses mm.

The root origin is midway between the two **closed inner carrier mounting faces**, at the original base-reference plane Z=0. X crosses the jaw opening, positive toward the right jaw. Y crosses fingertip width. Z points distally toward the LEGO. The center plunger travels along +Z at X=0, Y=0.

At zero, the left carrier inner face is X=-4.1 and the right is X=+4.1. Thus the assumed closed mounting-face spacing is 8.2. Inner pad faces are X=-3.5 and +3.5, giving a 7 mm free gap. This spacing is a design assumption, not a measured SMC dimension. The J6-to-root transform is unknown.

All individual STEP/STL/OBJ files are **already positioned in root coordinates at the zero configuration**. This includes the left/right rotation and offset. Joint origins in JSON are [0,0,0]; apply only their delta translations to these pre-positioned meshes. Do not add the +/-4.1 placement a second time.

## Dimensions

| Feature | Rev B value |
|---|---|
| Carrier stock section | 12 in Y x 6 in X |
| Carrier length | Z=0..36 |
| Existing mount hole centers | Y=0; Z=10 and 23; axes parallel X |
| Existing mount hole diameter | 3.4 through, provisional from Rev A |
| Finger end | Z=60 |
| Insert envelope | Z=26..60; 2.5 thickness in X |
| Insert root width | 6 in Y at Z=26..35 |
| Slim tong width | 3.6 in Y at Z=35..60 |
| Free working zone | Z=36..60; 24 long |
| Jaw pad | 0.6 thick x 3 wide x 16.5 long; Z=43..59.5 |
| Pad corner radius | 0.25 |
| Jaw motion | 0..6 outward per jaw |
| Free gap | 7 closed; 19 open; general gap = 7 + q_left + q_right |
| Plunger shaft | Diameter 3; shaft Z=-20..45 at zero |
| Coupling spigot | Diameter 1.6; Z=45..49 at zero; thread geometry omitted |
| Plunger travel | 0..12 along +Z |
| All tool contact faces | Z=52 retracted; Z=64 extended |
| Contact face relative to fingers | 8 behind retracted; 4 beyond extended |
| Round end | Diameter 3; rigid contact face |
| Flat shoe footprint | 5 in X x 6 in Y |
| Bridge shoe footprint | 5 in X x 14 in Y |
| Flat/bridge contact pads | 0.6 thick, included in the Z=52/64 face position |
| Flat/bridge rigid plate | 2 thick; Z=49.4..51.4 at zero |
| Shoe coupling boss | Diameter 4; Z=45..49.4 |
| Shoe outer plan corners | R0.35 |
| Plunger housing envelope | X=-3.2..3.2; Y=-6..6; Z=-28..24 |
| Main guide channel | 3.2 square, with +Y keyway |
| Rod antirotation key | X=-0.8..0.8; Y=1..2.8; Z=-15..-7 at zero |

The 5 mm shoe width in X leaves 1 mm clearance to each uncompressed jaw pad at full closure. The bridge extends along Y, not X. The housing leaves 0.9 mm per side to the closed inner carrier faces. The round end has no soft pad in this revision. All three ends share the same contact-plane offset.

Rev A's inward hooks and stud saddle are omitted for Rev B. Straight padded tongs allow the seated part to slide axially and release when the jaws relax. The stock section, mounting centers, replaceable insert pocket and 60 mm overall finger length are retained. This is a geometry revision, not just an added plunger.

## Solid names / bill of geometry

| Names | Motion group | Role |
|---|---|---|
| `left_carrier`, `right_carrier` | Corresponding jaw | Mounting carrier |
| `left_finger_insert`, `right_finger_insert` | Corresponding jaw | Removable slim rigid tongs |
| `left_finger_pad`, `right_finger_pad` | Corresponding jaw | Compliant inner contacts |
| `plunger_housing` | Fixed root | Concept guide body |
| `plunger_rod` | Plunger | Shaft, proximal antirotation key and distal coupling spigot |
| `round_tip` | Plunger, round option | Small central contact |
| `flat_shoe`, `flat_shoe_pad` | Plunger, flat option | Tile/brick pressing end |
| `bridge_shoe`, `bridge_shoe_pad` | Plunger, bridge option | Wider Y-span pressing end |

The STEP assembly names match these solid names. Each named component is one valid solid. A round-end assembly contains 9 solids; flat and bridge assemblies contain 10.

## Kinematics

| Joint | Type | Axis in root | Limits | Zero |
|---|---|---|---|---|
| left_jaw | Prismatic | [-1,0,0] | 0..6 mm | Closed |
| right_jaw | Prismatic | [+1,0,0] | 0..6 mm | Closed |
| plunger | Prismatic | [0,0,+1] | 0..12 mm | Contact face Z=52 |

The two jaw joints are independently addressable in the simulation. For symmetric operation, set q_left=q_right. Independent actuation of the actual existing parallel gripper is **not asserted**: the real gripper may require a coupled jaw model. Asymmetric motion moves the center of the grasp while the plunger stays at X=0. Use symmetric gripping/relaxation for centered seating unless the robot pose compensates.

The keyed rod/guide geometry constrains rotation conceptually; it is not a precision bushing or actuator design. The housing is declared fixed to the root without inventing a mounting bracket. The actual bracket, force path and guide fit remain to be designed around measurements.

## Interchangeable connection and retention

All tips use a nominal 1.6-diameter blind socket from Z=45 to 49 matching the rod's reduced spigot. The round end seats at the shaft shoulder at Z=45; the wider ends share the same seat and contact plane. This is a **nominal threaded coupling placeholder**: threads, clearance, locking, tool clocking and fasteners are not modeled. Treat an installed end as rigidly attached in simulation. Do not fabricate a plain slip-fit socket and assume it will retain the end. The bridge shoe must be clocked along Y; its physical locking method is unresolved.

Finger insert retention follows the Rev A concept: two DIA 2.2 through holes with DIA 3.8 x 1.2 recesses at Z=28 and 33, and DIA 1.6 carrier tap pilots. Verify M2 fastener heads, engagement, material-specific tapping, assembly access and external clearances. The 6 mm insert root fits a 6.2 mm pocket; pocket starts at Z=25.9 and is 2.5 deep.

## Force and compliance

Intended controlled seating force is 15–20 N, with a 20 N simulation cap. **This is not a demonstrated force rating** of the shaft, coupling, guide, pads or mounting. No mass, inertia, friction coefficient, material strength or actuator calibration is invented.

An optional simulation-only compliance target is k=10 N/mm over up to 2 mm: 1.5 mm corresponds to 15 N and 2 mm to 20 N under the simple spring relation F=k*delta. Implement this in a contact/impedance controller if supported. It is disabled by default in JSON; it is not an extra mechanical joint or a spring contained in the CAD. It does not increase the 12 mm geometric stroke or permit a face past Z=64. Real stiffness/damping and control gains require measured contact behavior and actuator data. A 0.6 mm TPU pad alone should not be assumed to provide 1–2 mm axial compliance.

A physical spring cartridge would need travel space, preload, a guided compression element and positive stops; these are deliberately left unresolved rather than hidden inside the concept housing.

## Intended sequence

1. Select one end and retract the plunger to q=0. Open the tongs, approach and grip the part using a width-appropriate jaw position.
2. Carry the part and align it over its mating studs with the robot.
3. Relax both jaws symmetrically enough to reduce axial friction while retaining orientation. The opening increment is part-dependent, not a fixed value in this model.
4. Advance the center plunger with force/position monitoring and a 20 N target cap, stopping at the seating condition or the 12 mm travel limit.
5. Verify seating with measured pose/force or other feedback. No such sensor is modeled.
6. Retract the plunger while the fingers still stabilize the part.
7. Open the jaws and withdraw the robot.

The assumed contact plane must be checked against part height, exposed studs and how deeply the tongs carry the part. Retracted face Z=52 is 8 mm behind the fingertip end but may still contact a tall part. The bridge can overlap surrounding studs or pieces along Y; jaw clearance alone does not prove workpiece clearance.

## Collision meshes

Each component has a separate convex OBJ proxy. For most components this is a conservative convex hull of the tessellated solid: mounting holes, sockets, recesses and concavities are filled. The guide housing instead uses **six convex wall boxes** whose union preserves its rectangular shaft channel and keyway. OBJ vertices use the same mm/root-zero frame as the visual STL files.

These are simplified simulation proxies, not exact contact meshes. Disable collisions within a rigid motion group and between the rod and housing, as listed in JSON. Hulls fill the shoe socket and would otherwise report false shaft/shoe overlap; contact filtering is essential. Keep housing/jaw, shoe/jaw and all external workpiece contacts active. Use high-detail geometry where small clearance/contact features matter. Hulls approximate curved surfaces through tessellation.

## Validation and limitations

Checks performed by `build_revB.py`:
- All 13 component solids are valid and single-solid.
- All 12 assembly exports were re-imported with the expected number of valid solids.
- Exact CAD solid intersections checked at all four endpoint configurations for every shoe; no positive-volume internal interference above 1e-6 mm^3.
- Five additional intermediate/asymmetric jaw/plunger states checked for all three ends.
- Separate convex collision hulls generated from component geometry; housing channel preserved by wall decomposition.

This is discrete geometry validation, not continuous collision proof or structural validation. Physical fit, gripper independence, mounting screw diameter, J6 alignment, guide friction, actuator force control, plunger buckling, thread retention, pad adhesion, precision, payload and neighboring-part collisions are unverified.

Before a physical version, measure the actual closed mounting-face spacing, mounting-hole diameter/orientation, base datum, available proximal space, gripper body envelope, camera clearance and flange transform. The 8.2 mm assumed closed face spacing and 3.4 mm mounting holes are provisional. The fixed housing may intersect the unmodeled real gripper body; this cannot be checked without its CAD or measurements.

## Rebuild

Requires Python with CadQuery, NumPy, SciPy and matplotlib. Run `python build_revB.py`. It writes all geometry, kinematics, checks and the preview beside the script. Rev B dimensions are declared in the geometry code; update this README and any fixed JSON metadata alongside future parameter changes. No simulator-specific import code or controller is included.
