# Replacement fingertip kit - Rev A

Concept based on the supplied SMC MHZL2-16D2 requirements. Units: mm.
This is a reviewable CAD concept, not a verified drop-in or production release.
No vendor model, physical measurements, load tests or J6 transform were supplied.

## Included
- assembly_closed.step and assembly_open.step: six named solids each, colored carrier/insert/pad pairs; no robot, gripper body or screw hardware.
- carrier.step, edge_hook_insert.step, pad.step: individual solids in shared local coordinates. Make two of each. Both jaws use the same component geometry; the opposite jaw is rotated 180 degrees about Z.
- Matching STL files for prototype work, with numerical coordinates in mm (STL has no unit metadata).
- fingertip_drawing.pdf: two-sheet nominal dimension and coordinate drawing.
- build_model.py: editable CadQuery geometry and checks; draw.py: drawing source.
- preview.png and validation.json.

The selected design is the symmetric edge-hook pair with integrated upper stud saddles. The optional forked pair is not included.

## Geometry
Carrier retains the supplied 12-wide x 6-thick stock section with mounting centers 10 and 23 from the base. Carrier length is a chosen 36. Overall length is 60. A removable 6-wide x 2.5-thick insert extends from z=26 to 60, with a 24-long free working zone below the carrier. Its 10-long pocket engagement has 0.1 side clearance per side and 0.1 axial clearance at the pocket start. The pocket is open at the distal end; two screws retain the insert against the pocket floor.

The toe is 0.8 high and projects 0.8 inward, with a 0.6 x 0.6 distal lead-in ramp. Its upper ledge is flat; the rigid face above it is recessed behind the toe. Outside longitudinal insert edges are R0.2 and carrier longitudinal edges R0.4. Not all transverse edges are filleted: deburr and tune non-contact edges without changing the toe ledge.

The separate compliant pad is 5.4 wide x 13.5 long x 0.6 thick, with R0.3 outline corners. Suggested supplied material target: TPU or urethane 60-80A. Adhesive is not modeled; total pad-plus-adhesive protrusion must equal the specified 0.6 for the stated gap. The upper rigid-face saddle is R2.5 and 0.35 deep, with cylinder axis along Z, covering z=38..44. This stud mode has not been tested on surrounding brick geometry.

## Gap and datum control
Local origin: inner carrier mounting face (x=0), width centerline (y=0), base reference (z=0). X grows inward, Y across width, Z toward the part. Mount hole axes are parallel X. The toe ledge is at z=59.2: 49.2 beyond the first mounting hole and 36.2 beyond the second.

The reference closed assembly positions the left origin at X=-4.1, and the right at X=+4.1 after rotation 180 degrees around Z. This REQUIRES an 8.2 closed spacing between the inner carrier mounting faces. With two 0.6 pads, the free pad gap is 7.0. Moving each jaw outward by the user-specified 6.0 gives 19.0. Toe gaps are 6.6 and 18.6, respectively. Actual part width and pad compression determine the commanded grip position; do not treat full closure as the grip setpoint for every part.

The assembly is centered on the grasp plane, NOT positioned relative to J6. Convert to robot coordinates only after measuring the transform from the actual mounting-hole datum. If actual closed inner-face spacing is G, current pad gap is G-1.2; achieving 7.0 requires G=8.2. Adjust carrier offsets/insert reach based on the measured geometry; simply translating a CAD assembly does not change the physical gripper interface.

## Assumptions to resolve before fabrication
1. Confirm the base reference, mounting-face orientation, hole centerline across width, and both hole coordinates on the current finger.
2. DIA 3.4 through mounting holes are PROVISIONAL. Mount screw size, head seating, thread engagement and screw length are unknown. No mounting counterbores are included.
3. Insert retention is a proposed M2 arrangement: DIA 2.2 through insert, DIA 3.8 x 1.2-deep counterbores, and DIA 1.6 carrier tap pilots. Threads are not modeled. Confirm low-head screw dimensions, material-specific tapping diameter, engagement, access and no protrusion through the carrier back. Standard full-height socket heads may not fit this recess.
4. Confirm carrier total length, closed spacing, collision envelope, camera view and J6 transform using actual robot/gripper CAD or measurements.
5. Choose rigid material and manufacturing tolerances after grip force and payload are known. No strength or fatigue rating is claimed. A machined metal carrier/insert is a candidate; printed rigid parts are fit prototypes until tested. M2 threads and the thin toe need special attention in polymer prints.
6. Toe engagement needs underside clearance or an edge overhang. The toe cannot enter a perfectly flush, gap-free seam against a baseplate. Do not rely on it to pry apart installed LEGO parts without a separate test.
7. The modeled 0.2 protrusion beyond the uncompressed pad is small. Verify actual underlap, pad compression, surface radii and release behavior at low force before use.

## Regeneration
Install CadQuery, matplotlib and reportlab in a Python environment. Run `python build_model.py` from any directory; it writes outputs alongside the script. Run `python draw.py` afterward to regenerate the drawing (requires DejaVu Sans at the path in the script). Geometry parameters are near the top of build_model.py. Drawing dimensions are fixed to Rev A: update draw.py when changing geometry.

## Validation performed
All three component shapes are valid single solids. Both exported STEP assemblies were re-imported and checked for six valid solids. Pairwise volumetric intersections are zero in both endpoint configurations (mating faces touch). Checks cover geometry only, not screw threads, dynamics, structural capacity, physical tolerances, gripper fit or collisions with external objects. The PDF was rendered and visually reviewed.
