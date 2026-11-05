#!/usr/bin/env python3
"""Print authored articulation-drive settings for the two-arm LEGO cell."""

from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
try:
    import omni.usd
    from pxr import UsdPhysics

    import lego_cell

    stage = omni.usd.get_context().get_stage()
    lego_cell.build(stage)
    for prim in stage.Traverse():
        path = str(prim.GetPath())
        if not path.startswith("/World/ER4iA_A/") or not prim.IsA(UsdPhysics.Joint):
            continue
        rows = []
        for token in ("angular", "linear"):
            drive = UsdPhysics.DriveAPI.Get(prim, token)
            if not drive:
                continue
            rows.append(
                {
                    "token": token,
                    "type": drive.GetTypeAttr().Get(),
                    "stiffness": drive.GetStiffnessAttr().Get(),
                    "damping": drive.GetDampingAttr().Get(),
                    "max_force": drive.GetMaxForceAttr().Get(),
                }
            )
        if rows:
            print(path, rows, flush=True)
finally:
    app.close()
