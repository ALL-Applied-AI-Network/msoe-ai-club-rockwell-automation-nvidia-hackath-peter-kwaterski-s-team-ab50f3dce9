"""Concept fingertip kit, mm. Run with Python + cadquery + reportlab + matplotlib.
Mounting assumptions are documented in README.md. No vendor geometry is imported.
"""
from pathlib import Path
import cadquery as cq
import json
OUT=Path(__file__).resolve().parent
# Editable design parameters. x grows inward, y across finger, z toward part.
STOCK_W=12.; STOCK_T=6.; CARRIER_L=36.
MOUNT_Z=(10.,23.); MOUNT_D=3.4 # PROVISIONAL clearance; verify actual screws
INSERT_Z=26.; TIP_START=36.; END_Z=60.; TIP_W=6.; TIP_T=2.5
PAD_T=.6; TOE_H=.8; TOE_D=.8; SADDLE_R=2.5; SADDLE_DEPTH=.35
CLOSED_PAD_GAP=7.; JAW_STROKE=6.
MOUNT_FACE_SPACING=CLOSED_PAD_GAP+2*PAD_T # 8.2 REQUIRED, not measured

def box(x0,x1,y0,y1,z0,z1):
    return cq.Workplane('XY').box(x1-x0,y1-y0,z1-z0).translate(((x0+x1)/2,(y0+y1)/2,(z0+z1)/2))
def bore(r,x,z,length=10):
    return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x,0,z),cq.Vector(1,0,0)))
carrier=box(-STOCK_T,0,-STOCK_W/2,STOCK_W/2,0,CARRIER_L).edges('|Z').fillet(.4)
carrier=carrier.cut(box(-TIP_T,1,-3.1,3.1,INSERT_Z-.1,CARRIER_L+1))
for z in MOUNT_Z: carrier=carrier.cut(bore(MOUNT_D/2,-7,z))
for z in (28.,33.): carrier=carrier.cut(bore(.8,-7,z)) # M2 tap pilots
insert=box(-TIP_T,0,-3,3,INSERT_Z,END_Z).edges('|Z').fillet(.2)
# Full-width toe, flat upper load ledge and 0.6 mm distal lead-in ramp.
toe=cq.Workplane('XZ').polyline([(-.3,END_Z-TOE_H),(TOE_D,END_Z-TOE_H),(TOE_D,END_Z-.6),(.2,END_Z),(-.3,END_Z)]).close().extrude(TIP_W).translate((0,TIP_W/2,0))
insert=insert.union(toe)
# Upper vertical stud saddle, 6 mm axial extent. Depth 0.35 mm in rigid face.
saddle=cq.Workplane(obj=cq.Solid.makeCylinder(SADDLE_R,6,cq.Vector(SADDLE_R-SADDLE_DEPTH,0,38),cq.Vector(0,0,1)))
insert=insert.cut(saddle)
for z in (28.,33.):
    insert=insert.cut(bore(1.1,-3,z)).cut(bore(1.9,-1.2,z,2))
pad=box(0,PAD_T,-2.7,2.7,45,58.5).edges('|X').fillet(.3)
parts={'carrier':carrier,'edge_hook_insert':insert,'pad':pad}
for name,p in parts.items():
    assert p.val().isValid() and len(p.solids().vals())==1,name
    cq.exporters.export(p,str(OUT/(name+'.step')))
    cq.exporters.export(p,str(OUT/(name+'.stl')),tolerance=.02,angularTolerance=.1)

def positioned(p,side,opening):
    a=MOUNT_FACE_SPACING/2+opening
    return p.translate((-a,0,0)) if side=='left' else p.rotate((0,0,0),(0,0,1),180).translate((a,0,0))
for state,opening in [('closed',0),('open',JAW_STROKE)]:
    a=cq.Assembly(name='fingertips_'+state)
    solids=[]
    for side in ('left','right'):
        for name,p in parts.items():
            q=positioned(p,side,opening); solids.append(q.val())
            color={'carrier':(.36,.43,.51),'edge_hook_insert':(.15,.55,.75),'pad':(.95,.5,.13)}[name]
            a.add(q,name=side+'_'+name,color=cq.Color(*color))
    a.save(str(OUT/('assembly_'+state+'.step')))
    for i,s in enumerate(solids):
        for t in solids[i+1:]: assert s.intersect(t).Volume()<1e-6
    reread=cq.importers.importStep(str(OUT/('assembly_'+state+'.step')))
    assert len(reread.solids().vals())==6 and all(s.isValid() for s in reread.solids().vals())
checks={'units':'mm','valid_solids':True,'assembly_solids':6,'interference_volume_mm3':0,'closed_pad_gap':CLOSED_PAD_GAP,'open_pad_gap':CLOSED_PAD_GAP+2*JAW_STROKE,'closed_toe_gap':MOUNT_FACE_SPACING-2*TOE_D,'open_toe_gap':MOUNT_FACE_SPACING+2*JAW_STROKE-2*TOE_D,'required_closed_mount_face_spacing':MOUNT_FACE_SPACING,'mount_spacing_verified':False}
(OUT/'validation.json').write_text(json.dumps(checks,indent=2))
# Render actual tessellated solids for visual QA and drawing.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(9,8)); ax=fig.add_subplot(111,projection='3d')
for side in ('left','right'):
    for name,p in parts.items():
        q=positioned(p,side,0); verts,tris=q.val().tessellate(.06)
        pts=[(v.x,v.y,-v.z) for v in verts]
        poly=Poly3DCollection([[pts[i] for i in tri] for tri in tris],linewidths=0)
        poly.set_facecolor({'carrier':'#8292a2','edge_hook_insert':'#268db3','pad':'#ed8c35'}[name]);ax.add_collection3d(poly)
ax.set_xlim(-14,14);ax.set_ylim(-12,12);ax.set_zlim(-62,2);ax.set_box_aspect((28,24,64));ax.view_init(elev=18,azim=-55);ax.set_axis_off();fig.tight_layout();fig.savefig(OUT/'preview.png',dpi=170,bbox_inches='tight');plt.close(fig)
print(checks)
