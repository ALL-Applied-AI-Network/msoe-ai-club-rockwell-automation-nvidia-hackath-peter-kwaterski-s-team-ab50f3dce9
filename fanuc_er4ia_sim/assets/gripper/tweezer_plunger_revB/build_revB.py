from pathlib import Path
import cadquery as cq
from scipy.spatial import ConvexHull
import numpy as np
import json, shutil
P=Path(__file__).resolve().parent
for d in ['components','visual_meshes','collision_meshes','configurations']:(P/d).mkdir(exist_ok=True)
# Units mm; all exported components are already placed at zero-state root coordinates.
FACE_GAP=8.2; PAD=.6; TONG_W=3.6; LENGTH=60.; STROKE=12.
def box(a,b,c,d,e,f):return cq.Workplane('XY').box(b-a,d-c,f-e).translate(((a+b)/2,(c+d)/2,(e+f)/2))
def cyl(r,z,h):return cq.Workplane(obj=cq.Solid.makeCylinder(r,h,cq.Vector(0,0,z)))
def hole(r,z,x=-7,length=10):return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x,0,z),cq.Vector(1,0,0)))
carrier=box(-6,0,-6,6,0,36).edges('|Z').fillet(.4).cut(box(-2.5,1,-3.1,3.1,25.9,37))
for z in [10,23]:carrier=carrier.cut(hole(1.7,z))
for z in [28,33]:carrier=carrier.cut(hole(.8,z))
# Width transition is proximal to working zone, leaving 24 mm slim tong.
insert=box(-2.5,0,-3,3,26,35).union(box(-2.5,0,-1.8,1.8,35,60))
insert=insert.edges('|Z').fillet(.2)
for z in [28,33]:insert=insert.cut(hole(1.1,z,-3)).cut(hole(1.9,z,-1.2,2))
pad=box(0,.6,-1.5,1.5,43,59.5).edges('|X').fillet(.25)
parts={}; groups={}; colors={}
for side in ['left','right']:
 for name,p in [('carrier',carrier),('finger_insert',insert),('finger_pad',pad)]:
  p=p.translate((-4.1,0,0)) if side=='left' else p.rotate((0,0,0),(0,0,1),180).translate((4.1,0,0))
  k=side+'_'+name;parts[k]=p;groups[k]=side+'_jaw';colors[k]=(.95,.5,.13) if name.endswith('pad') else ((.2,.6,.75) if 'insert' in name else (.5,.56,.63))
# Concept guide with square shaft channel and a wider +Y keyway. No motor or bracket claimed.
wall_ranges=[(-3.2,-1.6,-6,6), (1.6,3.2,-6,6),(-1.6,1.6,-6,-1.6),(-1.6,-.9,1.6,6),(.9,1.6,1.6,6),(-.9,.9,3,6)]
walls=[box(a,b,c,d,-28,24) for a,b,c,d in wall_ranges]
housing=walls[0]
for w in walls[1:]:housing=housing.union(w)
parts['plunger_housing']=housing;groups['plunger_housing']='fixed';colors['plunger_housing']=(.32,.38,.45)
# Keyed proximal rod; reduced distal spigot represents an unmodeled threaded coupling.
rod=cyl(1.5,-20,65).union(cyl(.8,45,4)).union(box(-.8,.8,1,2.8,-15,-7))
parts['plunger_rod']=rod;groups['plunger_rod']='plunger';colors['plunger_rod']=(.75,.78,.8)
round_tip=cyl(1.5,45,7).cut(cyl(.8,44.9,4.1))
parts['round_tip']=round_tip;groups['round_tip']='plunger';colors['round_tip']=(.7,.73,.77)
for typ,w in [('flat_shoe',6),('bridge_shoe',14)]:
 shoe=box(-2.5,2.5,-w/2,w/2,49.4,51.4).edges('|Z').fillet(.35).union(cyl(2,45,4.4)).cut(cyl(.8,44.9,4.1))
 contact=box(-2.5,2.5,-w/2,w/2,51.4,52).edges('|Z').fillet(.35)
 for k,p in [(typ,shoe),(typ+'_pad',contact)]:parts[k]=p;groups[k]='plunger';colors[k]=(.95,.5,.13) if k.endswith('pad') else (.7,.73,.77)
collision={}
def convex_obj(p,path):
 v,_=p.val().tessellate(.03);xyz=np.unique(np.round([[q.x,q.y,q.z] for q in v],8),axis=0);h=ConvexHull(xyz)
 with open(path,'w') as f:
  f.write('# mm; conservative convex hull; root zero coordinates\n')
  for v in xyz:f.write('v '+' '.join(map(str,v))+'\n')
  for tri,eq in zip(h.simplices,h.equations):
   a,b,c=tri
   if np.dot(np.cross(xyz[b]-xyz[a],xyz[c]-xyz[a]),eq[:3])<0:b,c=c,b
   f.write(f'f {a+1} {b+1} {c+1}\n')
 return float(h.volume)
for k,p in parts.items():
 assert p.val().isValid() and len(p.solids().vals())==1,k
 cq.exporters.export(p,str(P/'components'/f'{k}.step'))
 cq.exporters.export(p,str(P/'visual_meshes'/f'{k}.stl'),tolerance=.015,angularTolerance=.08)
 collision[k]=[]
 for i,c in enumerate(walls if k=='plunger_housing' else [p]):
  f=f'collision_meshes/{k}_{i:02}.obj';convex_obj(c,P/f);collision[k].append(f)
ends={'round':['round_tip'],'flat':['flat_shoe','flat_shoe_pad'],'bridge':['bridge_shoe','bridge_shoe_pad']}
base=[k for k in parts if k.startswith(('left_','right_'))]+['plunger_housing','plunger_rod']
def pose(k,ql,qr,qp):return parts[k].translate({'left_jaw':(-ql,0,0),'right_jaw':(qr,0,0),'plunger':(0,0,qp),'fixed':(0,0,0)}[groups[k]])
configs=[];max_intersection=0.
for end,endparts in ends.items():
 for jaw,q in [('closed',0),('open',6)]:
  for pl,qp in [('retracted',0),('extended',12)]:
   name=f'{jaw}_{pl}_{end}';a=cq.Assembly(name=name);solids=[]
   for k in base+endparts:
    p=pose(k,q,q,qp);a.add(p,name=k,color=cq.Color(*colors[k]));solids.append((k,p.val()))
   for i,(k,s) in enumerate(solids):
    for kk,t in solids[i+1:]:
     volume=s.intersect(t).Volume();max_intersection=max(max_intersection,volume)
     assert volume<1e-6,(name,k,kk,volume)
   target=P/'configurations'/f'assembly_{name}.step';a.save(str(target))
   imported=cq.importers.importStep(str(target));assert len(imported.solids().vals())==len(solids) and all(s.isValid() for s in imported.solids().vals())
   configs.append({'file':str(target.relative_to(P)),'end':end,'joints_mm':{'left_jaw':q,'right_jaw':q,'plunger':qp},'solids':len(solids)})
for jaw in ['closed','open']:
 for pl in ['retracted','extended']:shutil.copyfile(P/'configurations'/f'assembly_{jaw}_{pl}_flat.step',P/f'assembly_{jaw}_{pl}.step')
for pl in ['retracted','extended']:shutil.copyfile(P/f'assembly_closed_{pl}.step',P/f'assembly_{pl}.step')
# Sample asymmetric and intermediate states as well as every endpoint.
for ql,qr,qp in [(0,6,6),(6,0,6),(1,2,3),(3,3,6),(5,4,9)]:
 for end in ends.values():
  solids=[pose(k,ql,qr,qp).val() for k in base+end]
  for i,s in enumerate(solids):
   for t in solids[i+1:]:assert s.intersect(t).Volume()<1e-6
kin={'schema':'revB_concept_kinematics_v1','units':{'length':'mm','force':'N','angle':'rad'},'frame':{'origin':'midpoint between closed inner carrier faces at base reference z=0','axes':{'X':'across jaw opening; right positive','Y':'across fingertip width','Z':'distal toward LEGO'},'J6_transform':None},'mesh_coordinates':'All STEP/STL/OBJ components are pre-positioned in root coordinates at joint zero. Apply only the joint delta translation. Do not add another +/-4.1 offset.','joints':[{'name':'left_jaw','type':'prismatic','parent':'root','axis':[-1,0,0],'origin_xyz':[0,0,0],'limits':[0,6],'zero':0},{'name':'right_jaw','type':'prismatic','parent':'root','axis':[1,0,0],'origin_xyz':[0,0,0],'limits':[0,6],'zero':0},{'name':'plunger','type':'prismatic','parent':'root','axis':[0,0,1],'origin_xyz':[0,0,0],'limits':[0,12],'zero':0}], 'links':{k:{'motion_group':groups[k],'step':f'components/{k}.step','visual_mesh':f'visual_meshes/{k}.stl','collision_meshes':collision[k]} for k in parts},'tool_options':ends,'default_tool':'flat','fixed_links':['plunger_housing'],'always_present':base,'invariants':{'pad_gap_expression_mm':'7 + left_jaw + right_jaw','finger_end_z':60,'contact_face_z_expression_mm':'52 + plunger','jaw_closed_face_locations_x':[-4.1,4.1],'shoe_x_width':5},'actuation_assumptions':{'jaw_dofs':'independent virtual joints; existing parallel gripper independence is not asserted','symmetric_command':'left_jaw = right_jaw','plunger_actuator':'not modeled; external axial force input'},'force_control_target':{'nominal_max_force_N':20,'suggested_range_N':[15,20],'measured_capability':False,'optional_virtual_compliance':{'enabled_by_default':False,'stiffness_N_per_mm':10,'maximum_deflection_mm':2,'force_cap_N':20,'implementation':'contact/impedance model only; not a mechanical spring or extra geometric stroke','note':'Do not command geometric extension above 12 mm. TPU stiffness and contact friction require measurement.'}},'collision_policy':{'disable_within_same_motion_group':True,'disable_pairs':[['plunger_housing','plunger_rod']],'note':'Convex hulls fill holes and recesses; internal guide/shaft and rigid fastenings need collision filtering. Housing uses six wall proxies preserving channel. External contacts remain active.'},'configurations':configs}
(P/'kinematics.json').write_text(json.dumps(kin,indent=2))
(P/'validation.json').write_text(json.dumps({'component_count':len(parts),'all_components_valid':True,'step_configurations_reimported':12,'max_endpoint_intersection_mm3':max_intersection,'additional_states_tested':5,'tool_options_per_state':3,'continuous_collision_proof':False,'physical_fit_verified':False},indent=2))
# Preview of actual tessellated geometry, open + extended bridge.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
fig=plt.figure(figsize=(10,9));ax=fig.add_subplot(111,projection='3d')
for k in base+ends['bridge']:
 p=pose(k,6,6,12);v,tris=p.val().tessellate(.05);xyz=np.array([[t.x,t.y,-t.z] for t in v]);faces=xyz[np.array(tris)]
 normals=np.cross(faces[:,1]-faces[:,0],faces[:,2]-faces[:,0]);normals/=np.maximum(np.linalg.norm(normals,axis=1)[:,None],1e-9)
 shade=.65+.35*np.abs(normals@np.array([.3,-.4,.866]));cs=np.clip(np.array(colors[k])[None,:]*shade[:,None],0,1)
 pc=Poly3DCollection(faces,facecolors=cs,linewidths=0);ax.add_collection3d(pc)
ax.set_xlim(-19,19);ax.set_ylim(-13,13);ax.set_zlim(-68,30);ax.set_box_aspect((38,26,98));ax.view_init(elev=15,azim=-65);ax.set_axis_off();fig.tight_layout();fig.savefig(P/'preview.png',dpi=150,bbox_inches='tight');plt.close(fig)
print('SUCCESS: 13 valid components, 12 assemblies, asymmetric samples, convex meshes.')
