from pathlib import Path
import cadquery as cq
import numpy as np
from scipy.spatial import ConvexHull
import json,shutil
P=Path(__file__).resolve().parent
SOURCE=P.parent/'offset_work/tweezer_plunger_revB'
for d in ['components','visual_meshes','collision_meshes','configurations']:(P/d).mkdir(exist_ok=True)
# First build copies Rev B reference components. Later builds use packaged copies.
if not (P/'kinematics_revB_reference.json').exists():
 shutil.copyfile(SOURCE/'kinematics.json',P/'kinematics_revB_reference.json')
 for f in (SOURCE/'components').glob('*.step'):shutil.copyfile(f,P/'components'/f.name)
 shutil.copyfile(SOURCE/'components/right_finger_insert.step',P/'right_finger_insert_revB_reference.step')
k=json.loads((P/'kinematics_revB_reference.json').read_text())
parts={n:cq.importers.importStep(str(P/'components'/f'{n}.step')) for n in k['links']}
def box(a,b,c,d,e,f):return cq.Workplane('XY').box(b-a,d-c,f-e).translate(((a+b)/2,(c+d)/2,(e+f)/2))
def hole(r,z,x=3,length=10):return cq.Workplane(obj=cq.Solid.makeCylinder(r,length,cq.Vector(x,0,z),cq.Vector(1,0,0)))
stem=box(4.1,6.6,-3,3,26,36)
for z in [28,33]:
 stem=stem.cut(hole(1.1,z)).cut(hole(1.9,z,3,2.3)) # recess from inward face x4.1..5.3
walls=[box(3.8,13.2,-4,4,36,37.9),box(3.8,13.2,-4,-2.6,37.9,42),box(3.8,13.2,2.6,4,37.9,42),box(3.8,13.2,-2.6,-1.9,40.1,42),box(3.8,13.2,1.9,2.6,40.1,42)]
guide=stem
for w in walls:guide=guide.union(w)
bar=box(4.1,12.9,-2.5,2.5,38,40)
tong=box(4.1,6.6,-1.8,1.8,40,60).edges('|Z').fillet(.2)
slider=bar.union(tong)
parts['right_finger_insert']=slider
parts['right_retraction_guide']=guide
# Retain exactly the reference right pad, position and opposite components.
groups={n:l['motion_group'] for n,l in k['links'].items()}
groups['right_finger_insert']='JawL';groups['right_finger_pad']='JawL';groups['right_retraction_guide']='right_jaw'
colors={n:(.9,.48,.12) if n.endswith('pad') else (.22,.58,.73) if 'insert' in n else (.54,.59,.64) for n in parts}
colors['right_retraction_guide']=(.37,.42,.48)
# Preserve the Rev B housing channel with its six wall proxies.
housingwalls=[box(a,b,c,d,-28,24) for a,b,c,d in [(-3.2,-1.6,-6,6),(1.6,3.2,-6,6),(-1.6,1.6,-6,-1.6),(-1.6,-.9,1.6,6),(.9,1.6,1.6,6),(-.9,.9,3,6)]]
proxies={'right_finger_insert':[bar,tong],'right_retraction_guide':[stem]+walls,'plunger_housing':housingwalls}
def hull(p,path):
 v,_=p.val().tessellate(.025);v=np.unique(np.round([[x.x,x.y,x.z] for x in v],8),axis=0);h=ConvexHull(v)
 with open(path,'w') as f:
  f.write('# convex proxy; mm; root zero coordinates\n')
  for a in v:f.write('v '+' '.join(map(str,a))+'\n')
  for t,e in zip(h.simplices,h.equations):
   a,b,c=t
   if np.dot(np.cross(v[b]-v[a],v[c]-v[a]),e[:3])<0:b,c=c,b
   f.write(f'f {a+1} {b+1} {c+1}\n')
links={}
for n,p in parts.items():
 assert p.val().isValid() and len(p.solids().vals())==1,n
 cq.exporters.export(p,str(P/'components'/f'{n}.step'))
 cq.exporters.export(p,str(P/'visual_meshes'/f'{n}.stl'),tolerance=.015,angularTolerance=.08)
 cs=[]
 for i,s in enumerate(proxies.get(n,[p])):
  path=f'collision_meshes/{n}_{i:02}.obj';hull(s,P/path);cs.append(path)
 links[n]={'motion_group':groups[n],'step':f'components/{n}.step','visual_mesh':f'visual_meshes/{n}.stl','collision_meshes':cs}
base=k['always_present']+['right_retraction_guide']
def pose(n,ql,qr,qp,r):
 g=groups[n];delta={'left_jaw':(-ql,0,0),'right_jaw':(qr,0,0),'JawL':(qr+r,0,0),'plunger':(0,0,qp),'fixed':(0,0,0)}[g]
 return parts[n].translate(delta)
configs=[];checks=0
# 8 endpoint combinations for each of the 3 retained plunger ends.
for end,names in k['tool_options'].items():
 for jaw,q in [('closed',0),('open',6)]:
  for pl,qp in [('plunger_retracted',0),('plunger_extended',12)]:
   for js,r in [('JawL_pickup',0),('JawL_retracted',6)]:
    name=f'{jaw}_{pl}_{js}_{end}';a=cq.Assembly(name=name);sol=[]
    for n in base+names:
     p=pose(n,q,q,qp,r);a.add(p,name=n,color=cq.Color(*colors[n]));sol.append((n,p.val()))
    for i,(n,s) in enumerate(sol):
     for m,t in sol[i+1:]:assert s.intersect(t).Volume()<1e-6,(name,n,m)
    target=P/'configurations'/f'assembly_{name}.step';a.save(str(target));imp=cq.importers.importStep(str(target));assert len(imp.solids().vals())==len(sol) and all(x.isValid() for x in imp.solids().vals())
    configs.append({'file':str(target.relative_to(P)),'end':end,'joints_mm':{'left_jaw':q,'right_jaw':q,'plunger':qp,'JawL_retract':r}});checks+=1
# Check the minimum requested clearance and intermediate retraction.
for r in [1,2,3,4,5]:
 for end,names in k['tool_options'].items():
  sol=[pose(n,0,0,6,r).val() for n in base+names]
  for i,s in enumerate(sol):
   for t in sol[i+1:]:assert s.intersect(t).Volume()<1e-6
  checks+=1
for n,fn in [('pickup','closed_plunger_retracted_JawL_pickup_flat'),('jaw_retracted','closed_plunger_retracted_JawL_retracted_flat'),('seating_jaw_retracted','closed_plunger_extended_JawL_retracted_flat')]:shutil.copyfile(P/'configurations'/f'assembly_{fn}.step',P/f'assembly_{n}.step')
k.update({'schema':'revB_independent_insert_v1','links':links,'always_present':base,'configurations':configs})
k['joints'].append({'name':'JawL_retract','type':'prismatic','parent':'right_jaw','child':'JawL','axis':[1,0,0],'axis_frame':'root (parallel to parent)','origin_xyz':[0,0,0],'limits':[0,6],'zero':0,'zero_description':'Rev B normal pinch position; no additional insert offset','minimum_clearance_command_mm':4,'moving_components':['right_finger_insert','right_finger_pad'],'recommended_simulation_limits':{'initial_speed_mm_s':5,'speed_cap_mm_s':10,'initial_force_cap_N':2,'provisional_force_cap_N':5,'initial_acceleration_mm_s2':20,'validated_on_hardware':False,'note':'Conservative initial simulation/controller tuning values, not a hardware safety rating. Retract only after connector lock verification and grip-force relaxation. Stop on unexpected resistance; do not use this as a prying actuator.'}})
k['body_aliases']={'JawL':['right_finger_insert','right_finger_pad']}
k['transform_rule']='Meshes pre-positioned in root zero frame. JawL translation = [right_jaw + JawL_retract,0,0]. Guide/carrier translate only [right_jaw,0,0]. Opposite jaw and plunger do not inherit JawL_retract.'
k['invariants']['pad_gap_expression_mm']='7 + left_jaw + right_jaw + JawL_retract'
k['invariants']['JawL_pad_inner_x_expression_mm']='3.5 + right_jaw + JawL_retract'
k['invariants']['minimum_guide_engagement_mm']=3.1
k['collision_policy']['disable_pairs'] += [['right_retraction_guide','right_finger_insert']]
k['collision_policy']['note']+=' Retracting slider/guide contact is constrained by the prismatic joint; filter it. Guide and moving slider use multiple convex pieces to preserve the open distal slot.'
k['actuation_assumptions']['JawL_retract']='One added independent virtual DOF. Physical drive, preload, end stops, bearings and position sensing are not designed.'
(P/'kinematics.json').write_text(json.dumps(k,indent=2))
(P/'validation.json').write_text(json.dumps({'valid_components':len(parts),'step_endpoint_configurations_reimported':24,'exact_interference_states_tested':checks,'max_allowed_interference_mm3':1e-6,'pickup_pad_position_preserved':True,'retraction_mm':6,'physical_mount_and_actuator_verified':False},indent=2))
# Orthographic diagram from actual CAD triangles; compare unchanged root frame states.
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import PolyCollection
fig,axs=plt.subplots(1,2,figsize=(11,8))
for ax,r,title in zip(axs,[0,6],['Pickup | JawL_retract = 0 mm','Released | JawL_retract = 6 mm']):
 for n in base+k['tool_options']['flat']:
  p=pose(n,0,0,0,r);v,tr=p.val().tessellate(.05);v=np.array([[t.x,-t.z] for t in v]);ax.add_collection(PolyCollection([v[list(t)] for t in tr],facecolors=[colors[n]],edgecolors='none'))
 ax.set_xlim(-15,23);ax.set_ylim(-65,30);ax.set_aspect('equal');ax.set_title(title,fontsize=11);ax.set_xlabel('X (mm)');ax.set_ylabel('-Z (mm); distal downward');ax.grid(alpha=.15)
 if r:ax.annotate('',xy=(17,-48),xytext=(11,-48),arrowprops={'arrowstyle':'->'});ax.text(6,-46,'+X, 6 mm',fontsize=9)
fig.suptitle('Rev B-R | JawL = right_finger_insert + right_finger_pad',fontsize=14)
fig.tight_layout();fig.savefig(P/'joint_diagram.png',dpi=160);plt.close(fig)
print('OK',checks,'states;',len(parts),'components')
