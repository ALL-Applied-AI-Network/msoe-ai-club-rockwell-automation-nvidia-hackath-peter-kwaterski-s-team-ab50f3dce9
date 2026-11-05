from pathlib import Path
from reportlab.pdfgen import canvas
from reportlab.lib.colors import HexColor
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
pdfmetrics.registerFont(TTFont('DejaVu','/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'))
P=Path(__file__).resolve().parent
c=canvas.Canvas(str(P/'fingertip_drawing.pdf'),pagesize=(842,595))
c.setTitle('Replacement fingertip concept - Rev A')
c.setAuthor('OpenAI')
BLUE=HexColor('#268db3'); GRAY=HexColor('#8292a2'); ORANGE=HexColor('#ed8c35')
def text(x,y,s,size=9):
 c.setFillColor(HexColor('#182a3a'));c.setFont('DejaVu',size);c.drawString(x,y,s)
def line(x1,y1,x2,y2):
 c.setStrokeColor(HexColor('#43576a'));c.setLineWidth(.6);c.line(x1,y1,x2,y2)
def rect(x,y,w,h,color):
 c.setFillColor(color);c.setStrokeColor(HexColor('#43576a'));c.setLineWidth(.6);c.rect(x,y,w,h,fill=1,stroke=1)
def dim(x1,y1,x2,y2,label,dx=4,dy=4):
 line(x1,y1,x2,y2)
 for x,y in [(x1,y1),(x2,y2)]: line(x-3,y-3,x+3,y+3)
 text((x1+x2)/2+dx,(y1+y2)/2+dy,label,8)
def header(title,n):
 text(30,561,title,19);text(30,541,'MHZL2-16D2 replacement fingertip concept | REV A | 2026-11-01',10)
 line(30,530,812,530);line(30,35,812,35)
 text(30,20,'ALL DIMENSIONS mm | NOT TO PRINT SCALE | CONCEPT - VERIFY INTERFACE BEFORE FABRICATION',8)
 text(760,20,f'{n} / 2',8)
header('01 / Assembly and mounting datums',1)
# Front elevation, z increases down.
s=5; ox=130; oy=480
rect(ox-30,oy-180,60,180,GRAY);rect(ox-15,oy-300,30,170,BLUE)
for z in [10,23]:
 c.setFillColor(HexColor('#ffffff'));c.circle(ox,oy-z*s,1.7*s,fill=1,stroke=1)
 line(ox-10,oy-z*s,ox+10,oy-z*s);text(168,oy-z*s-3,f'z = {z}',8)
for z in [28,33]: c.circle(ox,oy-z*s,1.9*s,fill=0,stroke=1)
rect(ox-13.5,oy-58.5*s,27,13.5*s,ORANGE)
dim(82,oy,82,oy-300,'60',-18,0);dim(100,oy+15,160,oy+15,'12',-5,4)
dim(115,oy-315,145,oy-315,'6',-2,-13)
text(91,506,'FRONT / look along X',10)
text(45,143,'Base reference: z = 0',9);text(45,127,'Mount hole centers: y = 0; z = 10, 23',9)
text(45,111,'2X DIA 3.4 through X (ASSUMED)',9)
text(45,95,'Hole pitch = 13; carrier section = 12 x 6',9)
text(45,79,'Insert screws: z = 28, 33; y = 0',9)
# side view local coordinates
ox=315
rect(ox-30,oy-180,30,180,GRAY);rect(ox-12.5,oy-300,12.5,170,BLUE)
rect(ox,oy-292.5,3,67.5,ORANGE)
# toe polygon
path=c.beginPath();pts=[(-.3,59.2),(.8,59.2),(.8,59.4),(.2,60),(-.3,60)]
for i,(x,z) in enumerate(pts): (path.moveTo if i==0 else path.lineTo)(ox+x*s,oy-z*s)
path.close();c.setFillColor(BLUE);c.drawPath(path,fill=1)
line(ox-1.75,oy-190,ox-1.75,oy-220)
dim(347,oy-180,347,oy-300,'24',4,0)
dim(285,oy+15,315,oy+15,'6',-2,4)
text(270,506,'SIDE / look along Y',10)
text(250,143,'x = 0: carrier inner face',9);text(250,127,'Insert body x = -2.5 to 0',9)
text(250,111,'Working zone z = 36 to 60',9);text(250,95,'Toe ledge z = 59.2',9)
text(250,79,'Toe distal end z = 60',9)
# image
c.drawImage(str(P/'preview.png'),410,163,width=390,height=355,preserveAspectRatio=True,anchor='c',mask='auto')
text(440,147,'CLOSED ASSEMBLY / reference placement',10)
text(440,128,'Pad gap 7.0 closed / 19.0 open (uncompressed)',9)
text(440,112,'Toe gap 6.6 closed / 18.6 open',9)
text(440,96,'Required inner mounting-face gap: 8.2 closed',9)
text(440,80,'Travel: 6.0 per jaw; screw hardware not modeled',9)
c.showPage()
header('02 / Insert details and interface control',2)
# toe zoom with local z offset
ox=185;oy=441;s=62
rect(ox-2.5*s,oy-4*s,2.5*s,4*s,BLUE)
rect(ox,oy-2.5*s,.6*s,2.5*s,ORANGE) # ends at58.5, base z56
pts=[(-.2,59.2),(.8,59.2),(.8,59.4),(.2,60),(-.2,60)]
path=c.beginPath()
for i,(x,z) in enumerate(pts): (path.moveTo if i==0 else path.lineTo)(ox+x*s,oy-(z-56)*s)
path.close();c.setFillColor(BLUE);c.drawPath(path,fill=1)
dim(ox,oy+14,ox+.6*s,oy+14,'0.6 pad',-12,7)
dim(ox,oy-4*s-20,ox+.8*s,oy-4*s-20,'0.8 toe depth',-22,-14)
dim(ox+.8*s+15,oy-3.2*s,ox+.8*s+15,oy-4*s,'0.8',4,0)
text(35,495,'TOE / enlarged side section',11)
text(35,478,'Orange: compliant pad; blue: rigid insert',9)
text(35,150,'0.6 x 0.6 distal lead-in chamfer',9)
text(35,134,'0.7 axial clearance: pad end to toe ledge',9)
text(35,118,'0.2 toe underlap beyond uncompressed pad',9)
text(35,102,'Insert outer longitudinal edges R0.2',9)
# table
x=310;y=495
text(x,y,'LOCAL COORDINATES / same component used twice',11)
rows=[
('Datum origin','(0, 0, 0): inner carrier face, width center, base'),
('Axis convention','+X inward; +Y across width; +Z toward workpiece'),
('Mount hole centers','(0, 0, 10), (0, 0, 23); axes parallel X'),
('Insert envelope','x -2.5..0; y -3..3; z 26..60'),
('Carrier pocket','x -2.5..0; y -3.1..3.1; z 25.9..36'),
('Insert fastening','2X DIA 2.2 through; DIA 3.8 counterbore depth 1.2'),
('Carrier fastening','2X DIA 1.6 pilot for M2 tap; z 28, 33'),
('Pad envelope','x 0..0.6; y -2.7..2.7; z 45..58.5; R0.3 corners'),
('Stud saddle','R2.5; depth 0.35; z 38..44; cylinder axis Z'),
('Saddle axis','x = 2.15; y = 0; rigid face, above pad'),
('Toe load ledge','z = 59.2; x 0..0.8; y -3..3'),
('Assembly left','local translated X = -4.1 closed / -10.1 open'),
('Assembly right','rotate 180 deg about Z, then X = +4.1 / +10.1'),
]
for i,(a,b) in enumerate(rows):
 yy=y-27-i*20
 text(x,yy,a,8);text(x+115,yy,b,8);line(x,yy-6,808,yy-6)
text(310,183,'INTEGRATION HOLD POINTS',11)
notes=[
'Confirm actual closed face spacing; 8.2 is a design requirement, not measured.',
'Confirm base datum, hole axis, diameter, screws and mounting-face orientation.',
'J6-to-finger rigid transform is unknown; no J6 placement is asserted.',
'M2 low-head screw envelope must fit the 3.8 x 1.2 recess; verify engagement.',
'Pad: nominal TPU/urethane 60-80A; adhesive thickness must be included in 0.6.',
'Prototype only: validate grip force, toe strength, reach and neighboring-part clearance.',
'Toe needs under-edge access; it cannot enter a perfectly flush, gap-free interface.'
]
for i,n in enumerate(notes): text(310,164-i*15,n,8)
c.save()
