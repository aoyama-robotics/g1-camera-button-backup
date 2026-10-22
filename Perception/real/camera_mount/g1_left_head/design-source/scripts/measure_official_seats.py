"""Measure actual bearing-to-contact seat thickness, not partial cylinder span."""
from pathlib import Path
import json,math,hashlib,os,sys,traceback
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder,GeomAbs_Plane
ROOT=Path(__file__).resolve().parents[1]
def run():
 result={}
 for kind in ['classic','renewed']:
  path=next((ROOT/'sources'/kind).rglob('stero-camera-mount*.step'))
  shape=cq.importers.importStep(str(path)).val();holes=[];seen=set()
  for f in shape.Faces():
   a=BRepAdaptor_Surface(f.wrapped)
   if a.GetType()!=GeomAbs_Cylinder or abs(a.Cylinder().Radius()-2.25)>1e-6:continue
   c=a.Cylinder();o=cq.Vector(*c.Location().Coord());d=cq.Vector(*c.Axis().Direction().Coord())
   key=tuple(round(x,4) for x in o.toTuple())
   if key in seen:continue
   seen.add(key);seats=[]
   for other in shape.Faces():
    b=BRepAdaptor_Surface(other.wrapped)
    if b.GetType()!=GeomAbs_Plane:continue
    n=cq.Vector(*b.Plane().Axis().Direction().Coord());r=other.Center()-o
    if abs(n.dot(d))<.999 or (r-d.multiply(r.dot(d))).Length>.1:continue
    t=(cq.Vector(*b.Plane().Location().Coord())-o).dot(n)/d.dot(n)
    seats.append(dict(axis_intersection_mm=t,area_mm2=other.Area(),center_mm=other.Center().toTuple()))
   assert len(seats)==2
   seats.sort(key=lambda p:p['axis_intersection_mm']);thickness=seats[1]['axis_intersection_mm']-seats[0]['axis_intersection_mm']
   u=cq.Vector(0,1,0);v=u.cross(d);samples=[]
   for radius in [2.5,3.0]:
    for angle in [0,90,180,270]:
     q=o+u.multiply(radius*math.cos(math.radians(angle)))+v.multiply(radius*math.sin(math.radians(angle)))
     cut=shape.intersect(cq.Edge.makeLine(q,q+d.multiply(30)))
     spans=[abs((e.endPoint()-e.startPoint()).dot(d)) for e in cut.Edges()]
     assert len(spans)==1 and abs(spans[0]-thickness)<.001
     samples.append(spans[0])
   holes.append(dict(diameter_mm=4.5,axis_origin_mm=o.toTuple(),axis=d.toTuple(),seat_thickness_mm=thickness,seats=seats,annular_ray_thicknesses_mm=samples,
    equivalent_underhead_length_mm=16-thickness+5,reference_screw_protrusion_beyond_contact_mm=16-thickness,
    note='Protrusion is not thread engagement. Calculation assumes same robot contact datum and seating stack.'))
  assert len(holes)==2
  result[kind]=dict(source=str(path.relative_to(ROOT)),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),holes=holes)
 (ROOT/'output/official_screw_seat_measurement.json').write_text(json.dumps(result,indent=2))
 print(json.dumps(result,indent=2))
if __name__=='__main__':
 code=0
 try:run()
 except BaseException:traceback.print_exc();code=1
 sys.stdout.flush();sys.stderr.flush();os._exit(code)
