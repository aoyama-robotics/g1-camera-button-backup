from pathlib import Path
import json
import os,sys
import cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
root=Path(__file__).resolve().parents[1]
data={}
for typ in ['classic','renewed']:
    p=next((root/'sources'/typ).rglob('stero-camera-mount*.step'))
    s=cq.importers.importStep(str(p)).val()
    bb=s.BoundingBox()
    faces=[]
    for f in s.Faces():
        a=BRepAdaptor_Surface(f.wrapped)
        if a.GetType()==GeomAbs_Cylinder:
            c=a.Cylinder();v=c.Axis().Direction();o=c.Location()
            faces.append(dict(radius=c.Radius(),origin=[o.X(),o.Y(),o.Z()],axis=[v.X(),v.Y(),v.Z()],center=f.Center().toTuple(),area=f.Area()))
    data[typ]=dict(source=str(p.relative_to(root)),bounds=[bb.xmin,bb.xmax,bb.ymin,bb.ymax,bb.zmin,bb.zmax],cylinders=faces)
    cq.exporters.export(s,str(root/'output'/f'{typ}_official_bracket.stl'))
(root/'output/bracket_measurements.json').write_text(json.dumps(data,indent=2))
print('Saved full analytical face measurements for classic and renewed brackets.')
sys.stdout.flush();sys.stderr.flush();os._exit(0)
