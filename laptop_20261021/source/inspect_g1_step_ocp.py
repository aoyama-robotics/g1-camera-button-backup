"""Offline STEP geometry import/tessellation. Source CAD remains untouched."""
from pathlib import Path
import json,time
import numpy as np
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE,TopAbs_REVERSED,TopAbs_SHELL
from OCP.TopoDS import TopoDS
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRep import BRep_Tool
from OCP.TopLoc import TopLoc_Location
from OCP.TopTools import TopTools_IndexedMapOfShape

src=next(Path(r'<LOCAL_USER>\Downloads\29b3cbeb4876420f912af2456a3e9855').glob('*.stp'))
out=Path('bottle_camera_discovery/cad_sources/ocp');out.mkdir(exist_ok=True)
r=STEPControl_Reader();assert r.ReadFile(str(src))==IFSelect_RetDone
print('STEP_READ_OK',flush=True);r.TransferRoots();shape=r.OneShape()
def bbox(s):
 b=Bnd_Box();BRepBndLib.Add_s(s,b);return list(b.Get())
print('GLOBAL_BOUNDS_MM',bbox(shape),flush=True)
solids=[];ex=TopExp_Explorer(shape,TopAbs_SOLID)
while ex.More():solids.append(ex.Current());ex.Next()
solid_count=len(solids)
known=TopTools_IndexedMapOfShape()
for solid in solids:
 sh=TopExp_Explorer(solid,TopAbs_SHELL)
 while sh.More():known.Add(sh.Current());sh.Next()
allsh=TopExp_Explorer(shape,TopAbs_SHELL)
while allsh.More():
 sh=allsh.Current()
 if not known.Contains(sh):solids.append(sh);known.Add(sh)
 allsh.Next()
print('SOLIDS_PLUS_ORPHAN_SHELLS',solid_count,len(solids),flush=True)
triangles=[];ids=[];report=[]
for n,s in enumerate(solids):
 report.append({'id':n,'bounds_mm':bbox(s)})
 BRepMesh_IncrementalMesh(s,1.,False,.35,True)
 f=TopExp_Explorer(s,TopAbs_FACE)
 while f.More():
  face=TopoDS.Face_s(f.Current());loc=TopLoc_Location();mesh=BRep_Tool.Triangulation_s(face,loc)
  if mesh is not None:
   points=[]
   for k in range(1,mesh.NbNodes()+1):
    p=mesh.Node(k).Transformed(loc.Transformation());points.append([p.X(),p.Y(),p.Z()])
   points=np.array(points)
   for k in range(1,mesh.NbTriangles()+1):
    ix=np.array(mesh.Triangle(k).Get())-1
    if face.Orientation()==TopAbs_REVERSED:ix=ix[::-1]
    triangles.append(points[ix]);ids.append(n)
  f.Next()
 extext='TESSELLATED '+str(n+1)+'/'+str(len(solids))
 if n%10==0:print(extext,flush=True)
np.savez_compressed(out/'g1_mesh_mm.npz',triangles=np.array(triangles,dtype=np.float32),solid_id=np.array(ids,dtype=np.int16))
(out/'geometry.json').write_text(json.dumps({'source':str(src),'units':'mm','bounds':bbox(shape),'solids':report,'triangles':len(triangles)},indent=2))
print('DONE_TRIANGLES',len(triangles),flush=True)
