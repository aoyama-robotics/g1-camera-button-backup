"""Offline orthographic CAD preview, millimetre source geometry."""
import numpy as np,json
from pathlib import Path
from PIL import Image,ImageDraw
p=Path('bottle_camera_discovery/cad_sources/ocp');a=np.load(p/'g1_mesh_mm.npz');tri=a['triangles'];ids=a['solid_id']
print('MESH',tri.shape,flush=True)
def render(name,H,V,D,region=None):
 t=tri.copy();si=ids.copy()
 if region is not None:
  keep=(t[:,:,V].max(1)>=region[0])&(t[:,:,V].min(1)<=region[1]);t=t[keep];si=si[keep]
 lo=t.reshape(-1,3).min(0);hi=t.reshape(-1,3).max(0)
 w,h=1000,1200;scale=min((w-80)/(hi[H]-lo[H]),(h-110)/(hi[V]-lo[V]));u=(t[:,:,H]-(hi[H]+lo[H])/2)*scale+w/2;v=h-45-(t[:,:,V]-lo[V])*scale
 n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);nn=np.linalg.norm(n,axis=1);n=n/np.maximum(nn[:,None],1e-9)
 shade=.35+.65*np.abs(n@np.array([.5,-.35,.79]));order=np.argsort(t[:,:,D].mean(1))
 im=Image.new('RGB',(w,h),'white');draw=ImageDraw.Draw(im)
 for j in order:
  c=int(np.clip(shade[j]*205+20,30,245));draw.polygon(list(zip(u[j],v[j])),fill=(c,c,int(c*.97)))
 draw.text((18,15),name+' | official downloaded simplified STEP | mm',fill='black')
 for k in np.unique(si):
  sub=t[si==k];cent=sub.reshape(-1,3).mean(0);x=(cent[H]-(hi[H]+lo[H])/2)*scale+w/2;y=h-45-(cent[V]-lo[V])*scale
  draw.text((x,y),str(k),fill=(180,15,15))
 im.save(p/(name+'.png'));print(name,lo.tolist(),hi.tolist(),flush=True)
render('front',1,2,0)
render('side',0,2,1)
top=float(tri[:,:,2].max());render('head_front',1,2,0,(top-260,top));render('head_side',0,2,1,(top-260,top))
