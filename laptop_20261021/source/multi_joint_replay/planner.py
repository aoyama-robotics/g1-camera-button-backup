"""Offline right-arm replay planning; no DDS or robot APIs."""
from pathlib import Path
import json,math,sys
import numpy as np
import mujoco
ROOT=Path(__file__).resolve().parent
sys.path.insert(0,str(ROOT.parent/'right_arm_20deg_preparation'))
from check_sweep import NAMES
sys.path.pop(0)
RIGHT=tuple(range(22,29))
class Geometry:
 def __init__(self):
  self.model=mujoco.MjModel.from_xml_path(str(ROOT/'model/rev1_collision.urdf'));self.data=mujoco.MjData(self.model)
  self.address=[int(self.model.joint(n).qposadr[0]) for n in NAMES];self.model.geom_margin[:]=.005
  self.limits=np.array([self.model.jnt_range[self.model.joint(n).id] for n in NAMES])
 def contacts(self,q):
  self.data.qpos[self.address]=q;mujoco.mj_forward(self.model,self.data);result=[]
  for c in self.data.contact:
   a=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,int(self.model.geom_bodyid[c.geom1])) or ''
   b=mujoco.mj_id2name(self.model,mujoco.mjtObj.mjOBJ_BODY,int(self.model.geom_bodyid[c.geom2])) or ''
   moving=lambda x:x.startswith(('right_shoulder','right_elbow','right_wrist'))
   if not (moving(a) or moving(b)):continue
   if {a,b}=={'right_elbow_link','right_wrist_pitch_link'} and c.dist>0:continue
   result.append({'a':a,'b':b,'d':float(c.dist)})
  return result
 def minimum(self,q):return min((x['d'] for x in self.contacts(q)),default=.010)
 def valid(self,q):
  return np.all(np.isfinite(q)) and np.all(q>=self.limits[:,0]-1e-8) and np.all(q<=self.limits[:,1]+1e-8) and self.minimum(q)>=0
 def edge(self,a,b,step_deg=1.):
  n=max(2,int(np.ceil(np.max(np.abs(np.array(b)-a))/math.radians(step_deg)))+1)
  return all(self.valid(np.array(a)+(np.array(b)-a)*t) for t in np.linspace(0,1,n))

def main():
 p=ROOT.parent/'bottle_camera_discovery'
 start=np.array(json.loads((p/'resting_photo_20261021/joint_resting.json').read_text())['samples'][-1]['q'])
 taught=np.array(json.loads((p/'contact_diagnostic_20261021/short_contact_analysis.json').read_text())['q_median'])
 end=start.copy();end[22:29]=taught[22:29];g=Geometry()
 report={'offline_only':True,'model':'official rev1 configured commit; nominal convex collision shapes','start_stale':True,'start_q':start.tolist(),'contact_q':end.tolist(),'non_commanded_indices':list(range(22)),'endpoint_contacts':g.contacts(end),'start_contacts':g.contacts(start),'path_found':False,'external_scene_modeled':False}
 if not g.valid(start) or not g.valid(end):
  report['reason']='Nominal start or exact taught endpoint is not collision-free; no guard exclusions added'
 else:
  # Deterministic clearance proposals: outward roll, elbow flexion, arm reorientation,
  # then gradual approach. q_elbow decreases when physically flexing this model.
  for opening in (10,20,30,40):
   for elbow in (60,45,30,15):
    a=start.copy();a[23]-=math.radians(opening)
    b=a.copy();b[25]=math.radians(elbow)
    c=b.copy();c[22]=end[22];c[24]=end[24];c[26:29]=end[26:29]
    d=c.copy();d[23]=end[23]
    path=[start,a,b,c,d,end]
    if all(g.edge(x,y) for x,y in zip(path,path[1:])):
     report['path_found']=True;report['planning_parameters']={'outward_roll_deg':opening,'elbow_q_deg':elbow};break
   if report['path_found']:break
  if report['path_found']:
   # Preserve the explicit departure/flexion sequence; reverse the same free path.
   report['waypoints_q']=[x.tolist() for x in path]
   report['segment_seconds']=[max(2.,1.5*float(np.max(np.abs(y[22:29]-x[22:29])))/math.radians(3)) for x,y in zip(path,path[1:])]
   report['max_speed_deg_s']=3.;report['retreat']='reverse same path, no waist/leg target displacement'
   points=[x+(y-x)*t for x,y in zip(path,path[1:]) for t in np.linspace(0,1,101)]
   report['dense_samples']=len(points);report['dense_min_distance_m']=min(g.minimum(x) for x in points)
  else:report['reason']='No candidate in bounded outward-roll/elbow-flexion proposal grid; no hardware execution'
 (ROOT/'offline_plan.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ('start_q','contact_q','waypoints_q','start_contacts','endpoint_contacts')},indent=2))
if __name__=='__main__':main()
