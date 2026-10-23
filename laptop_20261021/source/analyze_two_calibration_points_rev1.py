import json,math,xml.etree.ElementTree as E
from pathlib import Path
import numpy as np
root=Path(__file__).parent
u=E.parse(root/'bottle_camera_discovery/g1_29dof_rev_1_0.urdf').getroot()
names=[s+'_'+j+'_joint' for s in ['left','right'] for j in ['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll']]+['waist_yaw_joint','waist_roll_joint','waist_pitch_joint']+[s+'_'+j+'_joint' for s in ['left','right'] for j in ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']]
def rot(a,t):
 a=np.asarray(a,float);a=a/np.linalg.norm(a);x,y,z=a;K=np.array([[0,-z,y],[z,0,-x],[-y,x,0]]);return np.eye(3)+np.sin(t)*K+(1-np.cos(t))*(K@K)
def tr(xyz,rpy):
 T=np.eye(4);T[:3,:3]=rot([0,0,1],rpy[2])@rot([0,1,0],rpy[1])@rot([1,0,0],rpy[0]);T[:3,3]=xyz;return T
def fk(q):
 poses={'pelvis':np.eye(4)};pending=list(u.findall('joint'))
 while pending:
  n=0
  for j in list(pending):
   par=j.find('parent').get('link');child=j.find('child').get('link')
   if par not in poses:continue
   o=j.find('origin');v=lambda a:list(map(float,o.get(a,'0 0 0').split())) if o is not None else [0,0,0]
   T=tr(v('xyz'),v('rpy'));name=j.get('name')
   if name in names:T[:3,:3]=T[:3,:3]@rot(list(map(float,j.find('axis').get('xyz').split())),q[names.index(name)])
   poses[child]=poses[par]@T;pending.remove(j);n+=1
  if not n:raise ValueError('chain unresolved')
 return poses
Ropt=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
records=[]
for idx,jdir,idir,pix,roi in [(1,'point1_20261021_160025','point1_20261021_160025/unobscured',(431,326),(420,318,443,334)),(2,'point2_20261021_162249','point2_precontact_20261021_151908',(351,298),(346,294,357,303))]:
 p=root/'bottle_camera_discovery';samples=json.loads((p/jdir/'joint_samples.json').read_text())['samples'];meta=json.loads((p/idir/'metadata.json').read_text());d=np.load(p/idir/'depth_aligned.npy');x1,y1,x2,y2=roi;vals=d[y1:y2,x1:x2];z=float(np.median(vals[vals>0])*meta['depth_scale']);i=meta['intrinsics'];opt=np.array([(pix[0]-i['ppx'])*z/i['fx'],(pix[1]-i['ppy'])*z/i['fy'],z]);corr=[];fingers=[];cams=[]
 for s in samples:
  poses=fk(s['q']);f=(poses['right_wrist_yaw_link']@np.r_[.1733,.0283,.0067,1])[:3];C=poses['torso_link']@tr([.0576235,.01753,.42987],[0,.8307767239493009,0]);cam=(C@np.r_[Ropt@opt,1])[:3];corr.append(f-cam);fingers.append(f);cams.append(cam)
 a=np.array(corr);records.append({'point':idx,'pixel':pix,'depth_m':z,'optical_m':opt.tolist(),'camera_nominal_pelvis_m':np.median(cams,axis=0).tolist(),'fingertip_nominal_pelvis_m':np.median(fingers,axis=0).tolist(),'translation_m':np.median(a,axis=0).tolist(),'translation_std_m':np.std(a,axis=0).tolist(),'translation_peak_to_peak_m':np.ptp(a,axis=0).tolist(),'sample_count':len(samples)})
delta=np.array(records[1]['translation_m'])-records[0]['translation_m'];result={'convention':'pelvis x forward y left z up; optical x right y down z forward; correction = FK_fingertip - nominal_camera_point','model':'Exact official configured rev1 commit ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee; existing camera and fingertip nominal configuration','points':records,'correction_difference_m':delta.tolist(),'difference_norm_m':float(np.linalg.norm(delta)),'full_extrinsic_calibration':False}
out=root/'bottle_camera_discovery/two_point_translation_rev1.json';out.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
