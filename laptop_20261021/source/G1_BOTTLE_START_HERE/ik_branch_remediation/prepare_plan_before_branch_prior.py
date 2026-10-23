"""Offline RGB-D target conversion and continuous right-arm planning. No robot I/O."""
import argparse,json,time,sys,math
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'vision_ik'))
from torso_ik import RightArmIK
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--capture',type=Path,required=True);p.add_argument('--state',type=Path,required=True)
p.add_argument('--pixel',nargs=2,type=int,required=True);p.add_argument('--out',type=Path,required=True)
p.add_argument('--correction',type=Path,default=ROOT/'bottle_camera_discovery/scene_refresh_contact_20261021_172741/local_correction_config.json')
a=p.parse_args();OUT=a.out.resolve();OUT.mkdir(parents=True,exist_ok=False)
cam=a.capture.resolve();meta=json.loads((cam/'metadata.json').read_text());depth=np.load(cam/'depth.npy');u,v=a.pixel
if not (2<=u<depth.shape[1]-2 and 2<=v<depth.shape[0]-2):raise ValueError('Target pixel outside supported image interior')
z=depth[v-2:v+3,u-2:u+3];valid=z[z>0]*meta['depth_scale']
if not len(valid):raise ValueError('Target has no valid depth; no guessed coordinate')
z=float(np.median(valid));i=meta['intrinsics'];cx=i.get('cx',i.get('ppx'));cy=i.get('cy',i.get('ppy'))
opt=np.array([(u-cx)*z/i['fx'],(v-cy)*z/i['fy'],z]);pitch=.8307767239493009
R=np.array([[math.cos(pitch),0,math.sin(pitch)],[0,1,0],[-math.sin(pitch),0,math.cos(pitch)]])
Ropt=np.array([[0,0,1],[-1,0,0],[0,-1,0]])
nominal=np.array([.0576235,.01753,.42987])+R@Ropt@opt
cfg=json.loads(a.correction.read_text());bias=np.array(cfg['translation_torso_m']);corrected=nominal+bias
old=dict(pixel=[u,v],depth_m=z,optical_m=opt.tolist(),nominal_torso_m=nominal.tolist(),local_bias_m=bias.tolist(),corrected_torso_m=corrected.tolist(),image_received_unix=meta['received_unix'],rgb_exposure_unix=meta.get('rgb_exposure_unix'),local_correction_validated=False,reference_distance_mm=float(np.linalg.norm(nominal-cfg['reference_nominal_torso_m'])*1000),correction_source=str(a.correction.resolve()))
(OUT/'target_ik_candidate.json').write_text(json.dumps(old,indent=2))
trial=a.state.resolve();text=trial.read_text(encoding='utf-8')
try:record=json.loads(text)
except json.JSONDecodeError:record=json.loads(next(x for x in text.splitlines() if x.startswith('{')))
if 'state' in record:record=record['state']
anchor=np.array([j['q'] for j in record['joints']]);ik=RightArmIK()
target=np.array(old['corrected_torso_m']);rng=np.random.default_rng(87)
lo,hi=ik.g.limits[22:29].T.copy();results=[];started=time.perf_counter()
def solve(seed):
 q=seed.copy();weights=np.array([1,1,1,1,12,12,12])*1e-7
 for it in range(250):
  pos,J=ik.evaluate(q);e=target-pos
  if np.linalg.norm(e)<.0007:break
  prior=q[22:29]-anchor[22:29]
  step=np.linalg.solve(J.T@J+np.diag(weights)+1e-6*np.eye(7),J.T@e-weights*prior)
  step*=min(1.,.15/max(np.max(np.abs(step)),1e-9))
  cost=e@e+np.sum(weights*prior**2)
  accepted=False
  for scale in (1,.5,.25,.1,.03):
   test=q.copy();test[22:29]=np.clip(q[22:29]+scale*step,lo,hi)
   err=target-ik.evaluate(test)[0]
   if err@err+np.sum(weights*(test[22:29]-anchor[22:29])**2)<cost:
    q=test;accepted=True;break
  if not accepted:break
 return q,float(np.linalg.norm(target-ik.evaluate(q)[0]))
for n in range(75):
 seed=anchor.copy()
 if n:
  seed[22:29]=np.clip(anchor[22:29]+rng.normal(0,[.8,.8,.9,.9,.3,.5,.4]),lo,hi)
 q,err=solve(seed);clear=ik.g.minimum(q)
 results.append(dict(q=q.tolist(),error_m=err,min_m=clear,wrist_delta_deg=np.degrees(q[26:29]-anchor[26:29]).tolist()))
 if n>20 and sum(x['error_m']<.001 and x['min_m']>=0 for x in results)>=6:break
valid=[x for x in results if x['error_m']<.001 and x['min_m']>=0]
valid.sort(key=lambda x:np.linalg.norm((np.array(x['q'])-anchor)[22:29]*[1,1,1,1,3,3,3]))
path=None
def edge(a,b):
 n=max(3,int(np.ceil(np.max(np.abs(b-a))/np.radians(1)))+1)
 def valid(q):
  return bool(np.all(q[22:29]>=lo-1e-8) and np.all(q[22:29]<=hi+1e-8) and ik.g.minimum(q)>=0)
 return all(valid(a+(b-a)*t) for t in np.linspace(0,1,n))
for goal in valid[:8]:
 end=np.array(goal['q'])
 # Explicit outward departure / elbow waypoints precede sampled tree search.
 for opening in (0,3,8,18):
  lead=anchor.copy();lead[23]-=np.radians(2)
  a=lead.copy();a[23]=np.clip(a[23]-np.radians(opening),lo[1],hi[1])
  for flex in (10,25,40):
   b=a.copy();b[25]=np.clip(b[25]-np.radians(flex),lo[3],hi[3])
   if edge(anchor,lead) and edge(lead,a) and edge(a,b) and edge(b,end):path=[anchor,lead,a,b,end];break
  if path:break
 if path:break
 if not edge(anchor,anchor):break
 lead=anchor.copy();lead[23]-=np.radians(2)
 if not edge(anchor,lead):continue
 nodes=[lead];parents=[-1]
 for k in range(1600):
  sample=end.copy()
  if k%5:sample[22:29]=rng.uniform(lo,hi)
  idx=int(np.argmin([np.linalg.norm((x-sample)[22:29]) for x in nodes]));near=nodes[idx]
  delta=sample-near;dist=np.linalg.norm(delta[22:29]);new=near+delta*min(1.,.18/max(dist,1e-9))
  if not edge(near,new):continue
  nodes.append(new);parents.append(idx)
  if np.linalg.norm((new-end)[22:29])<.6 and edge(new,end):
   chain=[end];j=len(nodes)-1
   while j>=0:chain.append(nodes[j]);j=parents[j]
   path=[anchor]+list(reversed(chain));break
 if path:break
report=dict(offline_only=True,motor_commands=0,target_torso_m=target.tolist(),source_image=old['image_received_unix'],
 source_joint_trial=str(trial),source_joint_is_saved=True,fresh_status_tick=record['tick'],outward_leadin_deg=2,local_correction_validated=False,
 candidates=len(results),valid_endpoints=len(valid),start_contacts=ik.g.contacts(anchor),path_found=bool(path),
 best_position_error_mm=min(x['error_m'] for x in results)*1000,
 failed_candidates_are_not_impossibility_proof=True,controller_gains_or_guards_changed=False)
trajectory=[]
if path:
 # Reverse exactly the same model-free path; 3 deg/s cubic peak, 20ms samples.
 way=path+list(reversed(path[:-1]));at=0.
 for index,(a,b) in enumerate(zip(way,way[1:])):
  seconds=max(2.,1.5*np.max(np.abs(b[22:29]-a[22:29]))/np.radians(3))
  for t in np.arange(0,seconds,.02):
   f=t/seconds;q=a+(b-a)*(f*f*(3-2*f));trajectory.append(dict(t=at+t,q=q.tolist()))
  at+=seconds
  if index==len(path)-2:
   trajectory.append(dict(t=at,q=b.tolist()));at+=1.
 trajectory.append(dict(t=at,q=anchor.tolist()))
 report.update(waypoints=[q.tolist() for q in way],duration_s=at,
  sampled_min_m=min(ik.g.minimum(np.array(x['q'])) for x in trajectory),
  target_right_deg=np.degrees(path[-1][22:29]).tolist(),right_delta_deg=np.degrees(path[-1][22:29]-anchor[22:29]).tolist(),
  ik_error_mm=float(np.linalg.norm(target-ik.evaluate(path[-1])[0]))*1000,
  execution_ready=False,unresolved='Absolute target uses candidate local correction; extrapolation is unvalidated; physical scene/tool uncertainty remains')
else:
 best=min(results,key=lambda x:x['error_m']+max(0,-x['min_m']))
 report.update(best_candidate=best,execution_ready=False,
  unresolved='No collision-free sampled contact path found in bounded search; absolute correction independently unvalidated')
report['elapsed_s']=time.perf_counter()-started
(OUT/'touch_generation_report.json').write_text(json.dumps(report,indent=2))
(OUT/'touch_trajectory_CANDIDATE.json').write_text(json.dumps(dict(metadata=report,samples=trajectory),indent=2))
(OUT/'touch_ik_candidates.json').write_text(json.dumps(results,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('waypoints','best_candidate')},indent=2))
