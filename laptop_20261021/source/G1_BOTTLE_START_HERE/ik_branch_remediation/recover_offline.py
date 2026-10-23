"""Saved-data-only branch audit. Never imports a robot transport."""
import json,sys,time
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent.parent
sys.path.insert(0,str(ROOT/'vision_ik'))
from torso_ik import RightArmIK
old=json.loads((ROOT/'bottle_camera_discovery/reboot1052/touch_trajectory_CANDIDATE.json').read_text())
new=json.loads((ROOT/'bottle_camera_discovery/before_continuous1059/touch_trajectory_CANDIDATE.json').read_text())
a=np.array(new['samples'][0]['q']);target=np.array(new['metadata']['target_torso_m']);ik=RightArmIK()
lo,hi=ik.g.limits[22:29].T;rng=np.random.default_rng(890);results=[]
oldend=np.array(old['metadata']['waypoints'][4]);seeds=[a.copy()]
s=a.copy();s[22:29]=oldend[22:29];seeds.append(s)
for i in range(36):
 s=seeds[i%2].copy();s[22:29]=np.clip(s[22:29]+rng.normal(0,[.25,.2,.25,.3,.1,.15,.1]),lo,hi);seeds.append(s)
for n,s in enumerate(seeds):
 # Position continuation retains the preceding nearby solution instead of random global restarts.
 origin=ik.evaluate(s)[0];q=s.copy();success=True
 for f in np.linspace(0,1,16)[1:]:
  r=ik.solve(origin+(target-origin)*f,q,max_iterations=350);q=np.array(r['q'])
  if not r['success']:success=False;break
 err=float(np.linalg.norm(target-ik.evaluate(q)[0]));clear=float(ik.g.minimum(q))
 results.append(dict(seed=n,q=q.tolist(),error_m=err,min_m=clear,continuation_success=success,
  weighted_distance=float(np.linalg.norm((q-a)[22:29]*[1,1,1,1,3,3,3]))))
for item in results:
 # Historical endpoint is a posture/branch prior only; current target and saved new anchor remain unchanged.
 item['reference_branch_distance']=float(np.linalg.norm((np.array(item['q'])-oldend)[22:29]))
valid=sorted([x for x in results if x['error_m']<.001 and x['min_m']>=0],key=lambda x:x['reference_branch_distance'])
def edge(x,y):
 for t in np.linspace(0,1,max(3,int(np.ceil(np.max(np.abs(y-x))/np.radians(.5)))+1)):
  q=x+(y-x)*t
  if np.any(q[22:29]<lo) or np.any(q[22:29]>hi) or ik.g.minimum(q)<0:return False
 return True
path=None
for item in valid:
 goal=np.array(item['q']);lead=a.copy();lead[23]-=np.radians(2)
 for opening in (0,3,8,18):
  w=lead.copy();w[23]=np.clip(w[23]-np.radians(opening),lo[1],hi[1])
  for flex in (10,25,40):
   b=w.copy();b[25]=np.clip(b[25]-np.radians(flex),lo[3],hi[3])
   trial=[a,lead,w,b,goal]
   if all(edge(x,y) for x,y in zip(trial,trial[1:])):path=trial;break
  if path:break
 if path:break
report=dict(offline_only=True,motor_commands=0,source_new='before_continuous1059',source_old='reboot1052',
 target_torso_m=target.tolist(),source_state_is_stale=True,local_correction_validated=False,
 old_start_right_deg=np.degrees(np.array(old['samples'][0]['q'])[22:29]).tolist(),
 new_start_right_deg=np.degrees(a[22:29]).tolist(),valid_endpoints=len(valid),path_found=path is not None,
 candidates=results,execution_ready=False,selection='distance to historical endpoint branch; never replay historical start/target')
if path:
 way=path+list(reversed(path[:-1]));samples=[];at=0
 for index,(x,y) in enumerate(zip(way,way[1:])):
  seconds=max(2.,1.5*np.max(np.abs(y[22:29]-x[22:29]))/np.radians(3))
  for t in np.arange(0,seconds,.02):
   f=t/seconds;samples.append(dict(t=float(at+t),q=(x+(y-x)*(f*f*(3-2*f))).tolist()))
  at+=seconds
  if index==len(path)-2:samples.append(dict(t=at,q=y.tolist()));at+=1
 samples.append(dict(t=at,q=a.tolist()))
 Q=np.array([s['q'] for s in samples]);T=np.array([s['t'] for s in samples])
 assert np.all(np.diff(T)>0) and np.allclose(Q[0],Q[-1]) and np.allclose(Q[:,:22],a[:22])
 assert np.max(np.abs(np.diff(Q[:,22:29],axis=0)/np.diff(T)[:,None]))<=np.radians(3)+1e-7
 assert all(ik.g.minimum(q)>=0 for q in Q)
 report.update(duration_s=at,waypoints=[q.tolist() for q in way],
  right_delta_deg=np.degrees(path[-1][22:29]-a[22:29]).tolist(),
  ik_error_mm=float(np.linalg.norm(target-ik.evaluate(path[-1])[0])*1000),
  sampled_min_m=float(min(ik.g.minimum(q) for q in Q)),validation='joint limits,0.5degree edges,20ms samples,speed3deg/s,non-target fixed,return checked')
 (HERE/'CONTINUITY_CANDIDATE_OFFLINE.json').write_text(json.dumps(dict(metadata={k:v for k,v in report.items() if k!='candidates'},samples=samples),indent=2))
(HERE/'BRANCH_AUDIT.json').write_text(json.dumps(report,indent=2))
print(json.dumps({k:v for k,v in report.items() if k not in ('candidates','waypoints')},indent=2),flush=True)
