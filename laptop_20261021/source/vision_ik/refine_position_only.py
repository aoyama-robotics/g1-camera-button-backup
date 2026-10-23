"""Offline comparison only; no robot transport or controller changes."""
import json
from pathlib import Path
import numpy as np
from torso_ik import RightArmIK
ROOT=Path(__file__).resolve().parent.parent
out=ROOT/'bottle_camera_discovery/moved_stream1012'
old=json.loads((out/'target_ik_candidate.json').read_text())
status=ROOT/'G1_START_HERE/runs/20261021_191044_974453_status/status_console.txt'
s=json.loads(next(x for x in status.read_text(encoding='utf-8').splitlines() if x.startswith('{')))
anchor=np.array([j['q'] for j in s['joints']]);ik=RightArmIK();target=np.array(old['corrected_torso_m'])

def solve(target,seed,wrist_window):
 q=seed.copy();lo,hi=ik.g.limits[22:29].T.copy()
 lo[4:]=np.maximum(lo[4:],anchor[26:29]-np.radians(wrist_window))
 hi[4:]=np.minimum(hi[4:],anchor[26:29]+np.radians(wrist_window))
 W=np.diag([1.,1.,1.,1.,.08,.08,.08])
 for it in range(350):
  pos,J=ik.evaluate(q);e=target-pos
  if np.linalg.norm(e)<.0005:break
  pinv=W@J.T@np.linalg.solve(J@W@J.T+.007**2*np.eye(3),np.eye(3))
  center=(lo+hi)/2;half=np.maximum((hi-lo)/2,.001)
  regular=(anchor[22:29]-q[22:29])*np.array([1,1,1,1,5,5,5])
  barrier=np.clip((center-q[22:29])/half,-1,1)*np.maximum(0,np.abs((q[22:29]-center)/half)-.8)*.1
  step=pinv@e+.025*(np.eye(7)-pinv@J)@(regular+barrier)
  step*=min(1.,.04/max(np.max(np.abs(step)),1e-12))
  q[22:29]=np.clip(q[22:29]+step,lo,hi)
 return q,float(np.linalg.norm(target-ik.evaluate(q)[0])),it

rows=[]
for window in (10,20,35):
 for roll,elbow in ((0,0),(-10,-15),(-20,-30),(0,-35),(10,-20)):
  seed=anchor.copy();seed[23]+=np.radians(roll);seed[25]+=np.radians(elbow)
  q,err,it=solve(target,seed,window)
  row=dict(wrist_window_deg=window,seed_roll_elbow_delta_deg=[roll,elbow],error_mm=err*1000,iterations=it,
   right_target_deg=np.degrees(q[22:29]).tolist(),right_delta_deg=np.degrees(q[22:29]-anchor[22:29]).tolist())
  if err<.001:
   path=[anchor,q];best=None
   for opening in (0,3,6,10):
    a=anchor.copy();a[23]-=np.radians(opening)
    for flex in (0,10,20):
     b=a.copy();b[25]-=np.radians(flex)
     way=[anchor,a,b,q]
     samples=[x+(y-x)*t for x,y in zip(way,way[1:]) for t in np.linspace(0,1,61)]
     clearance=min(ik.g.minimum(x) for x in samples)
     if best is None or clearance>best['min_m']:
      best=dict(min_m=clearance,opening_deg=opening,elbow_flex_deg=flex,waypoints=[x.tolist() for x in way],samples=len(samples))
   row['best_sampled_path']=best
  rows.append(row)
origin=ik.evaluate(anchor)[0];direction=target-origin;small=origin+.003*direction/np.linalg.norm(direction)
q,err,it=solve(small,anchor,1.)
smallrow=dict(target_torso_m=small.tolist(),error_mm=err*1000,right_delta_deg=np.degrees(q[22:29]-anchor[22:29]).tolist(),
 path_min_m=min(ik.g.minimum(anchor+(q-anchor)*t) for t in np.linspace(0,1,61)),q=q.tolist())
cfg=json.loads((ROOT/'bottle_camera_discovery/scene_refresh_contact_20261021_172741/local_correction_config.json').read_text())
r=dict(offline_only=True,motor_commands=0,orientation_constraint=False,existing_solver_already_position_only=True,
 local_target_distance_from_fitted_reference_mm=float(np.linalg.norm(np.array(old['nominal_torso_m'])-cfg['reference_nominal_torso_m'])*1000),
 local_region_validated=False,full_candidates=rows,small_3mm=smallrow,
 initial_contacts=ik.g.contacts(anchor),no_controller_or_guard_changes=True)
(out/'position_only_refinement.json').write_text(json.dumps(r,indent=2))
print(json.dumps({k:v for k,v in r.items() if k!='full_candidates'},indent=2))
for x in rows:print(json.dumps({k:v for k,v in x.items() if k!='best_sampled_path'}),x.get('best_sampled_path',{}).get('min_m'))
