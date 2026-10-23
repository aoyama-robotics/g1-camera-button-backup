"""Offline-only packet/trajectory review. Never constructs a transport."""
import importlib.util,json,sys,math
from pathlib import Path
import numpy as np
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
sys.path.insert(0,str(HERE))
spec=importlib.util.spec_from_file_location('continuous_review',HERE/'continuous_touch.py')
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
sys.path.insert(0,str(ROOT/'vision_ik'))
from torso_ik import RightArmIK
ik=RightArmIK();t=m.TIMES;Q=np.array([s['q'] for s in m.PLAN]);a=Q[0]
assert np.all(np.diff(t)>0)
assert np.all(Q[:,22:29]>=ik.g.limits[22:29,0]-1e-8)
assert np.all(Q[:,22:29]<=ik.g.limits[22:29,1]+1e-8)
assert m.TOTAL_DEADLINE_S==math.ceil(t[-1]+14)==146
assert np.allclose(m.reference(a,0),a[22:29])
assert np.allclose(m.reference(a,1),Q[-1,22:29])
assert np.allclose(Q[-1],a)
for k in np.linspace(0,len(Q)-1,19).astype(int):
 msg=m.fill(unitree_hg_msg_dds__LowCmd_(),a.tolist(),Q[k,22:29].tolist(),5)
 for i in range(29):
  assert abs(msg.motor_cmd[i].q-(Q[k,i] if i>=22 else a[i]))<1e-6
  expected=(40.,1.5) if i in m.WRIST else ((80.,3.) if i in m.WEAK else (300.,3.))
  assert (msg.motor_cmd[i].kp,msg.motor_cmd[i].kd)==expected
  assert msg.motor_cmd[i].dq==msg.motor_cmd[i].tau==0
code=(HERE/'continuous_touch.py').read_text()
for token in ['time.sleep(TOTAL_DEADLINE_S)',"mark_stop('keyboard')","mark_stop('SIGINT')",'now-next_at>.05','math.radians(3)','now-at<=.75','>.25','Observed nominal mesh penetration']:
 assert token in code,token
r=dict(hardware_commands=0,result='pass',samples=len(Q),planned_duration_s=float(t[-1]),total_deadline_s=m.TOTAL_DEADLINE_S,
 allowance='10s startup incl8s initial wait;1s initial hold;1s smooth start transition;.5s final hold;.5s observation;1s reserve',
 max_sampled_speed_deg_s=float(np.max(np.abs(np.diff(Q[:,22:29],axis=0)/np.diff(t)[:,None]))*180/math.pi),
 joint_limits='pass',non_target_fixed='pass',original_gains='pass',start_end_same='pass',
 unchanged='communication freshness,50ms deadline,3degree tracking,operator/signal stops,shape rejection',
 pending='fresh pose/scene/model validity and parent review of total deadline; hardware NOT executed')
(HERE/'OFFLINE_VALIDATION.json').write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
