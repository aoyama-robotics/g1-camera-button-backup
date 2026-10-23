"""Right-arm position IK in torso metres, official rev1 and fixed dummy-tip model.
No communication, motor publishing or camera calibration is performed here.
"""
from pathlib import Path
import sys
import numpy as np,mujoco
ROOT=Path(__file__).resolve().parent.parent
sys.path.insert(0,str(ROOT/'multi_joint_replay'))
from planner import Geometry,NAMES
sys.path.pop(0)
RIGHT=np.arange(22,29)
class RightArmIK:
 def __init__(self):
  self.g=Geometry();self.m=self.g.model;self.d=self.g.data
  self.wrist=self.m.body('right_wrist_yaw_link').id;self.torso=self.m.body('torso_link').id
  self.dofs=np.array([int(self.m.joint(NAMES[k]).dofadr[0]) for k in RIGHT]);self.offset=np.array([.1733,.0283,.0067])
 def evaluate(self,q):
  self.d.qpos[self.g.address]=q;mujoco.mj_forward(self.m,self.d)
  point=self.d.xpos[self.wrist]+self.d.xmat[self.wrist].reshape(3,3)@self.offset
  R=self.d.xmat[self.torso].reshape(3,3);position=R.T@(point-self.d.xpos[self.torso])
  jp=np.zeros((3,self.m.nv));jr=np.zeros((3,self.m.nv));mujoco.mj_jac(self.m,self.d,jp,jr,point,self.wrist)
  return position,R.T@jp[:,self.dofs]
 def solve(self,target,seed,max_iterations=200):
  target=np.asarray(target,float);q=np.array(seed,float).copy();seed=q.copy();lo,hi=self.g.limits[RIGHT].T
  for it in range(max_iterations):
   position,J=self.evaluate(q);error=target-position
   if np.linalg.norm(error)<.0005:return {'success':True,'q':q.tolist(),'error_m':float(np.linalg.norm(error)),'iterations':it}
   pinv=J.T@np.linalg.solve(J@J.T+.015**2*np.eye(3),np.eye(3));step=pinv@error+.03*(np.eye(7)-pinv@J)@(seed[RIGHT]-q[RIGHT]);step*=min(1.,.05/max(np.max(np.abs(step)),1e-12));q[RIGHT]=np.clip(q[RIGHT]+step,lo,hi)
  return {'success':False,'q':q.tolist(),'error_m':float(np.linalg.norm(target-self.evaluate(q)[0])),'iterations':max_iterations}

def main():
 import argparse,json
 p=argparse.ArgumentParser();p.add_argument('--delta-torso',nargs=3,type=float,default=[.003,0,0]);p.add_argument('--seed-file',type=Path,default=ROOT/'bottle_camera_discovery/motion_baseline_20261021/joint_resting.json');a=p.parse_args()
 saved=json.loads(a.seed_file.read_text());seed=saved['samples'][-1]['q'];ik=RightArmIK();origin=ik.evaluate(seed)[0];r=ik.solve(origin+np.array(a.delta_torso),seed);r.update(origin_torso_m=origin.tolist(),target_torso_m=(origin+np.array(a.delta_torso)).tolist(),seed_is_saved_not_live=True,robot_commands=0)
 out=Path(__file__).with_name('ik_solution.json');out.write_text(json.dumps(r,indent=2));print(json.dumps(r,indent=2))
if __name__=='__main__':main()
