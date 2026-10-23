"""No hardware access. Check scoped roll/pitch bounds and unchanged gyro."""
import dataclasses,math,sys,time,json
from pathlib import Path
ROOT=Path(__file__).resolve().parent.parent;sys.path.insert(0,str(ROOT/'continuous_suspended_pitch10_executor'))
from arm_runtime import State,SafetyError
from shadow_motion_guard import ShadowGuard,ShadowFault
q=[0.]*29;state=State(tuple(q),tuple(q),tuple([0]*29),tuple([True]*29),(0,0,0),(0,0,0),5,time.perf_counter(),1)
rows=[]
for axis in [0,1]:
 for name,value,expected in [('minus10',-math.radians(10),True),('plus10',math.radians(10),True),('below10',-math.radians(10)-1e-7,False),('above10',math.radians(10)+1e-7,False)]:
  for layer in ['state','guard']:
   rpy=[0.,0.,0.];rpy[axis]=value;s=dataclasses.replace(state,rpy=tuple(rpy),received_at=time.perf_counter());g=ShadowGuard();g.allowed_motor_modes=(0,1)
   try:
    if layer=='state':s.validate(time.perf_counter(),5,False,True)
    else:g.observe(1,time.perf_counter(),s.q,s.dq,s.rpy,s.gyro,5,s.motor_modes,0)
    accepted=True
   except (SafetyError,ShadowFault):accepted=False
   assert accepted==expected;rows.append(dict(axis=['roll','pitch'][axis],case=name,layer=layer,accepted=accepted))
for name,rpy,gyro in [('gyro',(0,0,0),(.050001,0,0))]:
 s=dataclasses.replace(state,rpy=rpy,gyro=gyro,received_at=time.perf_counter())
 try:s.validate(time.perf_counter(),5,False,True);raise AssertionError(name)
 except SafetyError:rows.append(dict(case=name,accepted=False))
result=dict(result='pass',cases=rows,hardware_commands=0)
(ROOT/'continuous_suspended_pitch10_executor/IMU_BOUNDARY_TESTS.json').write_text(json.dumps(result,indent=2))
print(json.dumps(result))
