import ast
from pathlib import Path
from arm_runtime import State,SafetyError
from shadow_motion_guard import ShadowGuard,ShadowFault
p=Path(__file__).resolve().parent
old=ast.parse((p.parent/'successful_supported_vr_local'/'supported_vr_lowcmd.py').read_text())
new=ast.parse((p/'one_degree_reference.py').read_text())
f=lambda tree:next(x for x in tree.body if isinstance(x,ast.FunctionDef) and x.name=='fill')
assert ast.dump(f(old))==ast.dump(f(new))
s=State((0.,)*29,(0.,)*29,(0,)*29,(True,)*29,(0.,)*3,(0.,)*3,5,1.,100)
s.validate(1.,check_body_velocity=False,allow_startup_modes=True)
try:s.validate(1.,check_body_velocity=False)
except SafetyError:pass
else:raise AssertionError('Strict mode requirement lost')
g=ShadowGuard();g.position_envelope=.03;g.allowed_motor_modes=(0,1)
g.observe(100,1.,s.q,s.dq,s.rpy,s.gyro,5,s.motor_modes)
g.allowed_motor_modes=(1,)
try:g.observe(101,1.001,s.q,s.dq,s.rpy,s.gyro,5,s.motor_modes)
except ShadowFault:pass
else:raise AssertionError('Mode-drop guard lost')
g=ShadowGuard();g.position_envelope=.03;dq=list(s.dq);dq[2]=.1
g.observe(1000,1.,s.q,dq,s.rpy,s.gyro,5,(1,)*29)
try:g.observe(1020,1.02,s.q,dq,s.rpy,s.gyro,5,(1,)*29)
except ShadowFault as e:assert 'persistent_dq' in str(e)
else:raise AssertionError('Persistence stop lost')
print('PASS: original packet/gains identical; startup mode0 acceptance bounded by executor; strict mode and persistence guard retained')
