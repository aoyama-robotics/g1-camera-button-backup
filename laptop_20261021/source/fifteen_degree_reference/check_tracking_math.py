import math,time,threading
from fifteen_degree_reference import Transport
from arm_runtime import State,SafetyError
anchor=[0.]*29
obj=Transport.__new__(Transport);obj.lock=threading.Lock();obj.error=None;obj.mode='';obj.startup_at=time.perf_counter()-.5;obj.all_enabled=True
for returning in (False,True):
 for f in (0.,.5,1.):
  blend=f*f*(3-2*f);target=-math.radians(15)*((1-blend) if returning else blend)
  for err,should_pass in ((0.,True),(math.radians(2.9),True),(math.radians(3.1),False)):
   q=anchor.copy();q[22]=target+err;now=time.perf_counter();obj.mode_at=now
   obj.state=State(tuple(q),(0.,)*29,(1,)*29,(True,)*29,(0.,)*3,(0.,)*3,5,now,100)
   try:obj.check(anchor,target);passed=True
   except SafetyError:passed=False
   assert passed==should_pass,(returning,f,err,passed)
now=time.perf_counter();obj.mode_at=now;q=anchor.copy();q[23]=math.radians(3.1)
obj.state=State(tuple(q),(0.,)*29,(1,)*29,(True,)*29,(0.,)*3,(0.,)*3,5,now,100)
try:obj.check(anchor,0)
except SafetyError:pass
else:raise AssertionError('Non-target 3 degree guard changed')
print('PASS: start/mid/end outgoing and returning use current command; 3deg threshold and other-joint reference preserved')
