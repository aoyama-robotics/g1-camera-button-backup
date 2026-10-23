import json,time,os,threading
from pathlib import Path
import unitree_sdk2py.core.channel as c
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
p=Path('bottle_camera_discovery')/('point1_'+time.strftime('%Y%m%d_%H%M%S'));p.mkdir()
threading.Timer(8,lambda:os._exit(2)).start()
c.ChannelConfigHasInterface=c.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((p/'dds.log').resolve()).replace('\\','/'))
c.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
samples=[]
lock=threading.Lock()
def cb(m):
 with lock:samples.append({'unix':time.time(),'perf_counter':time.perf_counter(),'tick':m.tick,'mode_machine':m.mode_machine,'q':[v.q for v in m.motor_state[:29]],'dq':[v.dq for v in m.motor_state[:29]],'motor_mode':[v.mode for v in m.motor_state[:29]]})
s=c.ChannelSubscriber('rt/lowstate',LowState_);s.Init(cb,10)
t=time.perf_counter()
while not samples and time.perf_counter()-t<4:time.sleep(.005)
time.sleep(.15)
with lock:saved_samples=list(samples)
r={'samples':saved_samples,'motor_commands':0,'operator_point':'right middle fingertip at robot-facing white label center','correspondence':'PROVISIONAL; no current unobscured depth yet'}
(p/'joint_samples.json').write_text(json.dumps(r,indent=2))
print(json.dumps({'path':str(p),'count':len(saved_samples),'first_unix':saved_samples[0]['unix'] if saved_samples else None}),flush=True)
os._exit(0 if samples else 1)
