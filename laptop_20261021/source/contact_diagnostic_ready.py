"""Read-only DDS ring buffer. Explicit file trigger, bounded lifetime, no publisher."""
import collections,json,os,threading,time
from pathlib import Path
import unitree_sdk2py.core.channel as c
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

out=Path('bottle_camera_discovery/contact_diagnostic_20261021');out.mkdir(exist_ok=True)
trigger=out/'TRIGGER';done=out/'joint_contact.json'
if trigger.exists() or done.exists():raise RuntimeError('Existing trigger/result; preserve prior capture')
lock=threading.Lock();samples=collections.deque(maxlen=15000)
def cb(m):
 s={'unix':time.time(),'perf_counter':time.perf_counter(),'tick':m.tick,'mode_machine':m.mode_machine,'q':[x.q for x in m.motor_state[:29]],'dq':[x.dq for x in m.motor_state[:29]],'motor_mode':[x.mode for x in m.motor_state[:29]]}
 with lock:samples.append(s)
c.ChannelConfigHasInterface=c.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((out/'dds.log').resolve()).replace('\\','/'))
c.ChannelFactoryInitialize(0,'<PRIVATE_IP>');sub=c.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(cb,10)
start=time.perf_counter()
while not samples and time.perf_counter()-start<5:time.sleep(.005)
if not samples:raise RuntimeError('No fresh DDS state')
print(json.dumps({'status':'DDS_READY_NO_CONTACT_SAMPLE','latest_tick':samples[-1]['tick'],'latest_unix':samples[-1]['unix'],'lifetime_s':480}),flush=True)
while not trigger.exists() and time.perf_counter()-start<480:time.sleep(.01)
if not trigger.exists():print('READY_EXPIRED_NO_CONTACT_CAPTURE',flush=True);os._exit(2)
at=time.time();time.sleep(3)
with lock:saved=[s for s in samples if at-5<=s['unix']<=at+3]
done.write_text(json.dumps({'trigger_unix':at,'saved_count':len(saved),'samples':saved,'motor_commands':0,'correction_frame':'torso','contact_reference':'right middle fingertip at robot-facing white label center'},indent=2))
print(json.dumps({'status':'SAVED','count':len(saved),'path':str(done)}),flush=True);os._exit(0)
