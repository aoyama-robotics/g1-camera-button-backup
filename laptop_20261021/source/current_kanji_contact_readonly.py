"""Bounded read-only contact buffer; camera completion controls finalization."""
import collections,json,threading,time
from pathlib import Path
import unitree_sdk2py.core.channel as c
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
out=Path('bottle_camera_discovery/current_kanji_contact_1020');out.mkdir(exist_ok=True)
if (out/'joint_contact.json').exists():raise RuntimeError('Preserve existing recording')
lock=threading.Lock();samples=collections.deque(maxlen=30000)
def cb(m):
 now=time.perf_counter()
 with lock:
  if samples and now-samples[-1]['monotonic']<.01:return
  samples.append(dict(unix=time.time(),monotonic=now,tick=m.tick,machine=m.mode_machine,
   q=[x.q for x in m.motor_state[:29]],dq=[x.dq for x in m.motor_state[:29]],motor_modes=[x.mode for x in m.motor_state[:29]]))
c.ChannelConfigHasInterface=c.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((out/'dds.log').resolve()).replace('\\','/'))
c.ChannelFactoryInitialize(0,'<PRIVATE_IP>');sub=c.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(cb,10)
start=time.perf_counter()
while not samples and time.perf_counter()-start<5:time.sleep(.01)
if not samples:raise RuntimeError('No LowState')
print(json.dumps(dict(status='READY',lifetime_s=600,path=str(out),latest_unix=samples[-1]['unix'])),flush=True)
while not (out/'CAMERA_DONE').exists() and time.perf_counter()-start<600:time.sleep(.01)
complete=(out/'CAMERA_DONE').exists();time.sleep(.5)
with lock:saved=list(samples)
sub.Close()
(out/'joint_contact.json').write_text(json.dumps(dict(samples=saved,saved_count=len(saved),camera_completed=complete,
 trigger_present=(out/'TRIGGER').exists(),motor_commands=0,units='radians, radians/s, Unix UTC seconds',
 mapping=dict(waist=[12,13,14],right_arm=[22,23,24,25,26,27,28])),indent=2))
print(json.dumps(dict(status='SAVED',saved_count=len(saved),camera_completed=complete)),flush=True)
