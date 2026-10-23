"""Read-only DDS ring buffer. Explicit file trigger, bounded lifetime, no publisher."""
import collections,json,os,threading,time
from pathlib import Path
import unitree_sdk2py.core.channel as c
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_

out=Path('bottle_camera_discovery/recovery0916_20261021');out.mkdir(exist_ok=True)
trigger=out/'TRIGGER';done=out/'joint_resting.json'
if trigger.exists() or done.exists():raise RuntimeError('Existing trigger/result; preserve prior capture')
lock=threading.Lock();samples=collections.deque(maxlen=30000)
def cb(m):
 now=time.perf_counter()
 with lock:
  if samples and now-samples[-1]['perf_counter']<.02:return
 s={'unix':time.time(),'perf_counter':time.perf_counter(),'tick':m.tick,'mode_machine':m.mode_machine,'q':[x.q for x in m.motor_state[:29]],'dq':[x.dq for x in m.motor_state[:29]],'motor_mode':[x.mode for x in m.motor_state[:29]]}
 with lock:samples.append(s)
c.ChannelConfigHasInterface=c.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((out/'dds.log').resolve()).replace('\\','/'))
c.ChannelFactoryInitialize(0,'<PRIVATE_IP>');sub=c.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(cb,10)
start=time.perf_counter()
while not samples and time.perf_counter()-start<5:time.sleep(.005)
if not samples:raise RuntimeError('No fresh DDS state')
print(json.dumps({'status':'DDS_RESTING_PHOTO_BUFFER_READY','latest_tick':samples[-1]['tick'],'latest_unix':samples[-1]['unix'],'lifetime_s':600}),flush=True)
while not trigger.exists() and time.perf_counter()-start<600:time.sleep(.01)
expired=not trigger.exists()
at=time.time()
with lock:saved=list(samples)
done.write_text(json.dumps({'trigger_unix':at,'saved_count':len(saved),'samples':saved,'motor_commands':0,'correction_frame':'torso','purpose':'Natural resting right-arm photo posture matching only, no extrinsics fitting','expired':expired,'joint_mapping':{'waist_yaw':12,'waist_roll':13,'waist_pitch':14,'right_shoulder_pitch':22,'right_shoulder_roll':23,'right_shoulder_yaw':24,'right_elbow':25,'right_wrist_roll':26,'right_wrist_pitch':27,'right_wrist_yaw':28},'units':'q radians, dq radians/second; unix seconds wall time and perf_counter monotonic seconds','sample_rate_max_hz':50},indent=2))
print(json.dumps({'status':'SAVED','count':len(saved),'path':str(done)}),flush=True);os._exit(0)

