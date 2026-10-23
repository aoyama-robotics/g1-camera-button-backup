"""Four-second raw state + ownership read; no publisher or mode mutation."""
import json,time,threading
from pathlib import Path
import unitree_sdk2py.core.channel as c
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
out=Path('bottle_camera_discovery/ankle5_readonly_1049');out.mkdir(exist_ok=True)
if (out/'samples.json').exists():raise RuntimeError('Preserve prior data')
c.ChannelConfigHasInterface=c.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((out/'dds.log').resolve()).replace('\\','/'))
c.ChannelFactoryInitialize(0,'<PRIVATE_IP>');samples=[];lock=threading.Lock()
def callback(m):
 s=dict(unix=time.time(),perf=time.perf_counter(),tick=m.tick,q=[x.q for x in m.motor_state[:29]],dq=[x.dq for x in m.motor_state[:29]],motor_modes=[x.mode for x in m.motor_state[:29]],machine=m.mode_machine,rpy=list(m.imu_state.rpy),gyro=list(m.imu_state.gyroscope))
 with lock:samples.append(s)
sub=c.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(callback,10)
client=MotionSwitcherClient();client.SetTimeout(.5);client.Init();before=client.CheckMode();time.sleep(4);after=client.CheckMode();sub.Close()
with lock:saved=list(samples)
r=dict(before=before,after=after,samples=saved,motor_commands=0)
(out/'samples.json').write_text(json.dumps(r,indent=2));print(json.dumps(dict(count=len(saved),before=before,after=after,motor_commands=0)),flush=True)
