"""One documented AI ReleaseMode call, then read back. No lowcmd writer."""
import json,os,time,threading
from pathlib import Path
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
def watchdog():time.sleep(12);print('RELEASE_READBACK_TIMEOUT',flush=True);os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('supported_vr_release_cdds.log').resolve()).replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
box={}
def state(m):box['state']=m;box['at']=time.perf_counter()
sub=channel.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(state,10)
client=MotionSwitcherClient();client.SetTimeout(.75);client.Init()
before=client.CheckMode()
report={'before':before,'release_called':False,'motor_commands_sent':0}
if before[0]!=0 or not isinstance(before[1],dict):
    report['error']='Cannot establish current mode; no mutation'
elif before[1].get('name'):
    if before[1]['name']!='ai':report['error']='Unexpected active controller; no mutation'
    else:
        report['release_called']=True
        report['release_result']=client.ReleaseMode()
checks=[]
for _ in range(4):
    time.sleep(.3)
    result=client.CheckMode();checks.append(result)
    if result[0]==0 and isinstance(result[1],dict) and not result[1].get('name'):break
report['after_checks']=checks
time.sleep(.5)
if 'state' in box:
    m=box['state'];report['state']=dict(age_s=time.perf_counter()-box['at'],tick=m.tick,machine=m.mode_machine,
       rpy=list(m.imu_state.rpy),gyro=list(m.imu_state.gyroscope),
       joints=[dict(index=i,mode=x.mode,q=x.q,dq=x.dq,vol=x.vol,tau_est=x.tau_est) for i,x in enumerate(m.motor_state[:29])])
Path(__file__).with_name('after_operator_release.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report),flush=True);os._exit(0)
