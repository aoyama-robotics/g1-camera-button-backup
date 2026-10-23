"""Subscribe ONLY to existing lowcmd and lowstate, 2 seconds, no writers."""
import json,os,threading,time
from pathlib import Path
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_
def watchdog():
    time.sleep(8);print('READONLY_TIMEOUT',flush=True);os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('controller_readonly_cdds.log').resolve()).replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
box={'cmd_count':0,'state_count':0}
def cmd(m):box['cmd_count']+=1;box['cmd']=m;box['cmd_time']=time.time()
def state(m):box['state_count']+=1;box['state']=m;box['state_time']=time.time()
s=channel.ChannelSubscriber('rt/lowstate',LowState_);s.Init(state,10)
c=channel.ChannelSubscriber('rt/lowcmd',LowCmd_);c.Init(cmd,10)
time.sleep(2)
result={'context':'post-trial current controller; not trial-time capture','motor_commands_sent':0,'cmd_count':box['cmd_count'],'state_count':box['state_count']}
if 'cmd' in box:
    m=box['cmd'];result['command']=dict(time=box['cmd_time'],mode_pr=m.mode_pr,mode_machine=m.mode_machine,
      joints=[dict(index=i,mode=m.motor_cmd[i].mode,q=m.motor_cmd[i].q,dq=m.motor_cmd[i].dq,tau=m.motor_cmd[i].tau,kp=m.motor_cmd[i].kp,kd=m.motor_cmd[i].kd) for i in range(12,30)])
if 'state' in box:
    m=box['state'];result['state']=dict(time=box['state_time'],tick=m.tick,mode_machine=m.mode_machine,
      joints=[dict(index=i,mode=m.motor_state[i].mode,q=m.motor_state[i].q,dq=m.motor_state[i].dq,tau_est=m.motor_state[i].tau_est) for i in range(12,29)])
Path('trial2_evidence/post_trial_controller_readonly.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result),flush=True);os._exit(0)
