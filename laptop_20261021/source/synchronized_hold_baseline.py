"""Read-only 3s nearest-time command/state/torque baseline. No writers."""
import json,os,time,threading,bisect
from pathlib import Path
import numpy as np
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_,LowState_
def watchdog():time.sleep(10);os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('synchronized_cdds.log').resolve()).replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
commands=[];states=[]
def command(m):
    commands.append(dict(host=time.perf_counter(),machine=m.mode_machine,mode_pr=m.mode_pr,
      joints=[dict(mode=x.mode,q=x.q,dq=x.dq,tau=x.tau,kp=x.kp,kd=x.kd) for x in m.motor_cmd[12:29]]))
def state(m):
    states.append(dict(host=time.perf_counter(),tick=m.tick,machine=m.mode_machine,
      joints=[dict(mode=x.mode,q=x.q,dq=x.dq,tau_est=x.tau_est,vol=x.vol) for x in m.motor_state[12:29]],
      rpy=list(m.imu_state.rpy),gyro=list(m.imu_state.gyroscope)))
s=channel.ChannelSubscriber('rt/lowstate',LowState_);s.Init(state,10)
c=channel.ChannelSubscriber('rt/lowcmd',LowCmd_);c.Init(command,10)
time.sleep(3)
c.Close();s.Close()
commands=list(commands);states=list(states)
cmdtimes=[x['host'] for x in commands];pairs=[]
for st in states:
    k=bisect.bisect_right(cmdtimes,st['host'])-1
    if k>=0 and st['host']-cmdtimes[k]<=.005:pairs.append(dict(command=commands[k],state=st,skew_s=st['host']-cmdtimes[k]))
summary=dict(command_count=len(commands),state_count=len(states),pairs_within_5ms=len(pairs),motor_commands_sent=0)
if pairs:
    summary['skew_ms_p50_p99_max']=np.percentile([p['skew_s']*1000 for p in pairs],[50,99,100]).tolist()
    summary['joints']=[]
    for i in range(17):
        cs=[p['command']['joints'][i] for p in pairs];ss=[p['state']['joints'][i] for p in pairs]
        summary['joints'].append(dict(index=i+12,
          command_min={k:min(x[k] for x in cs) for k in cs[0]},
          command_max={k:max(x[k] for x in cs) for k in cs[0]},
          q_command_range=max(x['q'] for x in cs)-min(x['q'] for x in cs),
          q_measured_range=max(x['q'] for x in ss)-min(x['q'] for x in ss),
          max_abs_dq=max(abs(x['dq']) for x in ss),
          tau_est_range=[min(x['tau_est'] for x in ss),max(x['tau_est'] for x in ss)],
          pd_tau_last=cs[-1]['kp']*(cs[-1]['q']-ss[-1]['q'])+cs[-1]['kd']*(cs[-1]['dq']-ss[-1]['dq'])+cs[-1]['tau'],
          last_command=cs[-1],last_state=ss[-1]))
Path('trial2_evidence/synchronized_baseline_raw.json').write_text(json.dumps(dict(commands=commands,states=states)),encoding='utf-8')
Path('trial2_evidence/synchronized_baseline_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary),flush=True);os._exit(0)
