"""Bounded 10s full-state baseline; no motor writer/mode mutation."""
import csv, json, os, threading, time
from pathlib import Path
import numpy as np
import unitree_sdk2py.core.channel as channel
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

def watchdog():
    time.sleep(22)
    print('BASELINE_WATCHDOG_TIMEOUT',flush=True);os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('baseline_cdds.log').resolve()).replace('\\','/'))
channel.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
rows=[]
def receive(m):
    rows.append((time.time(),time.perf_counter(),m.tick,m.mode_machine,
         [float(x.q) for x in m.motor_state[:29]],
         [float(x.dq) for x in m.motor_state[:29]],
         list(m.imu_state.rpy),list(m.imu_state.gyroscope),
         [int(x.mode) for x in m.motor_state[:29]],
         [float(x.vol) for x in m.motor_state[:29]]))
sub=channel.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(receive,10)
switcher=MotionSwitcherClient();switcher.SetTimeout(.5);switcher.Init()
loco=LocoClient();loco.SetTimeout(.5);loco.Init()
before={'mode':switcher.CheckMode(),'fsm':loco.GetFsmId()}
print('READ_ONLY_BASELINE 10 seconds; motor commands=0',flush=True)
time.sleep(10)
after={'mode':switcher.CheckMode(),'fsm':loco.GetFsmId()}
sub.Close()
data=list(rows)
summary={'count':len(data),'before':before,'after':after,'motor_commands_sent':0}
if data:
    q=np.array([r[4] for r in data]);dq=np.array([r[5] for r in data])
    host=np.array([r[1] for r in data]);ticks=np.array([r[2] for r in data],dtype=np.int64)
    delta=(np.diff(ticks)+2**32)%2**32
    names=['LHipPitch','LHipRoll','LHipYaw','LKnee','LAnklePitch','LAnkleRoll',
           'RHipPitch','RHipRoll','RHipYaw','RKnee','RAnklePitch','RAnkleRoll',
           'WaistYaw','WaistRoll','WaistPitch']
    summary.update(start_unix=data[0][0],end_unix=data[-1][0],
       duration_s=float(host[-1]-host[0]),
       duplicate_tick_count=int(np.sum(delta==0)),
       tick_delta_ms_percentiles=np.percentile(delta,[50,95,99,100]).tolist(),
       host_interval_ms_percentiles=np.percentile(np.diff(host)*1000,[50,95,99,100]).tolist(),
       mode_machine_values=sorted(set(r[3] for r in data)),
       any_nonfinite=not bool(np.isfinite(q).all() and np.isfinite(dq).all()),
       max_abs_rpy=np.max(np.abs([r[6] for r in data]),axis=0).tolist(),
       max_abs_gyro=np.max(np.abs([r[7] for r in data]),axis=0).tolist(),
       min_voltage=np.min([r[9] for r in data],axis=0).tolist(),
       motor_mode_values=[sorted(set(r[8][i] for r in data)) for i in range(29)])
    summary['joints']=[]
    for i,name in enumerate(names):
        speed=np.abs(dq[:,i]);exceed=speed>.05
        # Finite difference over ~20ms robot tick span for independent q evidence.
        derivatives=[]
        j=0
        for k in range(len(data)):
            while j+1<k and ticks[k]-ticks[j+1]>=20:j+=1
            elapsed=(ticks[k]-ticks[j])/1000
            if .020<=elapsed<=.1:derivatives.append(abs((q[k,i]-q[j,i])/elapsed))
        peak=int(np.argmax(speed))
        summary['joints'].append(dict(index=i,name=name,
          abs_dq_p50_p95_p99_max=np.percentile(speed,[50,95,99,100]).tolist(),
          over_threshold_count=int(exceed.sum()),over_threshold_fraction=float(exceed.mean()),
          q_range_rad=float(np.ptp(q[:,i])),
          abs_20ms_q_velocity_p95_max=(np.percentile(derivatives,[95,100]).tolist() if derivatives else None),
          peak_sample_unix=data[peak][0],peak_tick=data[peak][2]))
    with Path('baseline_samples.csv').open('w',newline='',encoding='utf-8') as f:
        writer=csv.writer(f)
        writer.writerow(['unix_s','host_perf_s','tick','machine']+[f'q{i}' for i in range(29)]+[f'dq{i}' for i in range(29)]+['roll','pitch','yaw','gx','gy','gz'])
        for r in data:writer.writerow([*r[:4],*r[4],*r[5],*r[6],*r[7]])
Path('baseline_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
print(json.dumps(summary),flush=True)
os._exit(0)
