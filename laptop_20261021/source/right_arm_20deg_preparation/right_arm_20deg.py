"""Supported-rig VR lowcmd: fixed measured body pose, one shoulder nudge.
No AI balancing, mode transition, zero-pose initialization or walking API.
"""
import argparse,dataclasses,json,math,os,signal,threading,time
from pathlib import Path
from arm_runtime import State,SafetyError,angles,numeric
from precise_timing import PreciseSleeper
from shadow_motion_guard import ShadowGuard,ShadowFault

WEAK={4,10,15,16,17,18,22,23,24,25}
WRIST={19,20,21,26,27,28}
def fill(msg,anchor,target22,machine):
    if len(anchor)!=29 or machine!=5:raise SafetyError('Expected fresh G1 29-axis snapshot')
    for v in anchor:numeric(v)
    q=list(anchor);q[22]=numeric(target22);angles(q[15:29])
    if abs(q[22]-anchor[22])>math.radians(20)+1e-8:raise SafetyError('Twenty degree envelope')
    msg.mode_pr=0;msg.mode_machine=machine
    for m in msg.motor_cmd:m.mode=1 # exact local VR header convention
    for i in range(29):
        m=msg.motor_cmd[i];m.q=q[i];m.dq=0.;m.tau=0.
        m.kp,m.kd=(40.,1.5) if i in WRIST else ((80.,3.) if i in WEAK else (300.,3.))
    return msg

class Transport:
    def __init__(self):
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.lock=threading.Lock();self.state=None;self.tick=None;self.error=None
        self.mode=None;self.mode_at=None;self.stop=threading.Event();self.samples=[]
        self.guard=ShadowGuard();self.guard.position_envelope=.03
        ChannelFactoryInitialize(0,'<PRIVATE_IP>')
        self.sub=ChannelSubscriber('rt/lowstate',LowState_);self.sub.Init(self.receive,10)
        self.client=MotionSwitcherClient();self.client.SetTimeout(.3);self.client.Init()
        threading.Thread(target=self.mode_loop,daemon=True).start()
    def receive(self,m):
        try:
            s=State.parse(m,time.perf_counter())
            if self.tick is not None and ((s.robot_tick-self.tick)&0xffffffff)==0:return
            self.tick=s.robot_tick
            with self.lock:self.state=s
            self.guard.observe(s.robot_tick,s.received_at,s.q,s.dq,s.rpy,s.gyro,
                               s.mode_machine,s.motor_modes,0)
        except Exception as e:
            with self.lock:self.error=str(e);self.fault=dataclasses.asdict(s) if 's' in locals() else None
    def mode_loop(self):
        while not self.stop.is_set():
            try:
                code,result=self.client.CheckMode()
                if code!=0 or not isinstance(result,dict) or result.get('name')!='':
                    raise SafetyError('AI is active or debug mode unknown')
                with self.lock:self.mode='';self.mode_at=time.perf_counter()
            except Exception as e:
                with self.lock:self.error=str(e)
                return
            self.stop.wait(.2)
    def check(self,anchor=None,target22=None):
        with self.lock:s=self.state;error=self.error;mode=self.mode;at=self.mode_at
        if error:raise SafetyError(error)
        if s is None or at is None:raise SafetyError('Waiting for state and debug confirmation')
        now=time.perf_counter()
        s.validate(now,5,check_body_velocity=False)
        if mode!='' or not 0<=now-at<=.75:raise SafetyError('Debug status stale/changed')
        if not all(s.live[i] and s.motor_modes[i]==1 for i in range(29)):
            raise SafetyError('All controlled body motors must be live and enabled')
        if anchor is not None:
            for i in range(29):
                limit=.03 if i<15 else math.radians(3)
                expected=target22 if i==22 and target22 is not None else anchor[i]
                if abs(s.q[i]-expected)>limit:raise SafetyError('Uncommanded displacement motor '+str(i))
        return s
    def close(self):self.stop.set();self.sub.Close()

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');args=p.parse_args()
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    crc=CRC();packet=fill(unitree_hg_msg_dds__LowCmd_(),[0.]*29,-math.radians(20),5)
    print(json.dumps({'offline_packet_crc':crc.Crc(packet),'execute':args.execute}),flush=True)
    if not args.execute:return
    logfile=Path('right_arm_20deg_result.json')
    if logfile.exists():raise SafetyError('Trial log already exists; no automatic retry')
    import unitree_sdk2py.core.channel as ch
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_
    ch.ChannelConfigHasInterface=ch.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(Path('supported_lowcmd_cdds.log').resolve()).replace('\\','/'))
    stop=threading.Event();signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
    def watch():
        time.sleep(40);stop.set();time.sleep(5);os._exit(2)
    threading.Thread(target=watch,daemon=True).start()
    def keys():
        import msvcrt
        while not stop.is_set():
            if msvcrt.kbhit() and msvcrt.getwch() in ('\r','\n','\x03'):stop.set();return
            time.sleep(.02)
    threading.Thread(target=keys,daemon=True).start()
    t=None;pub=None;sleeper=PreciseSleeper();report={'sent':0,'error':None,'observations':[],
      'transport':'rt/lowcmd','non_target_behavior':'all29 measured current-pose PD hold; no target displacement except motor22',
      'termination':'lowcmd has no arm_sdk weight release; sender closes; hardware watchdog behavior not assumed',
      'physical_stop_confirmed':False}
    try:
        t=Transport();deadline=time.perf_counter()+8;since=None
        while time.perf_counter()<deadline:
            if stop.is_set():raise SafetyError('Operator stop')
            with t.lock:s=t.state;at=t.mode_at;err=t.error
            if err:raise SafetyError(err)
            if s is not None and at is not None:
                s=t.check()
                if max(abs(x) for x in s.dq[15:29])<.05:
                    since=time.perf_counter() if since is None else since
                    if time.perf_counter()-since>=.5:break
                else:since=None
            sleeper(.02)
        else:raise SafetyError('Initial state/stillness timeout')
        s=t.check();anchor=tuple(s.q);report['initial_state']=dataclasses.asdict(s)
        angles(anchor[15:29]);angles([*anchor[15:22],anchor[22]-math.radians(20),*anchor[23:29]])
        from check_sweep import check as check_sweep
        clearance=check_sweep(anchor)
        report['nominal_geometry_clearance']=clearance
        if not clearance['nominal_geometry_clear']:raise SafetyError('Sweep geometry clearance not established')
        latest=t.check()
        if max(abs(a-b) for a,b in zip(latest.q,anchor))>.01:raise SafetyError('Pose changed during geometry check')
        pub=ch.ChannelPublisher('rt/lowcmd',LowCmd_);pub.Init()
        next_at=None;last_target=None;last_target_at=None
        def send(target):
            nonlocal next_at,last_target,last_target_at
            now=time.perf_counter()
            if stop.is_set():raise SafetyError('Operator stop')
            s=t.check(anchor,target)
            if next_at is not None and now-next_at>.05:raise SafetyError('Deadline exceeded')
            if last_target is not None and abs(target-last_target)>.10*(now-last_target_at)+1e-7:raise SafetyError('Command velocity exceeded')
            last_target=target;last_target_at=now
            msg=fill(unitree_hg_msg_dds__LowCmd_(),anchor,target,s.mode_machine);msg.crc=crc.Crc(msg)
            if pub.Write(msg) is not True:raise SafetyError('DDS Write failed')
            report['sent']+=1
            report['observations'].append(dict(host=time.perf_counter(),unix=time.time(),target22=target,
                 q=list(s.q),dq=list(s.dq),tick=s.robot_tick))
            end=time.perf_counter();next_at=now+.02 if end<now+.02 else end+.02
            sleeper(max(0,next_at-time.perf_counter()))
        def phase(seconds,fn):
            start=time.perf_counter()
            while True:
                f=min(1.,(time.perf_counter()-start)/seconds);send(fn(f))
                if f>=1:return
        phase(1.,lambda f:anchor[22])
        print('Fixed measured-pose hold established; beginning -20 degree shoulder trajectory',flush=True)
        phase(10.,lambda f:anchor[22]-math.radians(20)*(f*f*(3-2*f)))
        phase(1.,lambda f:anchor[22]-math.radians(20))
        phase(10.,lambda f:anchor[22]-math.radians(20)*(1-f*f*(3-2*f)))
        phase(.5,lambda f:anchor[22])
        report['trajectory_completed']=True
    except BaseException as e:report['error']=type(e).__name__+': '+str(e)
    finally:
        if pub is not None:pub.Close()
        report['sender_closed']=True
        if t is not None:
            report['receiver_fault']=getattr(t,'fault',None)
            # No new commands: observe for .5s after closing writer.
            time.sleep(.5)
            with t.lock:final=t.state
            if final is not None:
                report['terminal_state']=dataclasses.asdict(final)
                report['terminal_age_s']=time.perf_counter()-final.received_at
            t.close()
        stop.set();sleeper.close()
        if report['observations']:
            base=report['initial_state']['q'];obs=report['observations']
            report['commanded_peak_deg']=min(math.degrees(x['target22']-base[22]) for x in obs)
            report['measured_peak_deg']=min(math.degrees(x['q'][22]-base[22]) for x in obs)
            report['max_non_target_arm_drift_deg']=max(abs(math.degrees(x['q'][i]-base[i])) for x in obs for i in range(15,29) if i!=22)
            report['max_waist_drift_rad']=max(abs(x['q'][i]-base[i]) for x in obs for i in (12,13,14))
        logfile.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('observations','initial_state','terminal_state','receiver_fault')}),flush=True)
    return 0 if report['error'] is None else 1
if __name__=='__main__':raise SystemExit(main())
