"""Separate, finite direct-start experiment. No intentional pose displacement.
Original successful controller and mode1 requirement remain unchanged.
"""
import argparse,dataclasses,json,math,os,signal,threading,time
from pathlib import Path
from arm_runtime import State,SafetyError,angles,numeric
from precise_timing import PreciseSleeper
from engagement_proposal_offline import gains
from coordinated_geometry import Geometry
TOTAL_SECONDS=40.0

LOG=Path(__file__).with_name('coordinated_raise_result.json')

def packet(anchor,machine,elapsed):
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    if len(anchor)!=29 or machine!=5:raise SafetyError('Unexpected body configuration')
    angles(anchor[15:29])
    m=unitree_hg_msg_dds__LowCmd_();m.mode_pr=0;m.mode_machine=machine
    for i in range(29):
        c=m.motor_cmd[i];c.mode=1;c.q=numeric(anchor[i]);c.dq=0.;c.tau=0.
        gain_time=elapsed if elapsed<=1.5 else (1.5 if elapsed<=39 else elapsed-37)
        c.kp,c.kd=gains(i,gain_time)
    m.crc=CRC().Crc(m)
    return m

def main():
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');a=p.parse_args()
    for t in (0.,1.,1.5,2.,2.5,3.):
        m=packet([0.]*29,5,t)
        assert all(c.q==0 and c.tau==0 and c.dq==0 for c in m.motor_cmd[:29])
        assert max(c.kp for c in m.motor_cmd[:29])<=30.
    print('Offline packet checks passed; execute='+str(a.execute),flush=True)
    if not a.execute:return 0
    if LOG.exists():raise SafetyError('Prior result exists; no automatic retry')
    import unitree_sdk2py.core.channel as ch
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_,LowCmd_
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    ch.ChannelConfigHasInterface=ch.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(LOG.with_suffix('.dds.log')).replace('\\','/'))
    ch.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
    lock=threading.Lock();box={};stop=threading.Event();sleeper=PreciseSleeper()
    report={'sent':0,'error':None,'observations':[],'intentional_pitch_deg':-20,'intentional_roll_deg':None,
            'gain_fraction_max':.1,'physical_stop_confirmed':False}
    pub=None;sub=None
    signal.signal(signal.SIGINT,lambda *_:stop.set())
    def hard_timeout():
        if not stop.wait(55):stop.set();time.sleep(2);os._exit(2)
    threading.Thread(target=hard_timeout,daemon=True).start()
    def key_stop():
        import msvcrt
        while not stop.wait(.02):
            if msvcrt.kbhit() and msvcrt.getwch() in ('\r','\n','\x03'):stop.set()
    threading.Thread(target=key_stop,daemon=True).start()
    def receive(m):
        try:
            s=State.parse(m,time.perf_counter())
            with lock:
                if box.get('tick')==s.robot_tick:return
                box.update(state=s,tick=s.robot_tick)
        except Exception as e:
            with lock:box['error']=str(e)
    client=MotionSwitcherClient();client.SetTimeout(.3);client.Init()
    def ownership():
        while not stop.is_set():
            try:
                code,result=client.CheckMode()
                if code!=0 or not isinstance(result,dict) or result.get('name')!='':
                    raise SafetyError('AI ownership or control status changed')
                with lock:box['ownership_at']=time.perf_counter();box['ownership']=result
            except Exception as e:
                with lock:box['error']=str(e)
                return
            stop.wait(.2)
    def check(anchor=None,expected=None,elapsed=0):
        with lock:s=box.get('state');err=box.get('error');at=box.get('ownership_at')
        if err:raise SafetyError(err)
        now=time.perf_counter()
        if s is None or at is None:raise SafetyError('Waiting for fresh ownership/state')
        if not 0<=now-s.received_at<=.2 or not 0<=now-at<=.75:raise SafetyError('Stale telemetry/ownership')
        if s.mode_machine!=5 or not all(s.live):raise SafetyError('Invalid machine or unavailable motors')
        for v in (*s.q,*s.dq,*s.rpy,*s.gyro):numeric(v)
        if any(v not in (0,1) for v in s.motor_modes):raise SafetyError('Unexpected motor mode')
        angles(s.q[15:29])
        if max(abs(v) for v in s.rpy[:2])>.1 or max(abs(v) for v in s.gyro)>.05:raise SafetyError('IMU anomaly')
        if max(abs(v) for v in s.dq)>.1:raise SafetyError('Fast joint motion')
        if anchor is not None:
            for i in range(29):
                limit=math.radians(3) if i==22 and 10<=elapsed<=31 else (math.radians(1) if i==23 else .002)
                goal=expected[i] if expected is not None else anchor[i]
                if abs(s.q[i]-goal)>limit:raise SafetyError('Tracking/non-target displacement motor '+str(i))
            if s.q[23]>anchor[23]+.001:raise SafetyError('Inward motion toward body')
        if stop.is_set():raise SafetyError('Operator/watchdog stop')
        return s
    try:
        sub=ch.ChannelSubscriber('rt/lowstate',LowState_);sub.Init(receive,10)
        threading.Thread(target=ownership,daemon=True).start()
        deadline=time.perf_counter()+5;stable_since=None
        while True:
            if time.perf_counter()>deadline:raise SafetyError('Initial stationary state timeout')
            with lock:ready='state' in box and 'ownership_at' in box;err=box.get('error')
            if err:raise SafetyError(err)
            if ready:
                s=check()
                if max(abs(v) for v in s.dq)<.05:
                    stable_since=stable_since or time.perf_counter()
                    if time.perf_counter()-stable_since>=.5:break
                else:stable_since=None
            sleeper(.02)
        s=check();anchor=s.q;report['initial_state']=dataclasses.asdict(s)
        geometry=Geometry();plan=geometry.plan(anchor);report['geometry_plan']=plan
        report['intentional_roll_deg']=-plan['opening_deg']
        print('Plan: roll outward '+str(plan['opening_deg'])+'deg /8s; pitch -20deg /10s; hold1s; pitch return10s; roll reverse8s; gain ramp down1s',flush=True)
        check(anchor,anchor)
        def smooth(x):
            x=max(0.,min(1.,x));return x*x*(3-2*x)
        def targets(t):
            a=list(anchor);opening=math.radians(plan['opening_deg'])
            if t<10:a[23]-=opening*smooth((t-2)/8)
            elif t<31:a[23]-=opening
            elif t<39:a[23]-=opening*(1-smooth((t-31)/8))
            if 10<=t<20:a[22]-=math.radians(20)*smooth((t-10)/10)
            elif 20<=t<21:a[22]-=math.radians(20)
            elif 21<=t<31:a[22]-=math.radians(20)*(1-smooth((t-21)/10))
            angles(a[15:29]);return a
        for value in (0,2,10,20,21,31,39,40):targets(value)
        pub=ch.ChannelPublisher('rt/lowcmd',LowCmd_);pub.Init()
        start=time.perf_counter();due=start;latched=[False]*29;confirmed=False
        while True:
            now=time.perf_counter();elapsed=min(TOTAL_SECONDS,now-start)
            if now-due>.05:raise SafetyError('Send deadline exceeded')
            expected=targets(elapsed)
            s=check(anchor,expected,elapsed)
            if geometry.minimum(s.q)<.002:raise SafetyError('Actual nominal mesh clearance below 2mm')
            for i,v in enumerate(s.motor_modes):
                if latched[i] and v!=1:raise SafetyError('Enabled motor dropped mode:'+str(i))
                latched[i]=latched[i] or v==1
            confirmed=all(latched)
            if now-start>.25 and not confirmed:raise SafetyError('Direct enable not confirmed within 250ms')
            msg=packet(expected,s.mode_machine,elapsed)
            if pub.Write(msg) is not True:raise SafetyError('DDS Write failed')
            report['sent']+=1
            report['observations'].append(dict(host=now,tick=s.robot_tick,q=s.q,dq=s.dq,motor_modes=s.motor_modes,target_q=expected,kp=[c.kp for c in msg.motor_cmd[:29]],kd=[c.kd for c in msg.motor_cmd[:29]]))
            if elapsed>=TOTAL_SECONDS:break
            due=now+.02;sleeper(max(0,due-time.perf_counter()))
        report['coordinated_trajectory_completed']=True;report['enable_confirmed']=confirmed
    except BaseException as e:report['error']=type(e).__name__+': '+str(e)
    finally:
        if pub is not None:pub.Close()
        report['sender_closed']=True
        time.sleep(.5)
        with lock:final=box.get('state');owner=box.get('ownership')
        if final is not None:
            report['terminal_state']=dataclasses.asdict(final);report['terminal_age_s']=time.perf_counter()-final.received_at
        report['terminal_ownership']=owner
        stop.set()
        if sub is not None:sub.Close()
        sleeper.close()
        if report['observations']:
            base=report['initial_state']['q']
            report['pitch_peak_deg']=min(math.degrees(x['q'][22]-base[22]) for x in report['observations'])
            report['roll_peak_deg']=min(math.degrees(x['q'][23]-base[23]) for x in report['observations'])
            report['pitch_return_error_deg']=math.degrees(report['observations'][-1]['q'][22]-base[22])
            report['roll_return_error_deg']=math.degrees(report['observations'][-1]['q'][23]-base[23])
            report['max_displacement_deg']=max(abs(math.degrees(x['q'][i]-base[i])) for x in report['observations'] for i in range(29))
        LOG.write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('initial_state','terminal_state','observations')}),flush=True)
        if final is not None:print(json.dumps({'terminal_modes':final.motor_modes,'terminal_age_s':report['terminal_age_s']}),flush=True)
    return 0 if report['error'] is None else 1

if __name__=='__main__':raise SystemExit(main())
