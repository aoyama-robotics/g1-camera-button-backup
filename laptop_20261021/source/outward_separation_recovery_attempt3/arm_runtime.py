"""Opt-in 29DoF G1 arm_sdk runtime. No robot connection at import time.

Protocol follows pinned XR source; intentionally excludes lowcmd, locomotion,
waist commands, debug-mode changes, and automatic startup motion.
"""
from __future__ import annotations
import dataclasses
import math
import threading
import time

ARM = tuple(range(15,29))
WRIST = {19,20,21,26,27,28}
LIMITS = ((-3.0892,2.6704),(-1.5882,2.2515),(-2.618,2.618),(-1.0472,2.0944),
          (-1.9722,1.9722),(-1.6144,1.6144),(-1.6144,1.6144),
          (-3.0892,2.6704),(-2.2515,1.5882),(-2.618,2.618),(-1.0472,2.0944),
          (-1.9722,1.9722),(-1.6144,1.6144),(-1.6144,1.6144))
DT=.004  # XR G1_29: 250Hz; Python/Windows is not a realtime guarantee
STATE_TIMEOUT=.20
MODE_TIMEOUT=.75
MAX_SPEED=.10
MAX_ACCEL=.25
MAX_EXCURSION=math.radians(5)


class SafetyError(RuntimeError): pass


def numeric(value):
    if isinstance(value,bool): raise SafetyError('Boolean is not a numerical command')
    v=float(value)
    if not math.isfinite(v): raise SafetyError('Non-finite value')
    return v


def angles(q):
    if len(q)!=14: raise SafetyError('14 arm joint values required')
    q=tuple(numeric(v) for v in q)
    for v,(lo,hi) in zip(q,LIMITS):
        if not lo+.025 <= v <= hi-.025: raise SafetyError('Arm joint limit/margin violated')
    return q


@dataclasses.dataclass(frozen=True)
class State:
    q: tuple
    dq: tuple
    motor_modes: tuple
    live: tuple
    rpy: tuple
    gyro: tuple
    mode_machine: int
    received_at: float
    robot_tick: int = 0

    @classmethod
    def parse(cls,msg,now):
        if len(msg.motor_state)<29: raise SafetyError('Not a 29DoF LowState')
        m=msg.motor_state[:29]
        return cls(tuple(numeric(x.q) for x in m),tuple(numeric(x.dq) for x in m),
                   tuple(int(x.mode) for x in m),
                   tuple(numeric(x.vol)>.1 or numeric(x.temperature[0])>0 for x in m),
                   tuple(numeric(x) for x in msg.imu_state.rpy),
                   tuple(numeric(x) for x in msg.imu_state.gyroscope),int(msg.mode_machine),now,int(msg.tick))

    def validate(self,now,expected_machine=5,check_body_velocity=True,allow_startup_modes=False):
        if not 0 <= now-self.received_at <= STATE_TIMEOUT: raise SafetyError('Stale LowState')
        if len(self.q)!=29 or len(self.dq)!=29 or len(self.rpy)!=3 or len(self.gyro)!=3:
            raise SafetyError('State shape mismatch')
        for value in (*self.q,*self.dq,*self.rpy,*self.gyro): numeric(value)
        if self.mode_machine!=expected_machine: raise SafetyError('Unexpected mode_machine/robot model')
        if not all(self.live[i] and self.motor_modes[i] in ((0,1) if allow_startup_modes else (1,)) for i in ARM):
            raise SafetyError('29DoF arms not live/enabled; no auto mode change')
        angles(self.q[15:29])
        if max(abs(x) for x in self.rpy[:2])>.10 or max(abs(x) for x in self.gyro)>.05:
            raise SafetyError('Robot is not upright/stationary')
        exceeded=[(i,x) for i,x in enumerate(self.dq[:15]) if abs(x)>.05]
        if exceeded and check_body_velocity:
            error=SafetyError('Leg or waist motion detected: '+repr(exceeded))
            error.state_at_fault=dataclasses.asdict(self)
            raise error
        return self


def validate_profile(profile):
    if profile.get('protocol')!='g1_29_arm_sdk_arms_only': raise SafetyError('Only explicit arm_sdk supported')
    if profile.get('expected_mode_machine')!=5: raise SafetyError('Only reviewed G1 29DoF machine 5 supported')
    for key in ('robot_id','firmware_version','motion_mode_name','overlay_evidence','reviewed_by'):
        v=profile.get(key)
        if not isinstance(v,str) or not v.strip() or v.startswith('REPLACE'):
            raise SafetyError('Missing reviewed hardware profile: '+key)
    for key in ('arms_only_overlay_verified','external_body_support_confirmed','other_arm_publishers_stopped'):
        if profile.get(key) is not True: raise SafetyError('Required operator verification: '+key)
    return profile


def fill_command(msg,q,weight,mode_machine):
    """New zero message per frame; write ONLY arms15..28 and weight29.

    Non-arm zero fields are not guaranteed ignored on unverified firmware.
    Thus live enable requires a reviewed arms-only overlay profile.
    """
    q=angles(q); weight=numeric(weight)
    if not 0<=weight<=1: raise SafetyError('Invalid weight')
    msg.mode_pr=0; msg.mode_machine=mode_machine
    for i,v in zip(ARM,q):
        m=msg.motor_cmd[i];m.mode=1;m.q=v;m.dq=0.;m.tau=0.
        m.kp,m.kd=(40.,1.5) if i in WRIST else (80.,3.)
    msg.motor_cmd[29].q=weight
    for i,m in enumerate(msg.motor_cmd):
        if i in ARM:continue
        expected_q=weight if i==29 else 0.
        if m.mode!=0 or m.q!=expected_q or any(getattr(m,k)!=0 for k in ('dq','tau','kp','kd')):
            raise SafetyError('Non-arm motor slot is not neutral')
    return msg


class UnitreeTransport:
    """Subscribe + read-only CheckMode initially; arm publisher is lazy.

    SDK RPC may use its own request channel, but no locomotion/mode mutation is
    called. No ChannelPublisher(rt/arm_sdk) exists until enable_publisher().
    """
    def __init__(self,interface,clock=time.perf_counter):
        if not interface: raise SafetyError('Explicit NIC required')
        from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber,ChannelPublisher
        from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_,LowState_
        from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
        from unitree_sdk2py.utils.crc import CRC
        from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
        self.clock=clock;self.lock=threading.Lock();self.state=None;self.state_error=None;self.last_robot_tick=None
        self.mode=None;self.mode_at=None;self.mode_error=None;self.publisher=None
        self.stop=threading.Event();self.Publisher=ChannelPublisher;self.LowCmd=LowCmd_
        self.factory=unitree_hg_msg_dds__LowCmd_;self.crc=CRC()
        ChannelFactoryInitialize(0,interface)
        self.sub=ChannelSubscriber('rt/lowstate',LowState_);self.sub.Init(self._receive,10)
        self.client=MotionSwitcherClient();self.client.SetTimeout(.3);self.client.Init()
        self.worker=threading.Thread(target=self._poll_mode,daemon=True);self.worker.start()
    def _receive(self,msg):
        try:
            parsed=State.parse(msg,self.clock())
            with self.lock:
                if self.last_robot_tick is not None:
                    advance=(parsed.robot_tick-self.last_robot_tick)&0xffffffff
                    if advance==0 or advance>=0x80000000:return
                self.last_robot_tick=parsed.robot_tick
                self.state=parsed
        except Exception as e:
            with self.lock:self.state_error=str(e) # sticky, never silently resume
    def _poll_mode(self):
        while not self.stop.is_set():
            try:
                code,result=self.client.CheckMode()
                if code!=0 or not isinstance(result,dict) or not result.get('name'):
                    raise SafetyError('CheckMode failed or motion controller stopped')
                with self.lock:self.mode=result['name'];self.mode_at=self.clock()
            except Exception as e:
                with self.lock:self.mode_error=str(e) # sticky
                return
            self.stop.wait(.2)
    def read(self):
        with self.lock:
            if self.state_error:raise SafetyError('LowState parse failed: '+self.state_error)
            if self.mode_error:raise SafetyError(self.mode_error)
            return self.state,self.mode,self.mode_at
    def enable_publisher(self):
        if self.publisher is not None:raise SafetyError('Publisher already enabled')
        self.publisher=self.Publisher('rt/arm_sdk',self.LowCmd);self.publisher.Init()
    def write(self,q,weight,machine):
        if self.publisher is None:raise SafetyError('No publisher enabled')
        msg=fill_command(self.factory(),q,weight,machine);msg.crc=self.crc.Crc(msg)
        if self.publisher.Write(msg) is not True:raise SafetyError('DDS Write failed')
    def close(self):
        self.stop.set()
        self.worker.join(timeout=.5)
        for handle in (self.publisher,self.sub):
            if handle is not None and hasattr(handle,'Close'):handle.Close()


class ArmSession:
    def validate_state(self,state,now):
        return state.validate(now,self.profile['expected_mode_machine'])
    def __init__(self,transport,profile,stop=None,clock=time.perf_counter,sleep=None):
        self.transport=transport;self.profile=validate_profile(profile)
        self.stop=stop or threading.Event();self.clock=clock
        if sleep is None:
            from precise_timing import PreciseSleeper
            sleep=PreciseSleeper()
        self.sleep=sleep
        self.start=None;self.last=None;self.weight=0.;self.enabled=False;self.next_tick=None
    def check(self,allow_stop=False):
        if self.stop.is_set() and not allow_stop:raise SafetyError('Operator stop')
        s,mode,mode_at=self.transport.read();now=self.clock()
        if s is None or mode_at is None:raise SafetyError('No state/mode received')
        self.validate_state(s,now)
        if not 0<=now-mode_at<=MODE_TIMEOUT or mode!=self.profile['motion_mode_name']:
            raise SafetyError('Motion controller mode unknown, changed, or stale')
        if self.last is not None and max(abs(a-b) for a,b in zip(s.q[15:29],self.last))>.15:
            raise SafetyError('Arm tracking error')
        return s
    def wait_still(self,timeout=8.):
        end=self.clock()+timeout;since=None
        while self.clock()<end:
            if self.stop.is_set():raise SafetyError("Operator stop during discovery")
            # No state yet is normal for DDS discovery. Explicit errors do not retry.
            s,mode,mode_at=self.transport.read()
            if s is None or mode_at is None:
                self.sleep(.02);continue
            self.check()
            if max(abs(x) for x in s.dq[15:29])>.03:since=None
            else:since=self.clock() if since is None else since
            if since is not None and self.clock()-since>=.5:return s
            self.sleep(.02)
        raise SafetyError('Initial stationary state timeout')
    def enable(self):
        if self.enabled:raise SafetyError('Already enabled')
        s=self.wait_still();self.start=angles(s.q[15:29]);self.last=self.start
        self.transport.enable_publisher();self.enabled=True;self.next_tick=self.clock()
        self.ramp_weight(1.,2.)
    def tick(self,q,weight,allow_stop=False):
        s=self.check(allow_stop);q=angles(q)
        if max(abs(a-b) for a,b in zip(q,self.start))>MAX_EXCURSION+1e-9:
            raise SafetyError('Five-degree experiment envelope exceeded')
        period=getattr(self,'period',DT)
        if max(abs(a-b) for a,b in zip(q,self.last))>MAX_SPEED*period+1e-8:
            raise SafetyError('Joint rate limit exceeded')
        now=self.clock()
        if self.next_tick is not None and now-self.next_tick>.05:raise SafetyError('Control loop deadline missed')
        self.transport.write(q,weight,s.mode_machine)
        self.last=q;self.weight=weight
        # Next deadline follows actual send-start, not a historical ideal grid.
        # If processing consumed this period, start a new full period; no burst.
        finished=self.clock()
        self.next_tick=now+period if finished<now+period else finished+period
        self.sleep(max(0,self.next_tick-self.clock()))
    def ramp_weight(self,end,duration):
        start=self.weight;n=math.ceil(duration/DT)
        for k in range(1,n+1):self.tick(self.last,start+(end-start)*k/n)
    def move(self,goal,duration=3.):
        goal=angles(goal);duration=numeric(duration)
        delta=max(abs(a-b) for a,b in zip(goal,self.last))
        if duration<1 or duration>10 or 1.5*delta/duration>MAX_SPEED or 6*delta/duration**2>MAX_ACCEL:
            raise SafetyError('Segment velocity/acceleration/duration outside bounds')
        start=self.last;n=math.ceil(duration/DT)
        for k in range(1,n+1):
            t=k/n;f=t*t*(3-2*t)
            self.tick(tuple(a+f*(b-a) for a,b in zip(start,goal)),1.)
    def nudge(self,joint,delta_deg):
        if not 15<=joint<=28:raise SafetyError('Only arm motor15..28 supported')
        delta=numeric(delta_deg)
        if not 0<abs(delta)<=3.:raise SafetyError('Nudge must be nonzero and at most three degrees')
        goal=list(self.start);goal[joint-15]+=math.radians(delta);angles(goal)
        self.move(goal)
        for _ in range(round(.3/DT)):self.tick(self.last,1.)
        self.move(self.start)
    def release(self):
        if not self.enabled:return
        try:
            # Healthy exit: hold current measured pose while weight decreases.
            # Stop/fault: do NOT execute a return trajectory or hold stale high weight.
            if self.stop.is_set():raise SafetyError('Stop requested')
            self.check()
            self.ramp_weight(0.,2.)
        except Exception:
            # Best effort immediate weight zero uses last known valid header/angles.
            # This is not proof of receipt or a physical E-stop.
            self.transport.write(self.last,0.,self.profile['expected_mode_machine'])
            self.weight=0.
        finally:
            self.enabled=False
