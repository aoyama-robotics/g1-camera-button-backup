"""OFFLINE/SHADOW ONLY. Does not replace State.validate or enable sending."""
import math
from collections import deque

class ShadowFault(RuntimeError):pass

class ShadowGuard:
    speed=.05
    persistence=.020
    position_envelope=.001  # speed * maximum confirmation interval
    def __init__(self):
        self.anchor=None;self.history=deque();self.pending=[None]*15
        self.last_tick=None;self.last_host=None
    def observe(self,tick,host,q,dq,rpy,gyro,mode=5,motor_modes=None,age=0):
        values=[*q,*dq,*rpy,*gyro,host,age]
        if not all(math.isfinite(x) for x in values):raise ShadowFault('nonfinite')
        if mode!=5 or (motor_modes is not None and any(x!=1 for x in motor_modes)):
            raise ShadowFault('mode')
        if not 0<=age<=.2:raise ShadowFault('stale')
        if max(abs(x) for x in rpy[:2])>.1 or max(abs(x) for x in gyro)>.05:
            raise ShadowFault('imu')
        if self.last_host is not None and (host<self.last_host or host-self.last_host>.05):
            raise ShadowFault('timing_gap')
        if self.last_tick is not None:
            delta=(tick-self.last_tick)&0xffffffff
            if delta==0:
                return 'duplicate' # never extends persistence or replaces anchor
            if delta>=0x80000000 or delta>50:raise ShadowFault('tick_gap')
        self.last_tick=tick;self.last_host=host
        if self.anchor is None:self.anchor=tuple(q[:15])
        for i in range(15):
            if abs(q[i]-self.anchor[i])>self.position_envelope:
                raise ShadowFault('position_envelope:'+str(i))
        self.history.append((tick,tuple(q[:15])))
        while len(self.history)>1 and ((tick-self.history[1][0])&0xffffffff)>=10:
            self.history.popleft()
        elapsed=((tick-self.history[0][0])&0xffffffff)/1000
        if .010<=elapsed<=.050:
            for i in range(15):
                if abs(q[i]-self.history[0][1][i])/elapsed>self.speed:
                    raise ShadowFault('position_velocity:'+str(i))
        for i in range(15):
            if abs(dq[i])>self.speed:
                if self.pending[i] is None:self.pending[i]=tick
                elif ((tick-self.pending[i])&0xffffffff)/1000>=self.persistence:
                    raise ShadowFault('persistent_dq:'+str(i))
            else:self.pending[i]=None
        return 'shadow_pass'
