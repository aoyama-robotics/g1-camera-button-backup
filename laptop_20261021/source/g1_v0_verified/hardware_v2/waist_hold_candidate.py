"""Review-only candidate: arm motion plus explicit measured-waist hold.

No CLI, connection, or publisher at import. Original arms-only profile/gates
remain untouched. Requires a separately approved scope before construction.
The official SDK arm7 example controls 14 arms + 3 waist, leaving legs zero.
"""
import threading
import time
import math
from arm_runtime import (ARM, WRIST, SafetyError, numeric, angles,
                         UnitreeTransport, ArmSession)

WAIST = (12, 13, 14)
PROTOCOL = 'g1_29_arm_sdk_measured_waist_hold'
SOURCE = 'https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/high_level/g1_arm7_sdk_dds_example.cpp'

def validate_hold_profile(profile):
    if profile.get('protocol') != PROTOCOL:
        raise SafetyError('Explicit measured-waist-hold protocol required')
    if profile.get('expected_mode_machine') != 5:
        raise SafetyError('Only reviewed G1 29DoF machine 5 supported')
    # Identity/firmware are audit metadata, never fabricated authorization.
    # This design controls all 17 documented upper-body axes explicitly and
    # does not rely on the old firmware-specific zero-waist overlay assumption.
    for key in ('motion_mode_name', 'reviewed_by',
                'protocol_evidence', 'waist_configuration_evidence'):
        value = profile.get(key)
        if not isinstance(value, str) or not value.strip() or value.startswith('REPLACE'):
            raise SafetyError('Missing reviewed hardware profile: ' + key)
    for key in ('waist_hold_scope_approved', 'external_body_support_confirmed',
                'other_arm_publishers_stopped', 'three_waist_joints_verified_available'):
        if profile.get(key) is not True:
            raise SafetyError('Required operator verification: ' + key)
    if profile.get('waist_joint_indices') != [12,13,14]:
        raise SafetyError('This candidate requires three verified available waist joints; locked-waist configuration unsupported')
    return profile

def waist_angles(values):
    if len(values) != 3:
        raise SafetyError('Three measured waist angles required')
    return tuple(numeric(x) for x in values)

def fill_hold_command(msg, arm_q, waist_q, weight, mode_machine):
    """Official SDK arm7 gain convention 60/1.5 for all 17 joints.

    Waist target is an immutable measured anchor. This is waist authority,
    even though the target has zero displacement from that measured anchor.
    """
    arm_q = angles(arm_q)
    waist_q = waist_angles(waist_q)
    weight = numeric(weight)
    if mode_machine != 5 or not 0 <= weight <= 1:
        raise SafetyError('Invalid machine/weight')
    # Reject contaminated factories before touching any fields.
    for motor in msg.motor_cmd:
        if any(getattr(motor, key) != 0 for key in ('mode','q','dq','tau','kp','kd')):
            raise SafetyError('Command factory must be zero initialized')
    msg.mode_pr = 0
    msg.mode_machine = mode_machine
    for index, value in zip(ARM, arm_q):
        motor = msg.motor_cmd[index]
        motor.mode, motor.q = 1, value
        motor.kp, motor.kd = 60., 1.5
    for index, value in zip(WAIST, waist_q):
        motor = msg.motor_cmd[index]
        motor.mode, motor.q = 1, value
        motor.kp, motor.kd = 60., 1.5
    msg.motor_cmd[29].q = weight
    return msg

class WaistHoldTransport(UnitreeTransport):
    def __init__(self, interface, clock=time.perf_counter):
        self.waist_anchor = None
        super().__init__(interface, clock)

    def bind_waist(self, values):
        if self.publisher is not None or self.waist_anchor is not None:
            raise SafetyError('Waist anchor cannot change after binding')
        self.waist_anchor = waist_angles(values)

    def enable_publisher(self):
        if self.waist_anchor is None:
            raise SafetyError('Measured waist anchor required before publisher')
        super().enable_publisher()

    def write(self, q, weight, machine):
        if self.publisher is None or self.waist_anchor is None:
            raise SafetyError('Publisher/waist anchor not enabled')
        msg = fill_hold_command(self.factory(), q, self.waist_anchor, weight, machine)
        msg.crc = self.crc.Crc(msg)
        if self.publisher.Write(msg) is not True:
            raise SafetyError('DDS Write failed')

class WaistHoldSession(ArmSession):
    def __init__(self, transport, profile, stop=None, clock=time.perf_counter, sleep=None):
        self.transport = transport
        self.profile = validate_hold_profile(profile)
        self.stop = stop or threading.Event()
        self.clock=clock
        if sleep is None:
            from precise_timing import PreciseSleeper
            sleep=PreciseSleeper()
        self.sleep=sleep
        self.start = self.last = self.waist_anchor = None
        self.weight, self.enabled, self.next_tick = 0., False, None
        self.period=.020

    def validate_state(self,state,now):
        if not getattr(self.transport,'shadow_guard_enabled',False):
            # Test/mock and legacy callers retain the original raw-speed gate.
            return super().validate_state(state,now)
        return state.validate(now,self.profile['expected_mode_machine'],check_body_velocity=False)

    def _phase(self,duration,callback):
        start=self.clock()
        while True:
            fraction=min(1.,(self.clock()-start)/duration)
            callback(fraction)
            if fraction>=1:return

    def ramp_weight(self,end,duration):
        start=self.weight
        self._phase(duration,lambda f:self.tick(self.last,start+(end-start)*f))

    def move(self,goal,duration=3.):
        goal=angles(goal);duration=numeric(duration)
        delta=max(abs(a-b) for a,b in zip(goal,self.last))
        if duration<1 or duration>10 or 1.5*delta/duration>.10 or 6*delta/duration**2>.25:
            raise SafetyError('Segment velocity/acceleration/duration outside bounds')
        start=self.last
        self._phase(duration,lambda f:self.tick(tuple(a+(f*f*(3-2*f))*(b-a) for a,b in zip(start,goal)),1.))

    def nudge(self,joint,delta_deg):
        if not 15<=joint<=28 or abs(numeric(delta_deg))!=1.:
            raise SafetyError('This executor is restricted to one-degree arm test')
        goal=list(self.start);goal[joint-15]+=math.radians(delta_deg);angles(goal)
        self.move(goal)
        self._phase(.3,lambda f:self.tick(self.last,1.))
        self.move(self.start)

    def check(self, allow_stop=False):
        state = super().check(allow_stop)
        if not all(state.live[i] and state.motor_modes[i] == 1 for i in WAIST):
            raise SafetyError('Waist not live/enabled; no auto mode change')
        if self.waist_anchor is not None:
            if max(abs(state.q[i]-q) for i,q in zip(WAIST,self.waist_anchor)) > .03:
                raise SafetyError('Waist departed measured hold anchor')
        return state

    def enable(self):
        if self.enabled:
            raise SafetyError('Already enabled')
        state = self.wait_still()
        self.start = angles(state.q[15:29])
        self.last = self.start
        self.waist_anchor = waist_angles(state.q[12:15])
        self.transport.bind_waist(self.waist_anchor)
        self.transport.enable_publisher()
        self.enabled, self.next_tick = True, self.clock()
        self.ramp_weight(1., 2.)
