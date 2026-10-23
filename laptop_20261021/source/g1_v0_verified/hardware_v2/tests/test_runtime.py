import builtins
import contextlib
import dataclasses
import io
import json
from pathlib import Path
import sys
import threading
import types
import unittest
from unittest.mock import patch,Mock
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from arm_runtime import *
from g1_arm import main,nudge_preview,validate_plan

PROFILE=dict(protocol='g1_29_arm_sdk_arms_only',robot_id='test-fixture',firmware_version='test-version',
 expected_mode_machine=5,motion_mode_name='sport',arms_only_overlay_verified=True,
 external_body_support_confirmed=True,other_arm_publishers_stopped=True,
 overlay_evidence='test mock only',reviewed_by='test')

def message():
    return types.SimpleNamespace(mode_pr=0,mode_machine=0,crc=0,motor_cmd=[types.SimpleNamespace(mode=0,q=0.,dq=0.,tau=0.,kp=0.,kd=0.) for _ in range(35)])

def state(now=0.,q=None):
    return State(tuple(q or [0.]*29),(0.,)*29,(1,)*29,(True,)*29,(0.,)*3,(0.,)*3,5,now)

class Clock:
    def __init__(self):self.t=0.
    def __call__(self):return self.t
    def sleep(self,seconds):self.t+=max(0,seconds)

class Transport:
    def __init__(self,c):self.c=c;self.q=[0.]*29;self.writes=[];self.publishers=0;self.mode='sport';self.fail=False;self.stale=False;self.timepoints=[]
    def read(self):return state(self.c()-1 if self.stale else self.c(),self.q),self.mode,self.c()
    def enable_publisher(self):self.publishers+=1
    def write(self,q,w,machine):
        if self.fail:raise SafetyError('Write failed')
        self.writes.append((tuple(q),w,machine));self.timepoints.append(self.c());self.q[15:29]=q

class ProtocolTests(unittest.TestCase):
    def test_arm_only_fields(self):
        m=fill_command(message(),[.1]*14,.2,5)
        self.assertEqual((m.mode_pr,m.mode_machine),(0,5))
        for i in range(35):
            x=m.motor_cmd[i]
            if i in ARM:
                self.assertEqual((x.mode,x.q,x.dq,x.tau),(1,.1,0.,0.))
                self.assertEqual((x.kp,x.kd),(40.,1.5) if i in WRIST else (80.,3.))
            else:self.assertEqual((x.mode,x.q,x.kp,x.kd,x.tau),(0,.2 if i==29 else 0,0,0,0))
    def test_contaminated_factory_rejected(self):
        m=message();m.motor_cmd[12].kp=1
        with self.assertRaises(SafetyError):fill_command(m,[0.]*14,1,5)
    def test_joint_limit(self):
        q=[0.]*14;q[0]=4
        with self.assertRaises(SafetyError):angles(q)
    def test_nan(self):
        with self.assertRaises(SafetyError):fill_command(message(),[float('nan')]*14,0,5)
    def test_weight(self):
        with self.assertRaises(SafetyError):fill_command(message(),[0.]*14,2,5)
    def test_no_lowcmd_profile(self):
        p={**PROFILE,'protocol':'lowcmd'}
        with self.assertRaises(SafetyError):validate_profile(p)
    def test_unverified_overlay(self):
        with self.assertRaises(SafetyError):validate_profile({**PROFILE,'arms_only_overlay_verified':False})
    def test_bad_machine(self):
        with self.assertRaises(SafetyError):dataclasses.replace(state(),mode_machine=3).validate(0)
    def test_disabled_arm(self):
        modes=[1]*29;modes[22]=0
        with self.assertRaises(SafetyError):dataclasses.replace(state(),motor_modes=modes).validate(0)
    def test_missing_motor(self):
        live=[True]*29;live[28]=False
        with self.assertRaises(SafetyError):dataclasses.replace(state(),live=live).validate(0)
    def test_stale(self):
        with self.assertRaises(SafetyError):state().validate(.201)
    def test_tilt(self):
        with self.assertRaises(SafetyError):dataclasses.replace(state(),rpy=(.11,0,0)).validate(0)
    def test_leg_motion(self):
        dq=[0.]*29;dq[0]=.06
        with self.assertRaises(SafetyError):dataclasses.replace(state(),dq=dq).validate(0)

class SessionTests(unittest.TestCase):
    def setUp(self):
        self.c=Clock();self.t=Transport(self.c);self.s=ArmSession(self.t,PROFILE,clock=self.c,sleep=self.c.sleep)
    def test_measured_start_and_return(self):
        self.t.q[22]=.2;self.s.enable();self.s.nudge(22,-1);self.s.release()
        self.assertEqual(self.t.publishers,1)
        self.assertEqual(self.t.writes[0][0][7],.2)
        self.assertAlmostEqual(self.t.writes[-1][0][7],.2)
        self.assertEqual(self.t.writes[-1][1],0)
        self.assertTrue(all(q[:7]==(0.,)*7 for q,_,_ in self.t.writes))
    def test_wrong_mode_no_publisher(self):
        self.t.mode='other'
        with self.assertRaises(SafetyError):self.s.enable()
        self.assertEqual(self.t.publishers,0)
    def test_no_catchup_burst(self):
        self.s.enable();self.c.t+=.025
        for _ in range(20):self.s.tick(self.s.last,1)
        gaps=[b-a for a,b in zip(self.t.timepoints,self.t.timepoints[1:])]
        self.assertGreaterEqual(min(gaps),DT-1e-8)
    def test_loop_lateness_stops(self):
        self.s.enable();n=len(self.t.writes);self.c.t+=.06
        with self.assertRaises(SafetyError):self.s.tick(self.s.last,1)
        self.assertEqual(len(self.t.writes),n)
    def test_stale_stops_and_release_zero(self):
        self.s.enable();self.t.stale=True;n=len(self.t.writes)
        with self.assertRaises(SafetyError):self.s.tick(self.s.last,1)
        self.assertEqual(len(self.t.writes),n)
        self.s.release();self.assertEqual(self.t.writes[-1][1],0)
    def test_mode_change_stops(self):
        self.s.enable();self.t.mode='other'
        with self.assertRaises(SafetyError):self.s.tick(self.s.last,1)
        self.s.release();self.assertEqual(self.t.writes[-1][1],0)
    def test_write_failure_propagates(self):
        self.s.enable();self.t.fail=True
        with self.assertRaises(SafetyError):self.s.tick(self.s.last,1)
        with self.assertRaises(SafetyError):self.s.release()
        self.assertFalse(self.s.enabled)
    def test_operator_stop_no_return_motion(self):
        self.s.enable();self.s.stop.set();n=len(self.t.writes)
        with self.assertRaises(SafetyError):self.s.nudge(22,-1)
        self.s.release()
        self.assertEqual(len(self.t.writes),n+1);self.assertEqual(self.t.writes[-1][1],0)
    def test_nonarm_nudge_rejected(self):
        self.s.enable()
        with self.assertRaises(SafetyError):self.s.nudge(12,1)
    def test_three_degree_limit(self):
        self.s.enable()
        with self.assertRaises(SafetyError):self.s.nudge(22,3.1)
    def test_speed_gate(self):
        self.s.enable()
        with self.assertRaises(SafetyError):self.s.move([.2]*14,1)
    def test_step_gate(self):
        self.s.enable()
        with self.assertRaises(SafetyError):self.s.tick([.001]*14,1)

class ReceptionTests(unittest.TestCase):
    def test_duplicate_tick_does_not_refresh(self):
        c=Clock();t=UnitreeTransport.__new__(UnitreeTransport)
        t.clock=c;t.lock=threading.Lock();t.last_robot_tick=None;t.state=None;t.state_error=None
        m=types.SimpleNamespace(tick=42,mode_machine=5,
          motor_state=[types.SimpleNamespace(q=0.,dq=0.,mode=1,vol=24.,temperature=[20]) for _ in range(35)],
          imu_state=types.SimpleNamespace(rpy=[0.]*3,gyroscope=[0.]*3))
        t._receive(m);self.assertEqual(t.state.received_at,0)
        c.t=1;t._receive(m);self.assertEqual(t.state.received_at,0)
        with self.assertRaises(SafetyError):t.state.validate(c())
        m.tick=43;t._receive(m);self.assertEqual(t.state.received_at,1)
    def test_tick_wrap(self):
        c=Clock();t=UnitreeTransport.__new__(UnitreeTransport)
        t.clock=c;t.lock=threading.Lock();t.last_robot_tick=0xffffffff;t.state=None;t.state_error=None
        m=types.SimpleNamespace(tick=0,mode_machine=5,motor_state=[types.SimpleNamespace(q=0.,dq=0.,mode=1,vol=24.,temperature=[20]) for _ in range(35)],imu_state=types.SimpleNamespace(rpy=[0.]*3,gyroscope=[0.]*3))
        t._receive(m);self.assertIsNotNone(t.state)

class TransportWiringTests(unittest.TestCase):
    def setUp(self):
        self.publisher=Mock();self.publisher.Write.return_value=True
        self.pub_class=Mock(return_value=self.publisher)
        channel=types.ModuleType('unitree_sdk2py.core.channel')
        channel.ChannelFactoryInitialize=Mock();channel.ChannelSubscriber=Mock(return_value=Mock())
        channel.ChannelPublisher=self.pub_class
        idl=types.ModuleType('unitree_sdk2py.idl.unitree_hg.msg.dds_');idl.LowCmd_=object;idl.LowState_=object
        default=types.ModuleType('unitree_sdk2py.idl.default');default.unitree_hg_msg_dds__LowCmd_=message
        crc=types.ModuleType('unitree_sdk2py.utils.crc');crc.CRC=Mock(return_value=types.SimpleNamespace(Crc=lambda m:123))
        mode=types.ModuleType('unitree_sdk2py.comm.motion_switcher.motion_switcher_client')
        mode.MotionSwitcherClient=Mock(return_value=Mock())
        self.mods={m.__name__:m for m in (channel,idl,default,crc,mode)}
        self.channel=channel
    def test_readonly_has_no_arm_publisher(self):
        with patch.dict(sys.modules,self.mods),patch('threading.Thread.start'):
            t=UnitreeTransport('MOCK_NIC')
        self.pub_class.assert_not_called()
        self.channel.ChannelFactoryInitialize.assert_called_once_with(0,'MOCK_NIC')
        self.channel.ChannelSubscriber.assert_called_once_with('rt/lowstate',object)
    def test_explicit_publisher_crc_and_false_write(self):
        with patch.dict(sys.modules,self.mods),patch('threading.Thread.start'):
            t=UnitreeTransport('MOCK_NIC');t.enable_publisher();t.write([0.]*14,.2,5)
        self.pub_class.assert_called_once_with('rt/arm_sdk',object)
        msg=self.publisher.Write.call_args[0][0]
        self.assertEqual(msg.crc,123);self.assertEqual(msg.motor_cmd[29].q,.2)
        self.publisher.Write.return_value=False
        with self.assertRaises(SafetyError):t.write([0.]*14,.2,5)
    def test_missing_publisher_rejects_write(self):
        with patch.dict(sys.modules,self.mods),patch('threading.Thread.start'):
            t=UnitreeTransport('MOCK_NIC')
        with self.assertRaises(SafetyError):t.write([0.]*14,1,5)


class CliTests(unittest.TestCase):
    def test_default_dryrun_no_sdk_or_network(self):
        original=builtins.__import__
        def guard(name,*a,**k):
            if name.startswith(('unitree','cyclonedds')):raise AssertionError(name)
            return original(name,*a,**k)
        with patch('builtins.__import__',side_effect=guard),patch('socket.socket',side_effect=AssertionError('socket')),contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(main(['nudge']),0)
    def test_template_profile_blocks_live(self):
        p=Path(__file__).resolve().parents[1]/'profile.example.json'
        with self.assertRaises(SafetyError):validate_profile(json.loads(p.read_text()))
    def test_preview_schema_not_accepted(self):
        with self.assertRaises(SafetyError):validate_plan({'schema':'g1-button-offline-preview-v1'},PROFILE)
    def test_unreviewed_plan_rejected(self):
        p=dict(schema='g1-arms-reviewed-plan-v1',robot_id='test-fixture',firmware_version='test-version',joint_indices=list(ARM),reviewed_by='',collision_checked=False)
        with self.assertRaises(SafetyError):validate_plan(p,PROFILE)

if __name__=='__main__':unittest.main()
