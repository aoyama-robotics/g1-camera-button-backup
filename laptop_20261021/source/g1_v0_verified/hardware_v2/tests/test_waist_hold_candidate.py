import dataclasses
import unittest
from test_runtime import Clock, Transport, PROFILE, state, message
from arm_runtime import SafetyError, validate_profile
from waist_hold_candidate import (PROTOCOL, SOURCE, WAIST, fill_hold_command,
                                  WaistHoldSession, WaistHoldTransport)

HOLD_PROFILE = {**PROFILE, 'protocol': PROTOCOL, 'protocol_evidence': SOURCE,
                'waist_hold_scope_approved': True,
                'waist_joint_indices':[12,13,14],
                'waist_configuration_evidence':'mock configuration only',
                'three_waist_joints_verified_available':True}

class HoldMock(Transport):
    def bind_waist(self, values): self.anchor = tuple(values)

class HoldTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.transport = HoldMock(self.clock)
        self.transport.q[12:15] = [.1, -.02, .03]
        self.session = WaistHoldSession(self.transport, HOLD_PROFILE,
                           clock=self.clock, sleep=self.clock.sleep)

    def test_packet_has_17_controlled_joints_and_no_leg_control(self):
        msg = fill_hold_command(message(), [.1]*14, [.2,-.03,.04], .4, 5)
        self.assertEqual([i for i,m in enumerate(msg.motor_cmd) if m.mode], list(range(12,29)))
        for index,value in zip(WAIST,[.2,-.03,.04]):
            m = msg.motor_cmd[index]
            self.assertEqual((m.q,m.dq,m.tau,m.kp,m.kd),(value,0,0,60,1.5))
        for index in list(range(12))+list(range(30,35)):
            self.assertEqual(vars(msg.motor_cmd[index]),vars(message().motor_cmd[index]))
        self.assertEqual(msg.motor_cmd[29].q,.4)
        for i in range(12,29):
            self.assertEqual((msg.motor_cmd[i].kp,msg.motor_cmd[i].kd),(60.,1.5))

    def test_measured_waist_anchor_unchanged_through_nudge_release(self):
        self.session.enable()
        self.session.nudge(22,-1)
        self.session.release()
        self.assertEqual(self.transport.anchor,(.1,-.02,.03))
        self.assertEqual(self.transport.q[12:15],[.1,-.02,.03])
        self.assertEqual(self.transport.writes[-1][1],0)

    def test_scope_approval_required(self):
        with self.assertRaises(SafetyError):
            WaistHoldSession(self.transport,{**HOLD_PROFILE,'waist_hold_scope_approved':False})

    def test_unverified_or_locked_waist_configuration_rejected(self):
        for changes in ({'three_waist_joints_verified_available':False},
                        {'waist_joint_indices':[12]},
                        {'waist_configuration_evidence':''}):
            with self.subTest(changes=changes),self.assertRaises(SafetyError):
                WaistHoldSession(self.transport,{**HOLD_PROFILE,**changes})

    def test_original_zero_slot_gate_unchanged(self):
        with self.assertRaises(SafetyError): validate_profile(HOLD_PROFILE)
        with self.assertRaises(SafetyError): validate_profile({**PROFILE,'arms_only_overlay_verified':False})

    def test_stale_and_wrong_mode_create_no_publisher(self):
        for kind in ('stale','mode'):
            with self.subTest(kind=kind):
                t=HoldMock(self.clock)
                if kind=='stale':t.stale=True
                else:t.mode='other'
                s=WaistHoldSession(t,HOLD_PROFILE,clock=self.clock,sleep=self.clock.sleep)
                with self.assertRaises(SafetyError):s.enable()
                self.assertEqual(t.publishers,0)

    def test_waist_drift_stops_and_releases_weight(self):
        self.session.enable()
        self.transport.q[12]+=.031
        before=len(self.transport.writes)
        with self.assertRaises(SafetyError):self.session.tick(self.session.last,1)
        self.assertEqual(len(self.transport.writes),before)
        self.session.release()
        self.assertEqual(self.transport.writes[-1][1],0)

    def test_disabled_waist_blocks_enable(self):
        old=self.transport.read
        def read():
            s,mode,at=old();m=list(s.motor_modes);m[12]=0
            return dataclasses.replace(s,motor_modes=tuple(m)),mode,at
        self.transport.read=read
        with self.assertRaises(SafetyError):self.session.enable()
        self.assertEqual(self.transport.publishers,0)

    def test_anchor_required_and_immutable(self):
        t=WaistHoldTransport.__new__(WaistHoldTransport)
        t.publisher=None;t.waist_anchor=None
        with self.assertRaises(SafetyError):t.enable_publisher()
        t.bind_waist([.1,0,0])
        with self.assertRaises(SafetyError):t.bind_waist([.2,0,0])

    def test_contaminated_packet_or_bad_machine_rejected(self):
        msg=message();msg.motor_cmd[0].kp=300
        with self.assertRaises(SafetyError):fill_hold_command(msg,[0]*14,[0]*3,1,5)
        with self.assertRaises(SafetyError):fill_hold_command(message(),[0]*14,[0]*3,1,3)

    def test_raw_velocity_guard_preserved_and_exact_fault_state_recorded(self):
        s=state();dq=list(s.dq);dq[2]=.050001
        with self.assertRaises(SafetyError) as caught:
            dataclasses.replace(s,dq=tuple(dq)).validate(0)
        self.assertIn('(2, 0.050001)',str(caught.exception))
        self.assertEqual(caught.exception.state_at_fault['dq'][2],.050001)
        dq[2]=.05
        dataclasses.replace(s,dq=tuple(dq)).validate(0)

    def test_twenty_ms_elapsed_trajectory_and_communication_loss_release(self):
        self.transport.shadow_guard_enabled=True
        self.session.enable()
        start=self.clock()
        self.session.nudge(22,-1)
        self.assertGreaterEqual(self.clock()-start,6.3)
        self.assertLess(self.clock()-start,6.5)
        self.transport.stale=True
        before=len(self.transport.writes)
        with self.assertRaises(SafetyError):self.session.tick(self.session.last,1)
        self.assertEqual(len(self.transport.writes),before)
        self.session.release()
        self.assertEqual(self.transport.writes[-1][1],0)

    def test_twenty_ms_mode_change_and_stop_release(self):
        self.transport.shadow_guard_enabled=True
        self.session.enable();self.transport.mode='other'
        with self.assertRaises(SafetyError):self.session.tick(self.session.last,1)
        self.session.release()
        self.assertEqual(self.transport.writes[-1][1],0)

if __name__=='__main__':unittest.main()
