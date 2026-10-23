"""Exactly one authorized -1 degree motor22 test; no mode switching.

Default is offline packet/CRC preflight. Explicit --execute required.
"""
import argparse
import dataclasses
import json
import math
import os
import signal
import threading
import time
from pathlib import Path
from arm_runtime import SafetyError
from shadow_motion_guard import ShadowGuard,ShadowFault
from waist_hold_candidate import (WaistHoldTransport, WaistHoldSession,
                                  PROTOCOL, SOURCE, fill_hold_command)

PROFILE = dict(protocol=PROTOCOL, robot_id=None, firmware_version=None,
    identity_note='Unknown; not inferred from mode_machine or API version',
    expected_mode_machine=5,motion_mode_name='ai',
    protocol_evidence=SOURCE,
    reviewed_by='Codex official-source review; user authorized fixed waist hold',
    waist_hold_scope_approved=True,external_body_support_confirmed=True,
    other_arm_publishers_stopped=True,waist_joint_indices=[12,13,14],
    three_waist_joints_verified_available=True,
    waist_configuration_evidence='Operator completed specifically requested R1+Y transition; post-transition FSM501; all three waist motors live/mode1 with finite q/dq')

class ObservedTransport(WaistHoldTransport):
    def __init__(self, interface):
        self.sent=0;self.attempted=0;self.release_sent=0
        self.observations=[];self.fsm_id=None
        self.shadow_guard=ShadowGuard();self.shadow_guard_enabled=True
        super().__init__(interface)

    def _receive(self,msg):
        super()._receive(msg)
        with self.lock:state=self.state
        if state is None:return
        try:
            self.shadow_guard.observe(state.robot_tick,self.clock(),state.q,state.dq,
                 state.rpy,state.gyro,state.mode_machine,state.motor_modes,
                 self.clock()-state.received_at)
        except ShadowFault as error:
            with self.lock:
                self.state_error='Receiver motion guard: '+str(error)
                self.receiver_fault=dataclasses.asdict(state)

    def _poll_mode(self):
        from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient
        loco=LocoClient();loco.SetTimeout(.3);loco.Init()
        while not self.stop.is_set():
            try:
                code,result=self.client.CheckMode()
                fsm_code,fsm=loco.GetFsmId()
                if code!=0 or not isinstance(result,dict) or result.get('name')!='ai':
                    raise SafetyError('CheckMode changed/failed')
                if fsm_code!=0 or fsm!=501:
                    raise SafetyError('FSM changed/failed; expected observed501')
                with self.lock:
                    self.mode=result['name'];self.mode_at=self.clock();self.fsm_id=fsm
            except Exception as error:
                with self.lock:self.mode_error=str(error)
                return
            self.stop.wait(.2)

    def write(self,q,weight,machine):
        self.attempted+=1
        super().write(q,weight,machine)
        self.sent+=1
        if weight==0:self.release_sent+=1
        with self.lock:state=self.state
        self.observations.append(dict(at=time.time(),command_q22=q[7],weight=weight,
            measured_q22=state.q[22],waist=list(state.q[12:15]),
            tick=state.robot_tick,age=self.clock()-state.received_at))

def configure_windows_dds():
    import unitree_sdk2py.core.channel as channel
    channel.ChannelConfigHasInterface=channel.ChannelConfigHasInterface.replace(
        'name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace(
        '/tmp/cdds.LOG',str(Path('first_hold_cdds.log').resolve()).replace('\\','/'))

def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--log',default='first_hold_result.json')
    args=parser.parse_args()
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.utils.crc import CRC
    packet=fill_hold_command(unitree_hg_msg_dds__LowCmd_(),[0.]*14,[0.]*3,0,5)
    print(json.dumps({'packet_crc_preflight':CRC().Crc(packet),'execute':args.execute}),flush=True)
    if not args.execute:return 0
    if Path(args.log).exists():raise SafetyError('Refusing to overwrite existing trial log')
    configure_windows_dds()
    stop=threading.Event()
    def on_signal(*_):stop.set()
    signal.signal(signal.SIGINT,on_signal)
    signal.signal(signal.SIGTERM,on_signal)
    # Console Enter is an additional stop; physical remote stop remains essential.
    def console_stop():
        import msvcrt
        while not stop.is_set():
            if msvcrt.kbhit() and msvcrt.getwch() in ('\r','\n','\x03'):
                stop.set();return
            time.sleep(.02)
    threading.Thread(target=console_stop,daemon=True).start()
    report={'profile':PROFILE,'joint':22,'delta_deg':-1.,'error':None,
            'release_error':None,'physical_motion_observed_by_operator':None}
    transport=None;session=None
    try:
        transport=ObservedTransport('<PRIVATE_IP>')
        session=WaistHoldSession(transport,PROFILE,stop=stop)
        session.enable()
        report['start_arm_q']=list(session.start)
        report['waist_anchor']=list(session.waist_anchor)
        print(json.dumps({'phase':'enabled','start_q22':session.start[7],
             'waist_anchor':session.waist_anchor}),flush=True)
        session.nudge(22,-1.)
        report['trajectory_completed']=True
    except BaseException as error:
        stop.set()
        report['error']=type(error).__name__+': '+str(error)
        report['state_at_fault']=getattr(error,'state_at_fault',None)
    finally:
        if session is not None:
            report['start_arm_q']=(list(session.start) if session.start is not None else None)
            report['waist_anchor']=(list(session.waist_anchor) if session.waist_anchor is not None else None)
            try:session.release()
            except BaseException as error:report['release_error']=str(error)
        if transport is not None:
            with transport.lock:final_state=transport.state
            obs=transport.observations
            report.update(sent_count=transport.sent,attempted_count=transport.attempted,
                          release_weight_zero_sent_count=transport.release_sent,
                          receiver_fault=getattr(transport,'receiver_fault',None),
                          observations=obs)
            if final_state is not None:
                report['final_measured_q22']=final_state.q[22]
            if session is not None and session.start is not None and obs:
                base=session.start[7]
                report['commanded_peak_delta_deg']=math.degrees(min(x['command_q22']-base for x in obs))
                report['max_abs_measured_delta_deg']=max(abs(math.degrees(x['measured_q22']-base)) for x in obs)
                report['max_waist_drift_rad']=max(max(abs(a-b) for a,b in zip(x['waist'],session.waist_anchor)) for x in obs)
            transport.close()
        Path(args.log).write_text(json.dumps(report,indent=2),encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('observations','profile')}),flush=True)
    return 0 if report['error'] is None and report['release_error'] is None else 1

if __name__=='__main__':raise SystemExit(main())
