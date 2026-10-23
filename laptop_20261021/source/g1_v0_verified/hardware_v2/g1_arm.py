#!/usr/bin/env python3
"""G1 arm-only experiment CLI: offline default; explicit --execute enables DDS."""
from __future__ import annotations
import argparse
import dataclasses
import hashlib
import importlib
import importlib.util
import json
import math
import platform
import signal
import socket
import sys
import threading
import time
from pathlib import Path
from arm_runtime import (ARM,State,SafetyError,UnitreeTransport,ArmSession,angles,numeric,
                         validate_profile,MAX_EXCURSION,MAX_SPEED,MAX_ACCEL)


def write_json(path,obj):Path(path).write_text(json.dumps(obj,indent=2,allow_nan=False)+'\n')


def preflight():
    result={'os':platform.platform(),'python':sys.version,'machine':platform.machine(),
            'native_windows_tested':False,'hardware_tested':False,'sdk_imports':{}}
    try:result['interfaces']=socket.if_nameindex()
    except OSError as e:result['interfaces_error']=str(e)
    for name in ('unitree_sdk2py.core.channel','unitree_sdk2py.utils.crc',
                 'unitree_sdk2py.comm.motion_switcher.motion_switcher_client'):
        try:importlib.import_module(name);result['sdk_imports'][name]='OK'
        except Exception as e:result['sdk_imports'][name]=f'{type(e).__name__}: {e}'
    result['note']='Import/NIC inspection only. No DDS initialized. Import success is not connection verification.'
    return result


def load_profile(path):return validate_profile(json.loads(Path(path).read_text()))


def nudge_preview(snapshot,joint,delta):
    if not 15<=joint<=28:raise SafetyError('Arm motor15..28 only')
    delta=numeric(delta)
    if not 0<abs(delta)<=3:raise SafetyError('Nudge is limited to nonzero +/-3 degrees')
    q=angles(snapshot['motor_q_rad'][15:29]);goal=list(q);goal[joint-15]+=math.radians(delta);angles(goal)
    return {'schema':'g1-arm-nudge-preview-v1','hardware_executable':False,
            'joint':joint,'delta_deg':delta,'start_arm_q':q,'goal_arm_q':goal,
            'duration_s':3.,'return_to_measured_start':True,'protocol':'rt/arm_sdk',
            'non_arm_commands':'none; slot29 is weight','dds_initialized':False}


def validate_plan(doc,profile):
    if doc.get('schema')!='g1-arms-reviewed-plan-v1':raise SafetyError('Not a reviewed arm plan')
    if doc.get('robot_id')!=profile['robot_id'] or doc.get('firmware_version')!=profile['firmware_version']:
        raise SafetyError('Plan robot/firmware mismatch')
    if doc.get('joint_indices')!=list(ARM):raise SafetyError('Plan joint order mismatch')
    if doc.get('status')!='REVIEWED' or not doc.get('reviewed_by') or doc.get('collision_checked') is not True:
        raise SafetyError('Plan needs operator/collision review; a flag does not perform review')
    start=angles(doc['start_arm_q']);previous=start
    waypoints=doc['waypoints']
    if not isinstance(waypoints,list) or not 1<=len(waypoints)<=8:raise SafetyError('Plan needs 1..8 segments')
    total=0
    for point in waypoints:
        q=angles(point['arm_q']);duration=numeric(point['duration_s']);total+=duration
        delta=max(abs(a-b) for a,b in zip(q,previous))
        if (not 1<=duration<=10 or max(abs(a-b) for a,b in zip(q,start))>MAX_EXCURSION
                or 1.5*delta/duration>MAX_SPEED or 6*delta/duration**2>MAX_ACCEL):
            raise SafetyError('Plan exceeds bounded experimental envelope')
        previous=q
    if total>40:raise SafetyError('Plan duration exceeds 40 seconds')
    if max(abs(a-b) for a,b in zip(previous,start))>1e-6:raise SafetyError('Plan must return to recorded start')
    return doc


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    sub=p.add_subparsers(dest='command',required=True)
    sub.add_parser('preflight')
    state=sub.add_parser('state');state.add_argument('--connect',action='store_true')
    state.add_argument('--interface');state.add_argument('--output',default='robot_state.json')
    for name in ('nudge','run-plan'):
        a=sub.add_parser(name)
        a.add_argument('--execute',action='store_true');a.add_argument('--interface')
        a.add_argument('--profile');a.add_argument('--log',default='arm_session.json')
        if name=='nudge':
            a.add_argument('--joint',type=int,default=22);a.add_argument('--delta-deg',type=float,default=-1.)
            a.add_argument('--snapshot',default=str(Path(__file__).parent/'example_state.json'))
        else:a.add_argument('--plan',required=True)
    args=p.parse_args(argv)
    if args.command=='preflight':print(json.dumps(preflight(),indent=2));return 0
    if args.command=='state':
        if not args.connect or not args.interface:p.error('Read-only DDS requires --connect --interface')
        t=UnitreeTransport(args.interface)
        try:
            end=time.perf_counter()+8
            while time.perf_counter()<end:
                s,mode,mode_at=t.read()
                if s is not None and mode_at is not None:
                    now=time.perf_counter()
                    if not 0<=now-s.received_at<=.2:raise SafetyError('Stale LowState')
                    doc=dataclasses.asdict(s)
                    doc.update(schema='g1-arm-state-v2',motor_q_rad=list(s.q),motion_mode_name=mode,
                               captured_unix_s=time.time(),arm_publisher_created=False,
                               timestamp_s=time.time()-(now-s.received_at),imu_rpy_rad=list(s.rpy),
                               max_leg_speed_rad_s=max(abs(x) for x in s.dq[:12]),
                               max_gyro_rad_s=max(abs(x) for x in s.gyro))
                    write_json(args.output,doc);print(f'State saved to {args.output}; no arm publisher or motion commands.');return 0
                time.sleep(.02)
            raise SafetyError('No LowState / mode within 8 seconds')
        finally:t.close()
    if args.command=='nudge':
        # Validate even before any real connection is permitted.
        preview=nudge_preview(json.loads(Path(args.snapshot).read_text()),args.joint,args.delta_deg)
    else:
        if not args.profile:p.error('--profile is required to bind plan to robot')
        profile=load_profile(args.profile)
        preview=validate_plan(json.loads(Path(args.plan).read_text()),profile)
    if not args.execute:
        print(json.dumps(preview,indent=2));print('DRY RUN: no DDS initialized.');return 0
    if not args.profile or not args.interface:p.error('--execute requires --profile and --interface')
    profile=load_profile(args.profile)
    if not sys.stdin.isatty():raise SafetyError('Live enable requires an interactive terminal')
    print('LIVE arms only: external body support, verified firmware arms-only overlay, clear workspace, physical E-stop/operator required.')
    print('No waist/leg hold is sent. Ctrl+C or Enter stops trajectory and attempts weight release; not a physical E-stop.')
    print('Profile:',json.dumps(profile,indent=2))
    if args.command=='run-plan':print('Plan SHA256:',hashlib.sha256(Path(args.plan).read_bytes()).hexdigest())
    if input('Type ENABLE ARMS to connect and arm: ').strip()!='ENABLE ARMS':
        print('Canceled before connecting.');return 0
    stop=threading.Event();old_handlers={}
    def handler(signum,frame):stop.set()
    for sig in (signal.SIGINT,signal.SIGTERM):
        old_handlers[sig]=signal.signal(sig,handler)
    def watch_enter():
        try:sys.stdin.readline();stop.set()
        except Exception:stop.set()
    threading.Thread(target=watch_enter,daemon=True).start()
    t=None;session=None;report={'schema':'g1-arm-session-v2','started_unix_s':time.time(),
                               'operation':args.command,'physical_success_verified':False,'error':None}
    try:
        t=UnitreeTransport(args.interface);session=ArmSession(t,profile,stop=stop)
        state=session.wait_still()
        if args.command=='run-plan' and max(abs(a-b) for a,b in zip(state.q[15:29],preview['start_arm_q']))>.02:
            raise SafetyError('Live arm pose differs from reviewed plan start')
        # Second stationary check occurs in enable after all validation.
        session.enable();report['start_arm_q']=session.start
        if args.command=='nudge':session.nudge(args.joint,args.delta_deg)
        else:
            if max(abs(a-b) for a,b in zip(session.start,preview['start_arm_q']))>.02:
                raise SafetyError('Start pose changed before enable')
            for point in preview['waypoints']:session.move(point['arm_q'],point['duration_s'])
        report['trajectory_completed']=True
    except BaseException as e:
        report['error']=f'{type(e).__name__}: {e}';stop.set();raise
    finally:
        if session is not None:
            try:session.release();report['release_write_completed']=True
            except Exception as e:report['release_error']=str(e);print('RELEASE FAILED: use physical E-stop/support immediately.',file=sys.stderr)
        if t is not None:t.close()
        report['finished_unix_s']=time.time();write_json(args.log,report)
        for sig,old in old_handlers.items():signal.signal(sig,old)
    print('Bounded arm trajectory finished. Verify physical movement and return yourself.');return 0


if __name__=='__main__':
    try:raise SystemExit(main())
    except (SafetyError,ValueError,KeyError,OSError,ImportError) as e:
        print(f'STOP: {e}',file=sys.stderr);raise SystemExit(2)
