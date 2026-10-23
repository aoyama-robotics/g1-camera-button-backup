from pathlib import Path
import shutil
root=Path(__file__).resolve().parent.parent
p=root/'one_degree_reference_retry';p.mkdir(exist_ok=True)
original=root/'successful_supported_vr_local'
for name in ('arm_runtime.py','precise_timing.py','shadow_motion_guard.py'):
    shutil.copyfile(original/name,p/name)
shutil.copyfile(root/'right_arm_20deg_preparation'/'diagnostic_capture.py',p/'diagnostic_capture.py')
s=(p/'arm_runtime.py').read_text();s=s.replace('def validate(self,now,expected_machine=5,check_body_velocity=True):','def validate(self,now,expected_machine=5,check_body_velocity=True,allow_startup_modes=False):');s=s.replace('self.live[i] and self.motor_modes[i]==1 for i in ARM','self.live[i] and self.motor_modes[i] in ((0,1) if allow_startup_modes else (1,)) for i in ARM');(p/'arm_runtime.py').write_text(s)
s=(p/'shadow_motion_guard.py').read_text();s=s.replace('class ShadowGuard:\n','class ShadowGuard:\n    allowed_motor_modes=(1,)\n');s=s.replace('any(x!=1 for x in motor_modes)','any(x not in self.allowed_motor_modes for x in motor_modes)');(p/'shadow_motion_guard.py').write_text(s)
s=(original/'supported_vr_lowcmd.py').read_text()
s=s.replace('from precise_timing import PreciseSleeper','from precise_timing import PreciseSleeper\nfrom diagnostic_capture import Capture,command_dict\nimport sys\nsys.path.insert(0,str(Path(__file__).resolve().parent.parent / "right_arm_20deg_preparation"))\nfrom coordinated_geometry import Geometry')
# Path is needed at import, and prepend our own module location after loading geometry.
s=s.replace('import sys\nsys.path', 'import sys\nfrom pathlib import Path\nsys.path')
s=s.replace('from coordinated_geometry import Geometry','from coordinated_geometry import Geometry\nsys.path.pop(0)')
s=s.replace('self.guard=ShadowGuard();self.guard.position_envelope=.03','self.guard=ShadowGuard();self.guard.position_envelope=.03\n        self.guard.allowed_motor_modes=(0,1)\n        self.startup_at=None;self.all_enabled=False;self.latched=[False]*29;self.capture=Capture();self.command_stopped=False')
s=s.replace('s=State.parse(m,time.perf_counter())','''s=State.parse(m,time.perf_counter())
            with self.lock:self.capture.received(dataclasses.asdict(s))''')
s=s.replace('self.guard.observe(s.robot_tick', '''if self.command_stopped:return
            for i,value in enumerate(s.motor_modes):
                if self.latched[i] and value!=1:raise SafetyError('Enabled motor dropped mode '+str(i))
                self.latched[i]=self.latched[i] or value==1
            self.all_enabled=all(self.latched)
            if self.all_enabled:self.guard.allowed_motor_modes=(1,)
            elif self.startup_at is not None and time.perf_counter()-self.startup_at>.25:
                raise SafetyError('Direct enable not confirmed in 250ms')
            self.guard.observe(s.robot_tick''')
s=s.replace('s.validate(now,5,check_body_velocity=False)', '''allow_startup=self.startup_at is None or (now-self.startup_at<=.25 and not self.all_enabled)
        s.validate(now,5,check_body_velocity=False,allow_startup_modes=allow_startup)''')
s=s.replace('s.live[i] and s.motor_modes[i]==1 for i in range(29)', 's.live[i] and s.motor_modes[i] in ((0,1) if allow_startup else (1,)) for i in range(29)')
s=s.replace("logfile=Path('supported_vr_lowcmd_result.json')", "logfile=Path(__file__).with_name('one_degree_reference_result.json')")
s=s.replace('pub=ch.ChannelPublisher', '''geometry=Geometry();nominal=[]
        for k in range(21):
            q=list(anchor);q[22]-=math.radians(k*.05);nominal.append(geometry.minimum(q))
        report['nominal_geometry_min_m']=min(nominal)
        if min(nominal)<0:raise SafetyError('One degree nominal path has model penetration')
        latest=t.check(anchor)
        pub=ch.ChannelPublisher''')
s=s.replace('s=t.check(anchor)\n            if next_at', '''s=t.check(anchor)
            if geometry.minimum(s.q)<0:raise SafetyError('Observed nominal mesh penetration')
            if next_at''')
s=s.replace("if pub.Write(msg) is not True:raise SafetyError('DDS Write failed')", '''with t.lock:t.capture.preparing(command_dict(msg,now,0))
            if t.startup_at is None:t.startup_at=time.perf_counter()
            write_started=time.perf_counter();result=pub.Write(msg);write_finished=time.perf_counter()
            command=command_dict(msg,now,0);command.update(write_started=write_started,write_finished=write_finished,write_result=result)
            report['last_write_attempt']=command
            if result is not True:raise SafetyError('DDS Write failed')
            with t.lock:t.capture.sent(command)''')
s=s.replace('q=list(s.q),dq=list(s.dq),tick=s.robot_tick)', 'q=list(s.q),dq=list(s.dq),tick=s.robot_tick,motor_modes=list(s.motor_modes))')
s=s.replace("except BaseException as e:report['error']=type(e).__name__+': '+str(e)", '''except BaseException as e:
        report['error']=type(e).__name__+': '+str(e)
        if pub is not None:pub.Close();pub=None
        if t is not None:
            with t.lock:report['fault_context']=t.capture.fault(report['error'],time.perf_counter())''')
s=s.replace('if pub is not None:pub.Close()\n        report', 'if t is not None:t.command_stopped=True\n        if pub is not None:pub.Close()\n        report')
s=s.replace("'terminal_state','receiver_fault')", "'terminal_state','receiver_fault','fault_context','last_write_attempt')")
(p/'one_degree_reference.py').write_text(s,encoding='utf-8')
