from pathlib import Path
p=Path(__file__).resolve().parent
s=(p/'coordinated_raise.py').read_text(encoding='utf-8')
s=s.replace('from coordinated_geometry import Geometry', 'from coordinated_geometry import Geometry\nfrom diagnostic_capture import Capture,command_dict')
s=s.replace("coordinated_raise_result.json", "coordinated_raise_diagnostic_result.json")
s=s.replace('lock=threading.Lock();box={};stop=', 'lock=threading.Lock();box={};capture=Capture();stop=')
s=s.replace('s=State.parse(m,time.perf_counter())', """host=time.perf_counter()
            raw=dict(host=host,tick=m.tick,mode_machine=m.mode_machine,
                rpy=list(m.imu_state.rpy),gyro=list(m.imu_state.gyroscope),
                motors=[dict(index=i,mode=x.mode,q=x.q,dq=x.dq,tau_est=x.tau_est,
                             vol=x.vol,temperature=list(x.temperature)) for i,x in enumerate(m.motor_state[:29])])
            with lock:capture.received(raw)
            s=State.parse(m,host)""")
s=s.replace("with lock:box['error']=str(e)", "with lock:box['error']=str(e)",1)
s=s.replace("if err:raise SafetyError(err)\n        now=time.perf_counter()", """if s is not None:
            with lock:capture.checked(dict(host=time.perf_counter(),state=dataclasses.asdict(s),
                expected_q=list(expected) if expected is not None else None,elapsed=elapsed,
                ownership_at=at,receiver_error=err,
                fast_dq=[dict(index=i,dq=v) for i,v in enumerate(s.dq) if abs(v)>.1]))
        if err:raise SafetyError(err)
        now=time.perf_counter()""")
s=s.replace('expected=targets(elapsed)\n            s=check', """expected=targets(elapsed)
            preview=packet(expected,5,elapsed)
            with lock:capture.preparing(command_dict(preview,time.perf_counter(),elapsed))
            s=check""")
s=s.replace("if pub.Write(msg) is not True:raise SafetyError('DDS Write failed')", """write_started=time.perf_counter();write_result=pub.Write(msg);write_finished=time.perf_counter()
            cmd=command_dict(msg,now,elapsed)
            cmd.update(write_started=write_started,write_finished=write_finished,write_result=write_result)
            report['last_write_attempt']=cmd
            if write_result is not True:raise SafetyError('DDS Write failed')
            with lock:capture.sent(cmd)""")
s=s.replace("except BaseException as e:report['error']=type(e).__name__+': '+str(e)", """except BaseException as e:
        report['error']=type(e).__name__+': '+str(e)
        with lock:report['fault_context']=capture.fault(report['error'],time.perf_counter())""")
s=s.replace("'terminal_state','observations')", "'terminal_state','observations','fault_context','last_write_attempt')")
(p/'coordinated_raise_diagnostic.py').write_text(s,encoding='utf-8')
