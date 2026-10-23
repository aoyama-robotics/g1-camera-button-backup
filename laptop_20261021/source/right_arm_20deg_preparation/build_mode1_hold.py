from pathlib import Path
import json
p=Path(__file__).resolve().parent
from coordinated_geometry import Geometry
r=json.loads((p/'mode1_baseline_summary.json').read_text())
gap=Geometry().minimum(r['last_q'])
print(json.dumps({'baseline_max_q_range_rad':r['max_q_range_all29'],'baseline_max_dq':r['max_abs_dq_all29'],'nominal_min_gap_m':gap}))
s=(p/'bounded_direct_hold.py').read_text(encoding='utf-8')
s=s.replace('from engagement_proposal_offline import gains,TOTAL_SECONDS','from engagement_proposal_offline import gains,TOTAL_SECONDS\nfrom diagnostic_capture import Capture,command_dict\nfrom coordinated_geometry import Geometry')
s=s.replace('bounded_direct_hold_result.json','mode1_hold_result.json')
s=s.replace('lock=threading.Lock();box={};stop=', 'lock=threading.Lock();box={};capture=Capture();stop=')
s=s.replace('s=State.parse(m,time.perf_counter())', '''host=time.perf_counter()
            raw=dict(host=host,tick=m.tick,mode_machine=m.mode_machine,
                rpy=list(m.imu_state.rpy),gyro=list(m.imu_state.gyroscope),
                motors=[dict(index=i,mode=x.mode,q=x.q,dq=x.dq,tau_est=x.tau_est,vol=x.vol,temperature=list(x.temperature)) for i,x in enumerate(m.motor_state[:29])])
            with lock:capture.received(raw)
            s=State.parse(m,host)''')
s=s.replace('if err:raise SafetyError(err)\n        now=time.perf_counter()', '''if s is not None:
            with lock:capture.checked(dict(host=time.perf_counter(),state=dataclasses.asdict(s),ownership_at=at,
                receiver_error=err,fast_dq=[dict(index=i,dq=v) for i,v in enumerate(s.dq) if abs(v)>.1]))
        if err:raise SafetyError(err)
        now=time.perf_counter()''')
s=s.replace("s=check();anchor=s.q;report['initial_state']=dataclasses.asdict(s)", "s=check();anchor=s.q;report['initial_state']=dataclasses.asdict(s)\n        geometry=Geometry();report['initial_geometry_min_m']=geometry.minimum(anchor)\n        if report['initial_geometry_min_m']<.0025:raise SafetyError('Initial nominal mesh gap insufficient for finite hold')")
s=s.replace('s=check(anchor)\n            for i,v', '''preview=packet(anchor,5,elapsed)
            with lock:capture.preparing(command_dict(preview,time.perf_counter(),elapsed))
            s=check(anchor)
            if geometry.minimum(s.q)<.002:raise SafetyError('Nominal mesh clearance below 2mm')
            for i,v''')
s=s.replace("if pub.Write(msg) is not True:raise SafetyError('DDS Write failed')", '''write_started=time.perf_counter();write_result=pub.Write(msg);write_finished=time.perf_counter()
            cmd=command_dict(msg,now,elapsed);cmd.update(write_started=write_started,write_finished=write_finished,write_result=write_result)
            report['last_write_attempt']=cmd
            if write_result is not True:raise SafetyError('DDS Write failed')
            with lock:capture.sent(cmd)''')
s=s.replace("except BaseException as e:report['error']=type(e).__name__+': '+str(e)", '''except BaseException as e:
        report['error']=type(e).__name__+': '+str(e)
        if pub is not None:pub.Close();pub=None
        with lock:report['fault_context']=capture.fault(report['error'],time.perf_counter())''')
s=s.replace("'initial_state','terminal_state','observations')", "'initial_state','terminal_state','observations','fault_context','last_write_attempt')")
(p/'mode1_hold_diagnostic.py').write_text(s,encoding='utf-8')
