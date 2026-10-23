from pathlib import Path
p=Path(__file__).resolve().parent
s=(p/'bounded_direct_hold.py').read_text(encoding='utf-8')
s=s.replace('from engagement_proposal_offline import gains,TOTAL_SECONDS','from engagement_proposal_offline import gains\nfrom coordinated_geometry import Geometry\nTOTAL_SECONDS=40.0')
s=s.replace("bounded_direct_hold_result.json","coordinated_raise_result.json")
s=s.replace('c.kp,c.kd=gains(i,elapsed)', 'gain_time=elapsed if elapsed<=1.5 else (1.5 if elapsed<=39 else elapsed-37)\n        c.kp,c.kd=gains(i,gain_time)')
s=s.replace('if not stop.wait(15)', 'if not stop.wait(55)')
s=s.replace("'intentional_displacement_deg':0", "'intentional_pitch_deg':-20,'intentional_roll_deg':None")
s=s.replace('def check(anchor=None):', 'def check(anchor=None,expected=None,elapsed=0):')
s=s.replace("if anchor is not None and max(abs(a-b) for a,b in zip(s.q,anchor))>.005:raise SafetyError('Current-pose displacement exceeded 0.005rad')", """if anchor is not None:
            for i in range(29):
                limit=math.radians(3) if i==22 and 10<=elapsed<=31 else (math.radians(1) if i==23 else .002)
                goal=expected[i] if expected is not None else anchor[i]
                if abs(s.q[i]-goal)>limit:raise SafetyError('Tracking/non-target displacement motor '+str(i))
            if s.q[23]>anchor[23]+.001:raise SafetyError('Inward motion toward body')""")
s=s.replace("s=check();anchor=s.q;report['initial_state']=dataclasses.asdict(s)", """s=check();anchor=s.q;report['initial_state']=dataclasses.asdict(s)
        geometry=Geometry();plan=geometry.plan(anchor);report['geometry_plan']=plan
        report['intentional_roll_deg']=-plan['opening_deg']
        print('Plan: roll outward '+str(plan['opening_deg'])+'deg /8s; pitch -20deg /10s; hold1s; pitch return10s; roll reverse8s; gain ramp down1s',flush=True)
        check(anchor,anchor)
        def smooth(x):
            x=max(0.,min(1.,x));return x*x*(3-2*x)
        def targets(t):
            a=list(anchor);opening=math.radians(plan['opening_deg'])
            if t<10:a[23]-=opening*smooth((t-2)/8)
            elif t<31:a[23]-=opening
            elif t<39:a[23]-=opening*(1-smooth((t-31)/8))
            if 10<=t<20:a[22]-=math.radians(20)*smooth((t-10)/10)
            elif 20<=t<21:a[22]-=math.radians(20)
            elif 21<=t<31:a[22]-=math.radians(20)*(1-smooth((t-21)/10))
            angles(a[15:29]);return a
        for value in (0,2,10,20,21,31,39,40):targets(value)""")
s=s.replace('s=check(anchor)\n            for i,v', "expected=targets(elapsed)\n            s=check(anchor,expected,elapsed)\n            if geometry.minimum(s.q)<.002:raise SafetyError('Actual nominal mesh clearance below 2mm')\n            for i,v")
s=s.replace('msg=packet(anchor,s.mode_machine,elapsed)', 'msg=packet(expected,s.mode_machine,elapsed)')
s=s.replace("motor_modes=s.motor_modes,kp=", "motor_modes=s.motor_modes,target_q=expected,kp=")
s=s.replace("report['finite_hold_completed']=True", "report['coordinated_trajectory_completed']=True")
s=s.replace("report['max_displacement_deg']=max", "report['pitch_peak_deg']=min(math.degrees(x['q'][22]-base[22]) for x in report['observations'])\n            report['roll_peak_deg']=min(math.degrees(x['q'][23]-base[23]) for x in report['observations'])\n            report['pitch_return_error_deg']=math.degrees(report['observations'][-1]['q'][22]-base[22])\n            report['roll_return_error_deg']=math.degrees(report['observations'][-1]['q'][23]-base[23])\n            report['max_displacement_deg']=max")
(p/'coordinated_raise.py').write_text(s,encoding='utf-8')
