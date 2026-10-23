from pathlib import Path
root=Path(__file__).resolve().parent
source=root.parent/'successful_supported_vr_local'
for name in ['arm_runtime.py','precise_timing.py','shadow_motion_guard.py']:
    (root/name).write_bytes((source/name).read_bytes())
s=(source/'supported_vr_lowcmd.py').read_text(encoding='utf-8')
s=s.replace('math.radians(1)','math.radians(20)').replace('One degree envelope','Twenty degree envelope')
s=s.replace('def check(self,anchor=None):','def check(self,anchor=None,target22=None):')
s=s.replace('if abs(s.q[i]-anchor[i])>limit:',"expected=target22 if i==22 and target22 is not None else anchor[i]\n                if abs(s.q[i]-expected)>limit:")
s=s.replace("Path('supported_vr_lowcmd_result.json')","Path('right_arm_20deg_result.json')")
s=s.replace('time.sleep(25);stop.set()','time.sleep(40);stop.set()')
s=s.replace('s=t.check(anchor)','s=t.check(anchor,target)')
s=s.replace('phase(5.,','phase(10.,').replace('phase(.3,','phase(1.,')
s=s.replace('beginning -1 degree shoulder trajectory','beginning -20 degree shoulder trajectory')
s=s.replace("pub=ch.ChannelPublisher('rt/lowcmd',LowCmd_);pub.Init()", "from check_sweep import check as check_sweep\n        clearance=check_sweep(anchor)\n        report['nominal_geometry_clearance']=clearance\n        if not clearance['nominal_geometry_clear']:raise SafetyError('Sweep geometry clearance not established')\n        latest=t.check()\n        if max(abs(a-b) for a,b in zip(latest.q,anchor))>.01:raise SafetyError('Pose changed during geometry check')\n        pub=ch.ChannelPublisher('rt/lowcmd',LowCmd_);pub.Init()")
s=s.replace('next_at=None','next_at=None;last_target=None;last_target_at=None')
s=s.replace('nonlocal next_at','nonlocal next_at,last_target,last_target_at')
s=s.replace('msg=fill(unitree_hg_msg_dds__LowCmd_(),anchor,target,s.mode_machine);',"if last_target is not None and abs(target-last_target)>.10*(now-last_target_at)+1e-7:raise SafetyError('Command velocity exceeded')\n            last_target=target;last_target_at=now\n            msg=fill(unitree_hg_msg_dds__LowCmd_(),anchor,target,s.mode_machine);")
(root/'right_arm_20deg.py').write_text(s,encoding='utf-8')
print('Prepared executable; --execute required; geometry/debug/health gates retained')
