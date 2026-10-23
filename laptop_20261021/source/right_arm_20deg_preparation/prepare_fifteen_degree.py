from pathlib import Path
import shutil
root=Path(__file__).resolve().parent.parent
src=root/'one_degree_reference_retry';dst=root/'fifteen_degree_reference';dst.mkdir(exist_ok=True)
for name in ('arm_runtime.py','shadow_motion_guard.py','precise_timing.py','diagnostic_capture.py'):
 shutil.copyfile(src/name,dst/name)
s=(src/'one_degree_reference.py').read_text(encoding='utf-8')
s=s.replace('math.radians(1)','math.radians(15)').replace('math.radians(k*.05)','math.radians(k*(15./20))')
s=s.replace('one_degree_reference_result.json','fifteen_degree_reference_result.json').replace('One degree envelope','Fifteen degree envelope').replace('beginning -1 degree','beginning -15 degree')
s='''# Prepared amplitude-only variant; NOT EXECUTED.
# Existing 3-degree displacement-from-start guard still applies to target shoulder.
# A 15-degree completion requires resolving that existing guard reference first.
'''+s
(dst/'fifteen_degree_reference.py').write_text(s,encoding='utf-8')
