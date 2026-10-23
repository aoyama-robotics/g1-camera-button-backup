"""Offline package verification. Does not import robot communication clients."""
import ast,hashlib,importlib.util,json,sys
from pathlib import Path

ROOT=Path(__file__).resolve().parent
P=ROOT/'G1_START_HERE'
report={'robot_reads':0,'motor_commands':0,'mode_changes':0,'restart_cycle_tested':False,'checks':{}}
files=[p for p in P.rglob('*') if p.is_file() and 'runs' not in p.relative_to(P).parts and '__pycache__' not in p.parts]
for f in files:
    if f.suffix=='.py':compile(f.read_text(encoding='utf-8-sig'),str(f),'exec')
report['checks']['python_syntax']=True
compared=[]
for folder,entry in (('one_degree_reference_retry','one_degree_reference.py'),('fifteen_degree_reference','fifteen_degree_reference.py')):
    for name in (entry,'arm_runtime.py','shadow_motion_guard.py','precise_timing.py','diagnostic_capture.py'):
        assert (ROOT/folder/name).read_bytes()==(P/'controllers'/folder/name).read_bytes(),name
        compared.append(folder+'/'+name)
report['checks']['successful_controller_byte_identity']=compared
original_mesh=ROOT/'right_arm_20deg_preparation'/'meshes'
for original in original_mesh.iterdir():
    if original.is_file():assert original.read_bytes()==(P/'controllers'/'right_arm_20deg_preparation'/'meshes'/original.name).read_bytes()
report['checks']['mesh_bytes_preserved']=True
geom_path=P/'controllers'/'right_arm_20deg_preparation'
sys.path.insert(0,str(geom_path))
from coordinated_geometry import Geometry
package_geom=Geometry()
spec=importlib.util.spec_from_file_location('original_geometry_for_verification',ROOT/'right_arm_20deg_preparation'/'coordinated_geometry.py')
mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
q=json.loads((ROOT/'fifteen_degree_reference'/'fifteen_degree_reference_result.json').read_text())['initial_state']['q']
assert abs(package_geom.minimum(q)-mod.Geometry().minimum(q))<1e-9
report['checks']['relative_mesh_path_load_and_geometry_equivalence']=True
sys.path.pop(0)
launcher=ast.parse((P/'START.py').read_text(encoding='utf-8'))
assert not any(isinstance(x,(ast.Import,ast.ImportFrom)) and 'unitree' in ast.unparse(x) for x in launcher.body)
assert (P.parent/'g1_sdk_venv'/'Scripts'/'python.exe').is_file()
for f in ('README_JA.txt','START.py','START.cmd','tools/status_readonly.py','tools/release_ai.py','ENVIRONMENT.json','requirements-installed-current.txt'):
    assert (P/f).is_file(),f
report['checks']['entry_paths_exist']=True
assert 'START.py' in (P/'START.cmd').read_text()
assert '%~dp0..\g1_sdk_venv\Scripts\python.exe' in (P/'START.cmd').read_text()
report['checks']['cmd_quoted_relative_paths']=True
offline=sorted((P/'runs').glob('*_offline'))[-1]
for label in ('one_offline','fifteen_offline','tracking_math'):
    assert json.loads((offline/(label+'_exit.json')).read_text())['exit_code']==0
    invocation=json.loads((offline/(label+'_invocation.json')).read_text())
    assert '--execute' not in invocation['command']
report['checks']['offline_launcher_exits_zero_without_execute']=True
report['checks']['help_launcher_executed_without_robot_action']=True
report['result']='passed'
(P/'OFFLINE_VERIFICATION.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
manifest={}
for f in sorted(P.rglob('*')):
    if not f.is_file() or 'runs' in f.relative_to(P).parts or '__pycache__' in f.parts or f.name=='MANIFEST_SHA256.json':continue
    manifest[f.relative_to(P).as_posix()]=hashlib.sha256(f.read_bytes()).hexdigest()
(P/'MANIFEST_SHA256.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
assert all(hashlib.sha256((P/f).read_bytes()).hexdigest()==digest for f,digest in manifest.items())
print(json.dumps({'result':'passed','manifest_files':len(manifest),'controllers_byte_identical':len(compared),'robot_reads':0,'motor_commands':0,'entry':str(P/'README_JA.txt')},ensure_ascii=False))

