"""Assemble local, byte-preserved successful controllers. No robot operations."""
import hashlib,json,shutil,sys,xml.etree.ElementTree as ET
from pathlib import Path

ROOT=Path(__file__).resolve().parent
DEST=ROOT/'G1_START_HERE'
DEST.mkdir(exist_ok=True)
controllers=DEST/'controllers';controllers.mkdir(exist_ok=True)
for directory,entry,test in (
 ('one_degree_reference_retry','one_degree_reference.py','check_reference_equivalence.py'),
 ('fifteen_degree_reference','fifteen_degree_reference.py','check_tracking_math.py')):
    target=controllers/directory;target.mkdir(exist_ok=True)
    for name in (entry,'arm_runtime.py','shadow_motion_guard.py','precise_timing.py','diagnostic_capture.py',test):
        shutil.copyfile(ROOT/directory/name,target/name)
    evidence=DEST/'evidence'/directory;evidence.mkdir(parents=True,exist_ok=True)
    for source in (ROOT/directory).glob('*.json'):shutil.copyfile(source,evidence/source.name)
old=DEST/'evidence'/'original_supported_vr';old.mkdir(parents=True,exist_ok=True)
for name in ('supported_vr_lowcmd.py','supported_vr_lowcmd_result.json','SHA256_MANIFEST.json','environment.json','requirements-installed.txt'):
    shutil.copyfile(ROOT/'successful_supported_vr_local'/name,old/name)
geometry=controllers/'right_arm_20deg_preparation';geometry.mkdir(exist_ok=True)
for name in ('coordinated_geometry.py','check_sweep.py','g1_29dof.urdf','collision_only.urdf','geometry_manifest.json'):
    shutil.copyfile(ROOT/'right_arm_20deg_preparation'/name,geometry/name)
shutil.copytree(ROOT/'right_arm_20deg_preparation'/'meshes',geometry/'meshes',dirs_exist_ok=True)
# Only a resource-location change: preserve geometry and transforms verbatim.
xml=ET.parse(geometry/'collision_only.urdf')
xml.getroot().find('mujoco/compiler').set('meshdir','meshes')
xml.write(geometry/'collision_only.urdf',encoding='utf-8')
tools=DEST/'tools';tools.mkdir(exist_ok=True)
shutil.copyfile(ROOT/'right_arm_20deg_preparation'/'status_fresh_readonly.py',tools/'status_readonly.py')
shutil.copyfile(ROOT/'release_ai_for_supported_vr.py',tools/'release_ai.py')
provenance={
 'python':str(ROOT/'g1_sdk_venv'/'Scripts'/'python.exe'),
 'nic':{'alias':'Ethernet 3 / イーサネット 3','index':63,'description':'Realtek USB GbE Family Controller','mac':'98-FC-84-ED-8C-53','ipv4':'<PRIVATE_IP>/24','gateway':None,'dns':None},
 'dds':{'domain':0,'state':'rt/lowstate','command':'rt/lowcmd'},
 'restart_cycle_tested':False,'robot_operated_during_packaging':False,
 'controller_bytes_preserved':True,
 'geometry_resource_change':'collision_only.urdf compiler.meshdir set to relative meshes; mesh bytes/transforms unchanged',
 'sdk_source':'https://github.com/unitreerobotics/unitree_sdk2_python/archive/refs/heads/master.zip',
 'source_commit_pinned':False,
 'verified_15deg':{'sent':516,'measured_peak_deg':-12.844391007809712,'max_sampled_tracking_error_deg':2.4922874950387213,'pre_release_return_error_deg':-.05493179639982755,'final_motor_mode':0},
}
(DEST/'PROVENANCE.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2),encoding='utf-8')
print(str(DEST))
