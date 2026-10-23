"""Offline reconstruction, not a claim of captured/transmitted wire bytes."""
import json,hashlib
from pathlib import Path
from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
from unitree_sdk2py.utils.crc import CRC
import sys
sys.path.insert(0,str(Path('g1_v0_verified/hardware_v2').resolve()))
from waist_hold_candidate import fill_hold_command

trial=json.loads(Path('g1_v0_verified/hardware_v2/second_hold_result.json').read_text())
observations=trial['observations'];q=trial['start_arm_q'];waist=trial['waist_anchor']
records=[]
for obs in observations:
    msg=fill_hold_command(unitree_hg_msg_dds__LowCmd_(),q,waist,obs['weight'],5)
    msg.crc=CRC().Crc(msg)
    records.append(dict(unix_s=obs['at'],weight=obs['weight'],crc=msg.crc,
       serialized_sha256=hashlib.sha256(msg.serialize()).hexdigest(),
       measured_q22=obs['measured_q22'],command_q22=obs['command_q22']))
msg=fill_hold_command(unitree_hg_msg_dds__LowCmd_(),q,waist,max(x['weight'] for x in observations),5)
report=dict(provenance='Offline deterministic reconstruction from saved immutable start/waist and logged weights; no wire capture',
    mode_pr=msg.mode_pr,mode_machine=msg.mode_machine,
    static_controlled_fields=[dict(index=i,q=msg.motor_cmd[i].q,dq=msg.motor_cmd[i].dq,
       tau=msg.motor_cmd[i].tau,kp=msg.motor_cmd[i].kp,kd=msg.motor_cmd[i].kd,
       mode=msg.motor_cmd[i].mode) for i in range(12,29)],
    nonzero_leg_slots=[i for i in range(12) if any(getattr(msg.motor_cmd[i],k)!=0 for k in ['mode','q','dq','tau','kp','kd'])],
    records=records)
Path('trial2_evidence/reconstructed_packets.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps({k:v for k,v in report.items() if k!='records'}))
print(json.dumps({'first':records[0],'peak':max(records,key=lambda x:x['weight']),'last':records[-1],
 'max_q22_command_difference':max(abs(x['command_q22']-q[7]) for x in observations)}))
