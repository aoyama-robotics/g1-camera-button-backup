"""Offline preparation only. No SDK imports, network or motion execution."""
import hashlib,json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

root=Path(__file__).resolve().parent
lines=(root.parent/'battery_restart_state.txt').read_text(encoding='utf-8-sig').splitlines()
state=json.loads(next(x for x in reversed(lines) if x.startswith('{')))
robot=ET.parse(root/'g1_29dof.urdf').getroot()
joints={j.attrib['name']:j for j in robot.findall('joint')}
shoulder=joints['right_shoulder_pitch_joint']
axis=[float(x) for x in shoulder.find('axis').attrib['xyz'].split()]
limits=shoulder.find('limit').attrib
q={}
waist_names=['waist_yaw_joint','waist_roll_joint','waist_pitch_joint']
for item,name in zip(state['waist'],waist_names):q[name]=item['q']
arm_names=['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']
for side,start in [('left',15),('right',22)]:
    for offset,name in enumerate(arm_names):
        q[side+'_'+name+'_joint']=next(x['q'] for x in state['arms'] if x['index']==start+offset)
start=q['right_shoulder_pitch_joint'];goal=start-math.radians(20)
lower,upper=float(limits['lower']),float(limits['upper'])
assert lower+.025<=min(start,goal) and max(start,goal)<=upper-.025

def rot(axis,angle):
    a=np.array(axis,dtype=float);a/=np.linalg.norm(a)
    k=np.array([[0,-a[2],a[1]],[a[2],0,-a[0]],[-a[1],a[0],0]])
    return np.eye(3)+math.sin(angle)*k+(1-math.cos(angle))*(k@k)
def transform(xyz,rpy):
    t=np.eye(4);t[:3,3]=xyz
    r,p,y=rpy;t[:3,:3]=rot([0,0,1],y)@rot([0,1,0],p)@rot([1,0,0],r)
    return t
def fk(values):
    poses={'pelvis':np.eye(4)};pending=list(robot.findall('joint'))
    while pending:
        progress=False
        for j in list(pending):
            parent=j.find('parent').attrib['link'];child=j.find('child').attrib['link']
            if parent not in poses:continue
            origin=j.find('origin');xyz=[0.,0.,0.];rpy=[0.,0.,0.]
            if origin is not None:
                xyz=list(map(float,origin.attrib.get('xyz','0 0 0').split()))
                rpy=list(map(float,origin.attrib.get('rpy','0 0 0').split()))
            t=transform(xyz,rpy)
            if j.attrib['type']!='fixed':
                r=np.eye(4);r[:3,:3]=rot(list(map(float,j.find('axis').attrib['xyz'].split())),values.get(j.attrib['name'],0));t=t@r
            poses[child]=poses[parent]@t;pending.remove(j);progress=True
        if not progress:raise ValueError('Unresolved URDF chain')
    return poses
first=fk(q);last=fk({**q,'right_shoulder_pitch_joint':goal})
positions={}
for name in ['right_elbow_link','right_wrist_yaw_link']:
    positions[name]={'start_xyz_m':first[name][:3,3].tolist(),'goal_xyz_m':last[name][:3,3].tolist(),
                     'delta_xyz_m':(last[name][:3,3]-first[name][:3,3]).tolist()}
collisions=sorted(set(m.attrib['filename'] for m in robot.findall('.//collision/geometry/mesh')))
plan=dict(status='PREPARED_NOT_EXECUTABLE',motor_commands_sent=0,
    observed_state={'check_mode':state['check_mode'],'fsm':state['fsm'],'machine':state['mode_machine'],'tick':state['tick']},
    joint=dict(index=22,name='right_shoulder_pitch_joint',axis=axis,start_rad=start,goal_rad=goal,
               start_deg=math.degrees(start),goal_deg=math.degrees(goal),delta_deg=-20,limits_rad=[lower,upper]),
    trajectory=dict(outward_s=10,hold_s=1,return_s=10,initial_hold_s=1,final_hold_s=.5,
         interpolation='cubic smoothstep 3u^2-2u^3',peak_velocity_rad_s=1.5*math.radians(20)/10,
         peak_acceleration_rad_s2=6*math.radians(20)/100),
    nominal_fk=positions,
    self_collision={'verified':False,'reason':'Complete matching 29DoF collision meshes/model not available locally; FK and joint range are not a collision test','required_meshes':collisions},
    runtime_conditions={'AI_released':False,'fresh_start_must_be_resampled':True,'non_target_pose':'fresh measured all29 hold as successful VR lowcmd','gain_change':False},
    sources={'urdf':'https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description/g1_29dof.urdf',
             'urdf_sha256':hashlib.sha256((root/'g1_29dof.urdf').read_bytes()).hexdigest()})
(root/'planned_motion.json').write_text(json.dumps(plan,indent=2),encoding='utf-8')
print(json.dumps(plan))
