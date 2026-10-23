"""Offline nominal official geometry check for a single right-shoulder sweep."""
import json,math
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parent
NAMES=[s+'_'+j+'_joint' for s in ['left','right'] for j in ['hip_pitch','hip_roll','hip_yaw','knee','ankle_pitch','ankle_roll']]+['waist_yaw_joint','waist_roll_joint','waist_pitch_joint']+[s+'_'+j+'_joint' for s in ['left','right'] for j in ['shoulder_pitch','shoulder_roll','shoulder_yaw','elbow','wrist_roll','wrist_pitch','wrist_yaw']]

def check(q,delta_deg=-20):
    import mujoco
    robot=ET.parse(ROOT/'g1_29dof.urdf').getroot()
    for link in robot.findall('link'):
        for visual in list(link.findall('visual')):link.remove(visual)
    robot.find('mujoco/compiler').set('meshdir',str(ROOT/'meshes'))
    for mesh in robot.findall('.//mesh'):mesh.set('filename',Path(mesh.attrib['filename']).name)
    modelpath=ROOT/'collision_only.urdf'
    ET.ElementTree(robot).write(modelpath,encoding='utf-8')
    model=mujoco.MjModel.from_xml_path(str(modelpath))
    data=mujoco.MjData(model)
    addresses=[int(model.joint(n).qposadr[0]) for n in NAMES]
    assert len(q)==29 and model.nq==29
    model.geom_margin[:]=.005
    start=np.array(q,dtype=float);violations=[];min_distance=None
    for deg in np.linspace(0,delta_deg,81):
        values=start.copy();values[22]+=math.radians(deg)
        data.qpos[addresses]=values;mujoco.mj_forward(model,data)
        for c in data.contact:
            b1=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[c.geom1])) or ''
            b2=mujoco.mj_id2name(model,mujoco.mjtObj.mjOBJ_BODY,int(model.geom_bodyid[c.geom2])) or ''
            a=lambda x:x.startswith(('right_shoulder','right_elbow','right_wrist'))
            if not (a(b1) or a(b2)):continue
            if min_distance is None or c.dist<min_distance:min_distance=float(c.dist)
            violations.append(dict(delta_deg=float(deg),body1=b1,body2=b2,distance_m=float(c.dist)))
    return dict(nominal_geometry_clear=len(violations)==0,samples=81,
      collision_margin_m=.005,minimum_reported_contact_distance_m=min_distance,
      contacts=violations,model_nq=model.nq,scope='official bare-robot collision shapes; external support/accessories are not modeled')

if __name__=='__main__':
    r=json.loads((ROOT/'release_after_battery.json').read_text())
    result=check([x['q'] for x in r['state']['joints']])
    (ROOT/'sweep_clearance.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
    print(json.dumps({**result,'contacts':result['contacts'][:15]}))
