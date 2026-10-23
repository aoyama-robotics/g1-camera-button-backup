from pathlib import Path
import json,numpy as np,xml.etree.ElementTree as E,mujoco,cv2
exec(Path('analyze_two_calibration_points_rev1.py').read_text().split('records=[]')[0])
p=Path('bottle_camera_discovery/contact_diagnostic_20261021');q=np.array(json.loads((p/'short_contact_analysis.json').read_text())['q_median']);u=E.parse('bottle_camera_discovery/g1_29dof_rev_1_0.urdf').getroot()
for link in u.findall('link'):
 for tag in ['visual','collision']:
  for child in list(link.findall(tag)):link.remove(child)
mu=u.find('mujoco')
if mu is None:mu=E.SubElement(u,'mujoco')
co=mu.find('compiler')
if co is None:co=E.SubElement(mu,'compiler')
co.set('fusestatic','false');co.set('balanceinertia','true')
model=mujoco.MjModel.from_xml_string(E.tostring(u,encoding='unicode'));data=mujoco.MjData(model)
for name,v in zip(names,q):data.qpos[model.joint(name).qposadr[0]]=v
mujoco.mj_forward(model,data);P=fk(q);err={}
for name in ['torso_link','right_elbow_link','right_wrist_yaw_link','right_rubber_hand','d435_link']:
 j=model.body(name).id;err[name]={'position_error_m':float(np.linalg.norm(data.xpos[j]-P[name][:3,3])),'rotation_max_abs_error':float(np.max(np.abs(data.xmat[j].reshape(3,3)-P[name][:3,:3])))}
m=json.loads((p/'contact/metadata.json').read_text());d=np.load(p/'contact/depth_aligned.npy');i=m['intrinsics'];points={}
for name,(x,y,r) in {'middle_tip':(462,314,3),'palm':(475,356,5),'wrist_metal':(555,432,5)}.items():
 a=d[y-r:y+r+1,x-r:x+r+1];v=a[a>0]*m['depth_scale'];z=float(np.median(v));points[name]={'pixel':[x,y],'depth_m':z,'valid_fraction':float(np.mean(a>0)),'depth_p10_p90_m':np.percentile(v,[10,90]).tolist(),'optical_m':[(x-i['ppx'])*z/i['fx'],(y-i['ppy'])*z/i['fy'],z]}
v=np.array(points['middle_tip']['optical_m'])-points['wrist_metal']['optical_m'];proj=json.loads((p/'multi_landmark_projection.json').read_text())['landmarks'];mv=np.array(proj['middle_tip']['optical_m'])-proj['right_wrist_yaw_link']['optical_m'];out={'independent_mujoco_vs_explicit_urdf_chain':err,'elbow_urdf_origin':E.parse('bottle_camera_discovery/g1_29dof_rev_1_0.urdf').getroot().find("joint[@name='right_elbow_joint']/origin").attrib,'observed_surface_samples':points,'observed_wrist_surface_to_tip_surface_m':v.tolist(),'observed_surface_distance_m':float(np.linalg.norm(v)),'model_joint_origin_to_tip_m':mv.tolist(),'model_distance_m':float(np.linalg.norm(mv)),'surface_vs_model_note':'Visible metal surface is not wrist-yaw origin, glove contact tip not a measured mesh vertex; compare approximate size/orientation only','vector_angle_deg':float(np.degrees(np.arccos(np.clip(v@mv/np.linalg.norm(v)/np.linalg.norm(mv),-1,1))))};(p/'independent_fk_and_hand_depth.json').write_text(json.dumps(out,indent=2));print(json.dumps(out,indent=2))
