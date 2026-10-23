from pathlib import Path
import json,numpy as np,cv2
exec(Path('analyze_two_calibration_points_rev1.py').read_text().split('records=[]')[0])
p=Path('bottle_camera_discovery/contact_diagnostic_20261021');a=json.loads((p/'short_contact_analysis.json').read_text());q=np.array(a['q_median']);poses=fk(q);T=poses['torso_link'];m=json.loads((p/'contact/metadata.json').read_text());i=m['intrinsics'];C=tr([.0576235,.01753,.42987],[0,.8307767239493009,0]);cam_inv=np.linalg.inv(C)
def project(torso):
 opt=Ropt.T@(cam_inv@np.r_[torso,1])[:3]
 return [float(i['fx']*opt[0]/opt[2]+i['ppx']),float(i['fy']*opt[1]/opt[2]+i['ppy'])],opt
landmarks={}
for name in ['right_shoulder_pitch_link','right_shoulder_roll_link','right_shoulder_yaw_link','right_elbow_link','right_wrist_roll_link','right_wrist_pitch_link','right_wrist_yaw_link','right_rubber_hand']:
 torso=(np.linalg.inv(T)@poses[name])[:3,3];uv,opt=project(torso);landmarks[name]={'torso_m':torso.tolist(),'pixel':uv,'optical_m':opt.tolist()}
for name,offset in [('middle_tip',[.1733,.0283,.0067]),('index_tip',[.168,.0283,.03]),('ring_tip',[.168,.0283,-.02]),('palm_center',[.100,.02,.0067])]:
 torso=(np.linalg.inv(T)@poses['right_wrist_yaw_link']@np.r_[offset,1])[:3];uv,opt=project(torso);landmarks[name]={'torso_m':torso.tolist(),'pixel':uv,'optical_m':opt.tolist(),'note':'middle tip official derived; other tips/palm approximate mesh-region landmarks'}
im=cv2.imread(str(p/'contact/rgb.jpg'));canvas=np.zeros((760,900,3),np.uint8);canvas[:480,:640]=im
for x,y,w,h in [(445,300,33,20),(535,415,70,65),(445,345,70,55)]:cv2.rectangle(canvas,(x,y),(x+w,y+h),(0,255,255),1)
cv2.putText(canvas,'YELLOW: visually selected contact / wrist / palm regions',(12,735),cv2.FONT_HERSHEY_SIMPLEX,.45,(0,255,255),1)
chain=['right_shoulder_pitch_link','right_shoulder_roll_link','right_shoulder_yaw_link','right_elbow_link','right_wrist_roll_link','right_wrist_pitch_link','right_wrist_yaw_link','middle_tip']
for x,y in zip(chain,chain[1:]):
 u=landmarks[x]['pixel'];v=landmarks[y]['pixel'];cv2.line(canvas,tuple(np.clip(np.array(u).astype(int),-2000,2000)),tuple(np.clip(np.array(v).astype(int),-2000,2000)),(255,120,50),2)
for name,d in landmarks.items():
 xy=tuple(np.array(d['pixel']).astype(int));cv2.circle(canvas,xy,4,(255,150,0),-1);cv2.putText(canvas,name.replace('right_','').replace('_link',''),(xy[0]+5,xy[1]),cv2.FONT_HERSHEY_SIMPLEX,.34,(255,200,70),1)
cv2.putText(canvas,'NOMINAL MODEL PROJECTION - no fitted correction',(12,700),cv2.FONT_HERSHEY_SIMPLEX,.5,(255,200,70),1)
cv2.imwrite(str(p/'multi_landmark_overlay.png'),canvas);(p/'multi_landmark_projection.json').write_text(json.dumps({'landmarks':landmarks,'observed_regions_are_visual_estimates':True,'q_right_arm_deg':np.degrees(q[22:29]).tolist()},indent=2));print(json.dumps({k:np.round(v['pixel'],1).tolist() for k,v in landmarks.items()},indent=2))
