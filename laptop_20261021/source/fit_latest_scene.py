"""Offline local effective bias and IK candidate for the latest printed-kanji scene."""
from pathlib import Path
import json,sys
import numpy as np
ROOT=Path(__file__).resolve().parent
exec((ROOT/'analyze_two_calibration_points_rev1.py').read_text().split('records=[]')[0])
sys.path.insert(0,str(ROOT/'vision_ik'))
from torso_ik import RightArmIK
from local_camera_correction import LocalCameraCorrection
p=ROOT/'bottle_camera_discovery';out=p/'scene_refresh_contact_20261021_172741'
m=json.loads((out/'metadata.json').read_text());raw=json.loads((p/'scene_refresh1724_20261021/joint_resting.json').read_text());samples=raw['samples'];exposure=m['history'][-1][2]/1000
near=[s for s in samples if abs(s['unix']-exposure)<.05];synchronous=bool(near)
if not near:near=[s for s in samples if s['unix']>=samples[-1]['unix']-.1]
q=np.median([s['q'] for s in near],axis=0);P=fk(q);f=(np.linalg.inv(P['torso_link'])@P['right_wrist_yaw_link']@np.r_[.1733,.0283,.0067,1])[:3]
d=np.load(out/'depth_aligned.npy');i=m['intrinsics'];C=tr([.0576235,.01753,.42987],[0,.8307767239493009,0]);observations={}
for name,(px,py) in {'printed_kanji_center':(737,376),'visible_middle_tip_candidate':(742,395)}.items():
 a=d[py-3:py+4,px-3:px+4];z=a[a>0]*m['depth_scale'];depth=float(np.median(z));opt=np.array([(px-i['ppx'])*depth/i['fx'],(py-i['ppy'])*depth/i['fy'],depth]);cam=(C@np.r_[Ropt@opt,1])[:3]
 observations[name]={'pixel':[px,py],'depth_m':depth,'valid_fraction':float(np.mean(a>0)),'depth_p10_p90_m':np.percentile(z,[10,90]).tolist(),'optical_m':opt.tolist(),'nominal_torso_m':cam.tolist(),'correction_torso_m':(f-cam).tolist()}
target=observations['printed_kanji_center'];delta=np.array(target['correction_torso_m']);cfg={'enabled':False,'mode':'local_effective_translation','frame':'torso','translation_torso_m':delta.tolist(),'reference_camera_optical_m':target['optical_m'],'reference_nominal_torso_m':target['nominal_torso_m'],'reference_fk_tip_torso_m':f.tolist(),'reference_pixel':target['pixel'],'status':'CANDIDATE_TIMING_AND_CONTACT_UNCERTAINTY','absolute_extrinsics_validated':False,'validity_region':'At this demonstrated target only; no measured extrapolation radius','timing_gap_s':exposure-near[-1]['unix'],'no_new_stop_logic':True};config=out/'local_correction_config.json';config.write_text(json.dumps(cfg,indent=2))
layer=LocalCameraCorrection(config);layer.enabled=True # Explicit offline application only.
corrected=layer.correct_torso(target['nominal_torso_m']);rows=[{'sample':'latest_self_fit','used_for_fit':True,'before_mm':float(np.linalg.norm(np.array(target['nominal_torso_m'])-f)*1000),'after_mm':float(np.linalg.norm(corrected-f)*1000)}]
old=json.loads((p/'model_coordinate_diagnosis.json').read_text())['torso_points']
for x in old:
 cam=np.array(x['camera_torso_m']);tip=np.array(x['finger_torso_m']);rows.append({'sample':'older_point_'+str(x['point']),'used_for_fit':False,'before_mm':float(np.linalg.norm(cam-tip)*1000),'after_mm':float(np.linalg.norm(layer.correct_torso(cam)-tip)*1000)})
oc=json.loads((p/'contact_diagnostic_20261021/short_contact_analysis.json').read_text());oldq=np.array(oc['q_median']);oldP=fk(oldq);oldf=(np.linalg.inv(oldP['torso_link'])@oldP['right_wrist_yaw_link']@np.r_[.1733,.0283,.0067,1])[:3];od=json.loads((p/'contact_diagnostic_20261021/independent_fk_and_hand_depth.json').read_text())['observed_surface_samples']['middle_tip']['optical_m'];oldcam=(C@np.r_[Ropt@np.array(od),1])[:3];rows.append({'sample':'older_synchronized_glove_tip','used_for_fit':False,'before_mm':float(np.linalg.norm(oldcam-oldf)*1000),'after_mm':float(np.linalg.norm(layer.correct_torso(oldcam)-oldf)*1000)})
seed=np.median([s['q'] for s in samples if s['unix']<=samples[0]['unix']+.5],axis=0);ik=RightArmIK();solution=ik.solve(corrected,seed);solution.update(seed_is_saved_not_live=True,seed_q=seed.tolist(),target_torso_m=corrected.tolist(),motor_commands=0)
if solution['success']:
 end=np.array(solution['q']);vals=[ik.g.minimum(seed+(end-seed)*t) for t in np.linspace(0,1,101)];solution['direct_interpolation_nominal_min_m']=min(vals);solution['direct_interpolation_model_clear']=min(vals)>=0
(out/'offline_ik_candidate.json').write_text(json.dumps(solution,indent=2));report={'local_translation_mm':(delta*1000).tolist(),'error_comparison':rows,'contact_q_used':q.tolist(),'waist_deg':np.degrees(q[12:15]).tolist(),'selected_joint_interval':[near[0]['unix'],near[-1]['unix']],'selected_count':len(near),'exposure_unix':exposure,'synchronous_joint_coverage':synchronous,'nearest_joint_gap_s':exposure-near[-1]['unix'],'right_arm_stability_peak_to_peak_deg':np.degrees(np.ptp([s['q'] for s in near],axis=0))[22:29].tolist(),'observations':observations,'kanji_vs_visible_tip_pixel_distance':float(np.linalg.norm(np.array(target['pixel'])-observations['visible_middle_tip_candidate']['pixel'])),'uncertainty':'Printed kanji remains visible above glove tip. Operator confirmed alignment but exact tip-to-kanji point not independently verified.2.68s joint/image gap is unresolved. Candidate only; raw data preserved.','ik_success':solution['success'],'ik_error_m':solution['error_m'],'motor_commands':0};(out/'local_correction_evaluation.json').write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k not in ['contact_q_used','observations']},indent=2));print('IK_DIRECT_CLEAR',solution.get('direct_interpolation_model_clear'))
