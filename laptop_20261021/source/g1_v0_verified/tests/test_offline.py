import builtins
import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from dual_camera_preview import OfflineDualCamera
from offline_adapter import build_joint_preview, prepare_target
from vision import ButtonTarget
from trajectory import load_plan

ROOT=Path(__file__).resolve().parents[1]
EVIDENCE=ROOT/'fixtures'/'dual_camera'


def inputs(index):
    return (json.loads((EVIDENCE/f'obs_{index:02d}.json').read_text()),
            json.loads((EVIDENCE/f'decision_{index:02d}.json').read_text()))


class GuardTests(unittest.TestCase):
    def test_real_oct10_schema_replays_without_sdk_or_sockets(self):
        original=builtins.__import__
        def guarded(name,*args,**kwargs):
            if name.startswith(('unitree','mujoco','cyclonedds')):
                raise AssertionError('Forbidden import: '+name)
            return original(name,*args,**kwargs)
        with patch('builtins.__import__',side_effect=guarded), patch('socket.socket',side_effect=AssertionError('No sockets')):
            runner=OfflineDualCamera()
            reports=[runner.preview(EVIDENCE,*inputs(i)) for i in range(4)]
        self.assertEqual(reports[0]['geometry']['state'],'ALIGN_HEAD')
        self.assertTrue(reports[1]['geometry']['arm_motion_allowed'])
        self.assertTrue(reports[2]['geometry']['arm_motion_allowed'])
        self.assertEqual(reports[3]['geometry']['state'],'RETRACT_PREVIEW')
        self.assertTrue(all(not x['hardware_executable'] for x in reports))
        self.assertTrue(all(x['motor_commands_sent']==0 for x in reports))

    def test_missing_depth_holds_then_latches_stop(self):
        obs,decision=inputs(1)
        decision['inject_depth_dropout']=True
        runner=OfflineDualCamera()
        first=runner.preview(EVIDENCE,obs,decision)
        second=runner.preview(EVIDENCE,obs,decision)
        self.assertEqual(first['geometry']['state'],'HOLD_REACQUIRE')
        self.assertEqual(second['geometry']['state'],'STOP')
        self.assertFalse(second['geometry']['arm_motion_allowed'])
        with self.assertRaises(ValueError):
            runner.preview(EVIDENCE,*inputs(1))
        (ROOT/'dropout_replay.json').write_text(json.dumps([first,second],indent=2)+'\n')

    def test_wrong_image_hash_rejected(self):
        obs,decision=inputs(1)
        decision['observation_sha256']='0'*64
        with self.assertRaises(ValueError): OfflineDualCamera().preview(EVIDENCE,obs,decision)

    def test_wrong_label_holds(self):
        obs,decision=inputs(1)
        decision['label']='down'
        self.assertFalse(OfflineDualCamera().preview(EVIDENCE,obs,decision)['geometry']['arm_motion_allowed'])

    def test_stale_rgb_holds(self):
        obs,decision=inputs(1)
        obs['time_s']+=.101
        self.assertFalse(OfflineDualCamera().preview(EVIDENCE,obs,decision)['geometry']['arm_motion_allowed'])

    def test_depth_timestamp_skew_holds(self):
        obs,decision=inputs(1)
        obs['head']['depth_stamp']-=.016
        self.assertFalse(OfflineDualCamera().preview(EVIDENCE,obs,decision)['geometry']['arm_motion_allowed'])

    def test_camera_pair_skew_holds(self):
        obs,decision=inputs(1)
        obs['side']['stamp']-=.031
        obs['side']['pose_stamp']-=.031
        self.assertFalse(OfflineDualCamera().preview(EVIDENCE,obs,decision)['geometry']['arm_motion_allowed'])

    def test_future_timestamp_rejected(self):
        obs,decision=inputs(1)
        obs['head']['stamp']+=.001
        with self.assertRaises(ValueError): OfflineDualCamera().preview(EVIDENCE,obs,decision)

    def test_stale_retract_rejected(self):
        runner=OfflineDualCamera()
        runner.preview(EVIDENCE,*inputs(1))
        obs,decision=inputs(3)
        obs['time_s']+=.101
        with self.assertRaises(ValueError): runner.preview(EVIDENCE,obs,decision)

    def test_invalid_extrinsics_holds(self):
        obs,decision=inputs(1)
        obs['head']['T_world_optical'][0][0]=2.
        self.assertFalse(OfflineDualCamera().preview(EVIDENCE,obs,decision)['geometry']['arm_motion_allowed'])

    def test_nonfinite_stamp_rejected(self):
        obs,decision=inputs(1)
        obs['head']['depth_stamp']=float('nan')
        with self.assertRaises(ValueError): OfflineDualCamera().preview(EVIDENCE,obs,decision)

    def test_scan_after_localize_does_not_move(self):
        runner=OfflineDualCamera()
        runner.preview(EVIDENCE,*inputs(1))
        report=runner.preview(EVIDENCE,*inputs(0))
        self.assertEqual(report['geometry']['reason'],'waist_scan_requires_retracted_arm')
        self.assertFalse(report['geometry']['arm_motion_allowed'])

    def test_eight_action_limit(self):
        runner=OfflineDualCamera()
        runner.actions=8
        with self.assertRaises(ValueError): runner.preview(EVIDENCE,*inputs(0))


class TargetApiTests(unittest.TestCase):
    def setUp(self):
        obs,decision=inputs(1)
        m=obs['head']
        self.now=obs['time_s']
        self.intent={'schema':'g1-button-selection-v1','frame_id':m['rgb'], 'camera_id':'head',
                     'detection_id':'up','action':'press'}
        self.obs={'frame_id':m['rgb'],'camera_id':'head','camera_role':'primary_rgbd',
                  'depth_aligned_to_color':True,'rgb_timestamp_s':m['stamp'],
                  'depth_timestamp_s':m['depth_stamp'],'pose_timestamp_s':m['pose_stamp'],
                  'detections':[{'id':'up','bbox':decision['head_bbox']}],
                  'intrinsics':dict(fx=m['focal_px'],fy=m['focal_px'],cx=320,cy=240),
                  'depth_m':np.load(EVIDENCE/m['depth'])}
        self.cal={'camera_id':'head','base_frame':'stationary_floor','calibration_id':'test_fixture_not_robot_calibration',
                  'verified_on_robot':True,'T_base_optical':m['T_world_optical']}
    def target(self):return prepare_target(self.intent,self.obs,self.cal,self.now)
    def test_new_api_localizes_recorded_depth(self):
        self.assertAlmostEqual(self.target().face_xyz[0],.35046,places=4)
    def test_llm_joint_values_rejected(self):
        self.intent['joint_angles']=[0.]*7
        with self.assertRaises(ValueError):self.target()
    def test_side_camera_direct_target_rejected(self):
        self.obs['camera_role']='side_rgb'
        with self.assertRaises(ValueError):self.target()
    def test_pose_time_skew_rejected(self):
        self.obs['pose_timestamp_s']-=.02
        with self.assertRaises(ValueError):self.target()
    def test_unknown_selection_rejected(self):
        self.intent['detection_id']='other'
        with self.assertRaises(ValueError):self.target()
    def test_missing_depth_rejected(self):
        self.obs['depth_m']=np.zeros((480,640))
        with self.assertRaises(ValueError):self.target()


class FakeKin:
    def __init__(self): self.calls=[]
    def update_fixed_base(self,q,rpy): self.q=q
    def solve(self,point,normal,position_tolerance_m):
        self.calls.append(('solve',point))
        class Result: pass
        r=Result()
        r.joint_angles=self.q[22:29]+.001
        r.position_error_m=.00001
        r.axis_error_deg=0.
        r.min_joint_margin_rad=.1
        return r
    def validate_path(self,*args,**kwargs): self.calls.append(('path',args[0]))


class PlanningTests(unittest.TestCase):
    def setUp(self):
        self.target=ButtonTarget((.35,-.25,1.),(1.,0.,0.),.5,1.,1.)
        self.state={'motor_q_rad':[0.]*29,'imu_rpy_rad':[0.]*3,'timestamp_s':1.,
                    'max_leg_speed_rad_s':0.,'max_gyro_rad_s':0.}
        self.cal={'base_frame':'stationary_floor','verified_on_robot':True,'tip_radius_m':.015}
    def plan(self,kin=None):
        return build_joint_preview(self.target,self.state,self.cal,kin or FakeKin(),.0015,1.)
    def test_joint_preview_checks_all_paths_and_returns_home(self):
        kin=FakeKin()
        result=self.plan(kin)
        self.assertEqual(len([x for x in kin.calls if x[0]=='path']),5)
        self.assertEqual(result['poses']['home'],[0.]*7)
        self.assertEqual(result['motor_indices'],list(range(22,29)))
        self.assertFalse(result['hardware_executable'])
    def test_existing_executor_rejects_preview(self):
        with tempfile.TemporaryDirectory() as directory:
            p=Path(directory)/'preview.json';p.write_text(json.dumps(self.plan()))
            with self.assertRaises(ValueError): load_plan(p)
    def test_state_age_rejected(self):
        self.state['timestamp_s']=0.
        with self.assertRaises(ValueError):self.plan()
    def test_not_stationary_rejected(self):
        self.state['max_leg_speed_rad_s']=.06
        with self.assertRaises(ValueError):self.plan()
    def test_unverified_calibration_rejected(self):
        self.cal['verified_on_robot']=False
        with self.assertRaises(ValueError):self.plan()
    def test_collision_failure_propagates(self):
        kin=FakeKin()
        kin.validate_path=lambda *a,**k: (_ for _ in ()).throw(ValueError('collision'))
        with self.assertRaisesRegex(ValueError,'collision'): self.plan(kin)
    def test_ik_failure_propagates(self):
        kin=FakeKin()
        kin.solve=lambda *a,**k: (_ for _ in ()).throw(ValueError('unreachable'))
        with self.assertRaisesRegex(ValueError,'unreachable'): self.plan(kin)
    def test_nan_state_rejected(self):
        self.state['motor_q_rad'][22]=float('nan')
        with self.assertRaises(ValueError):self.plan()


if __name__=='__main__': unittest.main()
