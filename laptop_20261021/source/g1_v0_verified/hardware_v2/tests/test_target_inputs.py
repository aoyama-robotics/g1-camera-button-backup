import hashlib
import json
from pathlib import Path
import sys
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from prepare_inputs import make_inputs
from target_bridge import to_review_draft
ROOT=Path(__file__).resolve().parents[2]/'fixtures'/'dual_camera'

class InputTests(unittest.TestCase):
    def setUp(self):
        obs=json.loads((ROOT/'obs_01.json').read_text());m=obs['head']
        decision=json.loads((ROOT/'decision_01.json').read_text())
        self.capture=dict(timestamp_clock='unix_seconds',frame_id='fixture',camera_id='head',camera_role='primary_rgbd',
          depth_aligned_to_color=True,rgb_path=m['rgb'],depth_path=m['depth'],rgb_sha256=m['rgb_sha256'],
          depth_sha256=hashlib.sha256((ROOT/m['depth']).read_bytes()).hexdigest(),
          rgb_timestamp_s=m['stamp'],depth_timestamp_s=m['depth_stamp'],pose_timestamp_s=m['pose_stamp'],
          detections=[dict(id='up',bbox=decision['head_bbox'])],intrinsics=dict(fx=m['focal_px'],fy=m['focal_px'],cx=320,cy=240))
        self.intent=dict(schema='g1-button-selection-v1',frame_id='fixture',camera_id='head',detection_id='up',action='press')
        self.cal=dict(camera_id='head',calibration_id='test-only-not-real',verified_on_robot=True,
                      base_frame='stationary_floor',T_base_optical=m['T_world_optical'])
        self.state=dict(timestamp_s=m['stamp'],motor_q_rad=[0.]*29)
    def run_input(self):return make_inputs(self.capture,self.intent,self.cal,self.state,ROOT,.0015)
    def test_fixture_to_input_contract(self):
        result=self.run_input();self.assertTrue(result['offline_only']);self.assertEqual(result['snapshot'],self.state)
    def test_sim_calibration_rejected(self):
        self.cal['calibration_id']='sim_assumed_head_v1'
        with self.assertRaises(ValueError):self.run_input()
    def test_depth_hash_rejected(self):
        self.capture['depth_sha256']='0'*64
        with self.assertRaises(ValueError):self.run_input()
    def test_wrong_clock_rejected(self):
        self.capture['timestamp_clock']='monotonic_of_another_pc'
        with self.assertRaises(ValueError):self.run_input()
    def test_stale_state_rejected(self):
        self.state['timestamp_s']-=1
        with self.assertRaises(ValueError):self.run_input()
    def test_review_draft_not_automatically_approved(self):
        phases=('approach','contact','press','retract','home')
        preview=dict(schema='g1-button-offline-preview-v1',poses={p:[0.]*7 for p in phases},durations={p:3. for p in phases})
        draft=to_review_draft(preview,self.state,dict(robot_id='fixture',firmware_version='test'))
        self.assertEqual(draft['status'],'DRAFT_NOT_EXECUTABLE');self.assertFalse(draft['collision_checked'])
        self.assertEqual(draft['reviewed_by'],'')

if __name__=='__main__':unittest.main()
