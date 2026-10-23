"""Replay the exact Oct10 decision/observation boundary WITHOUT MuJoCo or DDS.

Uses original HandoffPolicy and image-derived depth geometry. Does not replay
physics, claim a new successful press, move waist, or produce motor commands.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path
import sys
import numpy as np
from PIL import Image
sys.path.insert(0, str(Path(__file__).parent / 'prior_oct10'))
from camera_handoff import CameraFrame, HandoffPolicy, coarse_yaw
from vision import CameraIntrinsics


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def local_path(root, relative):
    path = (root / relative).resolve()
    if not path.is_relative_to(root.resolve()):
        raise ValueError('Input path escapes the recording')
    return path


def load_frame(root, obs, label, drop_depth=False):
    m = obs[label]
    rgb_path = local_path(root, m['rgb'])
    if digest(rgb_path) != m['rgb_sha256']:
        raise ValueError('Recorded camera image hash mismatch')
    rgb = np.asarray(Image.open(rgb_path).convert('RGB'))
    # Exact Oct10 schema assumes 640x480, cx=320, cy=240. Fail, never rescale.
    if rgb.shape != (480,640,3):
        raise ValueError('Oct10 recording requires 640x480 RGB')
    depth = None if label == 'side' else np.load(local_path(root,m['depth']), allow_pickle=False)
    if drop_depth and depth is not None:
        depth = np.zeros_like(depth)
    f = m['focal_px']
    stamps = [m['stamp'],m['pose_stamp'],obs['time_s']]
    if label == 'head':
        stamps.append(m['depth_stamp'])
    if max(m['stamp'],m['pose_stamp'],m.get('depth_stamp',m['stamp'])) > obs['time_s']:
        raise ValueError('Future camera timestamp')
    if not np.all(np.isfinite(stamps)):
        raise ValueError('Non-finite timestamps')
    return CameraFrame(rgb,depth,CameraIntrinsics(f,f,320,240),np.array(m['T_world_optical']),
                       m['stamp'],m.get('depth_stamp'),m['pose_stamp'],m['calibration_id'])


class OfflineDualCamera:
    def __init__(self):
        self.policy = HandoffPolicy()
        self.actions = 0

    def preview(self, root, observation, decision):
        root = Path(root)
        p = self.policy
        if p.state == 'STOP' or self.actions >= 8:
            raise ValueError('Latched STOP or eight-action limit')
        if decision['observation_sha256'] != digest(local_path(root, observation['pair_rgb'])):
            raise ValueError('Decision does not match paired image')
        if observation.get('pair_sha256') != decision['observation_sha256']:
            raise ValueError('Observation metadata hash mismatch')
        side = load_frame(root,observation,'side')
        head = load_frame(root,observation,'head',decision.get('inject_depth_dropout',False))
        now = observation['time_s']
        kind = decision['action']
        self.actions += 1
        if kind == 'align_head':
            if p.last_target is not None:
                result = p.hold('waist_scan_requires_retracted_arm')
            else:
                torso = np.asarray(observation['T_world_torso'])
                if torso.shape != (4,4) or not np.all(np.isfinite(torso)):
                    raise ValueError('Invalid torso transform')
                yaw = coarse_yaw(side,decision['side_bbox'],torso,
                                 observation['waist_yaw_self_state_rad'],now)
                p.state = 'ALIGN_HEAD'
                result = {'state':p.state,'suggested_waist_yaw_rad':yaw,
                          'requires_separate_waist_control_design':True}
        elif kind in ('approach','press'):
            bbox = decision.get('head_bbox')
            partial = decision.get('track_previous_bbox',False)
            if partial and p.last_target is not None:
                t = head.T_world_optical
                optical = t[:3,:3].T @ (np.array(p.last_target.face_xyz)-t[:3,3])
                if not np.all(np.isfinite(optical)) or optical[2] <= 0:
                    result = p.hold('tracked_target_behind_camera')
                    return self._report(kind,result)
                k = head.intrinsics
                center = np.array([k.cx+k.fx*optical[0]/optical[2],k.cy+k.fy*optical[1]/optical[2]])
                old = p.last_bbox
                half = np.array([old[2]-old[0],old[3]-old[1]])/2
                bbox = [*(center-half),*(center+half)]
            result = p.localize(side,head,bbox,now,decision['label'],allow_partial=partial)
            if result.get('arm_motion_allowed'):
                # Original action geometry; not an executed trajectory.
                spacing = -.065 if kind == 'approach' else -.012
                result = {**result,'goal_xyz':(np.array(p.last_target.face_xyz)+spacing*np.array(p.last_target.press_direction)).tolist()}
        elif kind == 'retract' and p.last_target is not None:
            side.validate(now)
            head.validate(now, require_depth=True)
            if abs(side.rgb_stamp-head.rgb_stamp) > .030:
                raise ValueError('camera_pair_unsynchronized')
            tcp = np.asarray(observation['tcp_self_state_xyz'],dtype=float)
            if tcp.shape != (3,) or not np.all(np.isfinite(tcp)):
                raise ValueError('Invalid recorded TCP')
            result = {'state':'RETRACT_PREVIEW','goal_xyz':(tcp-.05*np.array(p.last_target.press_direction)).tolist()}
            p.last_target = p.last_bbox = p.last_stamp = None
            p.state = 'SEARCH_SIDE'
        else:
            raise ValueError('Unsupported action or missing target')
        return self._report(kind,result)

    @staticmethod
    def _report(kind,result):
        return {'schema':'g1-dual-camera-offline-preview-v1', 'action':kind,
                'geometry':result, 'hardware_executable':False,
                'dds_publishers_created':0,'motor_commands_sent':0,
                'warning':'Recorded-input replay only; geometry permission is not hardware authorization'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',required=True)
    p.add_argument('--output',required=True)
    args=p.parse_args()
    root=Path(args.recording)
    runner=OfflineDualCamera()
    results=[]
    for path in sorted(root.glob('decision_*.json')):
        decision=json.loads(path.read_text())
        observations=[json.loads(x.read_text()) for x in sorted(root.glob('obs_*.json'))]
        matching=[x for x in observations if x['pair_sha256']==decision['observation_sha256']]
        if len(matching)!=1:
            raise ValueError('Unique observation required')
        results.append(runner.preview(root,matching[0],decision))
    if not results:
        raise ValueError('No recorded decisions')
    Path(args.output).write_text(json.dumps(results,indent=2,allow_nan=False)+'\n')
    print(f'{len(results)} decisions replayed offline; no DDS or physics executed')


if __name__=='__main__':
    main()
