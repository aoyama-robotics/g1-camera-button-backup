"""Offline-only boundary from LLM selection to existing G1 RGB-D/IK code.

No SDK import, socket, robot connection, or hardware execution entrypoint exists.
The LLM selects a detection from one primary RGB-D image, never joint values.
"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np

UPSTREAM = Path(__file__).parent / 'prior_oct10'
sys.path.insert(0, str(UPSTREAM))
from vision import CameraIntrinsics, estimate_button_target, validate_reachable_target
from trajectory import ARM_JOINTS, PHASES, validate_arm_q, validate_button_stroke

FRAME = 'stationary_floor'


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name}: boolean is not a number')
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f'{name}: finite value required')
    return value


def prepare_target(intent, observation, calibration, now_s):
    """Validate matching recorded inputs, then compute depth-derived target.

    All timestamps share the recording's seconds clock. Passing historical now_s
    replays an archive; it is NOT proof of current robot state or calibration.
    Side-camera pixels must be re-localized in the primary RGB-D image first.
    """
    if set(intent) != {'schema', 'frame_id', 'camera_id', 'detection_id', 'action'}:
        raise ValueError('Intent must only select a detection, never coordinates or joints')
    if intent['schema'] != 'g1-button-selection-v1' or intent['action'] != 'press':
        raise ValueError('Unsupported intent')
    if not observation['frame_id'] or not observation['camera_id']:
        raise ValueError('Image and camera identity required')
    if (intent['frame_id'] != observation['frame_id'] or
            intent['camera_id'] != observation['camera_id']):
        raise ValueError('Selection does not belong to this RGB-D image')
    if observation['camera_role'] != 'primary_rgbd':
        raise ValueError('Side-camera selection needs primary RGB-D re-localization')
    if observation['depth_aligned_to_color'] is not True:
        raise ValueError('Color-aligned depth required')
    if calibration['camera_id'] != observation['camera_id'] or calibration['base_frame'] != FRAME:
        raise ValueError('Calibration camera/frame mismatch')
    if not isinstance(calibration.get('calibration_id'), str) or not calibration['calibration_id'].strip():
        raise ValueError('Calibration provenance ID required')
    if calibration.get('verified_on_robot') is not True:
        raise ValueError('Unverified calibration cannot generate a target')
    now = finite(now_s, 'now_s')
    rgb = finite(observation['rgb_timestamp_s'], 'rgb time')
    depth = finite(observation['depth_timestamp_s'], 'depth time')
    pose = finite(observation['pose_timestamp_s'], 'pose time')
    if pose > now or abs(pose-rgb) > .015:
        raise ValueError('Pose timestamp mismatch')
    if max(abs(now-rgb), abs(now-depth)) > 0.10 or now < max(rgb, depth):
        raise ValueError('Recorded image is stale or future-dated')
    if abs(rgb-depth) > 0.015:
        raise ValueError('RGB and depth are not synchronized')
    detections = observation['detections']
    ids = [d['id'] for d in detections]
    if len(ids) != len(set(ids)):
        raise ValueError('Detection IDs are ambiguous')
    chosen = [d for d in detections if d['id'] == intent['detection_id']]
    if len(chosen) != 1:
        raise ValueError('Exactly one known detection required')
    intr = CameraIntrinsics(**observation['intrinsics'])
    target = estimate_button_target(np.asarray(observation['depth_m'], dtype=float),
                                   tuple(chosen[0]['bbox']), intr,
                                   np.asarray(calibration['T_base_optical'], dtype=float))
    validate_reachable_target(target)  # upstream right-arm workspace and panel direction
    return target


def build_joint_preview(target, snapshot, calibration, kin, stroke_m, now_s):
    """Use upstream FK/IK and collision checks; return non-executable preview.

    kin is an upstream ArmKinematics object. Tests inject an in-memory fake.
    The preview is intentionally NOT g1-push-button-plan-v1 and has no
    sim_result.success claim. Existing hardware runner must reject this file.
    """
    validate_reachable_target(target)
    stroke = validate_button_stroke(stroke_m)
    if calibration['base_frame'] != FRAME or calibration.get('verified_on_robot') is not True:
        raise ValueError('Verified stationary-floor calibration required')
    q = np.asarray(snapshot['motor_q_rad'], dtype=float)
    rpy = np.asarray(snapshot['imu_rpy_rad'], dtype=float)
    if q.shape != (29,) or rpy.shape != (3,) or not np.all(np.isfinite(np.r_[q, rpy])):
        raise ValueError('Finite 29-DoF snapshot and three-axis IMU required')
    age = finite(now_s, 'now_s') - finite(snapshot['timestamp_s'], 'state time')
    if not 0 <= age <= 0.4:
        raise ValueError('State snapshot is stale or future-dated')
    if (abs(rpy[0]) > 0.10 or abs(rpy[1]) > 0.10 or
            abs(finite(snapshot['max_leg_speed_rad_s'], 'leg speed')) > 0.05 or
            abs(finite(snapshot['max_gyro_rad_s'], 'gyro')) > 0.05):
        raise ValueError('Snapshot is not stationary and upright')
    # Snapshot check does not establish a sustained live standstill.
    radius = finite(calibration['tip_radius_m'], 'tip radius')
    if not 0 < radius < 0.04:
        raise ValueError('Invalid tip radius')
    home = np.asarray(validate_arm_q(q[22:29].tolist()))
    face = np.asarray(target.face_xyz)
    normal = np.asarray(target.press_direction)
    contact = face - normal * radius
    positions = {'approach': contact-normal*0.05, 'contact': contact,
                 'press': contact+normal*stroke, 'retract': contact-normal*0.05}
    poses, durations, metrics = {}, {}, {}
    previous = home.copy()
    for phase in PHASES:
        kin.update_fixed_base(q.copy(), tuple(rpy))
        if phase == 'home':
            goal = home.copy()
        else:
            solution = kin.solve(positions[phase], normal, position_tolerance_m=0.00005)
            goal = np.asarray(validate_arm_q(solution.joint_angles.tolist()))
            if (not all(math.isfinite(float(x)) for x in
                        (solution.position_error_m, solution.axis_error_deg, solution.min_joint_margin_rad))
                    or not 0 <= solution.position_error_m <= 0.00005
                    or not 0 <= solution.axis_error_deg < 10.0
                    or solution.min_joint_margin_rad < 0.025):
                raise ValueError('IK validation failed')
            metrics[phase] = {'position_error_m': float(solution.position_error_m),
                              'axis_error_deg': float(solution.axis_error_deg),
                              'min_joint_margin_rad': float(solution.min_joint_margin_rad)}
        # During press->retract the starting tip is intentionally beyond panel.
        # All other robot geometry and self collision remain checked upstream.
        kin.validate_path(goal, face, normal, allow_tip_contact=phase in ('contact','press','retract'))
        delta = float(np.max(np.abs(goal-previous)))
        duration = max(2.0, 1.5*delta/0.5, math.sqrt(6*delta/1.0))
        if phase == 'press':
            duration = max(duration, stroke/0.00075 * 1.5)
        if duration > 20:
            raise ValueError('Preview segment exceeds bounded duration')
        poses[phase] = goal.tolist()
        durations[phase] = duration
        q[22:29] = goal
        previous = goal
    return {'schema': 'g1-button-offline-preview-v1', 'hardware_executable': False,
            'source_base_commit': '561c2d735f3da96d2e86e826e96f8ab2e026f275', 'source': 'Oct10 local/vlm-button-sim delivery (unpublished modifications)',
            'base_frame': FRAME, 'arm_joint_names': list(ARM_JOINTS),
            'motor_indices': list(range(22,29)), 'poses': poses, 'durations': durations,
            'ik_validation': metrics, 'stroke_m': stroke,
            'limitations': ['No DDS or robot connection', 'No physics simulation performed',
                           'No sustained live standstill verification',
                           'No contact-force or physical button-success verification']}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--inputs', required=True, help='Recorded input JSON')
    p.add_argument('--output', required=True, help='Offline preview JSON')
    p.add_argument('--model', help='Optional locally available Menagerie G1 29DoF XML')
    args = p.parse_args()
    doc = json.loads(Path(args.inputs).read_text())
    cal = doc['calibration']
    target = prepare_target(doc['intent'], doc['observation'], cal, doc['now_s'])
    result = {'schema': 'g1-button-target-preview-v1', 'hardware_executable': False,
              'base_frame': FRAME, 'face_xyz': target.face_xyz,
              'press_direction': target.press_direction}
    if args.model:
        from kinematics import ArmKinematics, load_robot
        model, _ = load_robot(Path(args.model), tuple(cal['tip_offset_m']),
                              cal['pelvis_height_m'], cal['tip_radius_m'])
        result = build_joint_preview(target, doc['snapshot'], cal, ArmKinematics(model),
                                     doc['stroke_m'], doc['now_s'])
    Path(args.output).write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print('Offline preview saved. No robot connection or DDS commands.')


if __name__ == '__main__':
    main()
