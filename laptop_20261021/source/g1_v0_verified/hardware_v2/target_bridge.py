"""Offline conversion from an Oct10 geometry result to a reviewed-plan DRAFT.

This does not capture real RGB-D, invoke an LLM, operate the robot, or turn a
sim-assumed camera calibration into a real calibration. Final operator review
and live state matching remain mandatory in g1_arm run-plan.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from offline_adapter import build_joint_preview
from arm_runtime import angles,SafetyError


def to_review_draft(joint_preview,snapshot,profile):
    if joint_preview['schema']!='g1-button-offline-preview-v1':raise SafetyError('Wrong joint-preview schema')
    start=angles(snapshot['motor_q_rad'][15:29]);points=[]
    for phase in ('approach','contact','press','retract','home'):
        q=list(start);q[7:]=joint_preview['poses'][phase]
        points.append({'phase':phase,'arm_q':q,'duration_s':joint_preview['durations'][phase]})
    return {'schema':'g1-arms-reviewed-plan-v1','robot_id':profile['robot_id'],
            'firmware_version':profile['firmware_version'],'joint_indices':list(range(15,29)),
            'start_arm_q':start,'waypoints':points,'reviewed_by':'','collision_checked':False,
            'status':'DRAFT_NOT_EXECUTABLE','calibration_note':'Operator must verify real calibration and collision review',
            'limits_note':'Five-degree total envelope and 0.1 rad/s enforced by runtime; large button approach is rejected'}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--joint-preview',required=True);p.add_argument('--snapshot',required=True)
    p.add_argument('--profile',required=True);p.add_argument('--output',required=True)
    a=p.parse_args()
    load=lambda x:json.loads(Path(x).read_text())
    draft=to_review_draft(load(a.joint_preview),load(a.snapshot),load(a.profile))
    Path(a.output).write_text(json.dumps(draft,indent=2,allow_nan=False)+'\n')
    print('Draft only. Not executable until operator review; runtime bounds remain unchanged.')


if __name__=='__main__':main()
