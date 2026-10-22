"""カメラ導入・同期・深度欠損時に腕を許可しないことをCPUだけで検証する。"""
from __future__ import annotations
from dataclasses import replace
import sys
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_handoff import CameraFrame,HandoffPolicy,coarse_yaw
from vision import CameraIntrinsics


def fixture() -> tuple[CameraFrame,CameraFrame,list]:
    rgb = np.zeros((480,640,3),dtype=np.uint8)
    depth = np.ones((480,640))
    depth[200:280,280:360] = .90
    t = np.eye(4)
    t[:3,:3] = [[0,0,1],[-1,0,0],[0,-1,0]]
    head = CameraFrame(rgb,depth,CameraIntrinsics(600,600,320,240),t,0.,0.,0.,"unit_test_calibration")
    return replace(head,depth_m=None,depth_stamp=None),head,[280,200,360,280]


def test_both_visible_registered_depth_localizes() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,head,bbox,0.,"up")
    assert result["arm_motion_allowed"]
    assert np.allclose(result["face_xyz"],[.9,0,0],atol=.003)


def test_initial_head_out_of_view_holds_arm_and_side_guides_yaw() -> None:
    side,head,_ = fixture()
    result = HandoffPolicy().localize(side,head,[650,200,700,280],0.,"up")
    assert not result["arm_motion_allowed"]
    assert result["reason"]=="head_target_out_of_view"
    yaw = coarse_yaw(side,[500,200,560,280],np.eye(4),0.,0.)
    assert -.35 < yaw < 0


def test_zero_depth_holds_then_latches_stop() -> None:
    side,head,bbox = fixture()
    missing = replace(head,depth_m=np.zeros_like(head.depth_m))
    p = HandoffPolicy()
    assert p.localize(side,missing,bbox,0.,"up")["state"]=="HOLD_REACQUIRE"
    assert p.localize(side,missing,bbox,0.,"up")["state"]=="STOP"
    assert not p.localize(side,head,bbox,0.,"up")["arm_motion_allowed"]


def test_rgb_depth_timestamp_mismatch_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,replace(head,depth_stamp=.020),bbox,0.,"up")
    assert not result["arm_motion_allowed"]
    assert result["reason"]=="rgb_depth_unsynchronized"


def test_unregistered_depth_shape_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,replace(head,depth_m=np.ones((240,320))),bbox,0.,"up")
    assert result["reason"]=="depth_registration_shape_mismatch"


def test_joint_pose_timestamp_mismatch_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,replace(head,pose_stamp=.020),bbox,0.,"up")
    assert result["reason"]=="stale_rgb_or_pose"


def test_camera_pair_timestamp_mismatch_rejected() -> None:
    side,head,bbox = fixture()
    shifted = replace(head,rgb_stamp=.04,depth_stamp=.04,pose_stamp=.04)
    result = HandoffPolicy().localize(side,shifted,bbox,.04,"up")
    assert result["reason"]=="camera_pair_unsynchronized"


def test_wrong_semantic_identity_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,head,bbox,0.,"down")
    assert not result["arm_motion_allowed"]


def test_missing_calibration_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,replace(head,calibration_id=""),bbox,0.,"up")
    assert result["reason"]=="calibration_missing"


def test_partial_occlusion_without_recent_track_rejected() -> None:
    side,head,bbox = fixture()
    result = HandoffPolicy().localize(side,head,bbox,0.,"up",allow_partial=True)
    assert result["reason"]=="occlusion_track_expired"
