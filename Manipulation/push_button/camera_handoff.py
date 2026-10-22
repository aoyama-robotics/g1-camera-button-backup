"""側頭部RGBで対象導入、主観RGB-Dで局所幾何を計測する制御境界。"""
from __future__ import annotations
from dataclasses import dataclass
import math
import numpy as np
from vision import CameraIntrinsics, ButtonTarget, estimate_button_target


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray
    depth_m: np.ndarray | None
    intrinsics: CameraIntrinsics
    T_world_optical: np.ndarray
    rgb_stamp: float
    depth_stamp: float | None
    pose_stamp: float
    calibration_id: str

    def validate(self, now: float, require_depth: bool = False) -> None:
        if not self.calibration_id:
            raise ValueError("calibration_missing")
        if not np.all(np.isfinite([self.intrinsics.fx,self.intrinsics.fy,self.intrinsics.cx,self.intrinsics.cy])) or min(self.intrinsics.fx,self.intrinsics.fy)<1:
            raise ValueError("invalid_intrinsics")
        if abs(now-self.rgb_stamp) > .10 or abs(self.pose_stamp-self.rgb_stamp) > .015:
            raise ValueError("stale_rgb_or_pose")
        if require_depth and (self.depth_m is None or self.depth_stamp is None):
            raise ValueError("depth_missing")
        if require_depth and self.depth_m.shape!=self.rgb.shape[:2]:
            raise ValueError("depth_registration_shape_mismatch")
        if require_depth and abs(self.depth_stamp-self.rgb_stamp) > .015:
            raise ValueError("rgb_depth_unsynchronized")
        transform = self.T_world_optical
        if (transform.shape != (4,4) or not np.all(np.isfinite(transform))
                or not np.allclose(transform[3],[0,0,0,1])
                or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-5)
                or abs(np.linalg.det(transform[:3,:3])-1) > 1e-5):
            raise ValueError("invalid_extrinsics")


def bbox_ok(bbox: list[float], frame: CameraFrame, margin: int = 0) -> bool:
    height,width = frame.rgb.shape[:2]
    return bool(len(bbox)==4 and margin <= bbox[0] < bbox[2] <= width-margin
                and margin <= bbox[1] < bbox[3] <= height-margin)


def coarse_yaw(frame: CameraFrame, bbox: list[float], T_world_torso: np.ndarray,
               current_yaw: float, now: float) -> float:
    """RGBだけのbearingを使う。距離不明なのでカメラ間bboxを直接転写しない。"""
    frame.validate(now)
    if not bbox_ok(bbox,frame):
        raise ValueError("side_target_out_of_view")
    u,v = (bbox[0]+bbox[2])/2,(bbox[1]+bbox[3])/2
    k = frame.intrinsics
    ray = np.array([(u-k.cx)/k.fx,(v-k.cy)/k.fy,1.])
    ray_torso = T_world_torso[:3,:3].T @ frame.T_world_optical[:3,:3] @ ray
    bearing = math.atan2(float(ray_torso[1]),float(ray_torso[0]))
    # 近距離のカメラ間parallaxを見込んで小刻みに導入、head側で意味を再検出する。
    return float(np.clip(current_yaw+np.clip(bearing,-math.radians(20),math.radians(20)),
                         -math.radians(45),math.radians(45)))


class HandoffPolicy:
    def __init__(self) -> None:
        self.state = "SEARCH_SIDE"
        self.failures = 0
        self.last_target: ButtonTarget | None = None
        self.last_bbox: list[float] | None = None
        self.last_stamp: float | None = None

    def hold(self, reason: str) -> dict:
        self.failures += 1
        self.state = "STOP" if self.failures >= 2 else "HOLD_REACQUIRE"
        return {"state":self.state,"reason":reason,"arm_motion_allowed":False,
                "reacquire_count":self.failures}

    def localize(self, side: CameraFrame, head: CameraFrame, bbox: list[float], now: float,
                 label: str, allow_partial: bool = False) -> dict:
        try:
            if self.state == "STOP":
                raise ValueError("latched_stop")
            if label != "up":
                raise ValueError("target_identity_mismatch")
            side.validate(now)
            head.validate(now,require_depth=True)
            if abs(side.rgb_stamp-head.rgb_stamp) > .030:
                raise ValueError("camera_pair_unsynchronized")
            if bbox is None:
                raise ValueError("head_target_missing")
            if not bbox_ok(bbox,head,margin=8):
                raise ValueError("head_target_out_of_view")
            expected = None
            if allow_partial:
                if self.last_target is None or now-self.last_stamp > 5:
                    raise ValueError("occlusion_track_expired")
                expected = np.array(self.last_target.face_xyz)
            target = estimate_button_target(head.depth_m,tuple(bbox),head.intrinsics,
                                            head.T_world_optical,expected_face_xyz=expected,
                                            min_panel_facing_cos=.40,
                                            min_face_valid_fraction=.60)
        except ValueError as error:
            return self.hold(str(error))
        self.last_target,self.last_bbox,self.last_stamp = target,list(bbox),now
        self.failures = 0
        self.state = "LOCALIZED_HEAD_DEPTH"
        return {"state":self.state,"arm_motion_allowed":True,"face_xyz":list(target.face_xyz),
                "press_direction":list(target.press_direction),
                "face_valid_fraction":target.depth_valid_fraction,
                "plane_inlier_fraction":target.plane_inlier_fraction,
                "source":"registered_head_depth_and_calibration","stamp":now}
