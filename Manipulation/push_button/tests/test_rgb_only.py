"""RGBの円の射影から、既知半径を使わず面法線を復元できることを検証する。"""
from __future__ import annotations
import sys
from pathlib import Path
import cv2
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/"sim"))
from run_rgb_only import circle_normal_candidates


def test_circle_normal_from_perspective_conic() -> None:
    normal = np.array([.8,.2,.56])
    normal /= np.linalg.norm(normal)
    first = np.cross(normal,[0.,1.,0.])
    first /= np.linalg.norm(first)
    second = np.cross(normal,first)
    angles = np.linspace(0,2*np.pi,200,endpoint=False)
    points = np.array([.1,-.1,1.])+.05*(np.cos(angles)[:,None]*first+np.sin(angles)[:,None]*second)
    pixels = np.column_stack((320+615*points[:,0]/points[:,2],240+615*points[:,1]/points[:,2]))
    center,axes,angle = cv2.fitEllipse(pixels.astype(np.float32).reshape(-1,1,2))
    obs = {"focal_px":615.,"camera_T_world_optical":np.eye(4).tolist()}
    candidates = circle_normal_candidates(obs,(np.array(center),np.array(axes),np.deg2rad(angle)))
    # 単眼の2候補のうち、真の面法線が含まれる。実試験では2画像の整合性で選ぶ。
    assert max(float(n@normal) for n in candidates) > .99999
