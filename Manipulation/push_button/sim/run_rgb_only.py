#!/usr/bin/env python3
"""胴体搭載RGBだけを観測する有限試行。深度・正解座標は目標生成に使わない。"""
from __future__ import annotations
import argparse
import json
import math
from pathlib import Path
import cv2
import mujoco
import numpy as np
from PIL import Image, ImageDraw
import yaml

from run_vlm_closed_loop import write_json, digest, save_state, physical_success
from run_mujoco import build_model, DEFAULT_TIP_OFFSET
from kinematics import arm_addresses, solve_ik
from trajectory import interpolate


def model_for(config: dict) -> tuple[mujoco.MjModel, np.ndarray]:
    model, stand = build_model(Path(config["model"]), .36, -.25, 1., .0015,
                              DEFAULT_TIP_OFFSET, True, rgbd_camera=True,
                              elevator_front=True)
    cam = config["head_camera"]
    # URDFのx前/y左/z上を、MuJoCoのx右/y上/-z前へ変換する。
    roll, pitch, yaw = cam["rpy"]
    if abs(roll) > 1e-9 or abs(yaw) > 1e-9:
        raise ValueError("この最小試験は確認済みのpitch-only取付だけに対応します")
    c, s = math.cos(pitch), math.sin(pitch)
    rotation = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    basis = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]])
    cid = model.camera("button_rgbd").id
    model.cam_bodyid[cid] = model.body(cam["parent_body"]).id
    model.cam_mode[cid] = int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
    model.cam_pos[cid] = cam["xyz"]
    mujoco.mju_mat2Quat(model.cam_quat[cid], (rotation @ basis).ravel())
    model.cam_fovy[cid] = cam["fovy_deg"]
    return model, stand


def observe(root: Path, model: mujoco.MjModel, data: mujoco.MjData, idx: int) -> dict:
    rgb_path = root / f"obs_{idx:02d}.png"
    renderer = mujoco.Renderer(model, height=480, width=640)
    try:
        renderer.update_scene(data, camera="button_rgbd")
        Image.fromarray(renderer.render().copy()).save(rgb_path)
    finally:
        renderer.close()
    cid = model.camera("button_rgbd").id
    transform = np.eye(4)
    transform[:3, :3] = data.cam_xmat[cid].reshape(3, 3) @ np.diag([1, -1, -1])
    transform[:3, 3] = data.cam_xpos[cid]
    focal = 240 / math.tan(math.radians(model.cam_fovy[cid])/2)
    tcp = data.site("button_tcp").xpos.copy()
    optical = transform[:3, :3].T @ (tcp-transform[:3, 3])
    pixel = [float(320+focal*optical[0]/optical[2]), float(240+focal*optical[1]/optical[2])]
    obs = {"index": idx, "rgb": rgb_path.name, "rgb_sha256": digest(rgb_path),
           "time_s": data.time, "camera_T_world_optical": transform.tolist(), "focal_px": focal,
           "tcp_self_state_xyz": tcp.tolist(), "tcp_projection_from_fk": pixel,
           "tcp_optical_z_from_fk": float(optical[2]),
           "waist_yaw_self_state_rad": float(data.qpos[model.joint("waist_yaw_joint").qposadr[0]]),
           "instruction": "Find and press UP using only this head RGB image, camera calibration and robot self state.",
           "depth_rendering": False, "target_ground_truth_in_policy": False}
    write_json(root / f"obs_{idx:02d}.json", obs)
    write_json(root / "observation.json", obs)
    print(json.dumps(obs))
    return obs


def load(root: Path) -> tuple[dict, mujoco.MjModel, mujoco.MjData, np.ndarray]:
    config = json.loads((root/"scene.json").read_text(encoding="utf-8"))
    model, stand = model_for(config)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, np.load(root/"state.npy"), mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    metrics = json.loads((root/"metrics.json").read_text(encoding="utf-8"))
    if physical_success(metrics):
        model.geom_rgba[model.geom("button_face_up").id] = (.95, .81, .49, 1.)
    return config, model, data, stand


def init(root: Path, model_path: str, camera_config: Path) -> None:
    if root.exists():
        raise ValueError("別の出力フォルダを指定してください")
    root.mkdir(parents=True)
    (root/"frames").mkdir()
    cfg = {"model": str(Path(model_path).resolve()), "head_camera": yaml.safe_load(camera_config.read_text(encoding="utf-8"))["head_camera"],
           "camera_source": "G1_Hackason Dev/ButtonPress e297a430 configs/robot.yaml; Unitree URDF ccfc6fd8",
           "camera_real_calibration_verified": False, "base_fixed": True, "rgb_only": True}
    write_json(root/"scene.json", cfg)
    model, stand = model_for(cfg)
    data = mujoco.MjData(model)
    data.qpos[:29], data.ctrl[:] = stand, stand
    mujoco.mj_forward(model, data)
    write_json(root/"metrics.json", {"max_stroke_m": 0., "max_other_stroke_m": 0., "contact_steps": 0,
                                    "contact_at_threshold_steps": 0, "max_contact_force_n": 0., "actions": 0, "frames": 0})
    save_state(root, model, data)
    observe(root, model, data, 0)


def ellipse_from_rgb(path: Path, bbox: list[float]) -> tuple[np.ndarray, np.ndarray, float]:
    # VLM選択ROIのRGB色だけから円形面のellipse中心を精緻化する。sim segmentationではない。
    rgb = np.asarray(Image.open(path).convert("RGB"))
    x0, y0, x1, y1 = map(int, bbox)
    if not (0 <= x0 < x1 <= 640 and 0 <= y0 < y1 <= 480):
        raise ValueError("bboxが画像範囲外です")
    roi = rgb[y0:y1, x0:x1].astype(int)
    mask = ((roi[:,:,0] > 100) & (roi[:,:,1] > 90) & (roi[:,:,2] > 70)
            & (roi[:,:,0]-roi[:,:,2] > 7)).astype(np.uint8)*255
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        raise ValueError("ROI内に淡色のボタン面がありません")
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < 100 or len(contour) < 5:
        raise ValueError("円形面のRGB点が不足しています")
    (x, y), axes, angle = cv2.fitEllipse(contour)
    return np.array([x+x0, y+y0]), np.array(axes), math.radians(angle)


def circle_normal_candidates(obs: dict, ellipse: tuple) -> list[np.ndarray]:
    """RGBの円形面の射影conicから法線候補を求める。半径・深度は不要。"""
    center, axes, angle = ellipse
    c,s = math.cos(angle),math.sin(angle)
    rot = np.array([[c,-s],[s,c]])
    matrix = rot @ np.diag(1/(axes/2)**2) @ rot.T
    conic = np.empty((3,3))
    conic[:2,:2] = matrix
    conic[:2,2] = conic[2,:2] = -matrix @ center
    conic[2,2] = center @ matrix @ center-1
    f = obs["focal_px"]
    intr = np.array([[f,0,320],[0,f,240],[0,0,1]])
    values,vectors = np.linalg.eigh(intr.T @ conic @ intr)
    low,mid,high = values
    if not low < 0 < mid <= high:
        raise ValueError("円形面のconicが不正です")
    a = math.sqrt(max(0.,(high-mid)/(high-low)))
    b = math.sqrt(max(0.,(mid-low)/(high-low)))
    view = np.array([(center[0]-320)/f,(center[1]-240)/f,1.])
    world_rot = np.array(obs["camera_T_world_optical"])[:3,:3]
    result = []
    for sign in (-1,1):
        normal = a*vectors[:,2]+sign*b*vectors[:,0]
        if normal @ view < 0:
            normal = -normal
        result.append(world_rot @ normal)
    return result


def triangulate(root: Path, first: dict, current: dict, first_bbox: list, bbox: list) -> dict:
    ellipses = [ellipse_from_rgb(root/o["rgb"], b) for o,b in ((first, first_bbox), (current, bbox))]
    pixels = [e[0] for e in ellipses]
    origins, rays = [], []
    for obs, pixel in zip((first,current), pixels):
        t = np.array(obs["camera_T_world_optical"])
        ray = t[:3,:3] @ np.array([(pixel[0]-320)/obs["focal_px"], (pixel[1]-240)/obs["focal_px"], 1.])
        origins.append(t[:3,3]); rays.append(ray/np.linalg.norm(ray))
    baseline = float(np.linalg.norm(origins[1]-origins[0]))
    if baseline < .005:
        raise ValueError("自己運動によるカメラbaselineが不足しています")
    distances = np.linalg.lstsq(np.column_stack((rays[0],-rays[1])), origins[1]-origins[0], rcond=None)[0]
    points = [origins[i]+rays[i]*distances[i] for i in (0,1)]
    gap = float(np.linalg.norm(points[0]-points[1]))
    if np.min(distances) < .1 or np.max(distances) > 1.5 or gap > .01:
        raise ValueError(f"RGB三角測量が不安定です: ray_gap={gap}, distances={distances}")
    face = (points[0]+points[1])/2
    candidates = [circle_normal_candidates(o,e) for o,e in zip((first,current),ellipses)]
    n0,n1 = max(((n0,n1) for n0 in candidates[0] for n1 in candidates[1]),key=lambda p:float(p[0]@p[1]))
    consistency = math.degrees(math.acos(float(np.clip(n0@n1,-1,1))))
    if consistency > 8:
        raise ValueError("2つのRGBによる面法線が一致しません")
    approach = n0+n1
    approach /= np.linalg.norm(approach)
    return {"face_xyz_from_rgb_parallax": face.tolist(), "approach_direction": approach.tolist(),
            "rgb_centers": [p.tolist() for p in pixels], "camera_baseline_m": baseline, "ray_gap_m": gap,
            "source_images": [first["rgb_sha256"],current["rgb_sha256"]], "depth_used": False,
            "normal_source": "Two calibrated RGB ellipse conics; circular-button assumption, no known radius",
            "normal_consistency_deg": consistency}


def act(root: Path, path: Path) -> None:
    config, model, data, stand = load(root)
    obs = json.loads((root/"observation.json").read_text(encoding="utf-8"))
    decision = json.loads(path.read_text(encoding="utf-8"))
    if decision["observation_sha256"] != digest(root/obs["rgb"]):
        raise ValueError("判断とRGB画像が一致しません")
    metrics = json.loads((root/"metrics.json").read_text(encoding="utf-8"))
    if metrics["actions"] >= 8:
        raise ValueError("有限試行の8行動上限に達しました")
    kind = decision["action"]
    qadr, dadr, actuator = arm_addresses(model)
    ik_info = None
    if kind == "yaw":
        yaw = math.radians(float(decision["yaw_deg"]))
        if not -.9 <= yaw <= .9:
            raise ValueError("yaw範囲を超えています")
        adr = model.joint("waist_yaw_joint").qposadr[0]
        ids, current, goal, duration = [model.actuator("waist_yaw_joint").id], [data.qpos[adr]], [yaw], 2.
    else:
        if kind == "triangulate_approach":
            first = json.loads((root/f"obs_{decision['first_index']:02d}.json").read_text(encoding="utf-8"))
            second = obs if "second_index" not in decision else json.loads((root/f"obs_{decision['second_index']:02d}.json").read_text(encoding="utf-8"))
            target = triangulate(root, first, second, decision["first_bbox"], decision["bbox"])
            write_json(root/"rgb_target.json", target)
            point = np.array(target["face_xyz_from_rgb_parallax"])-.070*np.array(target["approach_direction"])
            duration = 4.
        elif kind in ("press", "retract"):
            target = json.loads((root/"rgb_target.json").read_text(encoding="utf-8"))
            # contact判定値は制御には渡さない。少量の見た目補正だけを許す。
            correction = float(decision.get("offset_m", 0.))
            if not -.020 <= correction <= .020:
                raise ValueError("視覚補正は20mm以内です")
            spacing = (-.014+correction) if kind == "press" else -.070
            point = np.array(target["face_xyz_from_rgb_parallax"])+spacing*np.array(target["approach_direction"])
            duration = 3.
        elif kind == "delta":
            delta = np.array(decision["delta_optical_m"], dtype=float)
            if np.linalg.norm(delta) > .040:
                raise ValueError("RGB補正は40mm以内です")
            point = data.site("button_tcp").xpos + np.array(obs["camera_T_world_optical"])[:3,:3] @ delta
            duration = 2.
        else:
            raise ValueError("未知の行動です")
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        # 腕を下げた初期手先姿勢は高い目標へ届かないため、RGB視差から得た接近方向を使う。
        if kind in ("triangulate_approach", "press", "retract"):
            axis = np.array(target["approach_direction"])
        else:
            axis = data.site("button_tcp").xmat.reshape(3,3)[:,0].copy()
        solution = solve_ik(model, scratch, point, data.qpos[qadr].copy(), stand[actuator], qadr, dadr,
                            desired_axis=axis, position_tolerance_m=.0001)
        ids, current, goal = actuator, data.qpos[qadr].copy(), solution.joint_angles
        ik_info = {"goal_xyz_from_rgb_or_self_state": point.tolist(), "error_m": solution.position_error_m}
    up, down = [model.joint(f"button_slide_{d}").qposadr[0] for d in ("up","down")]
    tip, face_id = model.geom("button_tip").id, model.geom("button_face_up").id
    renderer = mujoco.Renderer(model, height=480, width=640)
    next_frame = data.time
    try:
        for q in interpolate(tuple(current), tuple(goal), duration, hz=1/model.opt.timestep):
            data.ctrl[ids] = q
            mujoco.mj_step(model, data)
            # evaluatorだけが環境の押下量/geom IDを読む。目標生成は上のRGB/FKだけ。
            metrics["max_stroke_m"] = max(metrics["max_stroke_m"], float(data.qpos[up]))
            metrics["max_other_stroke_m"] = max(metrics["max_other_stroke_m"], float(data.qpos[down]))
            for idx in range(data.ncon):
                contact = data.contact[idx]
                if {int(contact.geom1),int(contact.geom2)} == {tip,face_id}:
                    metrics["contact_steps"] += 1
                    if data.qpos[up] >= .00135:
                        metrics["contact_at_threshold_steps"] += 1
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model,data,idx,force)
                    metrics["max_contact_force_n"] = max(metrics["max_contact_force_n"],float(abs(force[0])))
            if physical_success(metrics):
                model.geom_rgba[face_id] = (.95,.81,.49,1.)
            if data.time >= next_frame:
                renderer.update_scene(data,camera="button_rgbd")
                frame = Image.fromarray(renderer.render().copy())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0,0,640,26),fill=(10,10,10))
                draw.text((8,7),f"Head RGB only | {kind} | t={data.time:.2f}s",fill="white")
                frame.save(root/"frames"/f"{metrics['frames']:05d}.png")
                metrics["frames"] += 1
                next_frame += .1
    finally:
        renderer.close()
    metrics["actions"] += 1
    metrics["success"] = physical_success(metrics)
    write_json(root/"metrics.json",metrics)
    save_state(root,model,data)
    with (root/"actions.jsonl").open("a",encoding="utf-8") as stream:
        stream.write(json.dumps({"decision":decision,"ik":ik_info,"time_s":data.time})+"\n")
    observe(root,model,data,metrics["actions"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command",choices=("init","act","evaluate","video"))
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--model")
    parser.add_argument("--camera-config",type=Path)
    parser.add_argument("--decision",type=Path)
    args = parser.parse_args()
    if args.command == "init":
        init(args.out,args.model,args.camera_config)
    elif args.command == "act":
        act(args.out,args.decision)
    elif args.command == "evaluate":
        print((args.out/"metrics.json").read_text(encoding="utf-8"))
    else:
        from run_vlm_closed_loop import video
        video(args.out)


if __name__ == "__main__":
    main()
