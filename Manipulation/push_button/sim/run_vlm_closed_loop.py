#!/usr/bin/env python3
"""画像を読むCodexセッションと、通信を使わないMuJoCo腕制御を接続する。"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from kinematics import arm_addresses, solve_ik
from trajectory import interpolate
from vision import CameraIntrinsics, estimate_button_target, validate_reachable_target
from run_mujoco import build_model, DEFAULT_TIP_OFFSET, TIP_RADIUS

WIDTH, HEIGHT, STROKE = 640, 480, 0.0015


def write_json(path: Path, value: dict) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def physical_success(metrics: dict) -> bool:
    """独立判定。画像の色やモデルの成功宣言を入力にしない。"""
    return bool(.9*STROKE <= metrics["max_stroke_m"] <= 1.1*STROKE
                and metrics["max_other_stroke_m"] < .1*STROKE
                and metrics["contact_at_threshold_steps"] >= 50)


def make_model(config: dict) -> tuple[mujoco.MjModel, np.ndarray]:
    # カメラ校正は全シーン共通。対象位置に追従するtargetbodyカメラを固定化する。
    path = Path(config["model"])
    calibration, stand = build_model(path, .36, -.25, 1., STROKE,
                                    DEFAULT_TIP_OFFSET, True, rgbd_camera=True)
    cal_data = mujoco.MjData(calibration)
    cal_data.qpos[:29], cal_data.ctrl[:] = stand, stand
    mujoco.mj_forward(calibration, cal_data)
    model, stand = build_model(path, .36, -.25, 1., STROKE, DEFAULT_TIP_OFFSET,
                              True, rgbd_camera=True, panel_offset=tuple(config["offset"]),
                              elevator_front=True)
    for name in ("button_rgbd", "demo_camera"):
        cid, ref = model.camera(name).id, calibration.camera(name).id
        model.cam_mode[cid] = int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
        model.cam_pos[cid] = cal_data.cam_xpos[ref]
        mujoco.mju_mat2Quat(model.cam_quat[cid], cal_data.cam_xmat[ref])
    return model, stand


def save_state(root: Path, model: mujoco.MjModel, data: mujoco.MjData) -> None:
    state = np.empty(mujoco.mj_stateSize(model, mujoco.mjtState.mjSTATE_INTEGRATION))
    mujoco.mj_getState(model, data, state, mujoco.mjtState.mjSTATE_INTEGRATION)
    np.save(root / "state.npy", state)


def load(root: Path) -> tuple[dict, mujoco.MjModel, mujoco.MjData, np.ndarray]:
    config = json.loads((root / "scene.json").read_text(encoding="utf-8"))
    model, stand = make_model(config)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model, data, np.load(root / "state.npy"),
                       mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model, data)
    metrics = json.loads((root / "metrics.json").read_text(encoding="utf-8"))
    if metrics.get("success"):
        model.geom_rgba[model.geom("button_face_up").id] = (.95, .81, .49, 1.)
    return config, model, data, stand


def observation(root: Path, model: mujoco.MjModel, data: mujoco.MjData, index: int) -> dict:
    name = f"obs_{index:02d}"
    renderer = mujoco.Renderer(model, height=HEIGHT, width=WIDTH)
    try:
        renderer.update_scene(data, camera="button_rgbd")
        Image.fromarray(renderer.render().copy()).save(root / f"{name}.png")
        renderer.enable_depth_rendering()
        depth = renderer.render().copy()
    finally:
        renderer.close()
    cid = model.camera("button_rgbd").id
    transform = np.eye(4)
    transform[:3, :3] = data.cam_xmat[cid].reshape(3, 3) @ np.diag([1, -1, -1])
    transform[:3, 3] = data.cam_xpos[cid]
    focal = HEIGHT / (2 * math.tan(math.radians(model.cam_fovy[cid]) / 2))
    np.savez(root / f"{name}_rgbd.npz", depth=depth, transform=transform, focal=focal)
    obs = {"index": index, "rgb": f"{name}.png", "rgb_sha256": digest(root / f"{name}.png"),
           "time_s": data.time, "tcp_self_state": data.site("button_tcp").xpos.tolist(),
           "instruction": "Press the UP arrow button; select its face bbox in this RGB image.",
           "policy_inputs": ["RGB image", "registered depth", "camera calibration", "robot self state"],
           "no_segmentation_or_target_ground_truth": True}
    write_json(root / "observation.json", obs)
    print(json.dumps(obs))
    return obs


def initial(root: Path, model_path: str, offset: list[float]) -> None:
    if (root / "scene.json").exists():
        raise ValueError("既存試験を上書きできません。別の出力フォルダを指定してください")
    root.mkdir(parents=True, exist_ok=True)
    (root / "frames").mkdir()
    config = {"model": str(Path(model_path).resolve()), "offset": offset,
              "stroke_m": STROKE, "base_fixed": True, "simulation_only": True,
              "model_xml_sha256": digest(Path(model_path)), "mujoco_version": mujoco.__version__}
    write_json(root / "scene.json", config)
    model, stand = make_model(config)
    data = mujoco.MjData(model)
    data.qpos[:29], data.ctrl[:] = stand, stand
    mujoco.mj_forward(model, data)
    save_state(root, model, data)
    write_json(root / "metrics.json", {"max_stroke_m": 0., "max_other_stroke_m": 0.,
                                      "contact_steps": 0, "contact_at_threshold_steps": 0,
                                      "max_contact_force_n": 0., "actions": 0, "frames": 0})
    observation(root, model, data, 0)


def action(root: Path, decision_path: Path) -> None:
    config, model, data, stand = load(root)
    obs = json.loads((root / "observation.json").read_text(encoding="utf-8"))
    decision = json.loads(decision_path.read_text(encoding="utf-8"))
    if decision["observation_sha256"] != digest(root / obs["rgb"]):
        raise ValueError("判断と観測画像のhashが一致しません")
    if decision["observation_sha256"] != obs["rgb_sha256"]:
        raise ValueError("保存済み観測画像が変更されています")
    phase = decision["phase"]
    if phase not in ("approach", "contact", "press", "retract", "home"):
        raise ValueError("未知の行動です")
    bbox = tuple(decision.get("bbox", []))
    if len(bbox) == 4:
        if not (0 <= bbox[0] < bbox[2] <= WIDTH and 0 <= bbox[1] < bbox[3] <= HEIGHT):
            raise ValueError("検出枠が画像範囲外です")
        image = np.load(root / f"obs_{obs['index']:02d}_rgbd.npz")
        intr = CameraIntrinsics(float(image["focal"]), float(image["focal"]), WIDTH/2, HEIGHT/2)
        target = estimate_button_target(image["depth"], bbox, intr, image["transform"])
        validate_reachable_target(target)
        perceived = {"face_xyz": list(target.face_xyz), "direction": list(target.press_direction),
                     "depth_valid_fraction": target.depth_valid_fraction,
                     "plane_inlier_fraction": target.plane_inlier_fraction,
                     "source_image_sha256": obs["rgb_sha256"]}
        write_json(root / "perceived_target.json", perceived)
    else:
        # 自己遮蔽時だけ、以前の画像から計算した目標を保持する。正解座標は読まない。
        if not decision.get("occluded_target_memory") and phase != "home":
            raise ValueError("bboxまたは明示的な遮蔽追跡判断が必要です")
        perceived = json.loads((root / "perceived_target.json").read_text(encoding="utf-8"))
    face, direction = np.array(perceived["face_xyz"]), np.array(perceived["direction"])
    distances = {"approach": -.050, "contact": -.003, "press": STROKE,
                 "retract": -.050}
    extension = float(decision.get("press_extension_m", 0.))
    if not 0 <= extension <= .002 or (extension and phase != "press"):
        raise ValueError("追加押込みはpress時だけ、0〜2mmに制限します")
    distances["press"] += extension
    qadr, dadr, actuator = arm_addresses(model)
    if phase == "home":
        goal, ik_log = stand[actuator], None
    else:
        target_xyz = face + (distances[phase] - TIP_RADIUS) * direction
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        solution = solve_ik(model, scratch, target_xyz, data.qpos[qadr].copy(),
                            stand[actuator], qadr, dadr, desired_axis=direction,
                            position_tolerance_m=.00003)
        goal = solution.joint_angles
        ik_log = {"target_xyz": target_xyz.tolist(), "error_m": solution.position_error_m,
                  "axis_error_deg": solution.axis_error_deg}
    metrics = json.loads((root / "metrics.json").read_text(encoding="utf-8"))
    up = model.joint("button_slide_up").qposadr[0]
    down = model.joint("button_slide_down").qposadr[0]
    tip, face_geom = model.geom("button_tip").id, model.geom("button_face_up").id
    duration = {"approach": 4., "contact": 2., "press": 2., "retract": 2., "home": 4.}[phase]
    renderer = mujoco.Renderer(model, height=480, width=640)
    next_frame = data.time
    try:
        for q in interpolate(tuple(data.qpos[qadr]), tuple(goal), duration,
                             hz=1/model.opt.timestep):
            data.ctrl[actuator] = q
            mujoco.mj_step(model, data)
            metrics["max_stroke_m"] = max(metrics["max_stroke_m"], float(data.qpos[up]))
            metrics["max_other_stroke_m"] = max(metrics["max_other_stroke_m"], float(data.qpos[down]))
            for idx in range(data.ncon):
                contact = data.contact[idx]
                if {int(contact.geom1), int(contact.geom2)} == {tip, face_geom}:
                    metrics["contact_steps"] += 1
                    if data.qpos[up] >= .9*STROKE:
                        metrics["contact_at_threshold_steps"] += 1
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model, data, idx, force)
                    metrics["max_contact_force_n"] = max(metrics["max_contact_force_n"], float(abs(force[0])))
            if metrics["contact_at_threshold_steps"] >= 50:
                model.geom_rgba[face_geom] = (.95, .81, .49, 1.)
            if data.time >= next_frame:
                renderer.update_scene(data, camera="demo_camera")
                frame = Image.fromarray(renderer.render().copy())
                draw = ImageDraw.Draw(frame)
                draw.rectangle((0, 0, 640, 28), fill=(10, 10, 10))
                draw.text((8, 7), f"MuJoCo fixed-base | {phase} t={data.time:.2f}s | travel={data.qpos[up]*1000:.2f}mm", fill="white")
                frame.save(root / "frames" / f"{metrics['frames']:05d}.png")
                metrics["frames"] += 1
                next_frame += .1
    finally:
        renderer.close()
    metrics["actions"] += 1
    metrics["success"] = physical_success(metrics)
    write_json(root / "metrics.json", metrics)
    save_state(root, model, data)
    with (root / "actions.jsonl").open("a", encoding="utf-8") as stream:
        stream.write(json.dumps({"decision": decision, "perceived_target": perceived,
                                 "ik": ik_log, "time_s": data.time, "metrics": metrics}) + "\n")
    observation(root, model, data, metrics["actions"])
    print(json.dumps({"phase": phase, "metrics": metrics}))


def video(root: Path) -> None:
    import imageio.v2 as imageio
    with imageio.get_writer(root / "simulation.mp4", fps=10, codec="libx264", quality=8) as writer:
        for path in sorted((root / "frames").glob("*.png")):
            writer.append_data(imageio.imread(path))
    print(root / "simulation.mp4")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("init", "act", "video"))
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--model")
    parser.add_argument("--offset", nargs=3, type=float, default=[0., 0., 0.])
    parser.add_argument("--decision", type=Path)
    args = parser.parse_args()
    if args.command == "init":
        initial(args.out, args.model, args.offset)
    elif args.command == "act":
        action(args.out, args.decision)
    else:
        video(args.out)


if __name__ == "__main__":
    main()
