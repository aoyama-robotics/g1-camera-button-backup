#!/usr/bin/env python3
"""仮想側頭部RGBで対象導入し、下向き頭部RGB-Dで腕を動かす。"""
from __future__ import annotations
import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import sys
import mujoco
import numpy as np
from PIL import Image,ImageDraw
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from camera_handoff import CameraFrame,HandoffPolicy,coarse_yaw
from vision import CameraIntrinsics,ButtonTarget
from kinematics import arm_addresses,solve_ik,ArmKinematics
from trajectory import interpolate
from run_vlm_closed_loop import write_json,digest,save_state,physical_success
from run_rgb_only import model_for as head_model


def model_for(config: dict) -> tuple[mujoco.MjModel,np.ndarray]:
    model,stand = head_model(config)
    side = config["side_camera"]
    p,y = side["pitch_rad"],side["yaw_rad"]
    ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    rz = np.array([[math.cos(y),-math.sin(y),0],[math.sin(y),math.cos(y),0],[0,0,1]])
    basis = np.array([[0,0,-1],[-1,0,0],[0,1,0]])
    cid = model.camera("demo_camera").id
    model.cam_bodyid[cid] = model.body("torso_link").id
    model.cam_mode[cid] = int(mujoco.mjtCamLight.mjCAMLIGHT_FIXED)
    model.cam_pos[cid] = side["xyz"]
    mujoco.mju_mat2Quat(model.cam_quat[cid],(rz@ry@basis).ravel())
    model.cam_fovy[cid] = side["fovy_deg"]
    return model,stand


def camera_pose(model: mujoco.MjModel,data: mujoco.MjData,name: str) -> tuple[np.ndarray,float]:
    cid = model.camera(name).id
    transform = np.eye(4)
    transform[:3,:3] = data.cam_xmat[cid].reshape(3,3)@np.diag([1,-1,-1])
    transform[:3,3] = data.cam_xpos[cid]
    f = 240/math.tan(math.radians(model.cam_fovy[cid])/2)
    return transform,f


def observe(root: Path,model: mujoco.MjModel,data: mujoco.MjData,index: int) -> dict:
    renderer = mujoco.Renderer(model,height=480,width=640)
    info = {"index":index,"time_s":data.time,"base_fixed":True,
            "calibration_assumption":"head_nominal_URDF; side_hypothetical_mount; zero_distortion",
            "depth_source":"registered_head_depth_only","target_ground_truth_in_policy":False}
    images = []
    try:
        for label,name in (("side","demo_camera"),("head","button_rgbd")):
            renderer.update_scene(data,camera=name)
            rgb = renderer.render().copy()
            image_path = root/f"obs_{index:02d}_{label}.png"
            Image.fromarray(rgb).save(image_path)
            transform,f = camera_pose(model,data,name)
            meta = {"rgb":image_path.name,"rgb_sha256":digest(image_path),"stamp":data.time,
                    "pose_stamp":data.time,"T_world_optical":transform.tolist(),"focal_px":f,
                    "calibration_id":f"sim_assumed_{label}_v1"}
            if label == "head":
                renderer.enable_depth_rendering()
                depth = renderer.render().copy()
                renderer.disable_depth_rendering()
                np.save(root/f"obs_{index:02d}_head_depth.npy",depth)
                meta["depth"] = f"obs_{index:02d}_head_depth.npy"
                meta["depth_stamp"] = data.time
            info[label] = meta
            images.append(rgb)
    finally:
        renderer.close()
    pair_path = root/f"obs_{index:02d}_pair.png"
    pair = Image.fromarray(np.concatenate(images,axis=1))
    draw = ImageDraw.Draw(pair)
    draw.rectangle((0,0,1280,24),fill=(8,8,8))
    draw.text((8,6),"SIDE RGB (hypothetical mount)",fill="white")
    draw.text((648,6),"HEAD RGB-D (nominal URDF mount)",fill="white")
    pair.save(pair_path)
    info["pair_rgb"],info["pair_sha256"] = pair_path.name,digest(pair_path)
    info["tcp_self_state_xyz"] = data.site("button_tcp").xpos.tolist()
    info["waist_yaw_self_state_rad"] = float(data.qpos[model.joint("waist_yaw_joint").qposadr[0]])
    torso = np.eye(4)
    torso[:3,:3],torso[:3,3] = data.body("torso_link").xmat.reshape(3,3),data.body("torso_link").xpos
    info["T_world_torso"] = torso.tolist()
    write_json(root/f"obs_{index:02d}.json",info)
    write_json(root/"observation.json",info)
    print(json.dumps({"index":index,"pair_rgb":pair_path.name,"pair_sha256":info["pair_sha256"],
                      "time_s":data.time,"tcp_self_state_xyz":info["tcp_self_state_xyz"]}))
    return info


def frame(root: Path,obs: dict,label: str,drop_depth: bool = False) -> CameraFrame:
    m = obs[label]
    depth = None if label=="side" else np.load(root/m["depth"])
    if drop_depth and depth is not None:
        depth = np.zeros_like(depth)
    f = m["focal_px"]
    return CameraFrame(np.asarray(Image.open(root/m["rgb"]).convert("RGB")),depth,
                       CameraIntrinsics(f,f,320,240),np.array(m["T_world_optical"]),m["stamp"],
                       m.get("depth_stamp"),m["pose_stamp"],m["calibration_id"])


def policy_load(root: Path) -> HandoffPolicy:
    p = HandoffPolicy()
    s = json.loads((root/"policy_state.json").read_text(encoding="utf-8"))
    for key in ("state","failures","last_bbox","last_stamp"):
        setattr(p,key,s[key])
    if s["last_target"] is not None:
        p.last_target = ButtonTarget(**s["last_target"])
    return p


def policy_save(root: Path,p: HandoffPolicy) -> None:
    write_json(root/"policy_state.json",{"state":p.state,"failures":p.failures,
               "last_target":asdict(p.last_target) if p.last_target else None,
               "last_bbox":p.last_bbox,"last_stamp":p.last_stamp})


def validate_robot_path(model: mujoco.MjModel,data: mujoco.MjData,goal: np.ndarray,
                        face: np.ndarray,normal: np.ndarray,press: bool) -> None:
    # 自己衝突モデルと画像から計測した面だけを使う。simの正解環境geomは予測に使わない。
    robot_bodies = {model.body("pelvis").id}
    for body in range(model.nbody):
        if int(model.body_parentid[body]) in robot_bodies:
            robot_bodies.add(body)
    env = np.array([g for g in range(model.ngeom) if int(model.geom_bodyid[g]) not in robot_bodies])
    types,affinity = model.geom_contype[env].copy(),model.geom_conaffinity[env].copy()
    try:
        model.geom_contype[env],model.geom_conaffinity[env] = 0,0
        kin = ArmKinematics(model)
        kin.update(data.qpos.copy())
        kin.validate_path(goal,face,normal,allow_tip_contact=press)
    finally:
        model.geom_contype[env],model.geom_conaffinity[env] = types,affinity


def init(root: Path,model_path: str,camera_config: Path,start_yaw_deg: float) -> None:
    if root.exists():
        raise ValueError("既存結果を上書きしません")
    root.mkdir(parents=True)
    (root/"frames").mkdir()
    config = {"model":str(Path(model_path).resolve()),
              "head_camera":yaml.safe_load(camera_config.read_text(encoding="utf-8"))["head_camera"],
              "side_camera":{"xyz":[.060,.100,.430],"pitch_rad":math.radians(15),
                             "yaw_rad":math.radians(-10),"fovy_deg":80.,
                             "hypothetical_mount":True,"real_extrinsics_known":False}}
    write_json(root/"scene.json",config)
    model,stand = model_for(config)
    data = mujoco.MjData(model)
    data.qpos[:29],data.ctrl[:] = stand,stand
    yaw_adr = model.joint("waist_yaw_joint").qposadr[0]
    data.qpos[yaw_adr] = math.radians(start_yaw_deg)
    data.ctrl[model.actuator("waist_yaw_joint").id] = math.radians(start_yaw_deg)
    mujoco.mj_forward(model,data)
    save_state(root,model,data)
    write_json(root/"metrics.json",{"max_stroke_m":0.,"max_other_stroke_m":0.,"contact_steps":0,
                                    "contact_at_threshold_steps":0,"max_contact_force_n":0.,"actions":0,"frames":0})
    policy_save(root,HandoffPolicy())
    observe(root,model,data,0)


def act(root: Path,decision_path: Path) -> None:
    config = json.loads((root/"scene.json").read_text(encoding="utf-8"))
    model,stand = model_for(config)
    data = mujoco.MjData(model)
    mujoco.mj_setState(model,data,np.load(root/"state.npy"),mujoco.mjtState.mjSTATE_INTEGRATION)
    mujoco.mj_forward(model,data)
    obs = json.loads((root/"observation.json").read_text(encoding="utf-8"))
    decision = json.loads(decision_path.read_text(encoding="utf-8"))
    if decision["observation_sha256"] != digest(root/obs["pair_rgb"]):
        raise ValueError("判断と観測画像が一致しません")
    p = policy_load(root)
    metrics = json.loads((root/"metrics.json").read_text(encoding="utf-8"))
    if p.state=="STOP" or metrics["actions"]>=8:
        raise ValueError("停止状態または8行動上限です")
    side,head = frame(root,obs,"side"),frame(root,obs,"head",decision.get("inject_depth_dropout",False))
    kind = decision["action"]
    geometry = None
    if kind=="align_head":
        if p.last_target is not None:
            geometry = p.hold("waist_scan_requires_retracted_arm")
            policy_save(root,p)
            print(json.dumps(geometry))
            return
        yaw = coarse_yaw(side,decision["side_bbox"],np.array(obs["T_world_torso"]),
                         obs["waist_yaw_self_state_rad"],data.time)
        ids = [model.actuator("waist_yaw_joint").id]
        current = [data.qpos[model.joint("waist_yaw_joint").qposadr[0]]]
        goal,duration = [yaw],2.
        p.state = "ALIGN_HEAD"
    else:
        if kind in ("approach","press"):
            bbox = decision.get("head_bbox")
            if decision.get("track_previous_bbox") and p.last_target is not None:
                t = head.T_world_optical
                optical = t[:3,:3].T@(np.array(p.last_target.face_xyz)-t[:3,3])
                k = head.intrinsics
                center = np.array([k.cx+k.fx*optical[0]/optical[2],k.cy+k.fy*optical[1]/optical[2]])
                old = p.last_bbox
                half = np.array([old[2]-old[0],old[3]-old[1]])/2
                bbox = [*(center-half),*(center+half)]
            geometry = p.localize(side,head,bbox,data.time,decision["label"],
                                  allow_partial=decision.get("track_previous_bbox",False))
            if not geometry["arm_motion_allowed"]:
                policy_save(root,p)
                with (root/"events.jsonl").open("a",encoding="utf-8") as stream:
                    stream.write(json.dumps({"decision":decision,"geometry":geometry,"motion_steps":0})+"\n")
                print(json.dumps(geometry))
                return
            target = p.last_target
            spacing = -.065 if kind=="approach" else -.012
            point = np.array(target.face_xyz)+spacing*np.array(target.press_direction)
            duration = 4. if kind=="approach" else 3.
        elif kind=="retract" and p.last_target is not None:
            point = data.site("button_tcp").xpos-.050*np.array(p.last_target.press_direction)
            target,duration = p.last_target,2.
        else:
            raise ValueError("対象が未計測または行動が不正です")
        if np.linalg.norm(point-data.body("right_shoulder_pitch_link").xpos)>.65:
            raise ValueError("腕の許容reachを超えています")
        qadr,dadr,ids = arm_addresses(model)
        scratch = mujoco.MjData(model)
        scratch.qpos[:] = data.qpos
        # 現姿勢seedで局所解に詰まる場合だけ、関節空間の肘曲げseedを1回試す。
        # seedは対象の正解位置を含まない。実際の軌道は現在姿勢から検証する。
        seeds = [data.qpos[qadr].copy(),np.array([-.8,-.35,0.,1.4,0.,-.2,0.])]
        errors = []
        for seed_index,seed in enumerate(seeds):
            scratch.qpos[:] = data.qpos
            try:
                solution = solve_ik(model,scratch,point,seed,stand[ids],qadr,dadr,
                                    desired_axis=np.array(target.press_direction),position_tolerance_m=.00005)
                break
            except ValueError as error:
                errors.append(str(error))
        else:
            p.state = "STOP"
            policy_save(root,p)
            write_json(root/"ik_stop.json",{"errors":errors,"motion_steps":0})
            raise ValueError("2つのIK seedで届かず、安全停止しました")
        current,goal = data.qpos[qadr].copy(),solution.joint_angles
        if kind!="retract":
            try:
                validate_robot_path(model,data,goal,np.array(target.face_xyz),
                                    np.array(target.press_direction),kind=="press")
            except ValueError as error:
                p.state = "STOP"
                policy_save(root,p)
                write_json(root/"path_stop.json",{"reason":str(error),"motion_steps":0})
                raise
        geometry = {**(geometry or {}),"ik_error_m":solution.position_error_m,
                    "goal_xyz":point.tolist(),"min_joint_margin_rad":solution.min_joint_margin_rad,
                    "ik_seed_index":seed_index,"earlier_ik_errors":errors}
    up,down = [model.joint(f"button_slide_{d}").qposadr[0] for d in ("up","down")]
    tip,face_id = model.geom("button_tip").id,model.geom("button_face_up").id
    renderer = mujoco.Renderer(model,height=480,width=640)
    next_frame = data.time
    try:
        for q in interpolate(tuple(current),tuple(goal),duration,hz=1/model.opt.timestep):
            data.ctrl[ids] = q
            mujoco.mj_step(model,data)
            metrics["max_stroke_m"] = max(metrics["max_stroke_m"],float(data.qpos[up]))
            metrics["max_other_stroke_m"] = max(metrics["max_other_stroke_m"],float(data.qpos[down]))
            for idx in range(data.ncon):
                contact = data.contact[idx]
                if {int(contact.geom1),int(contact.geom2)}=={tip,face_id}:
                    force = np.zeros(6)
                    mujoco.mj_contactForce(model,data,idx,force)
                    metrics["max_contact_force_n"] = max(metrics["max_contact_force_n"],float(abs(force[0])))
                    metrics["contact_steps"] += 1
                    if data.qpos[up]>=.00135:
                        metrics["contact_at_threshold_steps"] += 1
                    if abs(force[0])>10:
                        p.state = "STOP"
                        metrics["safety_stop_reason"] = "tip_contact_force_over_10N"
            if p.state=="STOP":
                # このsimulation-only実行では実測関節位置へ保持目標を戻して終了する。
                addresses = [model.joint(int(model.actuator_trnid[i,0])).qposadr[0] for i in ids]
                data.ctrl[ids] = data.qpos[addresses]
                break
            if physical_success(metrics):
                model.geom_rgba[face_id] = (.95,.81,.49,1.)
            if data.time>=next_frame:
                parts = []
                for name in ("demo_camera","button_rgbd"):
                    renderer.update_scene(data,camera=name)
                    parts.append(renderer.render().copy())
                image = Image.fromarray(np.concatenate(parts,axis=1))
                draw = ImageDraw.Draw(image)
                draw.rectangle((0,0,1280,26),fill=(8,8,8))
                draw.text((8,7),f"SIDE RGB | {kind} t={data.time:.2f}s",fill="white")
                draw.text((648,7),"HEAD RGB-D | fixed base, assumed calibration",fill="white")
                image.save(root/"frames"/f"{metrics['frames']:05d}.png")
                metrics["frames"] += 1
                next_frame += .2
    finally:
        renderer.close()
    metrics["actions"] += 1
    if kind!="align_head":
        tracking_error = float(np.linalg.norm(data.site("button_tcp").xpos-point))
        geometry["achieved_tcp_error_m"] = tracking_error
        if tracking_error>.015:
            p.state = "STOP"
            geometry["safety_stop_reason"] = "tcp_tracking_error_over_15mm"
    metrics["safety_stop"] = p.state=="STOP"
    metrics["success"] = physical_success(metrics) and not metrics["safety_stop"]
    if kind=="retract" and p.state!="STOP":
        p.state = "SAFE_RETRACTED"
        p.last_target,p.last_bbox,p.last_stamp = None,None,None
    write_json(root/"metrics.json",metrics)
    save_state(root,model,data)
    policy_save(root,p)
    with (root/"events.jsonl").open("a",encoding="utf-8") as stream:
        stream.write(json.dumps({"decision":decision,"geometry":geometry,"time_s":data.time,
                                 "state":p.state})+"\n")
    observe(root,model,data,metrics["actions"])


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command",choices=("init","act","evaluate","video"))
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--model")
    parser.add_argument("--camera-config",type=Path)
    parser.add_argument("--start-yaw-deg",type=float,default=0.)
    parser.add_argument("--decision",type=Path)
    args = parser.parse_args()
    if args.command=="init":
        init(args.out,args.model,args.camera_config,args.start_yaw_deg)
    elif args.command=="act":
        act(args.out,args.decision)
    elif args.command=="evaluate":
        print((args.out/"metrics.json").read_text(encoding="utf-8"))
    else:
        import imageio.v2 as imageio
        with imageio.get_writer(args.out/"simulation.mp4",fps=5,codec="libx264",quality=7) as writer:
            for path in sorted((args.out/"frames").glob("*.png")):
                writer.append_data(imageio.imread(path))
        print(args.out/"simulation.mp4")


if __name__=="__main__":
    main()
