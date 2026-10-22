# Two onboard cameras: RGB semantic search and head RGB-D manipulation

The physical simulation trial succeeded: UP travel **1.517 mm**, DOWN travel **0 mm**, contact at the success threshold for **198 steps / 396 ms cumulative**, maximum tip normal force **4.265 N**. Four visual decisions executed alignment, approach, press and retreat. The video is actual MuJoCo rendering, 59 frames, 5 fps, 1280 x 480 (11.8 seconds).

This is a fixed-base arm demonstration using the existing 15 mm spherical TCP contact proxy. It does not establish standing balance, whole-body control, real finger contact or hardware safety. No DDS, ROS, device communication or Isaac process was started. No new dependencies were installed for this continuation.

## What the model saw and did

MuJoCo onboard RGB PNG pair -> `functions.view_image` -> the parent-selected **gpt-6.1-sol/high Codex session** -> saved semantic action JSON -> deterministic RGB-D/IK controller -> MuJoCo PD steps -> new paired image. The model actually viewed `obs_00_pair.png` through `obs_03_pair.png`. `obs_04` is the final recorded observation. Decision JSONs bind each action to the SHA256 of its observed PNG and retain the visual reason. Fast was requested but no runtime switch was available; Fast activation is not claimed. This continuation did not use astra or an autonomous VLM API.

At the initial pose, the side RGB saw UP and the downward head camera did not. A calibrated side ray requested a bounded -20 degree waist turn. The controller then required a new head-local RGB bbox rather than copying the side bbox between images. Both images showed the upper arrow button after alignment. Head registered depth reconstructed the button face and panel normal. Approach kept 50 mm clearance from the sphere surface; fresh depth after approach refreshed the face/normal before press. The last image showed UP yellow and DOWN gray, prompting a 50 mm retreat. The yellow indicator is driven by simulator task state, so it is useful visual feedback, not an independent measurement of physical success.

Semantic identity across views was associated by the model viewing the pair. An automatic geometric cross-camera association/epipolar matcher is **not implemented**. No segmentation, target world coordinates or known button radius feed the perception target. Scene truth is confined to scene creation and physical scoring. Known camera FK and self joint state are controller inputs. The same Codex session authored/inspected the scene, so this is not a blinded model benchmark.

Specifically, the model chose the upper UP-arrow button in the side image, then visually selected the upper UP-arrow button in the newly acquired head image and confirmed it in both images. The controller checks the semantic string `up`, timing and head geometry quality; it does not independently prove cross-camera physical identity. This simple two-button scene therefore does not test ambiguous duplicate UP targets. The model selected a side bbox, not a fixed yaw angle. `coarse_yaw` converts its center through side intrinsics and camera/torso rotations, computes `atan2(y,x)`, and clips the requested increment to +/-20 degrees; this trial reached that -20 degree bound. Scene button coordinates do not determine the yaw.

Implemented transforms are the camera optical pose from the configured mount plus simulated robot FK, side-ray-to-torso rotation, head-depth-to-world position/normal and world-to-head reprojection of the previous depth-derived target. `inverse(T_world_head) * T_world_side` is documented as the camera relation but is not used to establish geometric identity between detections. Ground-truth button world coordinates are never passed to the policy. A **world-coordinate estimate reconstructed from measured head depth** is passed to IK and cached for the next ROI; this is an estimate, not an oracle position.

## Camera and coordinate contract

`CameraFrame` holds RGB, optional registered depth, intrinsics, `T_world_optical`, RGB/depth/FK timestamps and calibration identity. Optical axes are right/down/forward. The conversion is `p_world = T_world_head_optical * [z * K_head^-1 * (u,v,1), 1]`; normals rotate with the transform's rotation. `T_head_side = inverse(T_world_head) * T_world_side` defines the relation, but side RGB lacks depth, so a side bbox only supplies a bearing. It cannot alone specify a 3D point or exact head pixel.

| Camera | Simulation configuration | Status |
|---|---|---|
| Head RGB-D | torso xyz (0.0576235, 0.01753, 0.42987) m, pitch 47.6 degrees, 640 x 480, vertical FOV 42.6 degrees | Nominal repository/URDF mount; approximate pinhole intrinsics |
| Side RGB | torso xyz (0.060, 0.100, 0.430) m, pitch 15 degrees, yaw -10 degrees, vertical FOV 80 degrees | Explicit hypothetical mount; actual device/extrinsics unknown |

Both have zero simulated distortion. The head mount was found in `Dev/ButtonPress` commit `e297a4308531fa4f39c1a6afb94c8f5d3702214f`, `Button_Press/J1-gen/configs/robot.yaml`, and its xyz/rpy matched official unitree_ros URDF commit `ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee`, `d435_joint`. This does not verify the physical camera optical frame, factory RGB-depth alignment or real calibration. The saved YAML is `evidence/head_camera_config.yaml`.

Before a hardware adapter could use this contract, it would need actual per-stream intrinsics/distortion and resolution profile, depth scale/registration, RGB/depth optical-to-device transforms, calibrated torso-to-side camera and TCP extrinsics, plus camera/joint timestamps on one clock. Interpolate FK to exposure time. Calibrate mounting with a board observed at multiple known FK poses (hand-eye AX=XB as applicable), verify the board independently across both cameras and validate TCP position. Save serial/profile/calibration IDs and residual errors. None of this physical calibration was performed here.

Resource isolation: this continuation launched only short MuJoCo processes, not Isaac. No separate A-task process was stopped. At the initial check the GPU was 2% utilized with 1975/11264 MiB occupied, and other Python processes were present; zero GPU overlap with all background work cannot be proven. The final read-only check found GPU utilization 1%, memory 1870/11264 MiB, only `pythonw` PID2228 among Python/Isaac/Kit/ffmpeg names, and all our simulation/encoding command sessions had exited. Our renderers closed in `finally` blocks. Background memory is not claimed as released; it belongs to processes outside this task. Process command-line inspection was unavailable due to host access restrictions.

## Closed-loop gates

`SEARCH_SIDE -> ALIGN_HEAD -> LOCALIZED_HEAD_DEPTH -> approach -> reobserve -> fresh-depth press -> visual verification -> SAFE_RETRACTED`. Robot self-collision and the depth-derived panel plane gate the joint path. Predictive validation temporarily excludes environment geoms to avoid oracle obstacle geometry, then restores all geom flags before physical stepping. Other unknown environment obstacles are not reconstructed by this prototype.

Frames must be fresh within 100 ms, RGB/depth and RGB/FK within 15 ms, and camera pair within 30 ms. Registered depth must match RGB shape; transforms must be finite, orthonormal and proper; calibration IDs must exist. The real driver/calibration verification remains to be implemented: presence of an ID is not proof of its correctness.

Head out of view/missing bbox, stale/unsynchronized pose, identity mismatch and missing/invalid depth produce no arm movement: first `HOLD_REACQUIRE`, second latched `STOP`. A subsequent waist scan is rejected while an arm target remains active; retraction clears it. Initial side-only search bounds each yaw increment to 20 degrees, total yaw to +/-45 degrees. A real reacquisition adapter must request new synchronized frames and semantic confirmation; the file bridge does not autonomously operate a live camera.

Partial occlusion may project the last image-derived target to a head ROI for at most five seconds. Fresh depth remains mandatory with >=60% valid face support and the previous observed plane as a gate. Cached coordinates alone never authorize pressing. The actual partially occluded trial had 94.6% support. Reach is capped at 0.65 m, total motions at eight, final measured TCP tracking error at 15 mm and simulated tip contact force at 10 N. These are prototype simulation bounds, not hardware-qualified limits. IK tries measured joints first, then one bent-elbow seed; one original zero-step IK failure is retained in `failed_ik_attempt.json`.

## Evidence and verification

| Case | Outcome |
|---|---|
| Head initially out of view, side visible | Actual paired obs_00; arm stays still during waist alignment |
| Both cameras see target | Actual obs_01; head depth target then approach |
| Partial hand occlusion | Actual obs_02; fresh depth supports press, physical success |
| Full depth dropout after approach | Two explicit fault injections on copied measured state: HOLD then STOP, state SHA256 unchanged, zero additional steps |
| Final controller replay | Recorded semantic decisions replayed through final code; no new VLM calls; metrics in `evidence/dual_camera_replay/replay_result.json` |

The independent success predicate requires UP stroke 1.35..1.65 mm, DOWN <0.15 mm and at least 50 contact-at-threshold steps. Physical metrics were read after the image-driven action trial. The physical evaluator inspects slide state and tip/button contact separately from visual selection. The 32-test CPU suite covers dual-camera timestamp/depth/state gates plus the existing RGB, RGB-D, stroke and actual G1 IK tests. The geometry support threshold and stop/calibration guards were strengthened after the initial trial; saved depth recheck and final-code replay cover the final version. The earlier shifted-panel/frozen-target baseline is in `VLM_CLOSED_LOOP.md`; it is not a new two-camera variation benchmark. Broad appearance, mount variation and dynamic-scene robustness remain untested.

## Re-run from this workspace

Use the existing isolated `.venv`; official Menagerie G1 assets are pinned at `0059d4335f8156206f63a35662313385f7ad6d74`, repository base `561c2d735f3da96d2e86e826e96f8ab2e026f275`, local branch `local/vlm-button-sim`. No push/merge was made. Replaying decisions is not fresh VLM evaluation.

```powershell
$env:G1_MUJOCO_MODEL = (Resolve-Path menagerie\unitree_g1\g1.xml).Path
& .\.venv\Scripts\python.exe -m pytest G1_Hackason\Manipulation\push_button\tests\test_camera_handoff.py G1_Hackason\Manipulation\push_button\tests\test_rgb_only.py G1_Hackason\Manipulation\push_button\tests\test_vlm_closed_loop.py G1_Hackason\Manipulation\push_button\tests\test_vision.py G1_Hackason\Manipulation\push_button\tests\test_button_stroke.py G1_Hackason\Manipulation\push_button\tests\test_kinematics.py -q -p no:cacheprovider
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\replay_dual_camera.py --source evidence\dual_camera --out evidence\dual_replay_new --model menagerie\unitree_g1\g1.xml --camera-config evidence\head_camera_config.yaml
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\run_dual_camera.py video --out evidence\dual_replay_new
```

For a new actual model-driven run, use `run_dual_camera.py init` with the same model/config and a new output directory. View its paired PNG through the image tool, write a decision with its pair hash, then call `act --decision <json>`; repeat with each new observation. Coordinates for `head_bbox` and `side_bbox` are local 640 x 480 image coordinates. This intentionally requires a genuine new visual model decision; it cannot be replaced by asserting the saved replay is a VLM run. Dependencies remain those used by the existing simulator: MuJoCo 3.3.7, NumPy, Pillow, PyYAML, imageio/ffmpeg and pytest.
