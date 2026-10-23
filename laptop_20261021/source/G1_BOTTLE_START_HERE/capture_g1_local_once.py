"""Explicit finite G1-local camera capture, no motor API or service changes."""
import subprocess,datetime,zipfile
from pathlib import Path
root=Path(__file__).resolve().parent.parent
stamp=datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
remote="/tmp/g1_local_capture_"+stamp
dest=root/"bottle_camera_discovery"/("g1_local_"+stamp);dest.mkdir(exist_ok=False)
CODE='import json,time,zipfile,sys,dataclasses\nfrom pathlib import Path\nimport cv2,numpy as np\nsys.path.insert(0,"/home/unitree/button_press")\nfrom common.config import load_config\nfrom real.depth_server.rgbd_server import VideohubCamera\nout=Path("__REMOTE_DEST__");out.mkdir(exist_ok=False)\ncfg=load_config("depth_server.yaml")["camera"];cam=VideohubCamera(cfg)\nstarted=False\ntry:\n cam.start();started=True\n frame=None\n for _ in range(3):\n  frame=cam.complete(cam.read())\n  if frame is not None:break\n if frame is None:raise RuntimeError("No RGB from current videohub response; no cached fallback")\n saved=time.time();cv2.imwrite(str(out/"rgb.jpg"),frame.color_rgb[:,:,::-1]);np.save(out/"depth.npy",frame.depth)\n meta=dict(received_unix=saved,server_timestamp=frame.timestamp,rgb_exposure_unix=None,intrinsics=dataclasses.asdict(cam.intrinsics),depth_scale=cam.depth_scale,acquisition="G1-local VideohubCamera direct depth+official RGB RPC; no streaming socket",alignment="factory depth-to-color spatial mapping; independent RGB/depth exposure",motor_commands=0)\n (out/"metadata.json").write_text(json.dumps(meta,indent=2))\n with zipfile.ZipFile(out/"capture.zip","w",zipfile.ZIP_DEFLATED) as z:\n  for n in ["rgb.jpg","depth.npy","metadata.json"]:z.write(out/n,n)\n print(json.dumps(meta),flush=True)\nfinally:\n if started:cam.stop()\n'
code=CODE.replace("__REMOTE_DEST__",remote)
r=subprocess.run(["ssh","-T","-o","StrictHostKeyChecking=yes","-o","ConnectTimeout=5","<ROBOT_USER>@<PRIVATE_IP>","python3","-u","-"],input=code,text=True)
if r.returncode:raise SystemExit(r.returncode)
r=subprocess.run(["scp","-o","StrictHostKeyChecking=yes","<ROBOT_USER>@<PRIVATE_IP>:"+remote+"/capture.zip",str(dest/"capture.zip")])
if r.returncode:raise SystemExit(r.returncode)
with zipfile.ZipFile(dest/"capture.zip") as z:
 if set(z.namelist())!={"rgb.jpg","depth.npy","metadata.json"}:raise RuntimeError("Unexpected files")
 z.extractall(dest)
print("LOCAL_CAPTURE_SAVED",dest,flush=True)
