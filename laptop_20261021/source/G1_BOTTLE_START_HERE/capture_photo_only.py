"""One camera RGB only. No body state, motor, mode, depth, services or config edits."""
import subprocess,datetime
from pathlib import Path
root=Path(__file__).resolve().parent.parent
stamp=datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
remote="/tmp/g1_photo_"+stamp
dest=root/"bottle_camera_discovery"/("photo_only_"+stamp);dest.mkdir(exist_ok=False)
CODE='import sys,time,json\nfrom pathlib import Path\nimport cv2,numpy as np\nsys.path.insert(0,"/home/unitree/button_press")\nfrom unitree_sdk2py.core.channel import ChannelFactoryInitialize\nfrom unitree_sdk2py.go2.video.video_client import VideoClient\nout=Path("__REMOTE_DEST__");out.mkdir(exist_ok=False)\nChannelFactoryInitialize(0,"eth0")\nc=VideoClient();c.SetTimeout(2.);c.Init()\nrequested=time.time();status,data=c.GetImageSample();received=time.time()\nif status!=0:raise RuntimeError("Camera RGB request failed code="+str(status))\nraw=bytes(data);img=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)\nif img is None:raise RuntimeError("Camera response was not a decodable RGB image")\n(out/"photo.jpg").write_bytes(raw)\nm=dict(requested_unix=requested,received_unix=received,rgb_exposure_unix=None,shape=list(img.shape),source="G1-local official videohub single RGB request",motor_commands=0,body_state_reads=0)\n(out/"metadata.json").write_text(json.dumps(m,indent=2));print(json.dumps(m),flush=True)\n'
r=subprocess.run(["ssh","-T","-o","StrictHostKeyChecking=yes","-o","ConnectTimeout=5","<ROBOT_USER>@<PRIVATE_IP>","python3","-u","-"],input=CODE.replace("__REMOTE_DEST__",remote),text=True)
if r.returncode:raise SystemExit(r.returncode)
r=subprocess.run(["scp","-o","StrictHostKeyChecking=yes","<ROBOT_USER>@<PRIVATE_IP>:"+remote+"/photo.jpg","<ROBOT_USER>@<PRIVATE_IP>:"+remote+"/metadata.json",str(dest)])
if r.returncode:raise SystemExit(r.returncode)
print("PHOTO_SAVED",dest,flush=True)
