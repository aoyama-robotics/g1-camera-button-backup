"""Read existing5556 only. No motor API, installs, service or settings change."""
import json,time,zipfile,sys
from pathlib import Path
import zmq,cv2,numpy as np
sys.path.insert(0,'/home/unitree/button_press')
from common.rgbd_protocol import decode_rgbd
out=Path('__REMOTE_DEST__');out.mkdir(exist_ok=False)
context=zmq.Context();sub=context.socket(zmq.SUB);sub.setsockopt(zmq.SUBSCRIBE,b'');sub.setsockopt(zmq.CONFLATE,1);sub.connect('tcp://127.0.0.1:5556')
try:
 if not sub.poll(5000):raise RuntimeError('既存5556配信を5秒以内に受信できません。サービス変更はしていません')
 f=decode_rgbd(sub.recv());cv2.imwrite(str(out/'rgb.jpg'),f.color_bgr);np.save(out/'depth.npy',f.depth)
 m=dict(received_unix=time.time(),server_timestamp=f.timestamp,frame_id=f.frame_id,intrinsics=vars(f.intrinsics),depth_scale=f.depth_scale,rgb_exposure_unix=None,alignment='existing server spatial depth-to-color mapping; sensor exposure synchronization not proven')
 (out/'metadata.json').write_text(json.dumps(m,indent=2))
 with zipfile.ZipFile(out/'capture.zip','w',zipfile.ZIP_DEFLATED) as z:
  for n in ['rgb.jpg','depth.npy','metadata.json']:z.write(out/n,n)
 print(json.dumps(m),flush=True)
finally:sub.close();context.term()
