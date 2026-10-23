import json,os,time,threading,hashlib
from pathlib import Path
import cv2,numpy as np
import unitree_sdk2py.core.channel as ch
from unitree_sdk2py.go2.video.video_client import VideoClient
out=Path('bottle_camera_discovery/reboot_rgb_depth_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
threading.Timer(12,lambda:os._exit(2)).start();ch.ChannelConfigHasInterface=ch.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str((out/'dds.log').resolve()).replace('\\','/'));ch.ChannelFactoryInitialize(0,'<PRIVATE_IP>');v=VideoClient();v.SetTimeout(2);v.Init();rows=[]
for k in range(2):
 before=time.time();code,b=v.GetImageSample();after=time.time();assert code==0 and b
 raw=bytes(b);im=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR);assert im is not None;cv2.imwrite(str(out/('rgb_'+str(k)+'.jpg')),im);rows.append({'request_unix':before,'response_unix':after,'encoded_sha256':hashlib.sha256(raw).hexdigest(),'decoded_sha256':hashlib.sha256(im.tobytes()).hexdigest(),'shape':list(im.shape),'hardware_timestamp_available':False});time.sleep(.35)
r={'captures':rows,'distinct_encoded':rows[0]['encoded_sha256']!=rows[1]['encoded_sha256'],'distinct_decoded':rows[0]['decoded_sha256']!=rows[1]['decoded_sha256'],'motor_commands':0};(out/'rgb_freshness.json').write_text(json.dumps(r,indent=2));print(json.dumps({'path':str(out),**r}),flush=True);os._exit(0)
