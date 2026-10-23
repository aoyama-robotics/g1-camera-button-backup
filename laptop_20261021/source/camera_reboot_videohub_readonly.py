"""Single official videohub read. No motor APIs or direct camera access."""
import json,os,time,threading
from pathlib import Path
import numpy as np,cv2
import unitree_sdk2py.core.channel as ch
from unitree_sdk2py.go2.video.video_client import VideoClient

out=Path(__file__).resolve().parent/'bottle_camera_discovery'/'reboot_kanji_20261021';out.mkdir(parents=True,exist_ok=True)
def watchdog():time.sleep(12);print('VIDEО_READ_TIMEOUT',flush=True);os._exit(2)
threading.Thread(target=watchdog,daemon=True).start()
ch.ChannelConfigHasInterface=ch.ChannelConfigHasInterface.replace('name="$__IF_NAME__$"','address="$__IF_NAME__$"').replace('/tmp/cdds.LOG',str(out/'videohub_dds.log').replace('\\','/'))
ch.ChannelFactoryInitialize(0,'<PRIVATE_IP>')
client=VideoClient();client.SetTimeout(2.);client.Init()
version=client.GetServerApiVersion()
report={'server_api_version':version,'client_api_version':'1.0.0.1','motor_commands':0,'service_changes':0,'depth_received':False}
if version[0]==0 and isinstance(version[1],str):
    code,data=client.GetImageSample();report['image_status']=code
    if code==0 and data:
        raw=bytes(data);image=cv2.imdecode(np.frombuffer(raw,dtype=np.uint8),cv2.IMREAD_COLOR)
        if image is None:report['error']='Binary response did not decode as an image'
        else:
            path=out/('g1_rgb_'+time.strftime('%Y%m%d_%H%M%S')+'.jpg')
            if not cv2.imwrite(str(path),image):raise RuntimeError('Local image save failed')
            report.update(image_path=str(path),image_shape=list(image.shape),received_unix=time.time(),encoded_bytes=len(raw),calibration_for_this_image_verified=False)
else:report['error']='videohub protocol/version not confirmed; no image RPC attempted'
(out/'videohub_result.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
print(json.dumps(report),flush=True);os._exit(0)


