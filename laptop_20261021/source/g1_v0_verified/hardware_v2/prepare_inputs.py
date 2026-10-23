"""Package synchronized calibrated RGB-D/LLM-selection/state records for IK.

No camera/network/LLM calls. Acquisition is an external dependency. Camera and
state timestamps must be Unix seconds on the same synchronized clock.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import numpy as np
from PIL import Image
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from offline_adapter import prepare_target


def make_inputs(capture,intent,calibration,state,root,stroke_m):
    if capture.get('timestamp_clock')!='unix_seconds':raise ValueError('Synchronized Unix timestamps required')
    if not math.isfinite(float(state['timestamp_s'])):raise ValueError('Missing state timestamp')
    rgb_path=(root/capture['rgb_path']).resolve();depth_path=(root/capture['depth_path']).resolve()
    if not rgb_path.is_relative_to(root.resolve()) or not depth_path.is_relative_to(root.resolve()):
        raise ValueError('Capture files must stay inside capture directory')
    for path,key in ((rgb_path,'rgb_sha256'),(depth_path,'depth_sha256')):
        if hashlib.sha256(path.read_bytes()).hexdigest()!=capture[key]:raise ValueError('Capture hash mismatch')
    rgb=np.asarray(Image.open(rgb_path).convert('RGB'));depth=np.load(depth_path,allow_pickle=False)
    if depth.ndim!=2 or depth.shape!=rgb.shape[:2]:raise ValueError('RGB/depth registration shape mismatch')
    obs={k:capture[k] for k in ('frame_id','camera_id','camera_role','depth_aligned_to_color',
        'rgb_timestamp_s','depth_timestamp_s','pose_timestamp_s','detections','intrinsics')}
    obs['depth_m']=depth.tolist()
    if not calibration.get('calibration_id') or calibration['calibration_id'].startswith('sim_'):
        raise ValueError('Real measured camera calibration required, not sim assumed')
    # A replay clock for recorded inputs, NOT a declaration that they are live.
    now=max(float(obs['rgb_timestamp_s']),float(obs['depth_timestamp_s']),float(obs['pose_timestamp_s']),float(state['timestamp_s']))
    prepare_target(intent,obs,calibration,now)
    if now-float(state['timestamp_s'])>.4:raise ValueError('State not synchronized with RGB-D')
    return {'intent':intent,'observation':obs,'calibration':calibration,'snapshot':state,
            'stroke_m':stroke_m,'now_s':now,'offline_only':True,
            'capture_provenance':{'rgb_sha256':capture['rgb_sha256'],'depth_sha256':capture['depth_sha256']}}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for arg in ('capture','intent','calibration','state','output'):p.add_argument('--'+arg,required=True)
    p.add_argument('--stroke-m',type=float,default=.0015)
    a=p.parse_args();load=lambda path:json.loads(Path(path).read_text())
    result=make_inputs(load(a.capture),load(a.intent),load(a.calibration),load(a.state),Path(a.capture).parent,a.stroke_m)
    Path(a.output).write_text(json.dumps(result,allow_nan=False)+'\n')
    print('Offline synchronized inputs saved; no camera connection, LLM API, or motor commands.')


if __name__=='__main__':main()
