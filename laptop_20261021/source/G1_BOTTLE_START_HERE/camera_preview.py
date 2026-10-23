"""Camera-only PC preview. SSH to existing G1 videohub, no robot state/control APIs."""
import base64,collections,datetime,hashlib,json,queue,subprocess,threading,time
from pathlib import Path
import cv2,numpy as np
HERE=Path(__file__).resolve().parent
STATUS=HERE/'camera_preview_status.json';STOP=HERE/'CAMERA_PREVIEW_STOP'
REMOTE=r'''import sys,time,json,base64
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.video.video_client import VideoClient
ChannelFactoryInitialize(0,"eth0")
c=VideoClient();c.SetTimeout(2.);c.Init()
count=0
while True:
 started=time.time()
 try:
  status,data=c.GetImageSample();received=time.time()
  if status==0 and data:
   count+=1;print(json.dumps(dict(type="frame",frame=count,requested_unix=started,received_unix=received,rgb_exposure_unix=None,jpeg=base64.b64encode(bytes(data)).decode("ascii"))),flush=True)
  else:print(json.dumps(dict(type="error",code=status,received_unix=received)),flush=True)
 except BrokenPipeError:break
 except Exception as e:print(json.dumps(dict(type="error",message=str(e))),flush=True)
 time.sleep(max(0.,.5-(time.time()-started)))
'''
def main():
 if STOP.exists():STOP.unlink()
 q=queue.Queue(maxsize=2);name='G1 Camera Only - Q/ESC or close window to exit'
 cv2.namedWindow(name,cv2.WINDOW_NORMAL);cv2.resizeWindow(name,1280,720)
 image=np.zeros((720,1280,3),np.uint8);last=None;meta={};times=collections.deque();count=0;changes=0;digest=None;error='Waiting for SSH camera authentication';last_save=0;last_log=0
 p=subprocess.Popen(['ssh','-T','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=5','<ROBOT_USER>@<PRIVATE_IP>','python3','-u','-'],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=None)
 p.stdin.write(REMOTE.encode());p.stdin.close()
 def read():
  for line in p.stdout:
   try:item=json.loads(line)
   except (ValueError,UnicodeError):continue
   item['_pc_received_unix']=time.time();item['_pc_perf']=time.perf_counter()
   try:q.put_nowait(item)
   except queue.Full:
    try:q.get_nowait()
    except queue.Empty:pass
    q.put_nowait(item)
 threading.Thread(target=read,daemon=True).start()
 print('CAMERA_PREVIEW_STARTED; no body reads or motor commands. Q/ESC/window close or CAMERA_PREVIEW_STOP file stops only this preview.',flush=True)
 try:
  while True:
   now=time.perf_counter()
   while True:
    try:item=q.get_nowait()
    except queue.Empty:break
    if item.get('type')=='frame':
     raw=base64.b64decode(item['jpeg']);decoded=cv2.imdecode(np.frombuffer(raw,np.uint8),cv2.IMREAD_COLOR)
     if decoded is None:error='Invalid camera JPEG';continue
     image=decoded;last=item['_pc_perf'];meta={k:v for k,v in item.items() if k!='jpeg'};count+=1;times.append(last);newhash=hashlib.sha256(raw).hexdigest()
     if digest is not None and digest!=newhash:changes+=1
     digest=newhash;error=''
     if count==1:print('FIRST_CAMERA_FRAME_RECEIVED '+str(meta['received_unix']),flush=True)
    else:error='Camera error: '+str(item.get('code',item.get('message','unknown')))
   while times and now-times[0]>10:times.popleft()
   fps=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0.
   age=None if last is None else max(0.,now-last);stale=age is None or age>2.
   view=cv2.resize(image,(1280,720));cv2.rectangle(view,(0,0),(1280,100),(0,0,0),-1)
   title=('STALE / NO NEW FRAME' if stale else 'CAMERA RECEIVING')+f' | Received {fps:.2f} fps | Frames {count}'
   color=(0,0,255) if stale else (0,255,0);cv2.putText(view,title,(12,28),cv2.FONT_HERSHEY_SIMPLEX,.7,color,2)
   when=datetime.datetime.fromtimestamp(meta['_pc_received_unix'],datetime.timezone.utc).strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]+' UTC' if meta else 'not received'
   cv2.putText(view,'PC receive: '+when+' | age '+('unknown' if age is None else f'{age:.1f}s'),(12,57),cv2.FONT_HERSHEY_SIMPLEX,.6,(255,255,255),1)
   cv2.putText(view,(error or 'RGB exposure time unavailable; receive FPS is not verified sensor FPS')[:145],(12,84),cv2.FONT_HERSHEY_SIMPLEX,.55,(0,200,255),1)
   cv2.imshow(name,view);key=cv2.waitKey(30)&255
   visible=cv2.getWindowProperty(name,cv2.WND_PROP_VISIBLE)>=1
   if now-last_save>1:
    state=dict(running=True,window_visible=visible,received_frames=count,changed_encoded_frames=changes,receive_fps_10s=fps,last_frame_age_s=age,stale=stale,last_frame_metadata=meta,error=error,ssh_exit=p.poll(),motor_commands=0,body_state_reads=0)
    STATUS.write_text(json.dumps(state,indent=2),encoding='utf-8');last_save=now
   if now-last_log>30:print(json.dumps(dict(received_frames=count,receive_fps=fps,stale=stale,window_visible=visible)),flush=True);last_log=now
   if key in (27,ord('q'),ord('Q')) or not visible or STOP.exists():break
   if p.poll() is not None:
    error='SSH camera connection ended; image is stale';last=last if last else None
 finally:
  if p.poll() is None:p.terminate()
  try:p.wait(timeout=5)
  except subprocess.TimeoutExpired:p.kill();p.wait()
  cv2.destroyAllWindows()
  STATUS.write_text(json.dumps(dict(running=False,received_frames=count,motor_commands=0,body_state_reads=0),indent=2),encoding='utf-8')
if __name__=='__main__':main()
