"""Local experiment entry. Default help/offline; hardware actions are explicit."""
import argparse,datetime,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path
HERE=Path(__file__).resolve().parent;ROOT=HERE.parent
PY=ROOT/'g1_sdk_venv/Scripts/python.exe';SOURCE=ROOT/'continuous_suspended_pitch10_executor'
def main():
 p=argparse.ArgumentParser(description=__doc__)
 p.add_argument('action',nargs='?',default='help',choices=['help','offline','conditions','status','release','camera','plan','run'])
 p.add_argument('--plan',type=Path);p.add_argument('--capture',type=Path);p.add_argument('--state',type=Path)
 p.add_argument('--reference-plan',type=Path)
 p.add_argument('--pixel',nargs=2,type=int);p.add_argument('--out',type=Path)
 a=p.parse_args();env=os.environ.copy();env.update(PYTHONIOENCODING='utf-8',G1_WORKSPACE_ROOT=str(ROOT))
 if a.action=='help':
  p.print_help();print('\n最初に README_JA.txt と STOP_CONDITIONS_JA.md を参照。\nhelp/offline/conditions/plan はロボット通信なし。status/camera は読取。release はAI解除。runだけが腕を動かす。');return 0
 if a.action=='conditions':print((HERE/'STOP_CONDITIONS_JA.md').read_text(encoding='utf-8'));return 0
 if a.action=='offline':
  for command in [[str(PY),str(SOURCE/'validate_offline.py')],[str(PY),str(HERE/'verify_pitch_offline.py')]]:
   r=subprocess.run(command,cwd=ROOT,env=env)
   if r.returncode:return r.returncode
  return 0
 if a.action=='plan':
  if not all([a.capture,a.state,a.pixel,a.out]):p.error('planには --capture --state --pixel U V --out が必要')
  extra=['--reference-plan',str(a.reference_plan.resolve())] if a.reference_plan else []
  return subprocess.call([str(PY),str(HERE/'prepare_plan.py'),'--capture',str(a.capture.resolve()),'--state',str(a.state.resolve()),'--pixel',*map(str,a.pixel),'--out',str(a.out.resolve()),*extra],cwd=ROOT,env=env)
 if a.action=='camera':
  if not a.out:p.error('cameraには --out が必要')
  dest=a.out.resolve();dest.mkdir(parents=True,exist_ok=False)
  remote='/tmp/g1_camera_'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ')
  code=(HERE/'capture_remote.py').read_text().replace('__REMOTE_DEST__',remote)
  r=subprocess.run(['ssh','-o','StrictHostKeyChecking=yes','-o','ConnectTimeout=5','<ROBOT_USER>@<PRIVATE_IP>','python3','-'],input=code,text=True)
  if r.returncode:return r.returncode
  r=subprocess.run(['scp','-o','StrictHostKeyChecking=yes','<ROBOT_USER>@<PRIVATE_IP>:'+remote+'/capture.zip',str(dest/'capture.zip')])
  if r.returncode:return r.returncode
  import zipfile
  with zipfile.ZipFile(dest/'capture.zip') as z:
   if set(z.namelist())!={'rgb.jpg','depth.npy','metadata.json'}:raise RuntimeError('Unexpected capture contents')
   z.extractall(dest)
  print('保存先:',dest);return 0
 run=HERE/'runs'/(datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')+'_'+a.action);run.mkdir(parents=True)
 if a.action in ('status','release'):
  script=ROOT/'G1_START_HERE/tools'/('status_readonly.py' if a.action=='status' else 'release_ai.py')
  cmd=[str(PY),str(script)]
 else:
  if not a.plan:p.error('runには今回の --plan 絶対パスが必要。古い候補を自動選択しません')
  plan=a.plan.resolve()
  if not plan.is_file():p.error('指定planが見つかりません')
  data=json.loads(plan.read_text(encoding='utf-8'));samples=data['samples']
  if not samples:p.error('Empty explicit trajectory')
  summary=dict(explicit_plan=str(plan),sha256=hashlib.sha256(plan.read_bytes()).hexdigest(),duration_s=samples[-1]['t'],source_metadata=data.get('metadata',{}))
  print(json.dumps({k:v for k,v in summary.items() if k!='source_metadata'},ensure_ascii=False),flush=True)
  (run/'selected_plan.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
  target=run/'executor';shutil.copytree(SOURCE,target,ignore=shutil.ignore_patterns('*result.json','__pycache__','*.log'))
  shutil.copy2(plan,run/'input_plan.json');env['G1_PLAN_PATH']=str(run/'input_plan.json')
  cmd=[str(PY),str(target/'continuous_touch.py'),'--execute']
 (run/'invocation.json').write_text(json.dumps(dict(command=cmd,cwd=str(run),plan=str(a.plan) if a.plan else None),indent=2),encoding='utf-8')
 print('記録先:',run,flush=True)
 with (run/'console.txt').open('w',encoding='utf-8') as log:
  proc=subprocess.Popen(cmd,cwd=run,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1)
  while True:
   try:
    line=proc.stdout.readline()
    if line:print(line,end='',flush=True);log.write(line);log.flush()
    elif proc.poll() is not None:break
   except KeyboardInterrupt:print('既存の子プロセス停止処理を待ちます。強制終了やAI復帰はしません。',flush=True)
  code=proc.wait()
 (run/'exit.json').write_text(json.dumps(dict(exit_code=code)),encoding='utf-8');return code
if __name__=='__main__':raise SystemExit(main())
