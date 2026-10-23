"""Safe-default local launcher. No robot communication without explicit action."""
import argparse,datetime,hashlib,json,os,shutil,subprocess,sys
from pathlib import Path

HOME=Path(__file__).resolve().parent
ENTRIES={'one':('one_degree_reference_retry','one_degree_reference.py'),
         'fifteen':('fifteen_degree_reference','fifteen_degree_reference.py')}

def help_text():
    print('G1 起動入口（既定はヘルプ。開くだけでは通信・動作しません）')
    print('  START.cmd status   : 状態受信と制御所有権の読み取りだけ')
    print('  START.cmd release  : AIが稼働中の場合だけ明示的に解除、腕軌道なし')
    print('  START.cmd one      : 支持状態で右肩1度の往復を1回実行')
    print('  START.cmd fifteen  : 支持状態で右肩15度の往復を1回実行')
    print('  START.cmd offline  : 通信なしでパケット・ガード計算を確認')
    print('最初に README_JA.txt を読んでください。AI自動復帰・ネットワーク変更はありません。')

def create_run(action):
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    path=HOME/'runs'/(stamp+'_'+action);path.mkdir(parents=True,exist_ok=False)
    return path

def copy_controller(run,action):
    directory,entry=ENTRIES[action]
    target=run/directory
    shutil.copytree(HOME/'controllers'/directory,target)
    if not (run/'right_arm_20deg_preparation').exists():
        shutil.copytree(HOME/'controllers'/'right_arm_20deg_preparation',run/'right_arm_20deg_preparation')
    return target/entry

def invoke(python,script,args,run,label):
    command=[str(python),str(script),*args]
    meta={'command':command,'cwd':str(run),'script_sha256':hashlib.sha256(script.read_bytes()).hexdigest()}
    (run/(label+'_invocation.json')).write_text(json.dumps(meta,ensure_ascii=False,indent=2),encoding='utf-8')
    env=os.environ.copy();env['PYTHONIOENCODING']='utf-8';env['PYTHONUNBUFFERED']='1'
    print('記録先: '+str(run),flush=True)
    with (run/(label+'_console.txt')).open('w',encoding='utf-8') as log:
        proc=subprocess.Popen(command,cwd=run,env=env,stdin=None,stdout=subprocess.PIPE,
                              stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1)
        while True:
            try:
                line=proc.stdout.readline()
                if line:print(line,end='',flush=True);log.write(line);log.flush()
                elif proc.poll() is not None:break
            except KeyboardInterrupt:
                # The child shares this console and has its own verified stop handler.
                # Do not kill it or restore AI while it performs its existing stop path.
                print('停止要求を受け付けました。制御プログラムの終了を待ちます。',flush=True)
        code=proc.wait()
    (run/(label+'_exit.json')).write_text(json.dumps({'exit_code':code}),encoding='utf-8')
    return code

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action',nargs='?',default='help',choices=('help','status','release','one','fifteen','offline'))
    parser.add_argument('--python',type=Path,default=HOME.parent/'g1_sdk_venv'/'Scripts'/'python.exe')
    a=parser.parse_args()
    if a.action=='help':help_text();return 0
    if not a.python.is_file():parser.error('既存の専用Pythonが見つかりません: '+str(a.python))
    run=create_run(a.action)
    if a.action in ('status','release'):
        script=HOME/'tools'/('status_readonly.py' if a.action=='status' else 'release_ai.py')
        return invoke(a.python,script,[],run,a.action)
    if a.action in ENTRIES:
        script=copy_controller(run,a.action)
        return invoke(a.python,script,['--execute'],run,a.action)
    # This branch does not construct subscribers, publishers or RPC clients.
    for action in ENTRIES:
        script=copy_controller(run,action)
        code=invoke(a.python,script,[],run,action+'_offline')
        if code:return code
    # Existing math tests remain offline; original one-degree equivalence test
    # references its historical source layout, so use the verified 15-degree math test.
    check=run/'fifteen_degree_reference'/'check_tracking_math.py'
    return invoke(a.python,check,[],run,'tracking_math')

if __name__=='__main__':raise SystemExit(main())
