"""Sequential worker processes bound native solver memory lifetime."""
from pathlib import Path
import subprocess,sys,argparse,json
ROOT=Path(__file__).resolve().parents[1]
p=argparse.ArgumentParser();p.add_argument('--h',type=float,default=5);p.add_argument('candidates',nargs='*');a=p.parse_args()
ids=a.candidates or ['forward_plain','baseline_v1','forward_light','forward_balanced','forward_stiff']
for name in ids:
    target=ROOT/'output/v2/fea'/f'{name}_h{a.h:g}'/'result.json'
    print('START',name,a.h,flush=True)
    r=subprocess.run([sys.executable,str(ROOT/'scripts/fea_v2.py'),name,'--h',str(a.h)],cwd=ROOT)
    if r.returncode:sys.exit(r.returncode)
    assert target.is_file()
print('BATCH_COMPLETE',flush=True)
