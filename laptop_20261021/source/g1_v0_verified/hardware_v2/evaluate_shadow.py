import csv,json
from pathlib import Path
from shadow_motion_guard import ShadowGuard,ShadowFault

root=Path(__file__).resolve().parents[2]
def evaluate(records):
    guard=ShadowGuard();count=0
    try:
        for tick,host,q,dq,rpy,gyro in records:
            guard.observe(tick,host,q,dq,rpy,gyro);count+=1
    except ShadowFault as error:return dict(fault=str(error),samples_before_fault=count,tick=tick,q=q[:15])
    return dict(fault=None,samples=count)
rows=[]
with (root/'baseline_samples.csv').open() as f:
    for r in csv.DictReader(f):
        rows.append((int(r['tick']),float(r['host_perf_s']),
          [float(r['q'+str(i)]) for i in range(29)],
          [float(r['dq'+str(i)]) for i in range(29)],
          [float(r[x]) for x in ['roll','pitch','yaw']],
          [float(r[x]) for x in ['gx','gy','gz']]))
result={'baseline':evaluate(rows),'synthetic':{}}
for name,speed,raw in [('abrupt_.5',.5,.5),('persistent_.06',.06,.06),('position_only_.06',.06,0.),('persistent_dq_only',0.,.06)]:
    records=[]
    for t in range(101):
        q=[0.]*29;dq=[0.]*29
        if t>=20:q[2]=(t-20)/1000*speed;dq[2]=raw
        records.append((t,t/1000,q,dq,[0.]*3,[0.]*3))
    r=evaluate(records);r['onset_tick']=20
    if r['fault']:r['detection_delay_ms']=r['tick']-20;r['displacement_rad']=r['q'][2]
    result['synthetic'][name]=r
Path('shadow_evaluation.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result))
