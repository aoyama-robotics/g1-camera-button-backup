import json,math
from pathlib import Path
from check_sweep import check
p=Path(__file__).resolve().parent
r=next(json.loads(s) for s in reversed((p/'roll_departure_start.txt').read_text(encoding='utf-8-sig').splitlines()) if s.startswith('{'))
q=[j['q'] for j in r['joints']]
import check_sweep
src=Path(check_sweep.__file__).read_text().split("if __name__")[0].replace('values[22]','values[23]')
ns={'__file__':check_sweep.__file__};exec(src,ns)
roll=ns['check'](q,-10)
exclude=lambda c:set([c['body1'],c['body2']])==set(['right_elbow_link','right_wrist_pitch_link']) and c['distance_m']>0
contacts=[c for c in roll['contacts'] if not exclude(c)]
initial=[c for c in contacts if c['delta_deg']==0]
a=q.copy();a[23]-=math.radians(10);a[22]+=math.radians(3)
pitch=check(a,-26);pc=[c for c in pitch['contacts'] if not exclude(c)]
out={'q22':q[22],'q23':q[23],'roll_initial':initial,'roll_all_min':min(contacts,key=lambda c:c['distance_m']) if contacts else None,'roll_contacts':contacts,'pitch_contacts':pc,'roll_model_penetration':any(c['distance_m']<0 for c in contacts),'pitch_under10mm':bool(pc)}
(p/'roll_then_pitch_plan_geometry.json').write_text(json.dumps(out,indent=2))
print(json.dumps({k:v for k,v in out.items() if k not in ('roll_contacts','pitch_contacts')}))
