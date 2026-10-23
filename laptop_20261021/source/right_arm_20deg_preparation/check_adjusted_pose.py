import json
from pathlib import Path
from check_sweep import check
p=Path(__file__).resolve().parent
r=next(json.loads(s) for s in reversed((p/'after_outward_adjustment.txt').read_text(encoding='utf-8-sig').splitlines()) if s.startswith('{'))
q=[j['q'] for j in r['joints']];q[22]+=.05235987755982989
x=check(q,delta_deg=-26)
pairs=[c for c in x['contacts'] if set([c['body1'],c['body2']])!=set(['right_elbow_link','right_wrist_pitch_link'])]
print(json.dumps({'q22':q[22]-.05235987755982989,'q23':q[23],'other_contacts':pairs[:10],'minimum_other_m':min([c['distance_m'] for c in pairs],default=None)}))
(p/'adjusted_sweep_analysis.json').write_text(json.dumps(x,indent=2))
