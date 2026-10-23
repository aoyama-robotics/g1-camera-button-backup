import json
from pathlib import Path
from check_sweep import check
p=Path(__file__).resolve().parent
lines=(p/'after_operator_sequence_readonly.txt').read_text(encoding='utf-8-sig').splitlines()
r=next(json.loads(s) for s in reversed(lines) if s.startswith('{'))
x=check([j['q'] for j in r['joints']])
(p/'after_operator_sweep.json').write_text(json.dumps(x,indent=2))
external=[c for c in x['contacts'] if 'hip' in c['body1'] or 'hip' in c['body2']]
print(json.dumps({'clear':x['nominal_geometry_clear'],'external_contacts':external[:5],'minimum_external_m':min([c['distance_m'] for c in external],default=None),'all_contacts_first':x['contacts'][:3]}))
