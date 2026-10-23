import json
from pathlib import Path
from check_sweep import check
p=Path(__file__).resolve().parent
r=json.loads((p/'minimal_direct_release.json').read_text())
x=check([j['q'] for j in r['state']['joints']])
(p/'minimal_direct_sweep.json').write_text(json.dumps(x,indent=2))
external=[c for c in x['contacts'] if 'hip' in c['body1'] or 'hip' in c['body2']]
print(json.dumps({'clear':x['nominal_geometry_clear'],'external_contacts':external[:8],'minimum_external_m':min([c['distance_m'] for c in external],default=None)}))
