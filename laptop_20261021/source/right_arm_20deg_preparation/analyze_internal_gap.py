import json
from pathlib import Path
from collections import defaultdict
from check_sweep import check
p=Path(__file__).resolve().parent
r=json.loads((p/'after_operator_release.json').read_text())
x=check([j['q'] for j in r['state']['joints']])
pairs=defaultdict(list)
for c in x['contacts']:pairs[tuple(sorted([c['body1'],c['body2']]))].append(c['distance_m'])
summary=[dict(pair=k,min_m=min(v),max_m=max(v),variation_m=max(v)-min(v),count=len(v)) for k,v in pairs.items()]
report=dict(state_tick=r['state']['tick'],pairs=summary,note='Positive distances are clearance, not penetration. Shoulder pitch alone preserves downstream relative transforms. No official SRDF pair exclusions were loaded.')
(p/'elbow_wrist_geometry_analysis.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))
