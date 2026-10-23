import json,math
from pathlib import Path
from coordinated_geometry import Geometry
p=Path(__file__).resolve().parent
r=next(json.loads(s) for s in reversed((p/'one_degree_fresh_pose.txt').read_text(encoding='utf-8-sig').splitlines()) if s.startswith('{'))
q=[j['q'] for j in r['joints']];g=Geometry();rows=[]
for k in range(21):
 a=q.copy();a[22]-=math.radians(k*.05);rows.append(dict(deg=-k*.05,contacts=g.contacts(a)))
print(json.dumps({'start':rows[0],'end':rows[-1],'minimum':min([c['d'] for x in rows for c in x['contacts']],default=.01)}))
(p/'one_degree_geometry.json').write_text(json.dumps(rows,indent=2))
