import json,math
from pathlib import Path
from coordinated_geometry import Geometry
p=Path(__file__).resolve().parent
r=next(json.loads(s) for s in reversed((p/'resting_contact_pose.txt').read_text(encoding='utf-8-sig').splitlines()) if s.startswith('{'))
q=[j['q'] for j in r['joints']];g=Geometry();rows=[]
pair=set(('right_hip_roll_link','right_wrist_yaw_link'))
for k in range(61):
    deg=k*.05;a=q.copy();a[23]-=math.radians(deg)
    contacts=g.contacts(a);d=min([c['d'] for c in contacts if set((c['a'],c['b']))==pair],default=.01)
    rows.append({'roll_out_deg':deg,'pair_distance_m_capped10mm':d})
out={'tick':r['tick'],'q22':q[22],'q23':q[23],'monotone_non_decreasing_to3deg':all(b['pair_distance_m_capped10mm']>=a['pair_distance_m_capped10mm']-1e-8 for a,b in zip(rows,rows[1:])), 'every_half_degree':[x for x in rows if round(x['roll_out_deg']*20)%10==0], 'scope':'geometric only, all other q fixed; no contact force assurance'}
(p/'resting_contact_departure.json').write_text(json.dumps(out,indent=2))
print(json.dumps(out))
