import json,math
from pathlib import Path
from check_sweep import check
p=Path(__file__).resolve().parent
r=next(json.loads(s) for s in reversed((p/'after_outward_adjustment.txt').read_text(encoding='utf-8-sig').splitlines()) if s.startswith('{'))
q=[j['q'] for j in r['joints']]
rows=[]
for roll_delta in (0,-3,-5,-10):
  for envelope in (False,True):
    a=q.copy();a[23]+=math.radians(roll_delta)
    if envelope:a[22]+=math.radians(3)
    x=check(a,delta_deg=-26 if envelope else -20)
    ext=[c for c in x['contacts'] if set([c['body1'],c['body2']])!=set(['right_elbow_link','right_wrist_pitch_link'])]
    closest=min(ext,key=lambda c:c['distance_m']) if ext else None
    rows.append(dict(roll_adjust_deg=roll_delta,tracking_envelope_plusminus3deg=envelope,closest_hip=closest,hip_contact_under10mm=bool(ext),first_hip=ext[:1]))
report=dict(tick=r['tick'],q22=q[22],q23=q[23],results=rows,note='Nominal conservative convex collision geometry; distances positive mean separation; no-contact means beyond 10mm contact-generation envelope, not exact distance.')
(p/'adjusted_pose_options.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report))

