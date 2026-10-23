import json
from pathlib import Path
from coordinated_geometry import Geometry
p=Path(__file__).resolve().parent
r=json.loads((p/'mode1_baseline_summary.json').read_text())
c=Geometry().contacts(r['last_q'])
print(json.dumps(sorted(c,key=lambda x:x['d'])[:3]))
