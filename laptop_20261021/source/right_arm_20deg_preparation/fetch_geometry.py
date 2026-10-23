import concurrent.futures,json,hashlib,xml.etree.ElementTree as ET
from pathlib import Path
from urllib.request import urlopen
root=Path(__file__).resolve().parent
robot=ET.parse(root/'g1_29dof.urdf').getroot()
names=sorted(set(m.attrib['filename'] for m in robot.findall('.//collision/geometry/mesh')))
def fetch(name):
    path=root/name;path.parent.mkdir(exist_ok=True)
    if not path.exists():
        data=urlopen('https://raw.githubusercontent.com/unitreerobotics/unitree_ros/master/robots/g1_description/'+name,timeout=20).read()
        path.write_bytes(data)
    data=path.read_bytes()
    return dict(name=name,bytes=len(data),sha256=hashlib.sha256(data).hexdigest())
results=[]
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    for name,future in zip(names,[pool.submit(fetch,n) for n in names]):
        try:results.append(future.result())
        except Exception as e:results.append(dict(name=name,error=str(e)))
(root/'geometry_manifest.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results),flush=True)
