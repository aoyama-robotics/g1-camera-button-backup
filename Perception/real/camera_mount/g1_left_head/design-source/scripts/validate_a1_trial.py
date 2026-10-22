"""Bambu 2.8 result compatibility: validate real 3MF/G-code instead of missing objects key."""
from pathlib import Path
import json,zipfile,xml.etree.ElementTree as ET,re
import numpy as np
from src.manufacturing.bambu_3mf import validate_sliced_3mf,sha256_file
from src.generators.xl330_cradle import validate_binary_stl
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'output/a1_mini_trial';folder=out/'sliced'
path=folder/'G1_camera_trial_plate.gcode.3mf';source=out/'G1_camera_trial_plate.stl'
cli=json.loads((folder/'result.json').read_text());assert cli['return_code']==0
assert len(cli['sliced_plates'])==1
p=cli['sliced_plates'][0];assert not p.get('warning_message')
assert p['triangle_count']>0 and len(p['filaments'])==1 and p['filaments'][0]['total_used_g']>0
assert abs(cli['layer_height']-.2)<1e-6 and cli['wall_loops']==4 and cli['sparse_infill_density']==30
validation=validate_sliced_3mf(path)
with zipfile.ZipFile(path) as z:
    root=ET.fromstring(z.read('Metadata/model_settings.config'));assert len(root.findall('object'))==1
    assert int(root.find('object/metadata[@face_count]').attrib['face_count'])==p['triangle_count']
    model=ET.fromstring(z.read('3D/Objects/object_1.model'))
    triangles=model.findall('.//{*}triangle');assert len(triangles)==p['triangle_count']
    plate=json.loads(z.read('Metadata/plate_1.json'));assert len(plate['bbox_objects'])==1
    bbox=plate['bbox_all'];assert 0<=bbox[0]<bbox[2]<=180 and 0<=bbox[1]<bbox[3]<=180
    gcode=z.read('Metadata/plate_1.gcode').decode()
    settings=json.loads(z.read('Metadata/project_settings.config'))
    assert settings['printer_model']=='Bambu Lab A1 mini'
    assert settings['filament_type']==['PLA']
    assert settings['enable_support']=='1' and settings['wall_loops']=='4'
assert 'M83' in gcode and '; total layer number: 450' in gcode
# Review real deposited paths, not only the model thumbnail. Extrusion is relative.
selected={1:[],100:[],190:[],350:[]};colors={n:[] for n in selected}
x=y=0.;layer=0;feature='';counts=[0,0,0]
for line in gcode.splitlines():
    if line.startswith('; CHANGE_LAYER'):layer+=1
    if line.startswith('; FEATURE:'):feature=line.split(':',1)[1].strip()
    if not line.startswith(('G0 ','G1 ')):continue
    words=dict((a,float(b)) for a,b in re.findall(r'([XYZE])([-+]?\d*\.?\d+)',line.split(';')[0]))
    nx=words.get('X',x);ny=words.get('Y',y)
    if layer and words.get('E',0)>0 and (nx!=x or ny!=y) and feature!='Custom':
        assert 0<=nx<=180 and 0<=ny<=180
        if feature not in ['Support','Support interface','Brim']:
            if 19<=nx<=100 and 19<=ny<=98:counts[0]+=1
            elif 114<=nx<=156 and 19<=ny<=64:counts[1]+=1
            elif 114<=nx<=148 and 84<=ny<=102:counts[2]+=1
        if layer in selected:
            selected[layer].append([(x,y),(nx,ny)])
            colors[layer].append('#df9b25' if 'Support' in feature else '#95a1aa' if feature=='Brim' else '#148b91')
    x,y=nx,ny
assert min(counts)>100
fig,axes=plt.subplots(2,2,figsize=(10,10))
for ax,n in zip(axes.flat,selected):
    ax.add_collection(LineCollection(selected[n],colors=colors[n],linewidths=.5));ax.set_xlim(0,180);ax.set_ylim(0,180);ax.set_aspect('equal');ax.grid(alpha=.15);ax.set_title(f'Layer {n} | Z = {n*.2:.1f} mm');ax.set_xlabel('mm');ax.set_ylabel('mm')
fig.suptitle('A1 mini trial | actual G-code paths\nTeal: model / Orange: support / Grey: brim')
fig.tight_layout(rect=[0,0,1,.94]);fig.savefig(out/'toolpath_review.png',dpi=140);plt.close(fig)
report=dict(status='succeeded',input=dict(path=str(source),sha256=sha256_file(source),validation=validate_binary_stl(source)),
    output=dict(path=str(path),validation=validation),return_code=0,cli_result=cli,
    compatibility_note='Bambu Studio 2.8 omits sliced_plates.objects. Verified metadata object, actual model triangles, plate bbox, profile, and extrusion in all three part regions.',
    extrusion_segment_counts_by_part=counts,physical_part_count=3,profile=settings['printer_model'])
(folder/'slice_report.json').write_text(json.dumps(report,indent=2))
probe=json.loads((out/'printer_probe.json').read_text());assert probe['connected'] and probe['model_match'] and probe['telemetry_received']
assert probe['printer_state']['gcode_state']=='IDLE' and probe['printer_state']['print_error']==0
dry=dict(mode='dry_run',artifact=dict(local_path=str(path),sha256=validation['sha256']),profile=validation['profile'],
    estimates=validation['estimates'],printer_probe=str(out/'printer_probe.json'),checks=dict(geometry=True,sliced_3mf=True,toolpath_bounds=True,three_parts_extruded=True),
    security=dict(file_uploaded=False,print_started=False),note='Existing local slice and read-only probe composed; no second slicing run.')
(folder/'dry_run_report.json').write_text(json.dumps(dry,indent=2))
print(json.dumps(dict(total_seconds=p['total_predication'],weight_g=p['filaments'][0]['total_used_g'],segments=counts,bbox=bbox),indent=2))
import os,sys
sys.stdout.flush();sys.stderr.flush();os._exit(0)
