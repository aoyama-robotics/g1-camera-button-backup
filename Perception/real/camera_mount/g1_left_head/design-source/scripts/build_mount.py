"""Parametric concept hardware, with no invented robot mounting dimensions.

SANDRONE CadQuery/OCC pipeline and actual STL validator reused read-only.
Coordinates: X forward, Y left/outboard, Z up. All CAD/export units mm.
"""
from pathlib import Path
import argparse
import hashlib
import json
import math
import sys
import os
import traceback
import cadquery as cq
from src.generators.xl330_cradle import validate_binary_stl

ROOT=Path(__file__).resolve().parents[1]

def block(dx,dy,dz,center):
    return cq.Workplane('XY').box(dx,dy,dz).translate(center)

def cylinder(radius,y0,length,x,z):
    return cq.Workplane(obj=cq.Solid.makeCylinder(radius,length,cq.Vector(x,y0,z),cq.Vector(0,1,0)))

def rail(params,length=None,slots=None):
    p=params; length=length or p['length']; slots=slots or p['slot_count']
    h=p['height']; width=p['max_width']; neck=p['neck_width']
    lower=h-p['top_to_lower_datum']
    upper=lower+p['datum_vertical_separation']
    crest_low=lower+(width-p['datum_width'])/2
    crest_high=upper-(width-p['datum_width'])/2
    shoulder=lower-(p['datum_width']-neck)/2
    top_half=p['datum_width']/2-(h-upper)
    assert crest_high > crest_low
    assert lower >= .367*25.4
    # Section x coordinate is width, y coordinate is height; extrusion is rail length.
    # Small outer vertical flats truncate the theoretical sharp intersections;
    # all four 45 degree faces pass through the .748 x .108 datum rectangle.
    pts=[(-neck/2,-.1),(neck/2,-.1),(neck/2,shoulder),(width/2,crest_low),(width/2,crest_high),(top_half,h),(-top_half,h),(-width/2,crest_high),(-width/2,crest_low),(-neck/2,shoulder)]
    plane=cq.Plane(origin=(-length/2,0,0),xDir=(0,1,0),normal=(1,0,0))
    s=cq.Workplane(plane).polyline(pts).close().extrude(length)
    # Break exposed end and long edges before slot cuts. Roots remain supported.
    s=s.edges().chamfer(p['edge_break'])
    for i in range(slots):
        x=(i-(slots-1)/2)*p['slot_pitch']
        cutter=block(p['slot_width'],width+2,p['slot_depth']+1,(x,0,h-p['slot_depth']/2+.5))
        s=s.cut(cutter)
    return s

def descriptor(p):
    return {'path':str(p.relative_to(ROOT)),'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}

def export_checked(shape,name,out):
    s=shape.val();solids=s.Solids()
    assert len(solids)==1,name+' must have one solid'
    assert s.isValid() and all(sh.isValid() and sh.Closed() for sh in s.Shells()),name+' invalid/open BREP'
    step=out/(name+'.step');stl=out/(name+'.stl')
    cq.exporters.export(shape,str(step))
    cq.exporters.export(shape,str(stl),tolerance=.035,angularTolerance=.08)
    check=validate_binary_stl(stl)
    reread=cq.importers.importStep(str(step)).val()
    assert len(reread.Solids())==1 and reread.isValid()
    assert abs(reread.Volume()-s.Volume()) < .001
    b=s.BoundingBox()
    return {'solid_count':1,'valid_brep':True,'all_shells_closed':True,'volume_mm3':s.Volume(),
            'bounds_mm':[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax],
            'solid_PETG_mass_g_at_1_27':s.Volume()*.00127,'stl_check':check,'step_roundtrip':True,
            'files':[descriptor(step),descriptor(stl)]}

def build(config,out):
    p=json.loads(config.read_text());r=p['rail'];c=p['carrier'];a=p['adapter'];out.mkdir(exist_ok=True,parents=True)
    L=c['length'];h=c['half_height'];i=c['inner_y'];o=c['outer_y'];ov=c['fusion_overlap']
    body=block(L,c['inner_wall'],2*h,(0,i+c['inner_wall']/2,0))
    body=body.union(block(L,c['outer_wall'],2*h,(0,o-c['outer_wall']/2,0)))
    for z in [-h+c['cross_wall']/2,h-c['cross_wall']/2]:
        body=body.union(block(L,o-i, c['cross_wall'],(0,(i+o)/2,z)))
    rr=rail(r)
    body=body.union(rr.translate((0,c['rail_center_y'],h)))
    body=body.union(rr.rotate((0,0,0),(1,0,0),180).translate((0,c['rail_center_y'],-h)))
    body=body.union(rr.rotate((0,0,0),(1,0,0),-90).translate((0,o,0)))
    plate=block(a['length'],a['thickness'],a['height'],(0,a['thickness']/2,0)).edges('|Y').fillet(a['corner_radius'])
    for x,z in a['carrier_bolt_xz']:
        drill=cylinder(a['clearance_diameter']/2,-.1,i+c['inner_wall']+.2,x,z)
        plate=plate.cut(drill).cut(cylinder(a['head_recess_diameter']/2,-.1,a['head_recess_depth']+.1,x,z))
        body=body.cut(drill)
        # Open hex pockets from the central cavity. Nuts inserted through open ends.
        plane=cq.Plane(origin=(x,i+c['inner_wall']-a['nut_depth'],z),xDir=(1,0,0),normal=(0,1,0))
        pocket=cq.Workplane(plane).polygon(6,a['nut_across_flats']/math.cos(math.pi/6)).extrude(a['nut_depth']+.1)
        body=body.cut(pocket)
    # No guessed holes: an actual robot interface requires measured data and a new adapter revision.
    if a['robot_holes'] is not None:
        raise ValueError('Robot-specific adapter requires reviewed seat/axis geometry; do not silently drill guesses.')
    parts={'tri_rail_carrier_CONCEPT':body,'head_adapter_BLANK_NOT_FIT':plate,'rail_fit_coupon':rail(r,30,2)}
    report={name:export_checked(s,name,out) for name,s in parts.items()}
    overlap=body.intersect(plate).val().Volume()
    assert overlap < 1e-6
    # Shared mechanical interface: shafts clear each part, counterbore and nut seat geometries remain.
    for x,z in a['carrier_bolt_xz']:
        probe=cylinder(1.65,-.1,i+c['inner_wall']+.2,x,z)
        assert body.intersect(probe).val().Volume()<1e-7
        assert plate.intersect(probe).val().Volume()<1e-7
    assembly=cq.Assembly(name='G1_LEFT_TRI_RAIL_CONCEPT')
    assembly.add(plate,name='BLANK_ADAPTER_NO_ROBOT_HOLES',color=cq.Color(.82,.48,.12))
    assembly.add(body,name='TRI_RAIL_CARRIER',color=cq.Color(.18,.23,.27))
    assembly.export(str(out/'assembly_CONCEPT.step'))
    for name,s in list(parts.items())[:2]:
        moved=s.translate(tuple(p['preview']['translation_in_head_mesh_mm']))
        cq.exporters.export(moved,str(out/(name+'_head_coordinates.stl')),tolerance=.035,angularTolerance=.08)
    report.update(status=p['status'],geometry_checks_pass=True,adapter_carrier_overlap_mm3=overlap,
                  robot_holes_verified=False,robot_contact_verified=False,robot_screw_length_verified=False,
                  physical_ready=False,print_release=False,rail_standard_certified=False,
                  payload_rating=None,preview=p['preview'],config=descriptor(config),
                  sandrone_validator={'path':'E:/sandrone/src/generators/xl330_cradle.py','function':'validate_binary_stl'},
                  required_inputs=['Actual left head hole positions and axes','Seat geometry','Thread and original screw engagement','Payload and accessory clamp envelope','Head revision'])
    (out/'validation.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    print(json.dumps({k:{'volume_mm3':v['volume_mm3'],'stl':v['stl_check']} for k,v in report.items() if isinstance(v,dict) and 'volume_mm3' in v},indent=2))

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--config',type=Path,default=ROOT/'config/mount_v0.json');parser.add_argument('--out',type=Path,default=ROOT/'output/v0');args=parser.parse_args()
    code=0
    try:
        build(args.config,args.out)
    except BaseException:
        traceback.print_exc();code=1
    # Same explicit shutdown as SANDRONE: native OCC destructor teardown on this
    # Windows environment can crash after successful export. Preserve failures.
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
