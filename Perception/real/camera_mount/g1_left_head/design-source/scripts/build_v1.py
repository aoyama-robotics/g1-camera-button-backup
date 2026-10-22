"""53 mm underside screw pair, 20 mm L-bracket reach; actual fit remains to check."""
from pathlib import Path
import json,sys,os,traceback,hashlib
import cadquery as cq
from build_mount import block,cylinder,export_checked,descriptor
ROOT=Path(__file__).resolve().parents[1]

def zcyl(radius,z0,length,x,y=0):
    return cq.Workplane(obj=cq.Solid.makeCylinder(radius,length,cq.Vector(x,y,z0),cq.Vector(0,0,1)))

def make_bracket(p,thin=False):
    t=2 if thin else p['flange_thickness'];u=2 if thin else p['upright_thickness']
    d=p['hole_to_upright_inner_face'];margin=p['flange_inboard_margin'];L=p['flange_length'];top=10 if thin else p['upright_top_z']
    flange=block(L,margin+d+u,t,(0,(d+u-margin)/2,-t/2))
    # 0.1 mm overlap with flange avoids coincident-only joining.
    wall=block(L,u,top+.1,(0,d+u/2,(top-.1)/2))
    s=flange.union(wall)
    for x in [-p['hole_pitch']/2,p['hole_pitch']/2]:
        s=s.cut(zcyl(p['robot_clearance_diameter']/2,-t-.1,t+.2,x))
    if not thin:
        for x in [-25,25]:
            for z in [20,40]:
                s=s.cut(cylinder(1.7,d-.1,u+.2,x,z))
                s=s.cut(cylinder(3.2,d-.1,3.3,x,z))
    return s

def run():
    config=ROOT/'config/mount_v1.json';p=json.loads(config.read_text());out=ROOT/'output/v1';out.mkdir(parents=True,exist_ok=True)
    bracket=make_bracket(p);gauge=make_bracket(p,thin=True)
    carrier=cq.importers.importStep(str(ROOT/p['carrier_source'])).translate(tuple(p['carrier_translation']))
    parts={'L_bracket_53mm_20mm_FIT_PROTOTYPE':bracket,'fit_gauge_53mm_20mm':gauge,'tri_rail_carrier_v1':carrier}
    results={name:export_checked(s,name,out) for name,s in parts.items()}
    overlap=bracket.intersect(carrier).val().Volume();assert overlap<1e-6
    checks=[]
    for x in [-p['hole_pitch']/2,p['hole_pitch']/2]:
        shaft=zcyl(2.2,-6,18,x)
        access=zcyl(5,-35,30,x) # shaft/tool approaches from below, ends at flange underside
        for label,s in [('bracket',bracket),('carrier',carrier)]:
            sv=s.intersect(shaft).val().Volume();av=s.intersect(access).val().Volume()
            assert sv<1e-6 and av<1e-6
            checks.append(dict(part=label,x_mm=x,shaft_overlap_mm3=sv,tool_diameter_10mm_overlap_mm3=av))
    for x in [-25,25]:
        for z in [20,40]:
            probe=cylinder(1.65,19.9,12.2,x,z)
            assert bracket.intersect(probe).val().Volume()<1e-6
            assert carrier.intersect(probe).val().Volume()<1e-6
    # Reserve the nominal inboard head volume above the contact plane.
    reserved=block(70,29,60,(0,5.5,30))
    assert bracket.intersect(reserved).val().Volume()<1e-6
    assembly=cq.Assembly(name='G1_UNDERSIDE_L_MOUNT_V1')
    assembly.add(bracket,name='L_BRACKET_53_PITCH_20_REACH',color=cq.Color(.88,.58,.2))
    assembly.add(carrier,name='THREE_RAIL_CARRIER',color=cq.Color(.12,.53,.55))
    assembly.export(str(out/'assembly_L_mount_v1.step'))
    results.update(status=p['status'],geometry_checks_pass=True,bracket_carrier_intersection_mm3=overlap,
                   screw_access_checks=checks,config=descriptor(config),source_carrier=descriptor(ROOT/p['carrier_source']),
                   nominal_pitch_mm=p['hole_pitch'],nominal_offset_mm=p['hole_to_upright_inner_face'],
                   robot_fit_verified=False,thread_verified=False,physical_ready=False,print_release=False,
                   load_rating=None,head_mesh_registration_verified=False)
    (out/'validation.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
    print(json.dumps({k:dict(volume_mm3=v['volume_mm3'],stl=v['stl_check']) for k,v in results.items() if isinstance(v,dict) and 'volume_mm3' in v},indent=2))

if __name__=='__main__':
    code=0
    try:run()
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
