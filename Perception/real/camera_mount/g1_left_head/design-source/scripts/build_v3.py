"""Integrated version of the accepted forward-balanced geometry; mm."""
import json,os,sys,traceback
import cadquery as cq
from build_mount import ROOT,block,rail,cylinder,export_checked,descriptor
from build_v1 import zcyl

def run():
    cfg=json.loads((ROOT/'config/structural_v2.json').read_text())
    p=next(p.copy() for p in cfg['candidates'] if p['id']=='forward_balanced');p['id']='monolithic'
    cfg.update(candidates=[p],joint_note='Continuous single solid; no carrier-to-bracket fasteners or joint compliance.',selection_policy='Preserve accepted forward_balanced envelope and reinforcement; remove all four M3 interfaces.')
    (ROOT/'config/structural_v3.json').write_text(json.dumps(cfg,indent=2))
    out=ROOT/'output/v3/candidates/monolithic';out.mkdir(parents=True,exist_ok=True)
    base=json.loads((ROOT/'config/mount_v0.json').read_text());c=base['carrier']
    L=c['length'];h=c['half_height'];i=c['inner_y'];o=c['outer_y']
    body=block(L,c['inner_wall'],2*h,(0,i+c['inner_wall']/2,0))
    body=body.union(block(L,c['outer_wall'],2*h,(0,o-c['outer_wall']/2,0)))
    for z in [-h+c['cross_wall']/2,h-c['cross_wall']/2]:
        body=body.union(block(L,o-i,c['cross_wall'],(0,(i+o)/2,z)))
    rr=rail(base['rail'])
    body=body.union(rr.translate((0,c['rail_center_y'],h)))
    body=body.union(rr.rotate((0,0,0),(1,0,0),180).translate((0,c['rail_center_y'],-h)))
    body=body.union(rr.rotate((0,0,0),(1,0,0),-90).translate((0,o,0)))
    body=body.union(block(4,16.2,28.2,(0,20,0))).translate((20,22,30))
    b=block(90,37,5,(10,9.5,-2.5)).union(block(90,8,55.1,(10,24,27.45)))
    for x in [-15,10,45]:
        plane=cq.Plane(origin=(x-2.5,0,0),xDir=(0,1,0),normal=(1,0,0))
        b=b.union(cq.Workplane(plane).polyline([(-7,-4.9),(28,-4.9),(28,-17),(20,-17)]).close().extrude(5))
    for x in [-26.5,26.5]:b=b.cut(zcyl(2.25,-30,31,x))
    part=b.union(body).clean()
    report=export_checked(part,'G1_head_tri_rail_monolithic_v3',out)
    old=cq.importers.importStep(str(ROOT/'output/v2/candidates/forward_balanced/bonded_analysis.step'))
    removed=old.cut(part).val().Volume();added=part.cut(old)
    assert removed<1e-5 and added.val().Volume()>0
    neighborhoods=None
    for x in [-5,45]:
        for z in [20,40]:
            neighborhood=cylinder(3.5,19.8,14.4,x,z)
            neighborhoods=neighborhood if neighborhoods is None else neighborhoods.union(neighborhood)
            probe=cylinder(1.65,20,14,x,z)
            assert abs(part.intersect(probe).val().Volume()-probe.val().Volume())<1e-5
    assert added.cut(neighborhoods).val().Volume()<1e-5
    oldbb=old.val().BoundingBox();newbb=part.val().BoundingBox()
    assert all(abs(getattr(oldbb,k)-getattr(newbb,k))<1e-5 for k in ['xmin','ymin','zmin','xmax','ymax','zmax'])
    assert part.intersect(block(140,29,70,(0,5.5,35))).val().Volume()<1e-5
    for x in [-26.5,26.5]:
        for probe in [zcyl(2.2,-30,45,x),zcyl(5,-35,30,x)]:
            assert part.intersect(probe).val().Volume()<1e-5
    report.update(parameters=p,total_volume_mm3=part.val().Volume(),ideal_solid_mass_g=part.val().Volume()*.00127,
        analysis_step=descriptor(out/'G1_head_tri_rail_monolithic_v3.step'),removed_from_v2_mm3=removed,
        filled_M3_void_volume_mm3=added.val().Volume(),envelope_preserved=True,robot_hole_and_tool_access_pass=True,
        nominal_head_keepout_pass=True,physical_ready=False,robot_fit_verified=False,camera_FOV_verified=False)
    (out/'geometry.json').write_text(json.dumps(report,indent=2))
    print(json.dumps(report,indent=2),flush=True)

if __name__=='__main__':
    code=0
    try:run()
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
