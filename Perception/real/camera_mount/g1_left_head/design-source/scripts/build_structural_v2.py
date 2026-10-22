"""Parametric candidates preserving measured screw pitch and head-side keepout."""
from pathlib import Path
import json,os,sys,traceback
import cadquery as cq
from build_mount import block,cylinder,export_checked,descriptor
from build_v1 import zcyl
ROOT=Path(__file__).resolve().parents[1]

def build_candidate(p):
    shift=p['shift'];w=p['wall'];end=35+shift;length=70+shift
    b=block(length,29+w,5,(shift/2,(20+w-9)/2,-2.5))
    b=b.union(block(length,w,55.1,(shift/2,20+w/2,27.45)))
    if p['rib_width']:
        for x in [-15,10,45]:
            # Ribs wholly below seat; no reinforcement intrudes into the head.
            plane=cq.Plane(origin=(x-p['rib_width']/2,0,0),xDir=(0,1,0),normal=(1,0,0))
            yz=[(-7,-4.9),(20+w,-4.9),(20+w,-5-p['rib_depth']),(20,-5-p['rib_depth'])]
            rib=cq.Workplane(plane).polyline(yz).close().extrude(p['rib_width'])
            b=b.union(rib)
    for x in [-26.5,26.5]:b=b.cut(zcyl(2.25,-30,31,x))
    for x in [shift-25,shift+25]:
        for z in [20,40]:
            b=b.cut(cylinder(1.7,19.9,w+.2,x,z)).cut(cylinder(3.2,19.9,3.3,x,z))
    c=cq.importers.importStep(str(ROOT/'output/v0/tri_rail_carrier_CONCEPT.step'))
    if p['bulkhead']:
        # A transverse web in the hollow rail box, away from the four nut pockets.
        c=c.union(block(p['bulkhead'],16.2,28.2,(0,20,0)))
    c=c.translate((shift,14+w,30))
    return b,c

def run():
    cfg=json.loads((ROOT/'config/structural_v2.json').read_text());out=ROOT/'output/v2';out.mkdir(exist_ok=True)
    rows=[]
    for p in cfg['candidates']:
        folder=out/'candidates'/p['id'];folder.mkdir(parents=True,exist_ok=True)
        b,c=build_candidate(p)
        if p['id']=='baseline_v1':
            old=cq.importers.importStep(str(ROOT/'output/v1/L_bracket_53mm_20mm_FIT_PROTOTYPE.step'))
            assert abs(old.val().Volume()-b.val().Volume())<.001
            assert old.cut(b).val().Volume()<1e-5 and b.cut(old).val().Volume()<1e-5
        parts={'bracket':export_checked(b,'bracket',folder),'carrier':export_checked(c,'carrier',folder)}
        overlap=b.intersect(c).val().Volume();assert overlap<1e-5
        reserved=block(140,29,70,(0,5.5,35));assert b.intersect(reserved).val().Volume()<1e-5
        for x in [-26.5,26.5]:
            for probe in [zcyl(2.2,-30,45,x),zcyl(5,-35,30,x)]:
                assert b.intersect(probe).val().Volume()<1e-5
                assert c.intersect(probe).val().Volume()<1e-5
        for x in [p['shift']-25,p['shift']+25]:
            for z in [20,40]:
                probe=cylinder(1.65,19.9,p['wall']+6.2,x,z)
                assert b.intersect(probe).val().Volume()<1e-5 and c.intersect(probe).val().Volume()<1e-5
        bonded=b.union(c).clean()
        # FEA-only ideal bond uses actual coincident mating faces, no filler volume.
        assert len(bonded.solids().vals())==1 and bonded.val().isValid()
        assert abs(bonded.val().Volume()-b.val().Volume()-c.val().Volume())<.001
        cq.exporters.export(bonded,str(folder/'bonded_analysis.step'))
        cq.exporters.export(bonded,str(folder/'bonded_analysis.stl'),tolerance=.035,angularTolerance=.08)
        assembly=cq.Assembly(name=p['id']);assembly.add(b,name='L_bracket',color=cq.Color(.87,.58,.2));assembly.add(c,name='rails',color=cq.Color(.12,.55,.57))
        assembly.export(str(folder/'assembly.step'))
        row=dict(parameters=p,parts=parts,total_volume_mm3=bonded.val().Volume(),ideal_solid_mass_g=bonded.val().Volume()*.00127,
                 geometry_checks_pass=True,head_keepout_pass=True,screw_and_tool_access_pass=True,robot_fit_verified=False,
                 analysis_step=descriptor(folder/'bonded_analysis.step'))
        (folder/'geometry.json').write_text(json.dumps(row,indent=2));rows.append(row)
        print(p['id'],round(row['total_volume_mm3'],1),'mm3',flush=True)
    (out/'candidate_manifest.json').write_text(json.dumps(rows,indent=2))

if __name__=='__main__':
    code=0
    try:run()
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
