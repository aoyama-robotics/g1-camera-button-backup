"""Orient existing validated parts, pack a single A1 mini plate, and reuse slicer."""
import json,os,sys,traceback
import cadquery as cq
from pathlib import Path
from src.generators.xl330_cradle import validate_binary_stl
ROOT=Path(__file__).resolve().parents[1]
def run():
    out=ROOT/'output/a1_mini_trial';out.mkdir(parents=True,exist_ok=True)
    entries=[
        ('head_mount',ROOT/'output/v3/G1_head_tri_rail_monolithic_v3.step',(0,1,0),90,(20,20)),
        ('camera_body',ROOT/'output/camera_attachment_v1/camera_clamp_body.step',(0,1,0),-90,(115,20)),
        ('camera_jaw',ROOT/'output/camera_attachment_v1/camera_clamp_jaw.step',(1,0,0),-90,(115,85))]
    parts=[];rows=[]
    for name,path,axis,angle,xy in entries:
        s=cq.importers.importStep(str(path)).val().rotate((0,0,0),axis,angle)
        b=s.BoundingBox();s=s.translate((xy[0]-b.xmin,xy[1]-b.ymin,-b.zmin));b=s.BoundingBox()
        assert 10<=b.xmin<b.xmax<=170 and 10<=b.ymin<b.ymax<=170 and abs(b.zmin)<1e-5 and b.zmax<180
        for other in parts:assert s.intersect(other).Volume()<1e-6
        parts.append(s)
        cq.exporters.export(s,str(out/f'{name}_oriented.stl'),tolerance=.035,angularTolerance=.08)
        rows.append(dict(name=name,rotation_axis=axis,rotation_deg=angle,bounds_mm=[b.xmin,b.ymin,b.zmin,b.xmax,b.ymax,b.zmax],source=str(path)))
    assembly=cq.Compound.makeCompound(parts)
    target=out/'G1_camera_trial_plate.stl';cq.exporters.export(assembly,str(target),tolerance=.035,angularTolerance=.08)
    check=validate_binary_stl(target)
    assert check['valid'] and check['closed_manifold'] and len(assembly.Solids())==3
    config=json.loads(Path('E:/sandrone/configs/bambu/slicing_support_brim.json').read_text())
    config.update(auto_arrange=False,auto_orient=False,ensure_on_bed=True)
    config['process_overrides'].update(wall_loops='4',sparse_infill_density='30%',sparse_infill_pattern='gyroid',top_shell_layers='5',bottom_shell_layers='5')
    (out/'slicing_trial.json').write_text(json.dumps(config,indent=2))
    (out/'orientation.json').write_text(json.dumps(dict(parts=rows,solid_count=3,stl_check=check,prototype_only=True),indent=2))
    print(json.dumps(rows,indent=2),flush=True)
if __name__=='__main__':
    code=0
    try:run()
    except BaseException:traceback.print_exc();code=1
    sys.stdout.flush();sys.stderr.flush();os._exit(code)
