"""Deterministic finite-candidate selection and mesh comparison."""
from pathlib import Path
import json,argparse
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/v2'
def read(p):return json.loads(p.read_text())
def run():
    cfg=read(ROOT/'config/structural_v2.json');geo={r['parameters']['id']:r for r in read(OUT/'candidate_manifest.json')}
    rows=[];base=read(OUT/'fea/forward_plain_h5/result.json')
    for p in cfg['candidates']:
        name=p['id'];r=read(OUT/f'fea/{name}_h5/result.json')
        assert r['step_sha256']==geo[name]['analysis_step']['sha256'], 'Stale coarse CAD/FEA pairing'
        ratios={c['id']:c['max_displacement_mm']/b['max_displacement_mm'] for c,b in zip(r['cases'],base['cases'])}
        ratio=r['worst_displacement_mm']/base['worst_displacement_mm']
        rows.append(dict(id=name,volume_mm3=geo[name]['total_volume_mm3'],solid_mass_g=geo[name]['ideal_solid_mass_g'],
                         max_displacement_mm=r['worst_displacement_mm'],COM_displacement_mm=r['worst_COM_displacement_mm'],
                         p99_vm_MPa=r['worst_p99_vm_MPa'],peak_vm_MPa=max(c['peak_vm_MPa'] for c in r['cases']),
                         max_rotation_deg=max(c['rail_rotation_deg'] for c in r['cases']),ratio_to_forward_plain=ratio,
                         case_ratios=ratios,eligible=bool(p['shift']==20 and ratio<=.70 and max(ratios.values())<=1)))
    eligible=[r for r in rows if r['eligible']]
    assert eligible,'No candidate meets the stated selection gate; extend/revise search explicitly.'
    selected=min(eligible,key=lambda r:r['volume_mm3'])['id']
    report=dict(status='COARSE_SELECTION_PENDING_FINE_CHECK',selected=selected,selection_rule=cfg['selection_policy'],coarse=rows,physical_ready=False)
    fine={}
    for name in ['forward_plain','baseline_v1',selected]:
        path=OUT/f'fea/{name}_h3.5/result.json'
        if path.exists():
            result=read(path)
            if result['step_sha256']==geo[name]['analysis_step']['sha256']:
                fine[name]=result
    if len(fine)==3:
        basefine=fine['forward_plain'];chosen=fine[selected];convergence=[]
        for name,r in fine.items():
            coarse=read(OUT/f'fea/{name}_h5/result.json')
            percase={x['id']:abs(x['max_displacement_mm']-y['max_displacement_mm'])/y['max_displacement_mm'] for x,y in zip(coarse['cases'],r['cases'])}
            convergence.append(dict(id=name,max_case_displacement_change=max(percase.values()),case_changes=percase,
                                    max_p99_stress_change=abs(coarse['worst_p99_vm_MPa']-r['worst_p99_vm_MPa'])/r['worst_p99_vm_MPa']))
        cr=next(r['ratio_to_forward_plain'] for r in rows if r['id']==selected)
        fr=chosen['worst_displacement_mm']/basefine['worst_displacement_mm']
        ratio_change=abs(cr-fr)/fr
        no_worse=all(c['max_displacement_mm']<=b['max_displacement_mm'] for c,b in zip(chosen['cases'],basefine['cases']))
        agreement=max(r['max_case_displacement_change'] for r in convergence)<=.10 and ratio_change<=.10
        report.update(status='COMPARATIVE_STIFFNESS_PASS_PHYSICAL_UNVERIFIED' if agreement and fr<=.7 and no_worse else 'FINE_REVIEW_REQUIRED',
                      fine=fine,two_level_checks=convergence,ratio_relative_change=ratio_change,fine_ratio_to_forward_plain=fr,
                      two_level_displacement_agreement=agreement,fine_target_met=fr<=.7 and no_worse)
    (OUT/'selection.json').write_text(json.dumps(report,indent=2));print(report['status'],selected)
    return report
if __name__=='__main__':run()
