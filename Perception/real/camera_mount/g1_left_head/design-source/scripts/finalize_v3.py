"""Package the monolithic CAD and matched-mesh comparison."""
from pathlib import Path
import json,shutil,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cad_raster import render
ROOT=Path(__file__).resolve().parents[1]
out=ROOT/'output/v3';source=out/'candidates/monolithic'
name='G1_head_tri_rail_monolithic_v3'
for suffix in ['step','stl']:shutil.copy2(source/f'{name}.{suffix}',out/f'{name}.{suffix}')
geo=json.loads((source/'geometry.json').read_text())
dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
tri=np.frombuffer((out/f'{name}.stl').read_bytes(),dtype=dtype,offset=84)['v'].astype(float)
fig,axes=plt.subplots(1,2,figsize=(13,6),facecolor='#f4f7f9')
for ax,e,a,title in zip(axes,[17,-24],[45,-55],['Outer view: three integrated rails','Underside / head side: two robot mounting holes']):
    ax.imshow(render([(tri,'#428a93')],elev=e,azim=a));ax.axis('off');ax.set_title(title,fontsize=12)
fig.suptitle('G1 v3 | One continuous part | No M3 assembly hardware',fontsize=17)
fig.text(.5,.06,'53 mm screw pitch   /   20 mm inner-face offset   /   Rails shifted 20 mm forward',ha='center',fontsize=12)
fig.text(.5,.02,'Exact STL geometry. Nominal keepout checked; actual head fit and camera field of view remain unverified.',ha='center',fontsize=10)
fig.tight_layout(rect=[0,.1,1,.94]);fig.savefig(out/'v3_preview.png',dpi=150);plt.close(fig)

fine=json.loads((out/'fea/monolithic_h3.5/result.json').read_text())
coarse=json.loads((out/'fea/monolithic_h5/result.json').read_text())
old=json.loads((ROOT/'output/v2/fea/forward_balanced_h3.5/result.json').read_text())
change=max(abs(a['max_displacement_mm']/b['max_displacement_mm']-1) for a,b in zip(coarse['cases'],fine['cases']))
comparison=dict(v2_bonded_max_displacement_mm=old['worst_displacement_mm'],v3_max_displacement_mm=fine['worst_displacement_mm'],
    reduction_fraction=1-fine['worst_displacement_mm']/old['worst_displacement_mm'],max_mesh_change_fraction=change,
    ideal_solid_mass_g=geo['ideal_solid_mass_g'],physical_ready=False)
assert change<.05
(out/'comparison.json').write_text(json.dumps(comparison,indent=2))
text=f'''# G1 左側頭部 3面レール 一体型 v3

L字マウントと3面レールを1部品にしました。接合用M3穴4か所、頭部座ぐり、ナットポケットを削除しています。組立用M3ネジ・ナットは不要です。ロボット固定用の下からの2穴だけを残しています。

- STEP: `{name}.step` / STL: `{name}.stl`（mm）
- 取付ピッチ53 mm、穴径4.5 mm（M4は未確認）、座厚5 mm。
- 穴中心から立ち上がり内面まで20 mm、レール長70 mm、前方移動20 mm。
- v2で選択した壁厚8 mm・底面リブ3本・中央仕切り4 mmを維持。
- 外形90 × 77 × 79 mm。中実・密度1.27 g/cm³で約{geo['ideal_solid_mass_g']:.1f} g。スライサー条件と実材料で重量は変わります。
- v2形状からの除去体積0 mm³、追加体積{geo['filled_M3_void_volume_mm3']:.2f} mm³。追加は旧M3穴周囲のみ、外形寸法は同一。

## 形状検査

単一の有効なB-Repソリッド、全シェル閉鎖、閉じたマニフォールドSTL、STEP再読込の体積一致を確認済み。2本のロボットネジ軸と工具用空間、および公称の頭側禁止領域への干渉なしを確認しました。実機形状との適合確認ではありません。

## 同条件での剛性再確認

搭載0.5 kg、3g相当の静荷重と部品自重の慣性力、4方向。等方中実樹脂 E=2000 MPa、ν=0.35、密度1270 kg/m³。2穴周辺の座面半径6 mmを理想固定。P2変位の四面体有限要素法を使用。

| 指標 | v2（接合面を完全接着と仮定） | v3（一体型） |
|---|---:|---:|
| 全荷重方向の最大変位 | {old['worst_displacement_mm']:.4f} mm | {fine['worst_displacement_mm']:.4f} mm |
| 遠隔重心位置の最大推定変位 | {old['worst_COM_displacement_mm']:.4f} mm | {fine['worst_COM_displacement_mm']:.4f} mm |
| 体積加重99%点 von Mises応力の最大値 | {old['worst_p99_vm_MPa']:.3f} MPa | {fine['worst_p99_vm_MPa']:.3f} MPa |

最大変位はv2比{comparison['reduction_fraction']*100:.1f}%減。5 mmと3.5 mmのメッシュで各方向の最大変位差は最大{change*100:.2f}%。残差、力・モーメント釣合い、メッシュ体積差を検査しています。v2も接合部を完全接着と仮定していたため、この差は主に穴を埋めた効果です。変位差はメッシュによる変動より小さく、剛性は前版と同等と解釈します。現物のM3接合をなくす効果を定量保証する比較ではありません。

実機の座面・ネジ規格と有効ねじ込み長、座面から17 mm下に出るリブと首元の干渉、カメラ画角は未確認です。印刷の積層方向・材料異方性・クリープ・衝撃・ネジ接触は未評価で、0.5 kgの実機許容荷重を保証しません。試作して取付・レールクランプ適合を確認してください。STLは形状データでありG-codeではありません。

## 再現

ワークスペース直下で `./build_v3.ps1`。SANDRONE既存のCadQuery・Gmsh・scikit-fem環境とSTL検証関数を読み取り利用。設定は `config/structural_v3.json` と `config/mount_v0.json`、CADは `scripts/build_v3.py`、解析は `scripts/fea_v2.py --version v3`。比較元v2の形状と結果が必要です。ZIPは更新部品と設定・コード・結果の引渡し用で、SANDRONE依存環境は含みません。
'''
(out/'README_v3.md').write_text(text,encoding='utf-8')
shutil.copy2(source/'geometry.json',out/'geometry_validation.json')
with zipfile.ZipFile(ROOT/'output/G1_monolithic_v3.zip','w',zipfile.ZIP_DEFLATED) as z:
    for f in out.rglob('*'):
        if f.is_file() and f.suffix not in ['.npz','.msh'] and 'candidates' not in f.parts:z.write(f,str(f.relative_to(ROOT)))
    for rel in ['config/structural_v3.json','config/structural_v2.json','config/mount_v0.json','scripts/build_v3.py','scripts/fea_v2.py','scripts/build_mount.py','scripts/build_v1.py','scripts/cad_raster.py','scripts/finalize_v3.py','build_v3.ps1','output/v2/fea/forward_balanced_h3.5/result.json']:
        z.write(ROOT/rel,rel)
print(json.dumps(comparison,indent=2))

