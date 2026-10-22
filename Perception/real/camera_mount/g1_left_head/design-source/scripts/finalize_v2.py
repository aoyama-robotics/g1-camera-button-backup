"""Publish only after matched fine-mesh comparison passes; render actual CAD/FEA."""
from pathlib import Path
import json,shutil,hashlib,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from cad_raster import render as raster
ROOT=Path(__file__).resolve().parents[1];OUT=ROOT/'output/v2'
def read(path):return json.loads(path.read_text())
def stl(path):
    data=path.read_bytes();dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')]);return np.frombuffer(data,dtype=dtype,offset=84)['v'].astype(float)
def add_mesh(ax,t,color):
    from matplotlib.colors import to_rgb
    n=np.cross(t[:,1]-t[:,0],t[:,2]-t[:,0]);n/=np.maximum(np.linalg.norm(n,axis=1)[:,None],1e-12)
    shade=.55+.45*np.maximum(n@np.array([.4,-.5,.768]),0)
    ax.add_collection3d(Poly3DCollection(t,facecolors=shade[:,None]*np.array(to_rgb(color)),edgecolors='none'))
def scene(ax,elev=20,azim=-50):
    ax.set(xlim=(-40,61),ylim=(-14,80),zlim=(-26,68));ax.set_box_aspect((101,94,94));ax.view_init(elev=elev,azim=azim);ax.set_axis_off()

def render(selection,folder):
    selected=selection['selected'];candidate=OUT/'candidates'/selected
    b=stl(candidate/'bracket.stl');c=stl(candidate/'carrier.stl')
    fig=plt.figure(figsize=(15,8),facecolor='#f4f7f9')
    ax=fig.add_subplot(131);ax.imshow(raster([(b,'#e3a24a'),(c,'#299ca1')],17,45));ax.set_axis_off();ax.set_title('FORWARD 20 mm | three rails',fontsize=13)
    ax=fig.add_subplot(132);ax.imshow(raster([(b,'#e3a24a')],-24,-55));ax.set_axis_off();ax.set_title('UNDERSIDE RIBS | same 5 mm seats',fontsize=13)
    ax=fig.add_subplot(133);ax.add_patch(Rectangle((-35,0),70,8,facecolor='#c4cbd1',edgecolor='#82919c'))
    ax.add_patch(Rectangle((-15,20),70,8,facecolor='#299ca1',edgecolor='#16616b'))
    ax.text(-35,10,'v1 rail',fontsize=12);ax.text(-15,30,'v2 rail',fontsize=12)
    ax.plot([35,35],[8,44],'--',color='#82919c');ax.plot([55,55],[28,44],'--',color='#16616b')
    ax.annotate('',xy=(35,41),xytext=(55,41),arrowprops=dict(arrowstyle='<->',color='#1c4c5f'));ax.text(45,44,'20 mm',ha='center',fontsize=13)
    ax.scatter([-26.5,26.5],[-12,-12],s=55,c='#b34e35');ax.plot([-26.5,26.5],[-12,-12],color='#b34e35');ax.text(0,-19,'53 mm screw pair unchanged',ha='center',fontsize=10)
    ax.set(xlim=(-41,63),ylim=(-24,52),xlabel='X (mm) → forward');ax.set_yticks([]);ax.set_title('RAIL POSITION | length 70 mm',fontsize=13);ax.set_aspect('equal');ax.grid(axis='x',alpha=.2)
    fig.text(.05,.065,'8 mm upright / 3 underside ribs / 4 mm internal web. 20 mm head-side clearance retained.',fontsize=12,color='#244c5e')
    fig.text(.05,.03,'Comparative FEA only — camera field of view and physical robot fit remain unverified.',fontsize=11,color='#964d25')
    fig.subplots_adjust(top=.94,bottom=.13,left=.01,right=.97,wspace=.13);fig.savefig(folder/'v2_preview.png',dpi=170);plt.close(fig)
    # All coarse candidates compared at the same mesh level; fine three-way comparison below.
    labels={'baseline_v1':'v1 original','forward_plain':'+20 mm only','forward_light':'Light ribs','forward_balanced':'Selected','forward_stiff':'Thickest'}
    rows=selection['coarse'];colors=['#a1adb5','#e8a356','#63a9b5','#218990','#577782']
    fig,axs=plt.subplots(1,2,figsize=(12,5))
    for ax,key,ylabel in [(axs[0],'max_displacement_mm','Maximum displacement (mm)'),(axs[1],'solid_mass_g','Solid-material mass estimate (g)')]:
        bars=ax.bar([labels[r['id']] for r in rows],[r[key] for r in rows],color=colors)
        for bar,r in zip(bars,rows):ax.text(bar.get_x()+bar.get_width()/2,bar.get_height(),f"{r[key]:.3f}" if key.startswith('max') else f"{r[key]:.1f}",ha='center',va='bottom',fontsize=10)
        ax.set_ylabel(ylabel);ax.set_ylim(0,max(r[key] for r in rows)*1.18);ax.tick_params(axis='x',labelsize=9);ax.grid(axis='y',alpha=.2);ax.set_axisbelow(True)
    fig.suptitle('Five-candidate screen | matched 5 mm target mesh / quadratic tetrahedra')
    fig.text(.06,.015,'0.5 kg payload + CAD self-mass, 3 g static-equivalent, E = 2000 MPa, ideal clamped seats and bonded joint.',fontsize=10)
    fig.tight_layout(rect=(0,.055,1,.95));fig.savefig(folder/'candidate_comparison.png',dpi=170);plt.close(fig)
    # Actual FE surface field, same loading / units / color scale for all three.
    names=['baseline_v1','forward_plain',selected];maxu=max(selection['fine'][n]['cases'][0]['max_displacement_mm'] for n in names)
    fig=plt.figure(figsize=(15,6));norm=Normalize(0,maxu);cmap=plt.get_cmap('viridis')
    for i,name in enumerate(names):
        d=np.load(OUT/f'fea/{name}_h3.5/fields.npz');pts=d['points_mm'];faces=d['surface_faces'];du=d['vertex_displacements_mm'][0]
        values=np.linalg.norm(du,axis=1);ax=fig.add_subplot(1,3,i+1,projection='3d')
        ax.add_collection3d(Poly3DCollection(pts[faces],facecolors=cmap(norm(values[faces].mean(1))),edgecolors='none'))
        fixed=d['fixed_nodes_mm'];ax.scatter(fixed[::5,0],fixed[::5,1],fixed[::5,2],s=2,c='#ed7346')
        scene(ax,18,-55);ax.set_title(f"{labels[name]}\nmax {selection['fine'][name]['cases'][0]['max_displacement_mm']:.3f} mm",fontsize=13)
    cax=fig.add_axes([.92,.20,.015,.6]);fig.colorbar(ScalarMappable(norm=norm,cmap=cmap),cax=cax,label='Displacement (mm)')
    fig.suptitle('Downward 3 g case | fine mesh | true undeformed geometry colored by computed displacement',fontsize=13)
    fig.text(.035,.04,'Orange: ideal clamped seat nodes. Deformation is not visually exaggerated. Bonded M3 interface; no physical load rating.',fontsize=10)
    fig.subplots_adjust(left=.01,right=.9,top=.86,bottom=.10,wspace=.03);fig.savefig(folder/'fea_displacement.png',dpi=160);plt.close(fig)

def run():
    s=read(OUT/'selection.json');assert s['status']=='COMPARATIVE_STIFFNESS_PASS_PHYSICAL_UNVERIFIED'
    name=s['selected'];folder=OUT/'selected';folder.mkdir(exist_ok=True)
    for source,target in [('bracket.step','L_bracket_v2.step'),('bracket.stl','L_bracket_v2.stl'),('carrier.step','tri_rail_carrier_v2.step'),('carrier.stl','tri_rail_carrier_v2.stl'),('assembly.step','assembly_v2.step'),('geometry.json','geometry_validation.json')]:
        shutil.copy2(OUT/'candidates'/name/source,folder/target)
    render(s,folder)
    fine=s['fine'];old=fine['baseline_v1'];plain=fine['forward_plain'];new=fine[name]
    oldmass=next(r['solid_mass_g'] for r in s['coarse'] if r['id']=='baseline_v1');plainmass=next(r['solid_mass_g'] for r in s['coarse'] if r['id']=='forward_plain');newmass=next(r['solid_mass_g'] for r in s['coarse'] if r['id']==name)
    reduction=(1-new['worst_displacement_mm']/plain['worst_displacement_mm'])*100;fromold=(1-new['worst_displacement_mm']/old['worst_displacement_mm'])*100
    lines=[
    '# G1 前方20 mm・剛性改善版 v2',
    '',
    '**ユーザー承認条件：前へ20 mm、搭載機器合計0.5 kg。有限個の補強候補から比較解析で選択した試作版です。**',
    '',
    'レール3本を前方20 mm移動し、L字の前側も延長しました。53 mmの取付穴間隔、ネジ中心から立ち上がり内面まで20 mm、ネジ座厚さ5 mmを維持しています。',
    '',
    '選択案は、立ち上がり厚を6→8 mm、底面に幅5 mm・最大深さ12 mmのリブを3本、レール支持箱に厚さ4 mmの仕切りを追加したものです。内側の頭部領域へ補強は追加していません。外側へは壁厚増加分2 mm広がります。底面リブは座面から最大17 mm下へ出るので、首元の実物との干渉は確認が必要です。',
    '',
    '## 同条件の細密メッシュ結果',
    '',
    '| 形状 | 最大変位 mm | 仮重心位置の変位 mm | 最大レール回転 ° | 体積重み付き99%応力 MPa | 局所ピーク応力 MPa | 中実換算重量 g |',
    '|---|---:|---:|---:|---:|---:|---:|',
    ]
    for label,r,mass in [('v1・旧位置',old,oldmass),('前方移動のみ',plain,plainmass),('v2・選択補強',new,newmass)]:
        lines.append(f"| {label} | {r['worst_displacement_mm']:.4f} | {r['worst_COM_displacement_mm']:.4f} | {max(c['rail_rotation_deg'] for c in r['cases']):.3f} | {r['worst_p99_vm_MPa']:.3f} | {max(c['peak_vm_MPa'] for c in r['cases']):.2f} | {mass:.1f} |")
    lines += ['',f"前方移動のみ比で最大変位 **{reduction:.1f}%減少**、旧v1比で **{fromold:.1f}%減少**。重量は前方移動のみ比で約{newmass-plainmass:.1f} g増。重量は密度1.27 g/cm³の完全中実換算で、実際のスライサー重量ではありません。",'',
              f"メッシュ目標寸法5→3.5 mmでの最大の方向別変位変化は **{max(r['max_case_displacement_change'] for r in s['two_level_checks'])*100:.2f}%**、改善比の変化は **{s['ratio_relative_change']*100:.2f}%**。2段階での一致を確認しましたが、完全な収束証明ではありません。",'',
              '局所ピーク応力も表に残しています。固定境界・穴・鋭角部に依存するため、99%応力だけを材料強度と比較した安全率判定は行っていません。実機・印刷品の許容重量は未設定です。','',
              '## 解析の意味','',
              '0.5 kgの偏心搭載＋各形状の自重に、3g相当の静的荷重を下・外・前・斜めの4方向へ加えました。E=2000 MPa、ν=0.35の等方中実樹脂モデルです。実際のフィラメント・積層・インフィルは未反映です。',
              'G1の2つの座面は完全固定、M3継手の接合面は完全接着相当です。ネジや頭頂プレートの柔らかさ・滑りを含まないため、実物より硬く評価し得ます。値は同じ仮定の下での形状比較であり、実機耐荷重や撮像振動の保証ではありません。',
              '計算の釣合い・残差・メッシュ体積を検査し、独立した片持ち梁の既知解でも単位と変位の尺度を確認しました。詳細は同梱の `output/v2/analysis_model.md` と `selection.json`。','',
              '## カメラの画角','',
              'レール前端は取付2穴の中点から前へ35→55 mm。レンズの位置はカメラ・クランプに依存します。公式頭部メッシュと実機ネジ位置の対応、画角、レンズ中心は未確定のため「頭が絶対に映らない」とは判定していません。前方に延長した側を実機前方へ向けて取り付け、実カメラ画像で確認する必要があります。','',
              '## 使うファイルと締結','',
              '- `assembly_v2.step`：確認用の組立。',
              '- `L_bracket_v2.stl` / `.step`：L字ブラケット。',
              '- `tri_rail_carrier_v2.stl` / `.step`：内部仕切りを追加したレール本体。',
              '- STLの単位はmm。組立座標なのでスライサーで向きとベッド接地を設定してください。G-codeは含みません。',
              '- 内部締結はM3×12キャップボルト4本＋M3ナット4個が候補。壁厚を2 mm増したため、前版のM3×10から変更します。部品の実寸・頭径・頭高さを確認してください。',
              '- G1側はM4想定を継続。ねじ座厚さは5 mmのため、元のねじ込み量を維持する長さの基準は前版と同じ「元の首下長さ＋5 mm」。ワッシャー、座面凹み、底付き余裕等により調整が必要です。','',
              '## 再生成','',
              '`build_v2.ps1` はCAD生成、比較解析、選抜、図と配布ZIP作成まで実行します。SANDRONEの `.deps` と `.deps_cae_precision` を読み取り専用で再利用します。前版のレールSTEPも同梱しています。',
              '本版は計算比較を終えた試作設計で、実機の嵌合、底面リブの首元との干渉、実材料、動的荷重、疲労・クリープ・層間強度は未確認です。']
    (folder/'README_v2.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    provenance={'selected':name,'status':s['status'],'physical_ready':False,'files':[]}
    for path in [ROOT/'config/structural_v2.json',ROOT/'scripts/fea_v2.py',ROOT/'scripts/build_structural_v2.py',OUT/'selection.json',OUT/'beam_check.json']+list(folder.glob('*.step'))+list(folder.glob('*.stl')):
        provenance['files'].append({'path':str(path.relative_to(ROOT)),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()})
    (folder/'provenance.json').write_text(json.dumps(provenance,indent=2))
    archive_path=ROOT/'output/G1_forward20_stiffness_v2.zip'
    files=[ROOT/'build_v2.ps1',ROOT/'config/structural_v2.json',OUT/'analysis_model.md',OUT/'selection.json',OUT/'candidate_manifest.json',OUT/'beam_check.json',ROOT/'output/v0/tri_rail_carrier_CONCEPT.step',ROOT/'output/v1/L_bracket_53mm_20mm_FIT_PROTOTYPE.step']
    files+=list(folder.glob('*'))+list((OUT/'fea').glob('*/result.json'))
    for name in ['build_mount.py','build_v1.py','build_structural_v2.py','fea_v2.py','run_fea_v2.py','check_elasticity_v2.py','select_v2.py','finalize_v2.py','cad_raster.py']:files.append(ROOT/'scripts'/name)
    with zipfile.ZipFile(archive_path,'w',zipfile.ZIP_DEFLATED) as z:
        for path in files:
            if path.is_file():z.write(path,path.relative_to(ROOT).as_posix())
    with zipfile.ZipFile(archive_path) as z:assert z.testzip() is None
    print(json.dumps(dict(selected=s['selected'],reduction_percent=reduction,old_reduction_percent=fromold,zip_bytes=archive_path.stat().st_size),indent=2))
if __name__=='__main__':run()
