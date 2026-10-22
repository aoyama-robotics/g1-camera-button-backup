from pathlib import Path
import json,zipfile
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from cad_raster import render
ROOT=Path(__file__).resolve().parents[1];out=ROOT/'output/camera_attachment_v1'
dtype=np.dtype([('n','<f4',3),('v','<f4',(3,3)),('a','<u2')])
def mesh(p):return np.frombuffer(p.read_bytes(),dtype=dtype,offset=84)['v'].astype(float)
body=mesh(out/'camera_clamp_body.stl');jaw=mesh(out/'camera_clamp_jaw.stl')
parent=mesh(ROOT/'output/v3/G1_head_tri_rail_monolithic_v3.stl')
inplace=[(parent,'#727e85'),(mesh(out/'camera_clamp_body_assembly_coordinates.stl'),'#229eac'),(mesh(out/'camera_clamp_jaw_assembly_coordinates.stl'),'#eba23d')]
fig,axes=plt.subplots(1,3,figsize=(18,6),facecolor='#f4f7f9')
for ax,parts,e,a,title in [
    (axes[0],inplace,18,35,'On existing v3 | lens faces +X'),
    (axes[1],[(body,'#229eac'),(jaw+np.array([0,9,0]),'#eba23d')],24,40,'Camera side | four M2 adjustment slots'),
    (axes[2],[(body,'#229eac'),(jaw+np.array([0,9,0]),'#eba23d')],-25,140,'Rail side | two keys + detachable jaw')]:
    ax.imshow(render(parts,elev=e,azim=a,width=700,height=700));ax.axis('off');ax.set_title(title,fontsize=12)
fig.suptitle('U20CAM-1080P attachment | Rigid forward-facing frame + keyed two-bolt clamp',fontsize=17)
fig.text(.5,.055,'Teal: body / Orange: removable jaw / Grey: existing mount. Fasteners and camera omitted for clarity.',ha='center',fontsize=11)
fig.text(.5,.02,'PROTOTYPE: camera hole pitch and real printed fit are not yet verified. No physical slip or load rating.',ha='center',fontsize=11)
fig.tight_layout(rect=[0,.1,1,.94]);fig.savefig(out/'attachment_preview.png',dpi=140);plt.close(fig)
v=json.loads((out/'validation.json').read_text());mass=sum(p['solid_PETG_mass_g_at_1_27'] for p in v['parts'].values())
readme=f'''# InnoMaker U20CAM-1080P レールアタッチメント v1

対象はユーザー指定Amazon ASIN B0CNCSFQC1の裸基板カメラ。G1左側頭部のv3外側レールの前端へ取り付け、レンズをG1前方へ向ける固定角の試作部品です。カメラ側寸法に未確定事項が残るため、現物合わせ前の試作データです。

## 固定構造

1. レールの斜面を固定側と着脱側の顎で挟み、M4ボルト2本を交互に締める。締結には金属ナットを用い、樹脂に直接ねじを切らない。
2. 長さ32 mmのクランプ内に、レールの溝へ入る突起を2か所配置。中心間隔20.0152 mm、突起幅5.05 mm。溝幅5.334 mmとの差0.284 mmは印刷・組付け用の逃げ。
3. 前端のM3調整ネジでレール端面を軽く押し、両突起を同じ溝壁へ寄せてからM4を締める。M3は位置決め用であり、強く締めてレールを押し潰さない。溝突起はずれ止めの機械的な受けとなる。
4. カメラ支持フレームと固定顎を一体化。両側のリブで支え、角度調整ヒンジを設けない。基板は四隅をM2で固定。ケーブルタイ用の耳にUSBケーブルを固定し、短い余長を基板との間に残す。

CAD上の追加樹脂体積からの中実換算重量は約{mass:.1f} g（密度1.27 g/cm³、金具とカメラは含まない）。カメラ基板裏面はローカルX=46.1 mm、レール前端X=35 mmより11.1 mm前方。実際のレンズ位置とG1頭部の登録座標は未確定なので、映り込み解消は未確認。

## 確認済み仕様と未確定事項

- メーカー資料：基板32×32 mm、取付穴4か所・直径2.2 mm、M12レンズ。メーカーPDFでは水平画角が103°と106°の両方で記載されており、画角保証には使わない。
- 穴ピッチ・基板厚・背面部品の詳細寸法はメーカー資料に記載がない。四隅の穴を結ぶ正方形ピッチ26～30 mmに対応する斜め長穴を設けたが、これは調整範囲であり実機のピッチを断定したものではない。
- 基板を載せる4つの支持面は同一平面。周囲のフレーム面との間隔4.1 mm、中央は一部を除いて開放。前後調整ネジのハウジング付近では裏面空間が最小1.6 mmになるため、この位置の部品高さも現物確認する。USBコネクタを開口側へ向ける。基板上の部品に支持面やネジ頭が当たる場合は取付不可、寸法修正が必要。
- 裸基板専用。別売金属ケースを付けた状態には対応していない。
- 最終確定には穴中心間隔（横×縦）、基板厚、裏面部品とコネクタの干渉確認が必要。未確認寸法はconfigでnullのまま保存。

## 部品と金具

| 部品 | 数 | 備考 |
|---|---:|---|
| camera_clamp_body.stl | 1 | 顎とカメラ支持フレームの一体部品 |
| camera_clamp_jaw.stl | 1 | 着脱側の顎 |
| M4×40 六角穴付きボルト | 2 | クランプ用、首下40 mm |
| M4六角ナット | 2 | 二面幅7 mm・厚さ約3.2 mmの形状を想定、7.2 mm幅のポケットに捕捉 |
| M4平ワッシャー | 2 | 頭側、外径9 mm以下・厚さ約1 mmを想定 |
| M3×6 六角穴付きボルト | 1 | 前後位置決め用。頭径5.5 mm・高さ3 mmの包絡で検査 |
| M3六角ナット | 1 | 二面幅5.5 mm、厚さ約2.4 mm。下から挿入 |
| M2ボルト、ナット、平ワッシャー | 各4組 | 長さは基板厚確認後に確定。支持厚9.1 mm＋基板厚＋ワッシャー＋ナット＋必要な突出を合計。基板厚1.6 mmの場合は首下14 mmが仮候補 |
| 小型ケーブルタイ | 1～2 | 幅2.5 mm以下。ケーブルを保持しコネクタを引かない |

M2裏側ワッシャーは外径5 mm以下、基板側は部品と導体に接触しない寸法を選ぶ。穴部以外を締め付けて基板を曲げない。穴ピッチを確認できた場合は長穴を丸穴に変更して位置再現性を上げられる。

## 組立手順

1. バリを除去し、M4ナット2個とM3ナット1個をポケットへ入れる。M3は下面の開口から入れる。
2. 固定側をv3外側レール前端へ掛け、突起2個を対応する溝へ落とす。着脱側顎を外側から合わせてM4ボルトを緩く入れる。
3. カメラをまだ付けない状態で、前からM3を回して端面に接触させ、突起を前方の溝壁へ軽く寄せる。
4. M4を左右交互に締め、顎でレールを保持する。樹脂レールのため金属用の一般的な締付トルクを流用しない。顎の隙間が先に閉じたり白化・割れが出たりしたら使用を止め、嵌合を修正する。
5. 基板を4支持面へ載せて同一の辺を位置決め面へ寄せ、M2で均等に固定。ねじ頭・支持面・ワッシャーが電子部品に当たらないことを確認。
6. ケーブルに余長を作り、結束バンドで支持側へ固定する。手で軽く各方向に負荷を掛け、ずれ・ガタがないことを確認してから低速動作で評価する。

## 検証の範囲

各印刷部品は1つの有効なソリッド、閉じたSTL、STEP再読込一致を検証。v3親マウントとの体積干渉0、クランプの両顎が片側0.15 mmずつ閉じた位置で干渉なし、さらに0.04 mm閉じるとレールに接触することを検査。突起の前後位置決め、金具の包絡、カメラ側26/28/30 mmピッチでのM2軸と裏ワッシャーの空間を検査済み。

実物の摩擦・予圧・積層強度・繰り返し荷重は未検証。今回の検査は接触や締結を含むFEAではなく、滑り荷重や画像の無振動を保証しない。カメラの画角・電子シャッター、既存頭部マウント自体のたわみも画像のブレに影響する。

## 出力と再現

STLはmm単位。`assembly_on_v3.step`は取付位置確認用で、灰色の既存v3は再印刷不要。`*_assembly_coordinates.stl`は位置確認用、印刷用には通常のbody/jaw STLを使う。

既存環境で `build_camera_attachment.ps1` を実行。既存v3のSTEP/STLとSANDRONEのCadQuery/STL検証環境が必要。config・CAD生成・結果を分離して保存。

資料：[指定商品](https://www.amazon.co.jp/dp/B0CNCSFQC1)、[メーカー資料](https://github.com/INNO-MAKER/U20CAM-1080P-S1)。PDFのハッシュはvalidation.json。
'''
(out/'README_ja.md').write_text(readme,encoding='utf-8')
with zipfile.ZipFile(ROOT/'output/G1_U20CAM_rail_attachment_v1.zip','w',zipfile.ZIP_DEFLATED) as z:
    for f in out.glob('*'):
        if f.is_file():z.write(f,str(f.relative_to(ROOT)))
    for rel in ['config/camera_attachment_v1.json','config/mount_v0.json','scripts/build_camera_attachment.py','scripts/build_mount.py','scripts/cad_raster.py','scripts/finalize_camera_attachment.py','build_camera_attachment.ps1','sources/U20CAM-1080P-S1_UserManual-v1.1.pdf']:
        z.write(ROOT/rel,rel)
print('Packaged prototype; mass_g',mass)
