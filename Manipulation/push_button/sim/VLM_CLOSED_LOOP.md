# RGB-DでG1のエレベーターボタンを押す最小試験

2026-10-09。既存 `push-button` ブランチのMuJoCo場面と7関節IKを再利用。
実行モデルは親が指定した **gpt-6.1-sol / high** のCodexセッション。
モデル名はセッション指定を記録したもので、実行バックエンドをAPIで自己照会した値ではない。
今回はastraを使用していない。

## 結果

| 試験 | 上ボタン最大押下 | 下ボタン最大押下 | 閾値以上での手先接触step | 判定 |
|---|---:|---:|---:|---|
| 通常位置、実画像判断5回 | 1.504 mm | 0 mm | 244 | 成功 |
| 横65mm・高さ25mm変動、実画像判断3回 | 1.511 mm | 0 mm | 105 | 成功 |
| 変動場面、元のRGB-D目標を固定した比較 | 0 mm | 0 mm | 0 | 失敗 |
| 通常位置、保存済み判断の再実行 | 1.504 mm | 0 mm | 244 | 成功 |

2msの物理step。成功は押下1.35〜1.65mm、別ボタン0.15mm未満、
押下閾値以上でTCP球との接触が合計50step以上。画像の色やVLMの成功宣言は判定に使わない。
上ボタンの黄色表示はこの物理判定後の表示。
最初の押下は0.996mmに留まり、再画像と接触/押下量フィードバックから0.7mm追加して成功した。

固定ベースの腕試験。G1の自由立位、歩行、実機、手指の精密接触は未検証。
手先接触は既存simの半径15mm球を使う。1場面と1位置変動だけで、一般性能は示さない。

## 観測と制御

MuJoCo Rendererの640×480 RGB PNGを `functions.view_image` で当該モデルへ渡す。
モデルが上向き矢印の円形面のbboxと次のphaseをJSONで決める。
bbox内部の登録済み深度とカメラ校正から `vision.estimate_button_target` が面中心・法線を計算し、
`kinematics.solve_ik` が7関節目標へ変換、PD actuatorと `mj_step` で実際に動かす。
次のRGBを生成し、モデルが続行・追加押込み・撤退を決める。
自己遮蔽後は初回RGB-D推定を記憶し、関節自己状態と物理フィードバックを使う。

検出にsegmentation、geom ID、真のボタン座標を使わない。校正カメラは位置変動に追従しない。
場面生成と独立評価だけがボタン位置/押下関節を持つ。scene.jsonは評価側の設定で、
モデル向けobservation.jsonには正解座標を含めない。
各decisionには実画像hash、モデル指定、画像入力経路、選択理由を保存する。
actions.jsonlには画像由来目標・IK誤差・行動・独立計測値を保存する。
これはCodexツールによる画像判断の実験であり、無人運用向けVLM APIサーバーは実装していない。

## ローカル再実行（PowerShell）

作業ルート: `C:\Users\nico1\Documents\Codex\2026-10-09\task-4`

```powershell
Set-Location 'C:\Users\nico1\Documents\Codex\2026-10-09\task-4'
$python = '.\.venv\Scripts\python.exe'
$sim = '.\G1_Hackason\Manipulation\push_button\sim'
& $python "$sim\replay_vlm_trial.py" --source evidence\nominal --out evidence\rerun_new --model menagerie\unitree_g1\g1.xml
```

出力先は新しいフォルダを指定する。これは保存済みVLM判断の再実行で、新しいモデルの評価ではない。
新規の視覚閉ループ試験では以下のinitでPNGを作り、その画像をモデルへ渡し、
観測hashとbbox/phaseを含むdecisionを作ってactを呼ぶ。actごとに新しいPNGを再観測する。

```powershell
& $python "$sim\run_vlm_closed_loop.py" init --out evidence\live_new --model menagerie\unitree_g1\g1.xml
& $python "$sim\run_vlm_closed_loop.py" act --out evidence\live_new --decision evidence\live_new\decision_00.json
& $python "$sim\run_vlm_closed_loop.py" video --out evidence\live_new
```

decisionのphaseはapproach/contact/press/retract/home。
初回bboxは[x0,y0,x1,y1]ピクセル。以降、明示的なoccluded_target_memory=trueで画像由来目標を保持できる。
press_extension_mは0〜2mmだけ。古い画像hashは拒否する。
trialのscene設定やtarget ground truthを視覚選択の入力に渡さない。

## 依存と出典固定

- G1_Hackason: `push-button`, `561c2d735f3da96d2e86e826e96f8ab2e026f275`
- ローカル作業ブランチ: `local/vlm-button-sim`（未push）
- google-deepmind/mujoco_menagerie: `0059d4335f8156206f63a35662313385f7ad6d74`, `unitree_g1/g1.xml` とassets
- Python 3.13.9、MuJoCo 3.3.7、NumPy 2.5.0、Pillow、imageio、imageio_ffmpeg、pytest
- 専用venvは既存base環境の画像・数値ライブラリを参照。追加インストールはvenv内のみ。

別PCでは上記commitのrepoとモデルを取得し、分離venvへ依存を入れて、
同梱evidence/nominalとreplayコマンドのパスを調整する。
モデル assets は納品ZIPに含めず、公式Menagerieの固定commitから取得する。
DDS、ROS、Unitree SDK、socket通信はこの実行経路で使わない。

## 関連テスト

RGB-D、押下量範囲、実モデルIK、独立成功条件、観測hashの結び付けを含む21テストが成功。

```powershell
$env:G1_MUJOCO_MODEL = (Resolve-Path menagerie\unitree_g1\g1.xml).Path
& $python -m pytest G1_Hackason\Manipulation\push_button\tests\test_vlm_closed_loop.py G1_Hackason\Manipulation\push_button\tests\test_vision.py G1_Hackason\Manipulation\push_button\tests\test_button_stroke.py G1_Hackason\Manipulation\push_button\tests\test_kinematics.py -q -p no:cacheprovider
```

動画は実stepで記録したPNGを10fps/H.264に符号化したもの。
通常125frame/12.5秒、変動83frame/8.3秒。生成動画や編集した擬似成功画像は使わない。
