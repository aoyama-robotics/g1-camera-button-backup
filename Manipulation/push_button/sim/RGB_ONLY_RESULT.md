# 頭カメラRGBのみの最小試行（2026-10-09）

固定ベースG1で上ボタン押下に成功。**深度画像なし、外部視点なし、正解座標・sim segmentationなし**。
既知の機体FK、関節自己状態、カメラ取付校正、円形ボタンという形状仮定は使う。
sol6.1/highのCodexセッションが主観RGB `obs_00〜05.png` を実際に見て6つの行動判断を作成。
保存後の最終 `obs_06.png` は判断には使っていない。

## カメラの根拠と初期視野

実機向けrepo設定はD435i、torso_link基準xyz `[0.0576235,0.01753,0.42987]`m、
rpy `[0,0.8307767239493009,0]`rad（俯角47.6°）、640×480、垂直FOV42.6°。
xyz/rpyは公式Unitree URDFのd435_jointとも一致。FOVはrepoのfy約615pxによる近似。
repoには「実機日の較正で確認」と記載があり、実機の現在の取付・内部パラメータは未確認。

- [repo設定](https://github.com/airlayout/G1_Hackason/blob/e297a4308531fa4f39c1a6afb94c8f5d3702214f/Button_Press/J1-gen/configs/robot.yaml)
- [公式URDF](https://github.com/unitreerobotics/unitree_ros/blob/ccfc6fd8430a17ba3dacef9a1e2faf64ff3b0aee/robots/g1_description/g1_29dof_rev_1_0.urdf)

この設定を既存Menagerieのtorso_linkへ取り付けた初期画像では、パネルが右端で切れ、
上ボタンと手先は画面外。`evidence/rgb_only/obs_00.png` がその1枚。
胴体yaw −25°で上下ボタンが見えた。カメラ姿勢をソフトウェアで対象へ向けたのではなく、
固定ベースの実際のwaist_yaw関節をPD actuatorで動かし、取り付けたカメラが追従している。

以前の成功試験は別の仮想外部RGB-Dカメラ（世界位置[-0.10,-0.45,1.14]m、FOV60°、登録深度）であり、
頭部カメラの視野を検証するものではなかった。

## RGB-onlyの経路

1. 初期主観RGBで右端にパネルを認め、胴体を右25°、さらに右45°へ向ける。
2. 両主観RGBでVLMが上矢印のface ROIを選択。
3. ROI内のRGBの淡色面をellipse fitして中心を精緻化。これはRGB画像処理でありsimのgeom IDを使わない。
4. 関節FKによる2つのカメラ位置・校正rayから三角測量。カメラbaseline19.382mm、ray gap0.0813mm。
5. 円の2つの射影conicから面法線候補を求め、2視点の整合性で曖昧さを解く。法線一致角0.335°。
   ボタン実寸・半径・world poseはこの推定に使わない。
6. 推定面・FK自己状態からIK→PD→物理step→主観RGB。手先がずれた再画像から接近方向を修正。
7. 主観RGBの黄色表示を見て撤退。接触・押下量は試行終了後の独立評価にのみ使用。

初回IKは下げた手首姿勢の維持で失敗（物理stepなし）。次の水平接近では手先が目標からずれたため、
RGB円形面のconic法線を使うよう実装を修正し、再接近した。失敗もログに残す。
実行中にこの修正を行っており、最終コードによる全試行の再実行は行っていない。

## 独立評価

- 上ボタン最大押下: **1.505607mm**
- 下ボタン最大押下: **0mm**
- 上ボタンとTCP球の接触: 300step
- 1.35mm以上でのTCP接触: **81step**（合計162ms、2ms/step）
- 最大接触力: 2.028N
- 成功条件: 押下1.35〜1.65mm、下ボタン0.15mm未満、閾値以上のTCP接触合計50step以上
- 実行行動: 6（yaw2、接近2、押下1、撤退1）、上限8行動
- 主観動画: 640×480、10fps、186frame/18.6秒。実際の物理stepで記録したRGBのみ。

新規のconic法線復元の数学テスト1件成功。従来のRGB-D/IK/物理判定21テストは前試験で成功済み。

## 再利用

```powershell
Set-Location 'C:\Users\nico1\Documents\Codex\2026-10-09\task-4'
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\run_rgb_only.py init --out evidence\rgb_new --model menagerie\unitree_g1\g1.xml --camera-config evidence\head_camera_config.yaml
# 生成された頭部RGBをモデルへ渡してdecisionを作る。scene正解やmetricsはpolicyへ渡さない。
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\run_rgb_only.py act --out evidence\rgb_new --decision evidence\rgb_new\decision_00.json
# 必要な少数行動を行い、試行終了後だけ評価と動画化する。
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\run_rgb_only.py evaluate --out evidence\rgb_new
& .\.venv\Scripts\python.exe G1_Hackason\Manipulation\push_button\sim\run_rgb_only.py video --out evidence\rgb_new
```

`run_rgb_only.py` ではdepth rendering/registered depthを呼ばない。
RGB ellipse処理には既存環境のOpenCV4.13.0を使用。追加インストールなし。
DDS/ROS/実機通信なし。ユーザーの実カメラ画像による視野校正照合はまだ未実施。

これは校正を既知とした固定ベース・単一場面のRGB視覚デモ。
完全盲検VLMベンチマーク、RGB単眼1枚だけの距離推定、無人API運用、実機同等成功の主張はしない。
短い19mm baseline、円形面/色の仮定、実機の歪み・手先/取付ずれ・遮蔽への頑健性は未評価。
