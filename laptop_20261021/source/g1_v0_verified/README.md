# 最新版 v2 について

このZIPには実機送信可能なopt-inバックエンドを追加しています。
既定はdry-runですが、hardware_v2/g1_arm.py に --execute と確認済みprofileを指定し、対話enableすると実機へ送信します。
まず [hardware_v2/README.md](hardware_v2/README.md) を読んでください。現物/Windowsでの動作は未検証です。
以下はv1オフライン成果物の記録です。v2をoffline-onlyと解釈しないでください。

# G1: 前回のLLM判断を実機腕制御へつなぐためのオフライン接続試作

## 結論

VRテレオペは `Dev/Navigation` に見つかった。コンピュータから腕に指令するコードも既にある。
ただし、VR全体をコピーして起動するのではなく、**画像判断→深度/FK/IK→安全ゲート→腕の制御権**を分離する必要がある。
今回の成果物は **送信しないオフライン試作**。ロボットへ接続していない。インストール、DDS送信、push、mergeはしていない。

## 確認した元コード

- 全43リモートブランチのツリーを確認。
- VR: `Dev/Navigation` commit `7ccb66f9c0418b07f4db1c71a13b3a7ca14070c7`。
  `Teleop/vendor/g1-starter-kit/scripts/teleop.sh` が外部 `~/xr_teleoperate/teleop/teleop_hand_and_arm.py` を起動する。
  VR/IK本体はこのリポジトリに同梱されていない。キットは `yokoshaan/g1-starter-kit` の `45ead40` 由来。
- PC側の独立関節軌道制御: 同ブランチの `Teleop/vendor/g1-starter-kit/tools/replay_arm.py`。
  23/29DoF判定、50Hz、`rt/lowstate`、`rt/arm_sdk`、`rt/lowcmd`、制御重みランプ、状態通信断判定がある。
- ボタン用実機入口: `push-button` commit `74d384ad57f13d5d2c3924b8a1372533aab64b4b` の
  `Manipulation/push_button/real/push_button.py` と `arm_bridge.py`。`upstream/push_button` に未改変の参照コピー。
- 別系統の安全部品: `main` commit `81688f791dea282eb0c2cda16b458bc697d27a0e` の
  `Button_Press/J1-gen/common/arm/{backend_dds,commander,safety}.py`。`reference` に未改変で保存。
- 前回のLLM判断版はremote branch上ではなく、Oct10納品ZIPから回収。
  base `561c2d735f3da96d2e86e826e96f8ab2e026f275`、`local/vlm-button-sim` の未公開変更。
  `prior_oct10` の4ファイルはこの納品物から未改変でコピーした。ベースcommitのみではこの内容を復元できない。

参照:
- https://github.com/airlayout/G1_Hackason/blob/7ccb66f9c0418b07f4db1c71a13b3a7ca14070c7/Teleop/README.md
- https://github.com/airlayout/G1_Hackason/blob/74d384ad57f13d5d2c3924b8a1372533aab64b4b/Manipulation/push_button/real/push_button.py
- https://github.com/airlayout/G1_Hackason/blob/74d384ad57f13d5d2c3924b8a1372533aab64b4b/Manipulation/push_button/LOG/g1-arm-check-20261014/result.json

## 今回作ったもの

### dual_camera_preview.py

前回の `decision_*.json` と `obs_*.json`、RGB、深度をそのまま読み込む。
元 `camera_handoff.HandoffPolicy` と `vision.estimate_button_target` を呼び、判断と画像のSHA一致を確認する。
側頭部RGBからの導入角、主観RGB-Dからのボタン位置・法線、接近/押下/後退のCartesian目標を再計算する。
この入口にはSDK、DDS、socket、物理シミュレーション、腰/腕を動かす機能がない。

元の判断は **Codexが画像を見てJSONを作り、1行動ずつ進めた方式**。
自律的にVLM APIを呼び続けるループではない。今回も外部LLM/APIには接続していない。
`geometry.arm_motion_allowed` は元の幾何ゲートの判定であり、実機動作の許可ではない。
外側には常に `hardware_executable:false` を付ける。

### offline_adapter.py

実機側入力を想定した別途の境界案。LLMが直接関節角や3D距離を指示する形式は受け付けず、
同一主観RGB-D画像内の検出IDを選ぶ。既知の検出枠と深度から位置を計算する。
`build_joint_preview` は既存 `ArmKinematics` のFK/IK・経路衝突検査インターフェースにつなぐ。
右腕7軸だけの接近→接触→押下→後退→開始姿勢のプレビューを作る。
関節角、有限値、状態の鮮度、静止/直立、校正属性、速度、加速度、IK残差を確認し、経路検査失敗は伝播する。

この関節プレビューは**追加の保守的な設計案で、前回の4行動の逐語的再現ではない**。
今回の関節計画テストはfake IKを用いた境界検証。MuJoCo/モデル資産が無いため本物のIKや物理挙動は未検証。
出力schemaを既存実機runnerとは別にし、runnerの `load_plan` が拒否することもテストした。
`verified_on_robot` は入力申告に過ぎず、署名付き校正や人の現地確認を代替しない。

## 実行（オフラインのみ）

依存は Python、NumPy、Pillow。既存環境にあるものだけで今回検証した。

```sh
python dual_camera_preview.py --recording fixtures/dual_camera --output normal_replay.json
python -m unittest discover -s tests -v
```

関節計画案を本物のIKで計算する場合のみMuJoCoとMenagerieのローカルモデル資産が別途必要。
この納品物の検証のためにSDKやロボット接続を用意する必要はない。
`reference` と `upstream` は実機送信可能な元コードの**参照用コピー**。実行しないこと。

## 検証結果

- 27件の追加unittestに合格。詳細 `test_results.txt`。
- 前回記録4行動を読み、画像→深度幾何再計算に成功。`normal_replay.json`。
- 同じ記録の接近判断へ深度欠落を注入すると `HOLD_REACQUIRE` → `STOP`、復帰せず拒否。`dropout_replay.json`。
  これは新しいオフライン故障注入検証で、元 `dual_depth_dropout` ディレクトリの直接再実行ではない。
- 異なる画像SHA、誤ラベル、RGB鮮度、深度時刻ずれ、カメラ間時刻ずれ、不正外部行列、NaN、8行動上限を検査。
- SDK/MuJoCo/CycloneDDS importとsocket生成を禁止したテストでも4行動再計算が通る。
- 送信、実機接続、物理シミュレーションは未実施。27件はリポジトリ全体のテスト合格を意味しない。

## 実機化の前に残ること

1. 実際のPC上にある `xr_teleoperate` の版とパッチ状態、G1の29DoF/モータ構成を読み取りで確認する。
2. 側頭部RGBと主観RGB-Dの取り付け位置、内部パラメータ、歪み、外部行列、時刻同期を校正する。
   前回は側頭部カメラの仮置き・公称頭部位置。現物の校正ではない。
3. 元simの `align_head` は**腰yawを動かす**。腕だけの実装へそのまま移すと視野導入が成立しない。
   初期試験では操作者が主観RGB-Dの視野へボタンを入れるか、腰制御を別途設計・検証する。
4. 座標基準を揃える。Oct10/Menagerie側は床基準world座標、J1-gen側はpelvis基準。
   床高さをそのままpelvis入力に使わず、時刻付きFKと実測骨盤高/姿勢で変換する。
5. 関節名前順で対応付ける。29DoF右腕はmotor 22..28、左腕は15..21、weightは29。
   動作中の歩行コントローラへ `rt/arm_sdk` を使い、モード変更/lowcmd自動フォールバックを追加しない。
6. 実機runner/bridgeには接続前に安全レビューが必要。元コードのmode_machine/motor mode設定、
   腰保持の必要性、制御権獲得/返却、送信周期、制御プロセス競合、タイムアウトを確認する。
7. 手先形状、到達/自己衝突/パネル面、停止状態の継続、接触力/過押し、非常停止、監視者を含む段階試験を行う。
   最終成功は指令完了ではなくボタンの点灯/応答で確認する。

Oct14ログでは右肩の小動作が「静止・直立ガード」で中断され、指令軌道完走と動作振幅は未確認だった。
ガード原因の測定値も記録されておらず、現在の実機腕制御を検証済みとして扱えない。
今回の結果は実機への即時展開許可や、安全認証ではない。

追加境界: 27DoFなど29DoF以外の機体は対象外。機体ファームウェアの現在版は未確認。
future timestamp、retractのRGB-D鮮度も拒否条件に追加した。retractのTCPには独立timestampが無いため、
記録observation内の対応しか確認できない。実機では時刻付き状態snapshotが必要。
