# G1 腕だけ実験パッケージ v2

今回はpreviewだけでなく、**明示的に有効化した場合にrt/arm_sdkへ送信するバックエンド**を追加した。
実機、Windows実機、WSL実機ではまだ実行していない。全ての検証はクラウド上のmock/offline。
既定の `nudge` はDDSを初期化せず、腕も動かさない。実機実行には下記の段階を踏む。

## 対象と実装済み範囲

- G1 29DoF、unitree_hg LowCmd/LowState、expected mode_machine=5のみ。
- 腕motor15..28と制御重みslot29のみ。片腕の小動作中は反対の腕を開始時の実測姿勢で保持する。
- `rt/arm_sdk` のみ。lowcmd・脚・腰・歩行・Damp・ReleaseMode・デバッグモード切替は実装しない。
- 読み取り状態確認、default dry-run、1関節の小角度往復、review済みの小範囲関節plan実行。
- 元Oct10画像判断/深度/IK成果物をそのまま別層として残す。LLM判断を直接モータへ渡さない。

## まずWindows laptopで行う手順

### 0. ソフトウェアのみ

初期対象はPython 3.10 x64。Windows ARM64環境は確認していない。
既にUnitree SDK2 Pythonが使える環境があれば再利用する。SDK導入は公式手順を参照。
このパッケージはロボットやOSへ自動インストールせず、ネットワーク/Firewall設定も変更しない。

- SDK: https://github.com/unitreerobotics/unitree_sdk2_python
- SDK依存: https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/setup.py
- CycloneDDS 0.10.2: https://pypi.org/project/cyclonedds/0.10.2/

`requirements-sdk.txt` は依存一覧であり、SDK自体は公式checkoutから別途導入する。
CPython3.10向けWindows x86-64 CycloneDDS wheelは存在する。SDK CRCにも非Linux用Python実装がある。
ただしXRの公式検証OSはUbuntu20.04/22.04で、本コードのWindows実機動作を保証するものではない。
VR、Quest、ROS、Pinocchioは、小角度nudge実験には不要。

```powershell
cd <展開先>\hardware_v2
python g1_arm.py preflight
python -m unittest discover -s tests -v
python g1_arm.py nudge --joint 22 --delta-deg -1
```

`preflight.ps1` も上記preflightとdry-runだけを行う。PowerShellの実行ポリシーを変える必要はなく、
スクリプトがブロックされれば上記pythonコマンドを直接実行できる。
`preflight` はimport/NIC一覧のみ、DDS未初期化。SDKが入っていなければその旨を表示する。

### 1. 有線接続後、状態を読むだけ

SDKから見えるNIC名を明示する。Windows表示名とDDSが受け付けるNIC識別子が一致するかは現地確認する。
他の送信プログラムを起動しない。ネットワークは機体の既存設定を参照し、推測でIPを変更しない。

```powershell
python g1_arm.py state --connect --interface "<実際のNIC名>" --output robot_state.json
```

このコマンドはLowState購読と読み取りCheckModeのみ。arm_sdk publisherを生成しない。
CheckMode自体はSDK RPCの問い合わせを使うため、完全なネットワーク無送信ではない。
8秒以内に状態/稼働モードが取れなければ停止。モードを自動変更しない。
停止中のコントローラやRPC失敗は、lowcmdへ切り替える理由にはしない。

### 2. 実機profileを現地で確認

`profile.example.json` を `profile.local.json` にコピーして、実際の機体情報を記入する。
テンプレートは意図的に実行不可。特に次を未確認のままtrueにしない。

- firmware_version / robot_id / CheckModeで得たmotion_mode_name。
- **当該firmwareのarm_sdkが、腕以外の中立slotを適切に無視し、腰/脚の制御を奪わないこと**。
  ベンダー仕様または現地で既に確認した情報を `overlay_evidence` に残す。
- 腰の保持をこのプログラムに依存しない外部支持、退避空間、監視者、物理的な非常停止手段。
- VR、別のarm_sdk送信者、Navigation等の競合する動作指令が停止していること。

robot_id / firmwareは操作者申告で、接続先の機体IDを自動照合する機能ではない。
`mode_machine=5` と14腕モータのmode/通電状態はソフトウェアで検査するが、機体ID証明ではない。
未知firmwareで「非腕0フィールドなら安全」と断定して実行しない。

### 3. 送信前のdry-run

```powershell
python g1_arm.py nudge --snapshot robot_state.json --joint 22 --delta-deg -1
```

ここでは関節範囲と予定角を表示するのみ。保存snapshotはライブ状態ではない。
実行時は必ず新しいLowStateから開始姿勢を取り直す。

### 4. 現地の操作者が小角度実験を開始

```powershell
python g1_arm.py nudge --snapshot robot_state.json --joint 22 --delta-deg -1 --execute --interface "<実際のNIC名>" --profile profile.local.json --log first_nudge.json
```

profileと警告を表示し、対話端末で `ENABLE ARMS` を入力した場合だけ接続する。
新しい状態と稼働モード、0.5秒の静止を確認後に初めてarm_sdk publisherを生成する。
ゼロ姿勢へ飛ばさず実測姿勢を保持し、weightを2秒で上げる。
右肩1度を3秒で動かし、0.3秒保持、3秒で実測開始姿勢に戻し、weightを2秒で解除する。
最大指定は±3度。可動範囲、人・物との干渉、正負方向を現地で確認してから行う。
成功ログは指令軌道完了であり、実際の腕が正しく動いた証明ではない。必ず目視確認する。

## 停止と安全境界

- Enter / Ctrl+C / SIGTERMで軌道を中断し、制御重み0の送信を試みる。停止後に自動帰還軌道を走らせない。
- LowState0.2秒、CheckMode0.75秒で古い状態を拒否。モード変更/RPC失敗/NaN/無効モータも停止。
- LowState.tickが進まない再配信は鮮度を更新しない。32bit wrapを扱う。
- roll/pitch >0.10rad、gyro >0.05rad/s、脚/腰速度 >0.05rad/s、腕追従誤差 >0.15radで停止。
- 始点から最大5度、関節速度0.1rad/s、補間加速度0.25rad/s²。小実験用であって一般ボタン接近には狭い。
- XRと同じ250Hzを上限として使う。deadlineから50ms以上遅れると停止。遅延後のcatch-up連投はしない。
- 状態異常では高weightで古い姿勢を保持し続けず、最後の有効ヘッダ/腕目標でweight0を一度試す。
- DDS WriteがFalseなら失敗。解除失敗もログに残し、接続の後片付けを試みる。
- **ソフト停止は物理E-stopではない**。ケーブル断、OS停止、電源断、process強制終了では解除指令の受信を保証できない。
- 通信監視と送信周期はPython/OS依存。Windows通常スケジューラにリアルタイム保証はない。

## VR側への準拠と意図した差分

確認した公式コード:
https://github.com/unitreerobotics/xr_teleoperate/blob/817fb00c63cde15e5f24a0f8fa08e1e33ed89d3b/teleop/robot_control/robot_arm.py

G1_29_ArmControllerのトピック、unitree_hg型、mode_pr=0、受信mode_machine、CRC、腕motor番号、
weight29、250Hz、肩肘kp80/kd3・手首kp40/kd1.5を参照した。該当ソース抜粋 `XR_PROTOCOL_REFERENCE.txt`。

リポジトリ内のVRラッパーは `Dev/Navigation` commit `7ccb66f9...`。
キットは `g1-starter-kit 45ead40` を取り込み済みだが、install_env.shはxr_teleoperateとSDKのcommitをpinせずcloneする。
NumPy1.26.4、Pinocchio3.1.0、CycloneDDS0.10.2は固定。現物PCのXR checkoutとローカルパッチ状態は未読取。
したがって「今の現物VRとbit単位で同じコード」ではない。

意図した変更:
1. 元VRは脚腰を含む全関節へ実測角保持を設定する。本実装は腕だけをmode1とし、非腕は中立0。
   weight slot29はqのみ設定する。非腕が無視されることを現物firmwareで確認するprofileが必須。
2. lowcmdへの経路なし。元kitのRPC失敗が空mode→lowcmdとなる自動選択を採用しない。
3. 元VRの初期targetゼロ/高速度上限や追従開始時30rad/s復帰を採用せず、実測始点と小角度低速。
4. ゼロ姿勢へのgo_homeは呼ばず、nudgeのみ正常完了時に開始実測姿勢へ戻る。
5. 手/指や腰yawによる視野導入は未実装。前回simのalign_headは使わず、操作者が対象を主観カメラに入れる。

## LLM/ターゲット接続

元の `dual_camera_preview.py`、`offline_adapter.py` は親ディレクトリにある。
前回と同じ画像判断JSON→画像SHA確認→RGB-D計測→IKのオフライン境界を再利用できる。
`target_bridge.py` は、そこで作ったjoint previewと状態snapshotを **review用draft** に変換する。
画像判断を自動でmotor planへ変換し続ける自律ループでも、ライブカメラ接続でもない。

```powershell
python target_bridge.py --joint-preview <joint_preview.json> --snapshot robot_state.json --profile profile.local.json --output arm_plan.draft.json
python g1_arm.py run-plan --plan <review済みplan.json> --profile profile.local.json
```

draftはstatus=DRAFT_NOT_EXECUTABLE、reviewed_by空、collision_checked=falseなので、そのままでは実行不可。
実際にレビューした後だけstatusをREVIEWEDへ変更し、reviewerとcollision checkを記録する。
元previewとsnapshotに署名付き共通IDはないため、異なる試験/校正/機体のファイルを混ぜない。
現物のカメラ校正、床world/pelvis基準、手先形状、IK・衝突検査を確認した操作者がreviewする。
run-planは5度の小範囲に限定し、最後が始点に戻ること、最大8区間/40秒、ライブ始点との0.02rad以内一致を確認する。
元simの大きな腕接近をそのまま渡すと拒否される。大動作のために設定を緩めて実機へ直送しない。
review済み小範囲planのみ、nudgeと同じ `--execute --interface ... --profile ...` と対話enableで送信できる。

## Windows / WSL

Windows nativeのSDK importとCRCは可能性があるが現物未検証。最初に上記preflightとstateを通す。
WSL2既定NATでDDS multicastが届くとは仮定しない。Windows11 22H2+のmirrored networkingは
multicast/LANに対応するが、Windows/Hyper-V firewallと有線NIC経路は別途確認する。
Firewall全面無効化やセキュリティ設定の自動変更はしない。ネットワーク変更が必要なら操作者が確認して行う。

- https://github.com/unitreerobotics/xr_teleoperate/blob/main/README.md
- https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/utils/crc.py
- https://learn.microsoft.com/en-us/windows/wsl/networking

## 今回の検証

40件のhardware_v2 unittestがpass。SDKはfake、通信は未実施。
元offlineパッケージ27件とは別。新規テストは腕のみpacket、CRC/Write失敗、lazy publisher、read-only、
モード変化、低状態不鮮度、tick停止/wrap、deadline/連投防止、NaN/関節限界、手動停止、実測始点、
往復、profile/plan拒否、default dry-runのSDK/network不使用を検査した。
`cloud_preflight.json` はクラウドのSDK未導入結果で、Windows laptopの診断結果ではない。
実機/Windows/WSL/実ボタン/新しいMuJoCo検証は未実施。repo pushやrobot installも未実施。

### 校正済みRGB-Dからreview planまでの実行入口

以下の変換コードは実装済み。撮影・検出・LLM推論を行うループは未実装であり、外部からデータを保存する必要がある。
1. `g1_arm.py state` のJSONは全29関節、IMU、Unix秒の受信時刻、脚/gyro速度を保存する。
2. 外部カメラ受信側で同時刻のRGB PNG、カラー画素へ位置合わせした深度[m]の2次元NPY、capture.jsonを保存する。
3. 外部LLM/操作者はそのRGBに対応する検出IDを選ぶintent.jsonを保存する。3D値や関節角はLLMに渡させない。
4. `prepare_inputs.py` は画像/深度SHA、同じcamera/frameの選択、深度/姿勢/状態の時刻、校正IDを検査する。
5. 親の `offline_adapter.py` が深度計測と既存MuJoCo FK/IK/衝突検査を実行し、joint previewへ。
6. `target_bridge.py` がreview plan draftへ変換。人が現物条件と計画をレビュー後、小範囲runtimeへ。

```powershell
python prepare_inputs.py --capture capture.json --intent intent.json --calibration calibration.json --state robot_state.json --output inputs.json
python ..\offline_adapter.py --inputs inputs.json --model <Menagerieのg1.xml> --output joint_preview.json
python target_bridge.py --joint-preview joint_preview.json --snapshot robot_state.json --profile profile.local.json --output arm_plan.draft.json
python g1_arm.py run-plan --plan <レビュー済みplan.json> --profile profile.local.json
```

撮影とstate読み取りが離れすぎると拒否される。各時計は同じUnix秒へ同期する必要がある。
prepare_inputsのnowは記録再生用時刻であり、ファイルが今も新しいという証明ではない。
現場でリアルタイム利用するには**同時取得するカメラ/状態収集プロセス**の実装・時刻同期検証が必要。
単発stateコマンドだけで連続カメラ同期を保証しない。

capture.json の必要field:
- timestamp_clock="unix_seconds", frame_id, camera_id, camera_role="primary_rgbd", depth_aligned_to_color=true。
- rgb_path, depth_path（capture.jsonと同じディレクトリ配下）、rgb_sha256, depth_sha256。
- rgb_timestamp_s, depth_timestamp_s, pose_timestamp_s。取得時刻を使い、受信完了時刻で誤魔化さない。
- intrinsics={fx,fy,cx,cy}。
- detections=[{id:"up-001",bbox:[x1,y1,x2,y2]}]。このRGBの画素座標。

intent.json:
```json
{"schema":"g1-button-selection-v1","frame_id":"同じframe_id","camera_id":"同じcamera_id","detection_id":"up-001","action":"press"}
```

calibration.json の必要field:
- camera_id, calibration_id（sim_ではない現物校正ID）, verified_on_robot=true。
- base_frame="stationary_floor", T_base_optical（4×4の実測外部行列）。
- tip_offset_m（3要素）, tip_radius_m, pelvis_height_m。

姿勢/外部行列はその撮影時刻のもの。単なるtrueフラグやID名が正しい校正を作るわけではない。
この経路の本物のMuJoCo IKは本環境で未実行。カメラ acquisition、LLM API、連続視覚フィードバック、
接触力制御、点灯成功判定は追加作業であり、現段階の実機試験対象は腕の小動作に限る。
