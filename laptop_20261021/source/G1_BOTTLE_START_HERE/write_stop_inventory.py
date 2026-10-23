"""Generate source-linked effective stop inventory; reads local files only."""
from pathlib import Path
import json,math,hashlib
ROOT=Path(__file__).resolve().parent.parent;HERE=Path(__file__).resolve().parent
E=ROOT/'continuous_suspended_pitch10_executor/continuous_touch.py'
A=E.with_name('arm_runtime.py');S=E.with_name('shadow_motion_guard.py');G=ROOT/'right_arm_20deg_preparation/coordinated_geometry.py'
def ref(p,text):
 n=next(i for i,line in enumerate(p.read_text().splitlines(),1) if text in line)
 return f'[{p.name}:{n}]({p.as_posix()}:{n})'
def table(rows):
 return '|条件・目的|実効値／時間／対象|発動時|根拠／来歴|\n|---|---|---|---|\n'+''.join('|'+ '|'.join(r)+'|\n' for r in rows)
out='''# 停止条件一覧（PC作業中・実機停止）

選択版：`continuous_suspended_pitch10_executor/continuous_touch.py`。入口は同じフォルダーの`START.cmd`（既定help）。
この一覧は実際の呼出経路とソースから作成。メーカー認定の安全限界を列挙したものではない。
**roll/pitchとも±10度は専用版に実装済み。両層の境界試験対象で、実機未検証。**
固定25秒総時間制限は除去済み。総期限は`ceil(計画最終時刻+14秒)`で一度だけ計算し、区間ごとに延長しない。
現在の保存候補118.688499秒に対し133秒。保存候補の現在有効性は未確認で、再開時に新しく計画する。

## 1. 開始前の拒否（指令0回で終了）

'''
out+=table([
('既存結果の上書き防止','同じ実行先にcontinuous_touch_result.jsonがあれば拒否','開始しない',ref(E,"if logfile.exists()")+'／継承'),
('状態・所有待機と腕の静止','最大8秒。腕15～28の全dqが絶対値0.05rad/s未満を0.5秒連続（標本間に超過すると再計時）','待機時間切れで終了',ref(E,'deadline=time.perf_counter()+8')+'／継承'),
('開始状態の妥当性','下記動作中条件も受信開始から有効。過去試験のエラーは別プロセスへ持ち越さない','開始前なら指令0回で終了',ref(E,'s=t.check()')+'／継承'),
('始点の連続性','右腕22～28：保存計画の最初の角度との差は各3度以内。以後1秒の平滑接続。2度を全条件へ適用する設定はない','既存追従判定で拒否。大きな古い姿勢へ跳躍しない',ref(E,'latest=t.check(anchor,RIGHT_PLAN[0]')+'／3度を計画始点に適用'),
('計画の自己干渉','新鮮な非対象関節角を固定して計画を401点で評価。対象形状の符号付き距離min<0mで拒否','publisher生成前に終了',ref(E,'for k in range(401)')+'／連続軌道に適用'),
('計画端点と角度値','14腕角の長さ、有限値、下記関節限界と0.025rad内側余裕','開始しない',ref(E,'angles(anchor[15:29])')+'、'+ref(A,'lo+.025')+'／継承'),
])
out+='\n## 2. 開始時・動作中の停止\n\n'
out+=table([
('状態鮮度・時刻','LowState受信から0～0.20秒（負の時刻差も拒否）','例外→送信終了',ref(A,'now-self.received_at')+'／継承'),
('型・数値・機種','q,dq各29、rpy,gyro各3、有限数、bool数値拒否。mode_machine=5','同上',ref(A,'def numeric')+'、'+ref(A,'def validate(self')+'／継承'),
('生存判定','各関節のvol>0.1 または temperature[0]>0。開始後の全制御対象0～28に適用','同上',ref(A,'numeric(x.vol)')+'、'+ref(E,'All controlled body motors')+'／継承。熱・電圧上限ではない'),
('モーターmode','開始時0/1を許容。初送信から250ms以内に全29関節がmode1となること。mode1を観測した関節が0等へ落ちたら停止','同上',ref(E,'self.latched[i]')+'、'+ref(E,'>.25')+'／成功済み直接起動を継承'),
('制御所有','CheckMode成功(code0)、辞書name空文字。AIまたは不明なら拒否。約0.2秒間隔、RPCタイムアウト0.3秒、最終確認の古さ0.75秒以内','同上。自動AI解除／復帰は実行器内にない',ref(E,'self.client.SetTimeout(.3)')+'、'+ref(E,"result.get('name')")+'、'+ref(E,'now-at<=.75')+'／継承'),
('IMU roll','絶対値radians(10)=0.1745329252rad以下。持続猶予なし','同上',ref(A,'abs(self.rpy[0])')+'、'+ref(S,'abs(rpy[0])')+'／吊り下げ専用に0.1radから変更'),
('IMU pitch','絶対値radians(10)=0.1745329252rad以下。±10度境界を含み、超過は即時拒否','同上',ref(A,'abs(self.rpy[1])')+'、'+ref(S,'abs(rpy[1])')+'／吊り下げ専用に0.1radから変更'),
('IMU角速度','各軸絶対値0.05rad/s以下（約2.86479度/s）。持続猶予なし','同上',ref(A,'max(abs(x) for x in self.gyro)')+'、'+ref(S,'max(abs(x) for x in gyro)')+'／継承'),
('脚腰の静的位置ずれ','0～14：受信ガード最初の角から0.03rad以内（約1.71887度）。送信側でも実行開始角から同じ0.03rad以内','同上',ref(E,'self.guard.position_envelope=.03')+'、'+ref(S,'self.anchor[i]')+'、'+ref(E,'limit=.03')+'／継承。クラス既定0.001radは上書きされ実効値でない'),
('脚腰の位置差速度','0～14：robot_tickによる10～50msの履歴差分が0.05rad/sを超えたら停止。超過の追加持続判定はない','同上',ref(S,'if .010<=elapsed<=.050')+'／成功済みコードと同じ'),
('脚腰の生dq持続','0～14：abs(dq)>0.05rad/sが20ms以上継続。基準以下でその関節の計時をリセット','同上',ref(S,'if abs(dq[i])>self.speed')+'／継承'),
('状態サンプル間隔','perf_counter逆行または受信間隔>50msで停止。robot_tick逆行相当または差>50（msとして使用）でも停止。重複tickは持続や基準を更新しない','同上',ref(S,'host-self.last_host>.05')+'、'+ref(S,'delta>=0x80000000')+'／継承'),
('右腕追従','22～28：その時点の時間変化する指令角との差が各3度を超えたら停止','同上',ref(E,'expected=target22[i-22]')+'／単関節版の指令相対を全指令関節へ適用。閾値は変更なし'),
('左腕の非指令変位','15～21：開始角から各3度以内','同上',ref(E,'limit=.03')+'／継承'),
('関節限界・余裕','腕15～28：LIMITSの内側0.025rad（約1.43239度）。qと全指令に適用','同上',ref(A,'lo+.025')+'、'+ref(E,'angles(q[15:29])')+'／継承'),
('指令範囲','右腕各軸：現在anchorからの変位が、その計画の最大変位内（数値余裕約1e-7rad）','同上',ref(E,'Planned first-segment envelope')+'／旧1/15度固定範囲を計画範囲へ変更'),
('実測自己干渉','各送信前の実測qを既存形状で評価しmin<0mなら停止。グローブ、外部環境の完全な接触検出ではない','同上',ref(E,'Observed nominal mesh penetration')+'、'+ref(G,'def minimum')+'／継承'),
('送信周期遅延','次回予定時刻から>50ms遅延で停止。通常は送信開始から20ms周期、処理超過時は終了から20ms。追いつくための連射なし','同上',ref(E,'now-next_at>.05')+'、'+ref(E,'next_at=now+.02')+'／継承'),
('DDS書込結果','Write(msg)が厳密にTrueでなければ停止','同上',ref(E,'result is not True')+'／継承'),
('操作者・信号','Enter/改行/Ctrl+Cキー（約20ms監視）、SIGINT、SIGTERMでstop Event。次の制御境界で停止','同上。停止元と時刻を記録',ref(E,"mark_stop('SIGINT')")+'、'+ref(E,"mark_stop('keyboard')")+'／停止元記録を追加、停止操作は継承'),
])
out+='''
### 停止時の共通動作

例外またはstop Eventでpublisherを閉じる。異常中に自動帰還・姿勢ゼロ化・AI復帰を行わない。
受信は0.5秒続けて終了状態を保存する。`sender_closed=true`は物理的な脱力／トルク消失の確認ではない。
受信コールバックや所有監視で検出したエラーは実行中に記録され、次回の制御チェックで停止する。

## 3. 正常終了と総時間期限

'''
out+=table([
('正常終了','計画の接近1～3・接触候補保持・逆順退避を同じpublisherで連続実行。前後に1秒保持＋1秒平滑接続、最後0.5秒保持','送信終了→0.5秒受信→終了。AI復帰なし',ref(E,'phase(1.,lambda f:list(anchor')+'、'+ref(E,'phase(float(TIMES[-1])')+'／連続版'),
('有限総時間','ceil(計画最終秒+14秒)。現候補118.688499秒→133秒。14秒は起動10、保持1、接続1、終了保持0.5、受信0.5、余裕1秒','期限でstop Event。さらに5秒経って終了しない場合os._exit(2)',ref(E,'TOTAL_DEADLINE_S=')+'、'+ref(E,'finite_trajectory_total_deadline')+'／25秒試験用固定制限を置換。無期限化・区間毎リセットなし'),
])
out+='\n## 4. 形状・計画だけの条件／非実効コード\n\n'
out+=f'''- 選択実行器の形状は`right_arm_20deg_preparation/collision_only.urdf`。MuJoCoの各geom marginは0.005mだが、停止しきい値は符号付き距離0m。5mm未満を一律停止する設定ではない。{ref(G,'self.model.geom_margin')}。
- 右肩・肘・手首に関係する組だけを収集する。右肘－右手首pitchの正の距離は除外するが、負距離は除外しない。{ref(G,'c.dist>0')}。
- 接触候補がないときの戻り値0.010mは既定値であり、厳密な最近距離の実測値ではない。
- `Geometry.plan()`の2.5mm/10mm等は選択実行器から呼ばれない。**この一覧の実効停止条件ではない。**
- `separation_recovery_gate.py`の限定回復例外は、選択±10度連続実行器ではimportされず、**有効でない**。負距離を初期ゼロへ読み替える設定もない。
- オフラインIKは位置優先・関節限界内で探索し、通常の軌道生成は立方補間のピーク3度/s。これは生成時／オフライン検証値。選択実行器に独立した「各送信の3度/s上限チェック」はない。
- `arm_runtime.py`のarm_sdk用`validate_profile`、MAX_SPEED/MAX_ACCEL/MAX_EXCURSION、DT=0.004等は選択実行器の呼出経路で使わない。実効送信周期は20ms。架空のfirmware/model/operator承認欄は要求・作成しない。
- 2度は全ての条件に一括適用していない。現在の始点照合は既存3度。角度を速度(rad/s)、距離(m)、通信秒数へ流用していない。

## 5. 依頼値と実効値

|項目|依頼|実効|状態・理由|
|---|---|---|---|
|roll|±10度|±10度|吊り下げ専用として適用、gyro維持|
|LowState鮮度|1秒|0.20秒|未適用。3度/sでは1秒で3度進み、追従基準と同量の移動を未観測にする|
|所有確認鮮度|1.5秒|0.75秒|未適用。別制御との競合検出を倍遅らせる根拠がない|
|遅延|100ms|50ms|未適用。20ms送信の5周期欠落時の機体側挙動が未検証|

初期待機8秒、RPCタイムアウト0.3秒、所有ポーリング0.2秒は別。依頼値はINACTIVE_WATCHDOG_PROPOSAL.jsonへ非有効で保存。実行器は読み込まない。

## 6. このコードにないもの（あると誤認しない）

- タッチセンサーによる接触成功判定、手先力の上限、合計PDトルクの上限は実装していない。tau=0はフィードフォワードが0であり、PDトルクが0という意味ではない。
- 温度最大値や電圧上下限のアプリ独自停止はない。live判定は正常性の完全な証明ではない。
- 操作者の現場停止は別系統。PCのEnter/Ctrl+Cは安全認証された非常停止ではない。
- 外部環境・支持具・グローブを含む完全な衝突保証や、カメラ校正の精度保証はない。
- 左腕・腰・脚は新鮮な開始角で**PD保持する**。動作目標を変えるのは右腕のみであり、全身の保持力が入ることは承認済みのVR方式の一部。

## 7. 保護設定と確認状態

- ユーザー承認により専用版のroll/pitchを±10度へ変更。gyro±0.05rad/s、脚腰の速度／変位、追従／鮮度／形状は変更していない。
- roll/pitch両軸の±10度境界受理・境界外拒否、gyroの従来拒否を確認。保存済み`IMU_BOUNDARY_TESTS.json`と入口の`offline`を参照。旧pitch-only結果は履歴。
- ±10度版は**未実機検証**。pause直前にAI解除1回が成功、モーター送信0回、読戻し所有なし・全29関節mode0。その後新たな実機操作はしない。
- 直前の状態はpause後の現在状態を保証しない。再開時は所有、関節角、画像、始点と経路を取り直す。

### 腕の実効角度範囲（LIMITS内側0.025rad）

|関節|左15～21：rad|右22～28：rad|
|---|---|---|
'''
import ast
tree=ast.parse(A.read_text());limits=next(ast.literal_eval(n.value) for n in tree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='LIMITS' for t in n.targets))
for j,name in enumerate(['肩pitch','肩roll','肩yaw','肘','手首roll','手首pitch','手首yaw']):
 fmt=lambda v:f'{v[0]+.025:.4f} ～ {v[1]-.025:.4f}'
 out+=f'|{name}|{fmt(limits[j])}|{fmt(limits[j+7])}|\n'
out+='\n### ソース固定情報\n\n'
for p in [E,A,S,G]:out+=f'- `{p.relative_to(ROOT)}` SHA256 `{hashlib.sha256(p.read_bytes()).hexdigest()}`\n'
(HERE/'STOP_CONDITIONS_JA.md').write_text(out,encoding='utf-8')
print('STOP_CONDITIONS_JA.md generated from selected effective sources')
