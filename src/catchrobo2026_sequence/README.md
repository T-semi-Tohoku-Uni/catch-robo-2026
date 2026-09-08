# シーケンスノード

UIの初期化・開始・終了操作と確定したPICK／PLACEを、YAMLに記述した移動・ポンプ・エンドエフェクタ・待機へ展開するROS 2ノードです。UIのC制御コアが計画・保持・配置状態の正本を維持し、`sequence_node` は一度に1動作を実行します。

設定形式・エイリアス・継承・共通値の参照は [CONFIG.md](CONFIG.md) を参照してください。

## 起動

起動時の既定ファイルは [config/sequences.yaml](config/sequences.yaml) です。運動学パッケージのCSVから転記した赤青共通の16 PICK座標と、赤青それぞれ8 PLACE座標を持ち、デバッグ用の初期手順を読み込んで実行できます。[config/sequences.example.yaml](config/sequences.example.yaml) は全位置未設定の雛形として残しています。

各位置の絶対姿勢へ移動後、PICKは「吸引→相対接近→相対退避→PLACE用の幅に切替」、PLACEは「開放→相対接近→相対退避→オフ→PICK用の幅に切替」を実行します。PLACEでは赤 `[150, 0, 360, π]`／青 `[1200, 0, 360, π]` を経由して各位置へ1経路で移動します。初回はUIの「開始」でYAMLの開始シーケンスを実行し、PLACE用の幅を指令します。
幅指令はYAMLの `values.pick_endeffector_command: 0`／`place_endeffector_command: 1` に仮置きしています。0/1の広い／狭い対応は手動で確認し、必要なら両値を入れ替えてください。幅切替後の待ち時間は追加しておらず、ROSサービスの受理応答でそのPICK／PLACEを完了します。
現在の相対Zはapproachが `-10 mm`、retreatが `+10 mm` で、直近の絶対姿勢を固定基準にそれぞれZ−10／Z＋10 mmへ移動します。実機の高さ・移動量・PLACEの操作順は実機確認済みの値ではありません。

実機デバッグではワークスペースのルートから以下を実行し、編集するソースYAMLの絶対パスを指定します。`debug:=true` で初期化・開始・終了シーケンス・各PICK／PLACEの実行前に読み直すため、YAMLの編集を次の動作へ反映できます。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  team:=red \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  debug:=true
```

`automatic.launch.py` はUI・シーケンサ・経路生成・追従・ポンプ・エンドエフェクタを起動します。CAN接続と現在関節角の供給は既存の構成で別途行います。手動の `main.launch.py`／`handoperated.launch.py` と同時起動すると関節・ポンプ指令が競合するため、自動操縦時は手動ノードを停止してください。

シーケンサ単独は `sequence.launch.py` です。陣営 `team:=red` または `team:=blue` は必須です。ブラウザの表示反転は座標陣営を変更しません。

## UIの開始シーケンス

YAMLの `start_sequence: startup` で、`sequences.startup` を開始手順として指定します。同梱設定は `endeffector: '$place_endeffector_command'` の1手順です。ノード起動時には実行せず、UIの「開始」で実行します。初期化シーケンスとは独立しており、開始手順に初期化命令は含めません。

UIは開始中を「開始シーケンス」と表示し、完了後に通常のキューへ進みます。半自動でも開始手順は「開始」で実行し、その後のPICK／PLACEは「次へ」で進めます。開始手順の失敗・取消では終了状態へ移り、通常キューは実行しません。開始手順はワーク・保持・箱・PICK／PLACE履歴を更新しません。`start_sequence` を省略または `null` にすると、開始手順は何もせず完了します。

## 初期化前後・終了シーケンス

同梱YAMLの `sequences.before_initialization.steps` と `sequences.after_initialization.steps` に、初期化命令の前後に実行する手順を書けます。UIの「初期化」で前手順→初期化命令→後手順の順に実行し、全体成功後に初期化済みになります。初期化命令の応答は実機の原点復帰完了ではないため、必要な待機は後手順へ `wait` で設定してください。前後の相対移動は、それぞれの手順内の絶対移動を基準にします。

UIの「終了」では現在動作の取消完了後に `sequences.ending.steps` を実行します。cancel／reset・動作失敗・Ctrl+Cでは実行しません。終了手順を中断する場合は取消ボタンを操作します。同梱設定では初期化の前後手順は `steps: []`、終了手順は `[675, 200, 300, 0]` に到達してから `poses.lifecycle_pose: [670, -110, 220, 0]` へ移動します（x/y/zはmm、phiはrad）。2件とも通常の `move` で、それぞれの到達を待ちます。参照先はトップレベルの `before_initialization_sequence`／`after_initialization_sequence`／`end_sequence` で変更できます。省略または `null` なら追加動作なしです。

詳細と編集例は [CONFIG.md](CONFIG.md#初期化前後終了シーケンス) を参照してください。

## 次のPICK／PLACEへ向かう経由点

PICK／PLACEの末尾に`waypoint`を置くと、経由点を保留して現在の動作を完了します。次のPICK／PLACEが始まったとき、その最初の移動経路へ保留点を追加します。経由点へ単独では移動せず、次の動作がなければその場で待ちます。

```yaml
place_blue_0_0:
  steps:
    - call: place_pre_blue_waypoint
    - move: {absolute: place_blue_0_0_above}
    - call: place_common
    - call: place_pre_blue_waypoint
```

この例の末尾の青側経由点は、次のPICKが選ばれた時点で「現在位置→青側経由点→PICK直上」の1経路に含まれます。次動作に経由点があればその前に追加し、通常MOVEの終点で到達を待ちます。現在のPLACE中に退避先へ到達させたい場合は、末尾を通常の`move`にしてください。

保留点は元の動作で解決した絶対座標として保持し、次の動作の相対座標基準を変えません。`debug:=true`の再読込後も保留済み座標は維持します。引き継ぐのは同じepochの後続PICK／PLACEだけで、START／INITIALIZE／END、resetによるepoch変更、取消・失敗・再読込失敗・再起動では破棄します。ライフサイクル手順やgroup内部の末尾には未接続waypointを置けません。次の移動がgroup内なら保留点もそのgroupで事前計画します。詳しくは[末尾経由点の設定](CONFIG.md#pickplace末尾の経由点を次の動作へ持ち越す)を参照してください。

## シーケンスグループ

`sequence_group`の開始・終了で区間を作り、`rotation_group: true`で第4関節角`q4`の一方向制約、`max_phi_travel`でフィールド基準の手先角`phi`の総移動量上限を指定します。両者は独立したオプションです。`phi = q0 + q4`なので、第4関節を一方向に動かす制約だけでは、手先の360°回転を防げません。

次はAへの到達後からBまでの`phi`総移動量を90°（π/2rad）までに制限する書式例です。姿勢名と上限値は実際の手順に合わせて指定してください。

```yaml
steps:
  - move: {absolute: target_a}
  - sequence_group: {start: true, max_phi_travel: 1.5707963267948966}
  - move: {absolute: target_b}
  - sequence_group: end
```

総移動量は、巻き数を保った指令の`Σ|Δphi|`をグループ開始時から加算します。開始と終了が同じ角度でも、途中の1回転は360°です。単位はrad、有限の0以上を指定でき、`'$値名'`も使えます。`max_phi_travel`の省略時は上限なし、`rotation_group`の省略または`false`では第4関節の方向反転を許可します。両方の制約を使う場合は`sequence_group: {start: true, rotation_group: true, max_phi_travel: 1.5707963267948966}`とします。

グループ全経路を事前計画し、設定した制約を満たせない場合は最初の操作前に失敗します。実行時にも指令の総移動量を検査しますが、これは指令に対するソフト上の制限であり、実際の機構の回転量や停止を保証するものではありません。`sequence_group: start`は追加の一方向制約・総移動量上限を指定せず、グループとして事前計画します。

ポンプ・幅切替・待機・経由点を含められ、`call`／`extends`でも接続できます。同じUIアクション内で開始と終了を対応させ、初期化命令や経由点列の途中を境界にしないでください。従来の独立ステップ`rotation_group: start`／`end`は、第4関節の一方向制約を有効にする互換表記として残しています。現在の同梱endingは`sequence_group`内の`rotation_group: true`を指定し、`phi`の総移動量上限は未設定です。詳しくは[シーケンスグループの設定](CONFIG.md#シーケンスグループsequence_group)を参照してください。

## 実行前のオフライン検査

`check_sequence_config`は本番の設定展開・経路計算を使い、指定した開始状態からの到達可能性、第4関節の回転制約、`phi`総移動量の上限を検査します。ROSノードを起動せず、ロボットへ指令を送りません。

ワークスペースをビルドし、`source install/setup.bash`した後に実行します。実運用で使うYAMLと同じファイルを`--config`に指定してください。

```bash
# 設定の書式・参照・展開だけを確認
ros2 run catchrobo2026_sequence check_sequence_config \
  --config src/catchrobo2026_sequence/config/sequences.yaml --syntax-only

# lifecycle_poseを開始姿勢と仮定し、PICK → PLACE → ENDを連続して確認
ros2 run catchrobo2026_sequence check_sequence_config \
  --config src/catchrobo2026_sequence/config/sequences.yaml \
  --team red --initial-pose-name lifecycle_pose \
  --action pick:0,1 place:0,0 end
```

`--initial-pose-name`の代わりに、実際の開始関節角を`--initial-joints Q0 Q1 Q2 Q4`（すべてrad）で渡せます。姿勢の直接指定は`--initial-pose X Y Z PHI`（mm／rad）です。同じ姿勢に第4関節角0と−2πの両方が対応する場合は、`--initial-wrist`で実際の巻き数を指定しない限り`UNKNOWN`となります。

`--all`または対象指定の省略では、開始・初期化・終了・有効な全PICK／PLACEを同じ開始状態から**個別に**確認します。操作間の連続確認は`--action`を実際の順序で指定してください。名前付き手順は`--sequence ending`で選べます。`--team`の既定は`both`で、各陣営は別々に検査します。

結果は`FEASIBLE`（検査条件を満たす）、`INFEASIBLE`（不成立）、`UNKNOWN`（開始状態などが不明）です。終了コードはそれぞれ0・1・2で、設定不正は1、引数不正は2です。文法だけの検査の成功は、動作の実行可能性を意味しません。初期化を含める場合、完了後の関節角を`--after-initialization-joints Q0 Q1 Q2 Q4`で明示しないと、その先の経路は判定できません。

通常MOVEの第4関節角が経路サンプル間でπを超えて変化する場合は`wrist_wrap_jump`警告を出します。`--warnings-as-errors`で警告も終了コード1にできます。`--json`では対象名・失敗位置・予測最終関節角・回転量をJSONで出力します。

PICK／PLACEの末尾に経由点が残る場合、その時点では次の通常MOVEが決まらないため`UNKNOWN`として保留点を表示します。`--action place:0,0 pick:0,1`のような連続検査では、保留点と到達済み関節角を次の対象へ引き継ぎ、実際に連結した経路を検査します。保留だけを理由とする途中の`UNKNOWN`は、次の移動で解決できれば全体判定を妨げません。対象列の最後に保留点が残る場合と、`--all`で個別確認したPLACEの保留経路は未判定のままです。開始姿勢不明・初期化後不明・経路失敗はこの例外に含めず、後続を成功扱いにしません。`--action start`／`initialize`／`end`への切替では実行時と同じく保留点を破棄します。`--sequence`は汎用の手順列として扱うため、名前が`ending`でもライフサイクル境界にはなりません。UIの終了動作を再現する場合は`--action end`を使ってください。

これは目標に正確に到達すると仮定した経路計算上の判定です。通常MOVEは生成経路のサンプルを検査し、グループは本番と共通の計画処理で制約を検査します。衝突、吸着・機構動作、追従誤差、初期化の物理動作、実行時間やタイムアウトは判定対象外です。

## 設定の読込

- `debug:=false`（既定）: 起動時に1回読み込み、終了までその内容を使用します。
- `debug:=true`: 起動時の検証に加え、INITIALIZE／START／END／PICK／PLACEを受け取るたびにファイル全体を再読込・検証します。実行中の動作は開始時の展開結果を保持し、編集は次のINITIALIZE／START／END／PICK／PLACEから反映します。
- 読込・参照解決・対象位置の展開に失敗した場合、その動作の指令を出さず失敗を返します。デバッグ時に古い設定へフォールバックして実行することはありません。

編集対象は `sequence_file` に指定したファイルです。インストール済みコピーを既定で読む場合、ソース側ファイルだけの編集では反映されません。調整時は作業用YAMLの絶対パスを指定してください。

座標と手順の調整はYAMLを直接編集します。CSVは初期転記の参考資料で、実行時に読み込んだり、CSVの編集をYAMLへ自動反映したりする機能はありません。`config/sequences.yaml` の転記対応・共通高さ・仮の移動量は [CONFIG.md](CONFIG.md#座標を転記したデバッグ用設定) を参照してください。

## 実行とROSインターフェース

`execute_sequence`（`catchrobo2026_msgs/action/ExecuteSequence`）の要求には `control_epoch`・`step_id`・種類・UI位置・`collector_mask` を含めます。種類はINITIALIZE=5／START=4／END=6／PICK=1／PLACE=2です。INITIALIZE／START／ENDは位置を使用せず `collector_mask=7` で送信します。PICKはrow=0..3／column=1..4、PLACEはbox=0..3／box_column=0..1です。機構選択はUIのbit0/1/2（L/C/R）で、設定ファイルには記述しません。

シーケンスグループ外の `move` は `generate_route`（`GenerateRoute`）へ絶対目標の `x,y,z,phi` を渡し、成功応答の `path` を `follow_route`（`FollowRoute`）の `path` へ渡し、`start=true` で開始します。連続する `waypoint` は次の通常 `move` と一つの経路にまとめ、中間点での個別の到達待ちを省きます。`move` の `waypoints` オプションでも経由点列を指定でき、従来の `move.waypoint: true` も使用できます。経由点1件だけの共通手順を`call`／`extends`し、同じ動作内の後続`move`、またはPICK／PLACE末尾から次のUI動作の最初の移動へ接続できます。空の経路が返った場合は失敗とし、以前の経路を再利用しません。最後の移動先への追従成功を待ってから後続手順へ進みます。指定方法と制約は [経由点の設定](CONFIG.md#経由点waypoint) を参照してください。

シーケンサからの `GenerateRoute` は毎回 `use_explicit_waypoints=true` とし、要求内の `waypoints` に経由点を列挙します。経由点なしの移動では空配列です。YAMLと要求の最終目標はmm／rad、要求内の経由点はROS Poseのm／Quaternionへ変換します。生成結果の `route`／`path` もmです。要求内で経由点と終点をまとめるため、取消や失敗で共有の `waypoint` 蓄積へ経由点が残りません。

シーケンスグループでは `plan_rotation_group`（`PlanRotationGroup`）へ全目標・通常MOVEの終点位置・グループの制約を渡し、経路列と各点の第4関節角・方向をまとめて取得します。各経路を `FollowRoute` の `path`／`wrist_angles`／`wrist_direction`／`rotation_group_id` と、phi補間用の `phi_angles` へ渡して順に実行します。Joyの `wrist_control`（`WristControl`）はグループの開始・終了と、姿勢と第4関節角を一緒にした指令を受け付けます。

初期化前後・開始・終了・PICK／PLACEのすべての移動で、生成した経路を追従アクションへ直接渡します。`route` トピックの受信順に依存せず、実行中の経路は固定されます。追従中の追加ゴールは拒否します。手動の `FollowRoute(start=true)` は `path` を省略した場合、受理時に受信済みの `route` を固定して使い、未受信なら拒否します。手動の `GenerateRoute` は `use_explicit_waypoints=false`（既定）で従来の `waypoint` 蓄積を使います。明示要求はこの蓄積を参照・消費しません。

`GenerateRoute` の経由点指定、シーケンスグループ用の `PlanRotationGroup`／`WristControl`、`FollowRoute` の追加フィールドを使用します。更新時はワークスペース全体を再ビルドし、関係するノードをすべて再起動してください。

ポンプ手順では、UIで選択された機構に設定の指令値、選択されていない機構にオフ（0）を指定して、`set_pump_state` に全3状態を渡します。例えばLとRを選んで吸引する場合は `(left, center, right)=(1,0,1)` です。初期化前後・開始・終了手順のポンプ操作は全3機構が対象です。ポンプの現在状態の読出しは行いません。

姿勢の `phi` は経路追従の回転角です。`endeffector: 0`／`1` は回収機構の幅を切り替える既存の `set_endeffector_state` への指令です。0/1の広い／狭い対応は実機側で確認します。

ポンプ／エンドエフェクタのサービス成功は指令の受理です。吸着・開放・機構動作の完了をセンサで確認する機能はありません。設定の `wait` で必要な待機時間を与えます。シーケンス全体が成功した場合だけUIへ成功を返し、UIが元のepoch・step ID・RUNNING状態を照合し、PICK／PLACEでは保持／配置を更新、STARTでは通常キューへの進行を許可します。部分回収の検出は行いません。

## 取消・失敗

UIのcancel／end／resetで現在動作を取り消します。UIは旧動作の終端結果を受けるまで次を送信せず、古い成功通知を新しいステップへ適用しません。動作失敗時は完了を記録せずUIをFINISHEDへ進め、原因をcontrol_nodeのログへ出します。

シーケンサは待機・サービス応答・追従結果を非同期で処理します。取消時は後続手順を止め、実行中のFollowRouteを取り消し、次の動作用に保留した経由点も破棄します。シーケンスグループ外では従来どおり追従ループを終了し、実測角の保持指令は追加しません。シーケンスグループ内ではJoyのグループ解除も要求し、最後に有効だった4関節の指令を保持します。これはモータの非常停止や停止確認を意味しません。ポンプ／エンドエフェクタの指令は保持し、勝手に開放しません。既に送信したサービス要求は取り消せないため、応答を待ってから動作を終えます。

停止待ち時間を超えて結果が不明な場合は失敗を返し、以降の動作を拒否します。接続先の状態を確認したうえでノードを再起動する必要があります。プロセス強制終了・通信断時の実機停止、原点復帰、衝突回避、手動との自動切替は別途必要です。

ノードの読取専用パラメータは `sequence_file`、`team`、`debug` と以下です。`ros2 run ... --ros-args -p ...` または独自launchのparametersで指定できます。

| パラメータ | 既定秒 | 対象 |
|---|---:|---|
| `service_timeout_sec` | 3 | サービス待ち・応答・追従ゴールの受理 |
| `route_timeout_sec` | 30 | 経由点を含む1経路全体の追従。YAMLで指定した場合はそちらを優先 |
| `sequence_timeout_sec` | 120 | INITIALIZE／START／END／PICK／PLACE全体 |
| `stop_timeout_sec` | 3 | 取消・失敗後の未確定処理 |

タイムアウトは0秒超〜86,400秒の有限値を指定します。`wait` の設定値にも0〜86,400秒の上限があり、動作全体には `sequence_timeout_sec` が適用されます。

`following route timeout` は `sequences.yaml` のトップレベルに `route_timeout_sec: 60.0` のように書いて変更できます。同梱値は30秒です。省略時はROSパラメータを使います。`debug:=true` では次の動作から反映し、実行中の動作の値は固定します。通常モードではノード再起動が必要です。詳しくは [設定方法](CONFIG.md#経路追従のタイムアウト) を参照してください。

シーケンスグループでは開始・追従時に関節情報の鮮度と第4関節の制約を検査します。第4関節の指令速度には追従ノードの `rotation_speed_rad_sec`（既定1.0 rad/s）の上限を使います。グループ外の通常移動には従来の処理を使い、実機の到達可能性や衝突を保証する検査はありません。追従ノード自身に全経路の時間上限はなく、シーケンサ側のタイムアウトと取消処理を使用します。
