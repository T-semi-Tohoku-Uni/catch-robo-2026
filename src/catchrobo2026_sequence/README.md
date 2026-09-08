# シーケンスノード

UIの初期化・開始・終了操作と確定したPICK／PLACEを、YAMLに記述した移動・ポンプ・エンドエフェクタ・待機へ展開するROS 2ノードです。UIのC制御コアが計画・保持・配置状態の正本を維持し、`sequence_node` は一度に1動作を実行します。

設定形式・エイリアス・継承・共通値の参照は [CONFIG.md](CONFIG.md) を参照してください。

## 起動

起動時の既定ファイルは [config/sequences.yaml](config/sequences.yaml) です。運動学パッケージのCSVから転記した赤青共通の16 PICK座標と、赤青それぞれ8 PLACE座標を持ち、デバッグ用の初期手順を読み込んで実行できます。[config/sequences.example.yaml](config/sequences.example.yaml) は全位置未設定の雛形として残しています。

各位置の絶対姿勢へ移動後、PICKは「吸引→相対接近→相対退避→PLACE用の幅に切替」、PLACEは「開放→相対接近→相対退避→オフ→PICK用の幅に切替」を実行します。PLACEでは赤 `[150, 0, 400, π]`／青 `[1200, 0, 400, π]` を経由して各位置へ1経路で移動します。初回はUIの「開始」でYAMLの開始シーケンスを実行し、PLACE用の幅を指令します。
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

UIの「終了」では現在動作の取消完了後に `sequences.ending.steps` を実行します。cancel／reset・動作失敗・Ctrl+Cでは実行しません。終了手順を中断する場合は取消ボタンを操作します。同梱設定では初期化の前後手順は `steps: []`、終了手順は `[675, 200, 300, 0]` を経由して `poses.lifecycle_pose: [670, -110, 220, 0]` へ向かう1経路です（x/y/zはmm、phiはrad）。最初の点を `waypoint: {absolute: ...}` で指定しており、その点での個別の到達待ちを省きます。参照先はトップレベルの `before_initialization_sequence`／`after_initialization_sequence`／`end_sequence` で変更できます。省略または `null` なら追加動作なしです。

詳細と編集例は [CONFIG.md](CONFIG.md#初期化前後終了シーケンス) を参照してください。

## 設定の読込

- `debug:=false`（既定）: 起動時に1回読み込み、終了までその内容を使用します。
- `debug:=true`: 起動時の検証に加え、INITIALIZE／START／END／PICK／PLACEを受け取るたびにファイル全体を再読込・検証します。実行中の動作は開始時の展開結果を保持し、編集は次のINITIALIZE／START／END／PICK／PLACEから反映します。
- 読込・参照解決・対象位置の展開に失敗した場合、その動作の指令を出さず失敗を返します。デバッグ時に古い設定へフォールバックして実行することはありません。

編集対象は `sequence_file` に指定したファイルです。インストール済みコピーを既定で読む場合、ソース側ファイルだけの編集では反映されません。調整時は作業用YAMLの絶対パスを指定してください。

座標と手順の調整はYAMLを直接編集します。CSVは初期転記の参考資料で、実行時に読み込んだり、CSVの編集をYAMLへ自動反映したりする機能はありません。`config/sequences.yaml` の転記対応・共通高さ・仮の移動量は [CONFIG.md](CONFIG.md#座標を転記したデバッグ用設定) を参照してください。

## 実行とROSインターフェース

`execute_sequence`（`catchrobo2026_msgs/action/ExecuteSequence`）の要求には `control_epoch`・`step_id`・種類・UI位置・`collector_mask` を含めます。種類はINITIALIZE=5／START=4／END=6／PICK=1／PLACE=2です。INITIALIZE／START／ENDは位置を使用せず `collector_mask=7` で送信します。PICKはrow=0..3／column=1..4、PLACEはbox=0..3／box_column=0..1です。機構選択はUIのbit0/1/2（L/C/R）で、設定ファイルには記述しません。

通常の `move` は `generate_route`（`GenerateRoute`）へ絶対目標の `x,y,z,phi` を渡し、成功応答の `path` を `follow_route`（`FollowRoute`）の `path` へ渡し、`start=true` で開始します。連続する `waypoint` は次の通常 `move` と一つの経路にまとめ、中間点での個別の到達待ちを省きます。`move` の `waypoints` オプションでも経由点列を指定でき、従来の `move.waypoint: true` も使用できます。経由点1件だけの共通手順を `call`／`extends` し、同じ実行シーケンス内の後続 `move` へ接続できます。空の経路が返った場合は失敗とし、以前の経路を再利用しません。最後の移動先への追従成功を待ってから後続手順へ進みます。指定方法と制約は [経由点の設定](CONFIG.md#経由点waypoint) を参照してください。

シーケンサは毎回 `use_explicit_waypoints=true` とし、要求内の `waypoints` に経由点を列挙します。通常移動では空配列です。YAMLと要求の最終目標はmm／rad、要求内の経由点はROS Poseのm／Quaternionへ変換します。生成結果の `route`／`path` もmです。要求内で経由点と終点をまとめるため、取消や失敗で共有の `waypoint` 蓄積へ経由点が残りません。

初期化前後・開始・終了・PICK／PLACEのすべての移動で、生成した経路を追従アクションへ直接渡します。`route` トピックの受信順に依存せず、実行中の経路は固定されます。追従中の追加ゴールは拒否します。手動の `FollowRoute(start=true)` は `path` を省略した場合、受理時に受信済みの `route` を固定して使い、未受信なら拒否します。手動の `GenerateRoute` は `use_explicit_waypoints=false`（既定）で従来の `waypoint` 蓄積を使います。明示要求はこの蓄積を参照・消費しません。

`GenerateRoute`／`FollowRoute` の `nav_msgs/Path path` に加えて、`GenerateRoute` 要求に経由点指定を追加しています。更新時はワークスペース全体を再ビルドし、関係するノードをすべて再起動してください。

ポンプ手順では、UIで選択された機構に設定の指令値、選択されていない機構にオフ（0）を指定して、`set_pump_state` に全3状態を渡します。例えばLとRを選んで吸引する場合は `(left, center, right)=(1,0,1)` です。初期化前後・開始・終了手順のポンプ操作は全3機構が対象です。ポンプの現在状態の読出しは行いません。

姿勢の `phi` は経路追従の回転角です。`endeffector: 0`／`1` は回収機構の幅を切り替える既存の `set_endeffector_state` への指令です。0/1の広い／狭い対応は実機側で確認します。

ポンプ／エンドエフェクタのサービス成功は指令の受理です。吸着・開放・機構動作の完了をセンサで確認する機能はありません。設定の `wait` で必要な待機時間を与えます。シーケンス全体が成功した場合だけUIへ成功を返し、UIが元のepoch・step ID・RUNNING状態を照合し、PICK／PLACEでは保持／配置を更新、STARTでは通常キューへの進行を許可します。部分回収の検出は行いません。

## 取消・失敗

UIのcancel／end／resetで現在動作を取り消します。UIは旧動作の終端結果を受けるまで次を送信せず、古い成功通知を新しいステップへ適用しません。動作失敗時は完了を記録せずUIをFINISHEDへ進め、原因をcontrol_nodeのログへ出します。

シーケンサは待機・サービス応答・追従結果を非同期で処理します。取消時は後続手順を止め、実行中のFollowRouteを取り消します。従来の追従ノードは取消を処理すると追従ループを終了し、実測角の保持指令は追加しません。これはモータの非常停止や停止確認を意味しません。ポンプ／エンドエフェクタの指令は保持し、勝手に開放しません。既に送信したサービス要求は取り消せないため、応答を待ってから動作を終えます。

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

従来の経路生成・追従ノードには関節情報の鮮度検査、経路の到達可能性検査、追従ノード自身のタイムアウトはありません。シーケンサ側のタイムアウトと取消処理を使用します。
