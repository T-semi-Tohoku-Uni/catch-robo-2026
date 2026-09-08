# シーケンス設定

シーケンスノードは起動時に指定した陣営とUIのPICK／PLACE・位置を `bindings` で手順名へ変換し、移動・ポンプ・エンドエフェクタ・待機を順番に実行する。設定は通常のYAMLファイルであり、ROSパラメータファイルとは別に用意する。

既定の [sequences.yaml](config/sequences.yaml) は、座標とデバッグ用の初期手順を記入した読み込み可能な設定。[sequences.example.yaml](config/sequences.example.yaml) は赤青それぞれ16 PICK位置・8 PLACE位置の入力欄を持ち、全て `null` で無効にした雛形として残している。雛形から作る場合は必要な値・姿勢・手順を定義し、使用する位置の `null` を手順名へ変更する。欄の省略も未設定扱いとなり、選択時に実行を拒否する。

### 座標を転記したデバッグ用設定

[sequences.yaml](config/sequences.yaml) は [運動学パッケージの座標CSV](../CatchRobo2026_kinematics/config/flange_targets/) から初期座標を静的に転記した設定。ノードとlaunchの既定ファイルであり、稼働時に読むのはYAMLだけ。CSVローダや生成スクリプトによる同期は行わない。

| 対象 | 転記時の対応 |
|---|---|
| PICK | UIの `row=0..3`、`column=1..4` に対してCSVの `ID = row * 6 + column + 1`。16姿勢を赤青で共用 |
| PLACE | 陣営別CSVの `GroupIdx = box + 1`、`RowIdx = box_column + 1`、中央の `ColIdx = 2`。赤青それぞれ8姿勢 |

高さは `values.work_above_z=186.95`（自陣PICK）、`common_work_above_z=186.95`（共通エリアPICK）、`place_above_z=294.35`（PLACE）を姿勢から参照する。単位はmm。CSVを初期参考として調整した設定であり、`above` という名前が実機での上空高さや余裕量の確認済みを意味するものではない。

位置別PICKは絶対姿勢へ移動した後に `pick_common` を呼び、共通手順は「吸引→相対接近→相対退避→PLACE用の幅に切替」の順。PLACEは赤 `[150, 0, 360, π]`／青 `[1200, 0, 360, π]` の経由点から位置別の絶対姿勢へ1経路で移動し、`place_common` の「開放→相対接近→相対退避→オフ→PICK用の幅に切替」を実行する。初回はUIの「開始」で開始シーケンスを実行し、PLACE用の幅を指令する。このPLACE操作順は実機確認済みの手順ではない。

幅切替の指令値は `values.pick_endeffector_command: 0` と `values.place_endeffector_command: 1` に仮置きし、PLACE末尾で前者、PICK末尾で後者を参照する。0/1の広い／狭い対応は手動で確認し、必要なら値を入れ替える。ユーザー指定により幅切替後の `wait` は追加しておらず、ROSサービスの受理応答でそのPICK／PLACEを完了する。機構動作の完了通知は待たない。

現在の相対移動量は `pick_approach_dz`／`place_approach_dz` が `-10 mm`、`pick_retreat_dz`／`place_retreat_dz` が `+10 mm`。相対座標の基準は直近の絶対姿勢なので、各手順の2回の相対目標はその姿勢のZ−10／Z＋10 mmとなる。例えば自陣PICKのZ指令は `186.95 → 176.95 → 196.95 mm`。移動量の符号や復帰先を変える場合は、それぞれの値を直接編集する。

実機デバッグではワークスペースのルートからソースYAMLの絶対パスを指定して起動する。以下はJazzyの例。`debug:=true` では初期化・開始・終了シーケンス・各PICK／PLACEの実行前に再読込し、その動作の実行中は開始時の展開結果を使う。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  team:=red \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  debug:=true
```

## ファイルの構成

```yaml
version: 1
before_initialization_sequence: null
after_initialization_sequence: null
start_sequence: null
end_sequence: null
values: {}
poses: {}
sequences: {}
bindings:
  red: {pick: {}, place: {}}
  blue: {pick: {}, place: {}}
```

`version`、`poses`、`sequences`、`bindings` は必須。`values`、`route_timeout_sec` と `before_initialization_sequence`／`after_initialization_sequence`／`start_sequence`／`end_sequence` は省略可能。`bindings` には赤青両方と各 `pick`／`place` のマップを用意する。マップは空でもよい。未知キー・重複キー・複数YAMLドキュメントは拒否する。

## 経路追従のタイムアウト

トップレベルの `route_timeout_sec` で、`following route timeout` までの秒数を設定する。同梱設定は `30.0`。例えば60秒にする場合は次のように編集する。

```yaml
route_timeout_sec: 60.0
```

経由点を含む1経路全体に適用する。値は0秒超〜86,400秒の有限数で、`'$名前'` による `values` 参照も使用できる。指定時は同名のROSパラメータより優先し、省略時はROSパラメータ（既定30秒）を使う。

`debug:=true` は次のINITIALIZE／START／END／PICK／PLACE開始時に再読込し、実行中の動作では開始時の値を使い続ける。通常モードでは編集後にノードを再起動する。インストール済みYAMLを読む構成ではソースの編集後に再ビルドが必要。シーケンス全体の上限は別のROSパラメータ `sequence_timeout_sec`（既定120秒）のままで、経路のタイムアウトを延ばしても自動では変更しない。

## 開始シーケンス

トップレベルの `start_sequence` に、UIの「開始」で実行する `sequences` 内の手順名を指定する。同梱設定は以下のとおり。初期化シーケンスとは独立しており、開始シーケンスに初期化操作は含めない。

```yaml
start_sequence: startup
values:
  place_endeffector_command: 1
sequences:
  startup:
    steps:
      - endeffector: '$place_endeffector_command'
```

この例は設定の該当部分のみ。開始時の値もPICK末尾の幅切替と同じ `place_endeffector_command` を参照するため、0/1の確認後は共通値を変更すればよい。待ち時間は追加していない。

開始手順は全自動・半自動とも「開始」で1回実行し、完了後に全自動はキューへ進み、半自動は「次へ」を待つ。開始中はキューを取り出さず、開始手順の取消・失敗では終了状態にする。ワーク・保持・箱・通常動作の履歴は更新しない。

`start_sequence` の省略または `null` は何もしない開始手順として扱う。不明な参照や不正な手順は読込時に拒否する。指定した手順では通常と同じ `call`・`extends`・数値参照、移動・ポンプ・幅指令・待機を使える。相対移動にはその開始手順内で先行する絶対移動が必要。開始手順のポンプは全3機構を対象にする。`debug:=true` では開始シーケンスの実行前にも設定を読み直す。

## 初期化前後・終了シーケンス

`before_initialization_sequence`／`after_initialization_sequence` に名前を指定すると、UIの「初期化」で「前手順 → `request_initialization` への初期化命令1回 → 後手順」を実行する。`end_sequence` はUIの「終了」で実行する。いずれもノードの起動・プロセス終了時に自動実行する手順ではない。

```yaml
before_initialization_sequence: before_initialization
after_initialization_sequence: after_initialization
end_sequence: ending
poses:
  lifecycle_pose: [670, -110, 220, 0]
sequences:
  before_initialization:
    steps: []
  after_initialization:
    steps: []
  ending:
    steps:
      - move: {absolute: [675, 200, 300, 0]}
      - move: {absolute: lifecycle_pose}
```

同梱設定では初期化の前後手順は空で、終了手順は `[675, 200, 300, 0]` に到達してから `lifecycle_pose`（x=670 mm、y=-110 mm、z=220 mm、phi=0 rad）へ移動する。2件とも通常の `move` で、それぞれの到達を待つ。必要な移動・ポンプ・幅指令・待機を各 `steps` に追記する。各キーの省略または `null` は追加動作なしを意味し、初期化命令自体は実行する。`call`・`extends`・数値参照は開始手順と同様に利用できる。これらのポンプ手順は全3機構が対象。

初期化命令の成功応答は指令の受理であり、実機の原点復帰完了通知ではない。必要な待機は後手順の先頭へ `wait` を記述する。初期化前後では姿勢が変わるため、前手順の絶対座標を後手順の相対移動へ引き継がない。各手順内で絶対移動を先行させる必要がある。通常の `steps` に `initialize` を直接記述する構文は用意していない。

前手順の失敗、またはシーケンサが前手順中に取消を受信した場合は初期化命令を送らず、初期化命令の拒否・失敗では後手順を実行しない。UIからの取消は非同期で伝わるため、取消操作直後でもシーケンサが受信する前に次の命令が送られる場合がある。既に送信した初期化命令の効果は取消できない。終了は現在動作の取消完了を待ってから実行し、cancel／reset・動作失敗・Ctrl+Cで終了手順を自動実行しない。`debug:=true` では初期化・終了の実行前にもファイル全体を再検証する。

## 数値のエイリアスと継承

`values` に共通の有限数を定義し、数値を書く場所で `'$名前'` を参照できる。値の別名を定義して複数段の継承もできる。宣言順に制限はない。

```yaml
values:
  work_above_z: 300
  front_above_z: '$work_above_z'
  grip_depth: -20
  grip_wait: 0.2
  rotate_command: 1
poses:
  front: [100, 200, '$front_above_z', 0]
  back: {extends: front, x: 150, z: '$work_above_z'}
```

上の数値と後述の数値は構文説明用であり、実機の座標・高さ・待機時間を示すものではない。

参照は絶対・相対座標の4成分、姿勢継承の軸別上書き、`wait`、`endeffector` で使用できる。例えば `wait: '$grip_wait'` や `endeffector: '$rotate_command'` と書く。ポンプは `suction`／`off`／`release` の文字列を指定する。

値は単一数値または `'$別名'`。演算式やオブジェクトは扱わない。`values` 自体の継承専用構文はなく、各値のエイリアスと `poses` の `extends` を組み合わせて再利用する。YAML標準のスカラーアンカー `&height` とエイリアス `*height` も使用可能。YAMLの `<<` マージキーは扱わない。未使用の値についても未知参照・循環参照・NaN・無限大を読み込み時に拒否する。

## 絶対姿勢

姿勢は `[x, y, z, phi]` の4成分。位置は **mm**、角度は **rad** で、既存経路生成ノードと同じ座標系を使用する。陣営に応じた鏡映やUI座標からの自動変換は行わず、陣営ごとの `bindings` に使用する手順を明示する。同じ姿勢・手順を赤青両方の割り当てから参照して共用することもできる。

```yaml
poses:
  work_above: [100, 200, '$work_above_z', 0]
  work_alias: work_above
  next_work_above: {extends: work_above, y: 250}
  recovery_above: [200, 100, '$work_above_z', 0]
```

文字列は別の姿勢のエイリアス。`extends` は1姿勢を継承し、`x`／`y`／`z`／`phi` の指定された成分だけを絶対値で上書きする。配列の各成分と上書き値のどちらでも数値エイリアスを使える。

`phi` は経路の手先姿勢角である。後述の `endeffector` は回収機構の幅を切り替える既存サービスに送る0／1であり、物理角度への換算ではない。0／1と実機の幅との対応は実機側で確認する。

## 手順・別名・継承

```yaml
sequences:
  approach:
    steps:
      - move: {absolute: work_above}
  collect_standard:
    steps:
      - move: {relative: [0, 0, '$grip_depth', 0]}
      - pump: suction
      - wait: '$grip_wait'
      - move: {relative: [0, 0, 0, 0]}
  collect_front: collect_standard
  collect_rotated:
    steps:
      - endeffector: '$rotate_command'
      - call: collect_standard
  front_pick:
    extends: approach
    steps:
      - call: collect_front
  other_pick:
    steps:
      - move: {absolute: next_work_above}
      - call: collect_rotated
  recovery:
    steps:
      - move: {absolute: recovery_above}
      - pump: release
      - wait: '$grip_wait'
      - pump: off
```

シーケンスの文字列は別のシーケンス全体のエイリアス。`call` は指定位置へ別のシーケンスを展開する。`extends: approach` は親の全ステップの後ろへ自分の `steps` を追加する。複数親は `extends: [approach, collect_front]` のように指定し、列挙順に連結した後で自分の `steps` を追加する。親のステップ番号を上書きする機能は持たない。

`extends` のみの定義や `steps` のみの定義も可能だが、明示的な `steps: []` は初期化前後・開始・終了手順の編集用として使用できる。空手順の別名や継承・`call` も使用できるが、PICK／PLACEの割当が展開後に空なら拒否する。共通部分ごとに手順名を設け、位置別のシーケンスが使用する名前を切り替えることで、回収パターンを変更できる。

| ステップ | 内容 |
|---|---|
| `move: {absolute: 名前}` | 名前付き絶対姿勢へ移動 |
| `move: {absolute: [x, y, z, phi]}` | 直接指定した絶対姿勢へ移動 |
| `move: {relative: [dx, dy, dz, dphi]}` | 直近の絶対姿勢を基準にした姿勢へ移動 |
| `waypoint: {absolute: 名前または4成分配列}` | 次の通常MOVEへ向かう経由点 |
| `waypoint: {relative: [dx, dy, dz, dphi]}` | 直近の絶対姿勢を基準にした経由点 |
| `move: {absolute: 名前, waypoints: [経由点, ...]}` | 経由点列を通って目標へ移動（relative目標にも指定可） |
| `move: {absolute: 名前, waypoint: true}` | 従来形式の経由点。単独の `waypoint` と同じ（relativeにも指定可） |
| `call: 名前` | 指定シーケンスをその位置に展開 |
| `pump: suction` | UIが指定した回収機構を吸引（1） |
| `pump: off` | UIが指定した回収機構をオフ（0） |
| `pump: release` | UIが指定した回収機構を開放（-1） |
| `endeffector: 0` または `1` | エンドエフェクタサービスへの指令 |
| `wait: 秒数` | 0〜86,400の有限秒数だけ待機 |
| `sequence_group: start` | 共通の計画・制約を適用する区間を開始 |
| `sequence_group: {start: true, rotation_group: true}` | 第4関節の回転方向を固定する区間を開始 |
| `sequence_group: {start: true, max_phi_travel: ラジアン}` | 手先角`phi`の総移動量を制限する区間を開始 |
| `sequence_group: end` | シーケンスグループを終了 |
| `rotation_group: start`／`end` | 従来形式。第4関節の一方向制約を指定する互換表記 |

1ステップに操作を2つ書くことはできない。通常の `move` は経路生成と経路追従の成功応答を待ってから次へ進む。絶対姿勢のワーク直上への移動に続けて、共通回収動作を記述する。経路APIは [README.md](README.md#実行とrosインターフェース) を参照。ポンプとエンドエフェクタはサービス応答の成功を待つが、吸着成立や機構の回転完了を観測するものではない。必要な待機を `wait` で指定する。

### 経由点（waypoint）

`waypoint` ステップは、その座標を次の通常 `move` へ向かう経路の経由点にする。連続する複数の経由点と最後の通常移動を、1回の経路生成・追従で実行する。中間点ごとの位置・角度の到達待ちと経路切替がなくなり、最後の移動先で通常の到達判定を行う。絶対座標は `poses` の名前または4成分配列、相対座標は4成分配列で指定する。以下は終了手順を経由点方式に変更する例で、現在の同梱設定は通常の `move` 2件になっている。

```yaml
ending:
  steps:
    - waypoint: {absolute: [675, 200, 300, 0]}
    - move: {absolute: lifecycle_pose}
```

経由点1件だけの共通シーケンスも定義できる。次の例では `call` した青側経由点と、その後の `move` を一つの経路として実行する。同梱YAMLの赤青全16 PLACEもこの形式で指定している。

```yaml
sequences:
  place_pre_blue_waypoint:
    steps:
      - waypoint: {absolute: [1200, 0, 360, 3.14159265358979]}
  place_blue_0_0:
    steps:
      - call: place_pre_blue_waypoint
      - move: {absolute: place_blue_0_0_above}
      - call: place_common
```

`extends: place_pre_blue_waypoint` の後に `steps` で通常 `move` を追加してもよい。`call`／`extends` の展開後、経由点の次は別の経由点または `move` でなければならず、最後は通常の `move` で終える。途中にポンプ・幅切替・待機が入る場合や、実行シーケンス末尾が経由点の場合は読み込み時に拒否する。経由点だけの共通定義は可能だが、それだけを `bindings` やライフサイクル手順に直接割り当てて実行することはできない。別のUIアクションや初期化前後の境界へ経由点を持ち越すこともない。

既存の `move: {absolute: ..., waypoint: true}` も同じ意味で使用できる。`relative` にも指定でき、`waypoint: false` または省略時は通常の移動になる。このオプションの値は `true`／`false` のみで、距離の許容値ではない。

### moveの経由点列（waypoints）

通常の `move` に `waypoints` オプションを追加し、その移動だけの経由点を順番に指定することもできる。次の例は「青側経由点→直接指定の経由点→PLACE目標」の1経路になる。

```yaml
poses:
  place_blue_via: [1200, 0, 360, 3.14159265358979]
sequences:
  place_with_waypoints:
    steps:
      - move:
          absolute: place_blue_0_0_above
          waypoints:
            - place_blue_via
            - [1180, -40, 350, 3.14159265358979]
      - call: place_common
```

各要素は `poses` の名前、4成分の絶対座標配列、`{absolute: 名前または配列}`、`{relative: [dx, dy, dz, dphi]}` のいずれか。名前は姿勢名であり、`place_pre_blue_waypoint` のようなシーケンス名は指定しない。省略または `waypoints: []` は経由点列なしになる。通常の `move` の最終目標には、従来どおり `absolute`／`relative` のいずれか一つが必要。

`waypoints` 内の相対座標は、**そのmoveより前の直近absolute** が固定基準になる。列内の絶対座標は基準を更新せず、同じmoveの最終absoluteも先取りしない。例えば先行姿勢が `[100, 200, 300, 0]` なら、次の列の相対経由点は `[100, 200, 350, 0]`、相対の最終目標は `[100, 200, 280, 0]` になる。

```yaml
steps:
  - move: {absolute: [100, 200, 300, 0]}
  - move:
      relative: [0, 0, -20, 0]
      waypoints:
        - {absolute: [150, 200, 350, 0]}
        - {relative: [0, 0, 50, 0]}
```

先行する単独の `waypoint` と `move.waypoints` を併用した場合は「先行経由点→そのmoveの経由点列→そのmoveの目標」の順に連結する。旧形式の `move.waypoint: true` と `waypoints` も併用でき、そのmoveの目標も経由点として次の通常 `move` へつながる。

単独の `waypoint: {absolute: ...}` は従来の絶対MOVEと同様に相対座標の基準を更新する。`waypoints` オプション内の座標は基準を変更しない。`phi` は経路上で補間されるが、経由点の姿勢一致を個別には待たない。既存のスプラインと先読み追従を使うため、実際の軌跡は経由点の座標からずれる場合や、点間の直線より外側へ膨らむ場合がある。厳密な到達を要する位置には通常の `move` を使う。機構操作はまとめた経路の追従成功後に実行する。

`route_timeout_sec` はまとめた1経路全体に適用する。YAMLの編集反映には通常の再読込規則を使うが、この機能の初回導入ではサービス型が変わるため、ワークスペース全体の再ビルドと関係ノードの再起動が必要。

### シーケンスグループ（sequence_group）

`sequence_group` の開始・終了で対象区間を指定し、開始時のオプションで第4関節の一方向制約と手先角`phi`の総移動量制限を設定する。両者は独立して指定でき、併用もできる。`phi`はフィールド基準の手先角度、第4関節角`q4`は手首軸の角度で、ベース軸の角度`q0`を含む`phi = q0 + q4`の関係にある。

```yaml
values:
  phi_budget: 1.5707963267948966  # 90 degrees, in radians.
sequences:
  limited_motion:
    steps:
      - sequence_group: {start: true, max_phi_travel: '$phi_budget'}
      - move: {absolute: target_a}
      - pump: release
      - wait: 0.5
      - move: {absolute: target_b}
      - sequence_group: end
```

これは書式例で、`target_a`／`target_b`は別途`poses`に定義する。上限値は機構と手順に合わせて指定する。この例ではグループ開始時の姿勢からAまでと、AからBまでの両方を累計する。**A到達後からBまでだけを制限したい場合は、Aへの通常MOVEの後に開始フラグを置く。**

```yaml
steps:
  - move: {absolute: target_a}
  - sequence_group: {start: true, max_phi_travel: 1.5707963267948966}
  - move: {absolute: target_b}
  - sequence_group: end
```

| 開始時の設定 | 意味 |
|---|---|
| `sequence_group: start` または `{start: true}` | グループ全体を事前計画する。一方向制約・総移動量上限は追加しない |
| `rotation_group: true` | グループ内の第4関節角`q4`を一方向にだけ変化させる |
| `rotation_group: false` または省略 | 第4関節角`q4`の方向反転を許可する |
| `max_phi_travel: 数値` | 手先角`phi`の総移動量をラジアンで制限する。有限の0以上、`values`参照可 |
| `max_phi_travel`を省略 | 手先角`phi`の総移動量上限を追加しない |

オプションを併用するときは`sequence_group: {start: true, rotation_group: true, max_phi_travel: 1.5707963267948966}`のように書く。オプションのマップには`start: true`が必要で、終了は`sequence_group: end`とする。`rotation_group`の値は`true`／`false`のみ。単位はradであり、90°は約1.5708rad、360°は約6.2832radになる。

総移動量は、2πで丸めない関節指令から計算した`phi = q0 + q4`について、連続する指令間の角度差の絶対値を加算した`Σ|Δphi|`。グループ開始時から数え、通常MOVE・経由点・機構操作・待機をまたいで累計し、グループ終了でその区間を終える。開始角と終了角の差だけではない。たとえば0°→90°→0°は180°、0°→360°は360°として扱う。Quaternionや`atan2`で同じ姿勢に見えても、1回転を0として扱わない。上限0も有効だが、経路上の`phi`指令が一定であることを要求するため、同じ`phi`の終点を指定しただけでは実行可能とは限らない。

総移動量だけを制限する場合は、`rotation_group`を省略する。`q4`の方向反転を許すことで、ベース軸の回転に合わせて`phi`の変化を抑えられる場合がある。`rotation_group: true`の場合は、第4関節の既存ソフト範囲`[-2π, 0]`radに収まる正方向または負方向をグループ全体から自動選択し、停止や同じ角度の保持を許可する。`q4`が一方向でも、フィールド基準の`phi`は同じ方向に進むとは限らない。

方向制約を指定しない場合は、全目標の巻き数候補を先読みし、総量の小さい`phi`補間を選ぶ。方向制約と上限を併用する場合も、両条件を満たす`phi`補間を先に探し、必要に応じて第4関節角の補間を検査する。既存のXYZ経路に対する角度候補の計画であり、経路形状を変えて迂回する探索は行わない。

開始フラグに到達すると、グループ内の全経路を計画してから最初の操作を実行する。設定した制約を同時に満たせない場合や、開始時の関節情報が未受信・古い・不正な場合は、**そのグループの移動・ポンプ等を実行する前に失敗**する。グループより前の手順は既に実行されている場合がある。開始姿勢によって可否が変わるため、YAMLの読み込み成功だけで実行可能とは判定しない。

グループ内は、巻き数を保つ第4関節の指令角を使って経路を追従する。中間の`phi`は通常の最短角度補間から変わる場合がある。通常MOVEの終点では位置・手先角度に加えて、実測第4関節の角度差を2πで丸めずに到達判定する。経由点では個別の到達待ちを行わない。総移動量は事前計画と実行時の指令で検査する**ソフト上の指令制限**であり、実測した物理回転量や機構の停止・干渉回避を保証するものではない。

各開始・終了フラグは独立した1ステップとして記述する。次の規則を`call`／`extends`の展開後に検証する。

- グループを入れ子にせず、開始と終了を必ず対応させる。1グループに少なくとも1件の移動が必要。
- ポンプ・幅切替・待機をグループ内に置ける。全体を事前計画した後も、操作順と通常MOVEの到達待ちは維持する。
- 開始だけ、または終了だけの共通手順を定義できる。呼び出し側を含む同じUIアクション内で対応させる。別のPICK／PLACE／開始／終了へは持ち越さない。
- 初期化命令をまたぐグループは作れない。初期化前と初期化後の手順で、それぞれ開始・終了を対応させる。
- 経由点列をグループ境界で分割しない。開始は最初の`waypoint`より前、終了は最後の通常`move`より後に置く。`move.waypoints`も使用できる。
- グループ境界は相対移動の固定基準を変更しない。絶対移動の基準規則は前後で継続する。

グループ中はJoyの手動移動補正と直接の初期化操作を受け付けない。終了フラグ・取消・失敗ではグループを解除し、Joyは最後に有効だった4関節の指令を保持する。解除応答や追従停止の結果が不明なら以後の動作を拒否する。保持は実機の停止確認や非常停止を意味しない。

グループの第4関節指令の速度上限は、追従ノードのROSパラメータ`rotation_speed_rad_sec`（既定`1.0`rad/s、有限の正数）で指定する。これはソフト上の指令速度制限で、実機で保証された速度ではない。`route_timeout_sec`はグループ内の各経路、`sequence_timeout_sec`は待機と計画を含むUIアクション全体に適用される。

### 従来の回転グループ（rotation_group）

独立ステップの`rotation_group: start`／`rotation_group: end`は互換表記として使用できる。`sequence_group: {start: true, rotation_group: true}`／`sequence_group: end`と同じ意味で、`phi`の総移動量上限は設定しない。現在の同梱`sequences.yaml`のendingは`sequence_group`内の`rotation_group: true`で第4関節の一方向制約を指定し、`phi`の総移動量上限は未設定。**既存の回転グループ設定だけでは、A・Bの`phi`が同じでも途中の360°回転を制限しない。**

シーケンスグループには`PlanRotationGroup`／`WristControl`サービスと`FollowRoute`の追加フィールドを使用する。今回の型更新に合わせ、ワークスペース全体を再ビルドし、シーケンサ・経路生成・経路追従・Joyを含む関係ノードを再起動する。

### 相対姿勢の基準

相対姿勢の基準は、同じPICK／PLACEの中で**直近の `move.absolute` または単独の `waypoint.absolute`**。`move.waypoints` 内の絶対座標は含めない。直前の相対姿勢からの積算ではなく、軸は固定された経路座標系で、`phi` による局所軸の回転は行わない。`phi` も基準角に `dphi` を加算する。

例えば絶対姿勢 `[100, 200, 300, 0]` の後に、相対姿勢 `[0, 0, -20, 0]`、`[0, 0, 0, 0]` と指定すると、絶対姿勢 `[100, 200, 280, 0]` へ下がり、`[100, 200, 300, 0]` へ戻る。途中のポンプ・待機・エンドエフェクタ指令・シーケンスグループの開始と終了では基準は変わらない。別の絶対移動を挿入すると、以後はその姿勢が基準になる。`call`／`extends` の境界でも同じ規則が続く。

相対移動から始まる共通シーケンスは定義できる。ただし `bindings` に登録する実行シーケンスでは、参照の展開後に全ての相対移動より前に絶対移動が必要となる。開始時のロボット現在姿勢を暗黙の基準にはしない。

## UI位置の割り当て

```yaml
bindings:
  red:
    pick:
      '0,1': front_pick
      '0,2': other_pick
    place:
      '0,0': recovery
  blue:
    pick: {}
    place: {}
```

PICKのキーは `'row,column'`。UIの `row=0..3` とアーム中心の `column=1..4` を使う。PLACEのキーは `'box,box_column'`。`box=0..3`、`box_column=0` が奥、`1` が手前。キーには空白を入れない。既存CSVの1始まり添字とは異なる。

**回収機構の選択は設定ファイルに書かない。** PICKの `collector_mask` とPLACEの保持対象はUIから渡され、シーケンスノードがポンプ指令に適用する。各 `pump` 手順では選択された機構に設定値を送り、非選択機構は全てオフ（0）にする。`pump: off` では全3機構がオフとなる。L／C／Rごとの設定キーや `collector_mask` をYAMLに書くと未知キーとして拒否される。

## 再読み込みと検証

実行前の経路検査には`ros2 run catchrobo2026_sequence check_sequence_config --config PATH`を使う。開始関節角または姿勢と、実行するアクション列を指定し、本番と共通の計画処理で制約を検査できる。初期状態の指定がなければ、姿勢を仮定せず`UNKNOWN`とする。`--syntax-only`は従来の読込・参照・展開の検証だけを行う。コマンド例・結果の意味・検査範囲は[実行前のオフライン検査](README.md#実行前のオフライン検査)を参照。

通常時は起動時に1回読み込み、その内容を使い続ける。デバッグ時はUIから受け付けるPICK／PLACEの**動作全体の開始前に1回**再読み込みする。各ウェイポイントでは読み直さないため、実行中の1動作は同じ設定を使い切り、編集内容は次の動作へ反映される。

読み込み時に全ての数値・姿勢・シーケンス参照と有効な割り当てを検証する。不明な参照、循環、未知キー、重複キー、非有限値、不正なポンプ／エンドエフェクタ指令、範囲外の待機時間、実行シーケンスの基準なし相対移動、回転グループの開始・終了の不整合や入れ子・移動なしのグループは実機指令の前に拒否する。再読み込みに失敗した動作では古い設定にフォールバックして実行しない。

待機は1ステップ最大86,400秒（1日）という運用上限を設ける。数値が有限でも極端に大きい値はタイマーの時間表現を超えるため、数値エイリアスで指定した場合も同じ上限を適用する。

参照の深さは数値・姿勢・シーケンス各64まで、1シーケンスの展開後ステップ数と `move.waypoints` の点数の合計は10,000まで。初期化は前手順＋初期化命令1件＋後手順の合計にも同じ上限を適用する。座標・相対加算結果は有限であることを検証するが、実機の可動範囲・障害物・吸着成立をこのファイルの構文検証だけで保証するものではない。
