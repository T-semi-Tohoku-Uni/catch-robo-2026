# シーケンス設定

シーケンスノードは起動時に指定した陣営とUIのPICK／PLACE・位置を `bindings` で手順名へ変換し、移動・ポンプ・エンドエフェクタ・待機を順番に実行する。設定は通常のYAMLファイルであり、ROSパラメータファイルとは別に用意する。

既定の [sequences.yaml](config/sequences.yaml) は、座標とデバッグ用の初期手順を記入した読み込み可能な設定。[sequences.example.yaml](config/sequences.example.yaml) は赤青それぞれ16 PICK位置・8 PLACE位置の入力欄を持ち、全て `null` で無効にした雛形として残している。雛形から作る場合は必要な値・姿勢・手順を定義し、使用する位置の `null` を手順名へ変更する。欄の省略も未設定扱いとなり、選択時に実行を拒否する。

### 座標を転記したデバッグ用設定

[sequences.yaml](config/sequences.yaml) は [運動学パッケージの座標CSV](../CatchRobo2026_kinematics/config/flange_targets/) から初期座標を静的に転記した設定。ノードとlaunchの既定ファイルであり、稼働時に読むのはYAMLだけ。CSVローダや生成スクリプトによる同期は行わない。

| 対象 | 転記時の対応 |
|---|---|
| PICK | UIの `row=0..3`、`column=1..4` に対してCSVの `ID = row * 6 + column + 1`。16姿勢を赤青で共用 |
| PLACE | 陣営別CSVの `GroupIdx = box + 1`、`RowIdx = box_column + 1`、中央の `ColIdx = 2`。赤青それぞれ8姿勢 |

高さは `values.work_above_z=166.95`（自陣PICK）、`common_work_above_z=186.95`（共通エリアPICK）、`place_above_z=294.35`（PLACE）を姿勢から参照する。単位はmm。いずれもCSV値の初期転記であり、`above` という名前が実機での上空高さや余裕量の確認済みを意味するものではない。

位置別PICKは絶対姿勢へ移動した後に `pick_common` を呼ぶ。「吸引判定開始→吸引→相対接近→相対退避→判定結果待ち」を最大 `values.pick_max_attempts` 回繰り返し、成功時に `break` で抜けてPLACE用の幅へ切り替える。上限まで失敗した場合はPICK全体を失敗にする。仮設定は最大3回・判定期限 `values.pick_suction_timeout_sec=2.0` 秒。期限は判定要求から数えるため、接近・退避を含む動作に合わせて調整する。PLACEは陣営別の絶対姿勢に続けて「開放→相対接近→相対退避→オフ→PICK用の幅に切替」を実行する。初回はUIの「開始」でPLACE用の幅を指令する。移動量・再試行回数・期限は実機で調整する。

幅切替の指令値は `values.pick_endeffector_command: 0` と `values.place_endeffector_command: 1` に仮置きし、PLACE末尾で前者、PICK末尾で後者を参照する。0/1の広い／狭い対応は手動で確認し、必要なら値を入れ替える。ユーザー指定により幅切替後の `wait` は追加しておらず、ROSサービスの受理応答でそのPICK／PLACEを完了する。機構動作の完了通知は待たない。

現在の相対移動量は `pick_approach_dz`／`place_approach_dz` が `-10 mm`、`pick_retreat_dz`／`place_retreat_dz` が `+10 mm`。相対座標の基準は直近の絶対姿勢なので、各手順の2回の相対目標はその姿勢のZ−10／Z＋10 mmとなる。例えば自陣PICKのZ指令は `166.95 → 156.95 → 176.95 mm`。移動量の符号や復帰先を変える場合は、それぞれの値を直接編集する。

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

`version`、`poses`、`sequences`、`bindings` は必須。`values` と `before_initialization_sequence`／`after_initialization_sequence`／`start_sequence`／`end_sequence` は省略可能。`bindings` には赤青両方と各 `pick`／`place` のマップを用意する。マップは空でもよい。未知キー・重複キー・複数YAMLドキュメントは拒否する。

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
    steps:
      - move: {absolute: lifecycle_pose}
  ending:
    steps:
      - move: {absolute: lifecycle_pose}
```

上記は後手順に移動を追加した例。同梱設定の初期化前後はどちらも `steps: []` で、初期化命令のみを実行する。終了手順は `lifecycle_pose`（x=670 mm、y=-110 mm、z=220 mm、phi=0 rad）へ絶対移動する。必要な移動・ポンプ・幅指令・待機を各 `steps` に追記する。各キーの省略または `null` は追加動作なしを意味し、初期化命令自体は実行する。`call`・`extends`・数値参照は開始手順と同様に利用できる。これらのポンプ手順は全3機構が対象。

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
| `call: 名前` | 指定シーケンスをその位置に展開 |
| `pump: suction` | UIが指定した回収機構を吸引（1） |
| `pump: off` | UIが指定した回収機構をオフ（0） |
| `pump: release` | UIが指定した回収機構を開放（-1） |
| `endeffector: 0` または `1` | エンドエフェクタサービスへの指令 |
| `wait: 秒数` | 0〜86,400の有限秒数だけ待機 |
| `suction_check: {start: {timeout: 秒数}}` | UI選択機構の空気圧監視を要求し、次手順へ進む |
| `suction_check: wait` | 監視の成功または期限切れを待ち、結果を保存 |
| `if: {condition: suction_success, then: [...], else: [...]}` | 保存済み結果で分岐。`suction_failure` も使用可能、`else` は省略可 |
| `for: {max_iterations: 回数, steps: [...]}` | 初回を含む最大回数まで繰り返す |
| `break: true` | 最内側の `for` を抜ける |
| `fail: メッセージ` | アクションを失敗として終了する |

1ステップに操作を2つ書くことはできない。各 `move` は経路生成と経路追従の成功応答を待ってから次へ進む。絶対姿勢のワーク直上への移動に続けて、共通回収動作を記述する。経路APIは [README.md](README.md#実行とrosインターフェース) を参照。`pump` と `endeffector` の応答は指令受理を表す。吸引成否は独立した `suction_check` で確認する。幅切替・開放完了を観測する機能はなく、必要な待機を `wait` で指定する。

### 吸引判定と条件分岐・繰り返し

以下は `pick_common` の編集例。位置別の先行する絶対移動を基準に、同じ接近・退避を繰り返す。

```yaml
pick_common:
  steps:
    - for:
        max_iterations: '$pick_max_attempts'
        steps:
          - suction_check: {start: {timeout: '$pick_suction_timeout_sec'}}
          - pump: suction
          - move: {relative: [0, 0, '$pick_approach_dz', 0]}
          - move: {relative: [0, 0, '$pick_retreat_dz', 0]}
          - suction_check: wait
          - if:
              condition: suction_success
              then:
                - break: true
    - if:
        condition: suction_failure
        then:
          - fail: "Suction failed after the maximum number of attempts"
    - endeffector: '$place_endeffector_command'
```

`start` はポンプノードの `check_suction` へUIの `collector_mask` と期限を渡す。要求を送ったら次手順へ進むため、監視とポンプ・移動を並行して実行できる。選択機構すべてが同じ新規圧力サンプルで閾値条件を満たすと成功し、期限まで満たさなければ失敗結果になる。早く成功しても途中の移動は省略せず、`wait` まで進んでから分岐する。結果はその監視中の成功を保存したもので、以後の吸着維持や落下の継続監視ではない。閾値・比較方向・圧力配列の対応は [ポンプ設定](../catchrobo2026_pump/README.md) を参照。

`timeout` は0秒超〜86,400秒、`max_iterations` は1〜10,000の整数で、どちらも `'$名前'` を使える。最大回数は初回を含む。`for` 自体は回数を使い切ると次へ進むので、再試行の上限を失敗扱いにするには例のように `if` と `fail` を書く。上限失敗時に部分回収を成功扱いにはせず、ポンプ指令は保持する。

条件は直前の `suction_check: wait` の結果を使う。次の `start` で前結果を消し、同時監視は1件に限定する。すべての到達可能な経路で `start` → `wait` → 条件参照の順序が必要。重複開始・開始のない待ち・待ち前の条件参照・待ち忘れの正常終了は読込時に拒否する。初期化前後など別のライフサイクル手順へ監視を持ち越すことはできない。

`then` と `else`、`for.steps` には通常の手順や `call`、入れ子の `if`／`for` を書ける。`break` は構文上の最内側の `for` のみを抜ける。単独の共通手順に `break` だけを定義して呼び出す形式は使えない。`for` は読込時に有限回数へ展開し、分岐・breakを含む展開後の命令総数に10,000件の上限を適用する。アクション全体には従来どおり `sequence_timeout_sec` が適用される。

正常な吸引タイムアウトは `suction_failure` として分岐できる。サービス未起動・要求拒否・不正応答・通信応答期限切れは動作を停止し、吸引失敗の再試行には入らない。監視応答の通信期限は指定した `timeout` ＋ `service_timeout_sec`、サービスを見つける期限は `service_timeout_sec`。取消時は読み取り専用の監視結果を破棄し、後から届いた結果を次の動作へ使わない。ポンプ側の監視自体は成功か期限まで続く。

### 相対姿勢の基準

相対姿勢の基準は、同じPICK／PLACEの中で**直近に指定した絶対姿勢**。直前の相対姿勢からの積算ではない。軸は固定された経路座標系で、`phi` による局所軸の回転は行わない。`phi` も基準角に `dphi` を加算する。

例えば絶対姿勢 `[100, 200, 300, 0]` の後に、相対姿勢 `[0, 0, -20, 0]`、`[0, 0, 0, 0]` と指定すると、絶対姿勢 `[100, 200, 280, 0]` へ下がり、`[100, 200, 300, 0]` へ戻る。途中のポンプ・待機・エンドエフェクタ指令では基準は変わらない。別の絶対移動を挿入すると、以後はその姿勢が基準になる。`call`／`extends` の境界でも同じ規則が続く。

相対移動から始まる共通シーケンスは定義できる。ただし `bindings` に登録する実行シーケンスでは、参照の展開後に全ての相対移動より前に絶対移動が必要となる。開始時のロボット現在姿勢を暗黙の基準にはしない。

分岐や `break` の合流後に相対移動する場合、その位置へ到達する全経路で絶対基準が同じである必要がある。片側だけで異なる絶対移動をした場合は、合流後に `move: {absolute: ...}` を入れて基準を再確定する。実行しなかった枝の絶対姿勢を相対基準には使わない。

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

通常時は起動時に1回読み込み、その内容を使い続ける。デバッグ時はUIから受け付けるPICK／PLACEの**動作全体の開始前に1回**再読み込みする。各ウェイポイントでは読み直さないため、実行中の1動作は同じ設定を使い切り、編集内容は次の動作へ反映される。

読み込み時に全ての数値・姿勢・シーケンス参照と有効な割り当てを検証する。不明な参照、循環、未知キー、重複キー、非有限値、不正なポンプ／エンドエフェクタ指令、範囲外の待機時間、実行シーケンスの基準なし相対移動は実機指令の前に拒否する。再読み込みに失敗した動作では古い設定にフォールバックして実行しない。

待機は1ステップ最大86,400秒（1日）という運用上限を設ける。数値が有限でも極端に大きい値はタイマーの時間表現を超えるため、数値エイリアスで指定した場合も同じ上限を適用する。

参照の深さは数値・姿勢・シーケンス各64まで、1シーケンスの展開後ステップ数は10,000まで。初期化は前手順＋初期化命令1件＋後手順の合計にも10,000件の上限を適用する。座標・相対加算結果は有限であることを検証するが、実機の可動範囲・障害物・吸着成立をこのファイルの構文検証だけで保証するものではない。
