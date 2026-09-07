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

位置別PICKは絶対姿勢へ移動した後に `pick_common` を呼び、共通手順は「相対移動→吸引→相対移動→オフ→PLACE用の幅に切替」の順。PLACEは陣営別の絶対姿勢に続けて `place_common` を呼び、「相対移動→開放→相対移動→オフ→PICK用の幅に切替」を実行する。各動作の退避・ポンプオフ直後に次の動作用の幅を指令する。初回はUIの「開始」で開始シーケンスを実行し、PLACE用の幅を指令する。このPLACE操作順は実機確認済みの手順ではない。

幅切替の指令値は `values.pick_endeffector_command: 0` と `values.place_endeffector_command: 1` に仮置きし、PLACE末尾で前者、PICK末尾で後者を参照する。0/1の広い／狭い対応は手動で確認し、必要なら値を入れ替える。ユーザー指定により幅切替後の `wait` は追加しておらず、ROSサービスの受理応答でそのPICK／PLACEを完了する。機構動作の完了通知は待たない。

現在の相対移動量は `pick_approach_dz`／`place_approach_dz` が `-10 mm`、`pick_retreat_dz`／`place_retreat_dz` が `+10 mm`。相対座標の基準は直近の絶対姿勢なので、各手順の2回の相対目標はその姿勢のZ−10／Z＋10 mmとなる。例えば自陣PICKのZ指令は `166.95 → 156.95 → 176.95 mm`。移動量の符号や復帰先を変える場合は、それぞれの値を直接編集する。

実機デバッグではワークスペースのルートからソースYAMLの絶対パスを指定して起動する。以下はJazzyの例。`debug:=true` では開始シーケンス・各PICK／PLACEの実行前に再読込し、その動作の実行中は開始時の展開結果を使う。

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
start_sequence: null
values: {}
poses: {}
sequences: {}
bindings:
  red: {pick: {}, place: {}}
  blue: {pick: {}, place: {}}
```

`version`、`poses`、`sequences`、`bindings` は必須。`values` と `start_sequence` は省略可能。`bindings` には赤青両方と各 `pick`／`place` のマップを用意する。マップは空でもよい。未知キー・重複キー・複数YAMLドキュメントは拒否する。

## 開始シーケンス

トップレベルの `start_sequence` に、UIの「開始」で実行する `sequences` 内の手順名を指定する。同梱設定は以下のとおり。初期化ボタンの処理は独立したままで、開始シーケンスに初期化操作は含めない。

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

`extends` のみの定義や `steps` のみの定義も可能だが、展開後に空のシーケンスは拒否する。共通部分ごとに手順名を設け、位置別のシーケンスが使用する名前を切り替えることで、回収パターンを変更できる。

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

1ステップに操作を2つ書くことはできない。各 `move` は経路生成と経路追従の成功応答を待ってから次へ進む。絶対姿勢のワーク直上への移動に続けて、共通回収動作を記述する。既存の経路APIにおける受信順序と同時操作の制約は [README.md](README.md#実行とrosインターフェース) を参照。ポンプとエンドエフェクタはサービス応答の成功を待つが、吸着成立や機構の回転完了を観測するものではない。必要な待機を `wait` で指定する。

### 相対姿勢の基準

相対姿勢の基準は、同じPICK／PLACEの中で**直近に指定した絶対姿勢**。直前の相対姿勢からの積算ではない。軸は固定された経路座標系で、`phi` による局所軸の回転は行わない。`phi` も基準角に `dphi` を加算する。

例えば絶対姿勢 `[100, 200, 300, 0]` の後に、相対姿勢 `[0, 0, -20, 0]`、`[0, 0, 0, 0]` と指定すると、絶対姿勢 `[100, 200, 280, 0]` へ下がり、`[100, 200, 300, 0]` へ戻る。途中のポンプ・待機・エンドエフェクタ指令では基準は変わらない。別の絶対移動を挿入すると、以後はその姿勢が基準になる。`call`／`extends` の境界でも同じ規則が続く。

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

通常時は起動時に1回読み込み、その内容を使い続ける。デバッグ時はUIから受け付けるPICK／PLACEの**動作全体の開始前に1回**再読み込みする。各ウェイポイントでは読み直さないため、実行中の1動作は同じ設定を使い切り、編集内容は次の動作へ反映される。

読み込み時に全ての数値・姿勢・シーケンス参照と有効な割り当てを検証する。不明な参照、循環、未知キー、重複キー、非有限値、不正なポンプ／エンドエフェクタ指令、範囲外の待機時間、実行シーケンスの基準なし相対移動は実機指令の前に拒否する。再読み込みに失敗した動作では古い設定にフォールバックして実行しない。

待機は1ステップ最大86,400秒（1日）という運用上限を設ける。数値が有限でも極端に大きい値はタイマーの時間表現を超えるため、数値エイリアスで指定した場合も同じ上限を適用する。

参照の深さは数値・姿勢・シーケンス各64まで、1シーケンスの展開後ステップ数は10,000まで。座標・相対加算結果は有限であることを検証するが、実機の可動範囲・障害物・吸着成立をこのファイルの構文検証だけで保証するものではない。
