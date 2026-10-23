# 学祭用シーケンス設定

現在の実行対象は初期化・START・ENDです。PICK／PLACEの座標、bindings、回転ステップは運用設定から削除しています。任意のワーク位置への移動と吸引はUIまたはJoyで手動操作します。

[sequences.yaml](config/sequences.yaml)と[sequences.example.yaml](config/sequences.example.yaml)は同じ雛形です。位置の単位はmm、phi欄の単位はradです。

```yaml
version: 1
route_timeout_sec: 180.0
before_initialization_sequence: before_initialization
after_initialization_sequence: after_initialization
start_sequence: startup
end_sequence: ending
values:
  startup_width_command: 1
poses:
  lifecycle_pose: [675, -110, 220, 0]
sequences:
  before_initialization:
    steps: []
  after_initialization:
    steps: []
  startup:
    steps:
      - endeffector: '$startup_width_command'
  ending:
    steps:
      - move: {absolute: [675, 400, 300, 0]}
      - move: {absolute: lifecycle_pose}
bindings:
  red: {pick: {}, place: {}}
  blue: {pick: {}, place: {}}
```

`poses`、`sequences`、`bindings`は必須です。互換文法のため空のred/blue・pick/placeマップを保持しますが、公開actionはSTART=4、INITIALIZE=5、END=6だけを許可します。`control_epoch`と`step_id`は非ゼロ、`collector_mask`は1..7が必要で、実ポンプの選択は共通`selected_pump`が優先されます。

MOVEを含む動作は開始時の実測`current_joints`から`phi = q0 + q3`を固定し、すべてのMOVE・waypointのphi欄を置き換えます。4要素すべてが有限で1秒以内の実測が必要です。開始時に条件を満たさなければ手順全体を実行しません。回転グループ、sequenceグループ、phi区間は学祭運用では拒否されます。4軸のIK/FKと内部関節角出力は維持されます。実行時には各経路を連続関節角の内部グループに変換し、phi移動量0を強制します。公開YAMLの回転グループを有効にする変更ではありません。固定phiで到達できない場合は拒否し、2π回転へ切り替えません。

基本手順には次を指定できます。各ステップは操作を1種類だけ持ちます。

```yaml
steps:
  - move: {absolute: [670, -110, 220, 0]}
  - move: {relative: [0, 0, 20, 0]}
  - pump: suction
  - wait: 0.5
  - pump: release
  - pump: off
  - endeffector: 1
```

相対座標は直近のabsoluteを固定基準にし、前のrelativeから積算しません。`waypoint: {absolute: ...}`は次の通常MOVEまでを1経路にまとめ、通常MOVE終点で到達を待ちます。手順末尾の未接続waypointは初期化・START・ENDで使用できません。`move.waypoints: [[x, y, z, phi], ...]`も使用でき、経由点のphiも開始時実測値へ置き換えます。

`pump`の値は`suction`／`release`／`off`です。選択は[festival.yaml](../catchrobo2026_pump/config/festival.yaml)の`selected_pump: 1..3`で統一します。1/2/3は既存PumpControlのleft/center/rightというROSスロットであり、物理配置を意味しません。非選択スロットはOFFです。シーケンス内の`check_suction`も同じ選択スロット1bitだけを要求します。

`endeffector`は幅機構の既存0/1指令です。同梱STARTは従来の1を保持し、広い／狭いの実機対応をここで推測しません。サービスの受理応答で次へ進み、機構の物理完了通知は待ちません。

`call: sequence_name`は別手順を展開し、`extends`は継承元の後へ手順を追加します。`'$name'`は`values`内の数値を参照し、計算式は使えません。`manual: true`などの手動待ちステップは、学祭UIに完了操作がないため手順全体の最初の操作前に拒否します。

初期化前後の手順は既存Triggerの前後に実行します。初期化の受理は原点復帰完了ではありません。ENDは明示要求時のみ実行し、STOP・取消・reset・失敗・Ctrl+Cで自動実行しません。

`route_timeout_sec`は1経路の上限で、YAML指定が同名ROSパラメータより優先されます。値は有限の0秒超～86400秒です。全体上限は別の`sequence_timeout_sec`パラメータです。`debug:=true`は各動作開始前にYAMLを再読込します。実行中は変更しません。起動引数`sequence_file`が編集対象と一致していることを確認してください。

旧競技用の汎用文法・静的解析はライブラリに残っていますが、公開actionの受付制限と学祭用回転拒否が優先されます。旧設定を読み込めてもPICK／PLACEは実行できません。実行制約と停止動作は[README](README.md)を参照してください。
