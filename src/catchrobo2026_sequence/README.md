# 学祭用ライフサイクル制御

学祭では任意の1箇所へ手動で移動してワークを取得します。`execute_sequence` はUIの初期化・START・ENDだけを受け付け、PICK／PLACEとその他のkindを拒否します。競技用の位置・共通手順・bindingsは同梱YAMLから削除しました。

[config/sequences.yaml](config/sequences.yaml) は次の動作です。[設定仕様](CONFIG.md)と[同内容の雛形](config/sequences.example.yaml)も参照してください。

- 初期化: 前後手順は空。既存の`request_initialization`を呼び出します。受理応答は実機の原点復帰完了を意味しません。
- START: 既存の幅指令`1`を送ります。幅切替機構は保持しています。
- END: `[675, 400, 300]` → `[675, -110, 220]` mmの順で移動し、各到達を待ちます。旧回転グループとphi移動量区間は削除しました。終点Xはphi=0での手首上限越えを避けるため675 mmへ変更し、Joyの起動姿勢X=670 mmは維持しています。

MOVEを含む動作は開始時の`current_joints`から`phi = q0 + q3`を読み、すべての終点と経由点のphiを同じ値へ置き換えます。4要素の有限値を1秒以内に受信していない場合、幅指令を含め最初の操作前に失敗します。YAMLのphi欄は運用時には使いません。回転・phiグループ、`manual: true`等の手動待ちを含む設定も操作前に拒否します。学祭UIにシーケンスの手動待ちを完了する操作はありません。内部の4軸運動学・関節出力は維持され、並進に必要な第4軸の補償は行われます。各経路は内部で既存の連続関節角計画を使い、phi移動量0の制約で実行します。通常Poseの±π境界による第4軸の2π巻替えを防ぎ、固定phiで計画できない場合は動作を拒否します。

ポンプは[共通設定festival.yaml](../catchrobo2026_pump/config/festival.yaml)の`selected_pump: 1..3`で1つを選びます。UI・Joy・sequence・pumpノードが同じ`festival_config`を読みます。既定`1`は変更可能な仮値で、物理的な左右やMCU配線を確認した値ではありません。sequenceのpump手順は選択したROSスロットだけを操作します。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  festival_config:="$PWD/src/catchrobo2026_pump/config/festival.yaml" \
  debug:=true
```

`automatic.launch.py`はUI・sequence・経路生成／追従・Joy・幅・ポンプを起動します。CANと実測関節角は別途供給します。陣営引数は互換用に残り、既定`red`ですが競技位置の選択には使いません。Joy入力は`joy_source:=web`だけを許可します。`local`はUIのSTOP状態を迂回するため起動引数で拒否します。単独の手動launchは従来のローカルJoyを利用できます。他の手動launchと同時起動しないでください。

`debug:=true`は各動作前にsequence YAMLを読み直し、実行中は開始時の手順を維持します。`festival_config`は起動時だけ読み込むため、選択ポンプ変更後は関係する全ノードを再起動してください。STOPは実行中actionの取消として処理し、END手順を自動実行しません。取消の確認が取れない場合はfaultとして次動作を拒否し、ノード再起動が必要です。

構文だけをロボットへ指令せず確認できます。

```bash
ros2 run catchrobo2026_sequence check_sequence_config \
  --config src/catchrobo2026_sequence/config/sequences.yaml --syntax-only
```

`check_sequence_config`の名前付き手順・旧bindings・回転制約検査は設定ライブラリ用に残っています。これらの文法を検査できても、学祭用`execute_sequence`が競技goalや回転グループを許可することにはなりません。オフラインの経路検査はYAMLに記載されたphiを使うため、開始時実測phiを保持する運用動作の代わりにはなりません。
