# 学祭用ライフサイクル制御

学祭では任意の1箇所へ手動で移動してワークを取得します。`execute_sequence` はUIの初期化・START・ENDだけを受け付け、PICK／PLACEとその他のkindを拒否します。競技用の位置・共通手順・bindingsは同梱YAMLから削除しました。

[config/sequences.yaml](config/sequences.yaml) は次の動作です。[設定仕様](CONFIG.md)と[同内容の雛形](config/sequences.example.yaml)も参照してください。

- 初期化: 前後手順は空。既存の`request_initialization`を呼び出します。受理応答は実機の原点復帰完了を意味しません。
- START: 空手順です。幅機構は撤去しています。
- END: `[675, 400, 300]` → `[675, -110, 220]` mmの順で移動し、各到達を待ちます。旧回転グループとphi移動量区間は削除しました。既定の終点X=675 mmとJoyの起動姿勢X=670 mmを維持しています。

MOVEはXYZのみで通常の経路生成・追従を実行します。`current_joints`と`target_joint_angles`は3要素で、第4軸の補償・回転制約・phi保持はありません。有限の3関節値と1秒以内の鮮度を経路生成・追従で確認します。YAMLのphi欄は運用時には使いません。回転・phiグループ、幅指令、`manual: true`等の手動待ちを含む設定は操作前に拒否します。

ポンプは[共通設定festival.yaml](../catchrobo2026_pump/config/festival.yaml)の`selected_pump: 1..3`で1つを選びます。UI・Joy・sequence・pumpノードが同じ`festival_config`を読みます。既定`1`は変更可能な仮値で、物理的な左右やMCU配線を確認した値ではありません。sequenceのpump手順は選択したROSスロットだけを操作します。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  festival_config:="$PWD/src/catchrobo2026_pump/config/festival.yaml" \
  debug:=true
```

`automatic.launch.py`はUI・sequence・経路生成／追従・Joy・ポンプを起動します。CANと実測関節角は別途供給します。陣営引数は互換用に残り、既定`red`ですが競技位置の選択には使いません。Joy入力は`joy_source:=web`だけを許可します。`local`はUIのSTOP状態を迂回するため起動引数で拒否します。単独の手動launchは従来のローカルJoyを利用できます。他の手動launchと同時起動しないでください。

`debug:=true`は各動作前にsequence YAMLを読み直し、実行中は開始時の手順を維持します。`festival_config`は起動時だけ読み込むため、選択ポンプ変更後は関係する全ノードを再起動してください。STOPは実行中actionの取消として処理し、END手順を自動実行しません。取消の確認が取れない場合はfaultとして次動作を拒否し、ノード再起動が必要です。

構文だけをロボットへ指令せず確認できます。

```bash
ros2 run catchrobo2026_sequence check_sequence_config \
  --config src/catchrobo2026_sequence/config/sequences.yaml --syntax-only
```

`check_sequence_config`の名前付き手順・旧bindings・回転制約検査は設定ライブラリ用に残っています。これらの文法を検査できても、学祭用`execute_sequence`が競技goalや回転グループを許可することにはなりません。オフラインの経路検査はYAMLに記載されたphiを使うため、XYZのみの運用動作の代わりにはなりません。

初期化ステップは`request_initialization`経由でMCUの`arm_ready`結果と必須flagsを照会します。物理rehomeを追加しません。PumpControlもMCU APPLIEDまで待ち、false/unknownや期限超過では後続MOVE/吸引判定へ進みません。BOOT/再準備はROS状態操作で、旧set_value/end_stateの疑似成功は使用しません。
