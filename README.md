# catch-robo-2026

キャチロボ2026のロボット制御用ROS 2ワークスペースです。

## 取得とビルド

UIと運動学はサブモジュールです。親リポジトリが記録するコミットも含めて取得します。以下はROS 2 Jazzy、colcon、rosdep、Ninjaを導入済みの環境での例です。

```bash
git clone --recurse-submodules --branch 41_ryuzot_UI_Sequence_merge \
  git@github.com:T-semi-Tohoku-Uni/catch-robo-2026.git
cd catch-robo-2026
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -G Ninja
source install/setup.bash
```

実機の既存チェックアウトでは、ワークスペースのルートで次を一度実行します。

```bash
git config --local submodule.recurse true
git submodule update --init --recursive
```

以後は通常の `git pull` で、初期化済みのUI・運動学も親リポジトリが記録するコミットへ揃います。この設定は対応するブランチ切替操作にも適用されます。設定は各クローンの `.git/config` に保存され、Gitで配布されないため、実機ごとに設定してください。新しくサブモジュールが追加されたときは `git submodule update --init --recursive` で初期化します。設定の詳細は [Git公式ドキュメント](https://git-scm.com/docs/git-config#Documentation/git-config.txt-submodulerecurse) を参照してください。

更新後は上の手順で再ビルドし、稼働ノードを再起動します。UIの配信用ファイルは同梱しているため、通常のビルドにnpmは不要です。Humble向けにも開発していますが、上記手順のローカル検証環境はJazzyです。

## 実機の本番起動（CANブリッジ込み）

ルートの [run_production.sh](run_production.sh) でROS環境を読み直し、[production.launch.py](launch/production.launch.py) を起動します。スクリプトは実行場所に依存せず、このチェックアウトの `install` を使用します。

```bash
./run_production.sh team:=red
# YAMLを動作ごとに読み直す実機調整時
./run_production.sh team:=red debug:=true
```

ROSの選択順は `ROBOT_ROS_DISTRO`、現在の `ROS_DISTRO`、`/opt/ros` にある唯一の環境です。明示する場合は `ROBOT_ROS_DISTRO=humble ./run_production.sh team:=red` のように指定します。`sh run_production.sh ...` でも実行できます。スクリプト内部でBashへ切り替え、既存のROS検索パスをクリアしてROS本体と `install/local_setup.bash` を読み込みます。`ROS_DOMAIN_ID` や `RMW_IMPLEMENTATION` などの通信設定は引き継ぎます。

本番launchは既存の `raspi_can.launch.py` でCANブリッジを起動し、activeへの遷移を確認してから自動操縦launch（UI・シーケンサ・経路生成／追従・Joy・機構ノード）を1回起動します。CANブリッジが終了すると全体も終了します。CANの設定は既存どおり `can0`、1 Mbps／CAN FD 2 Mbpsです。設定変更が必要な場合は既存launchの `sudo -n ip ...` が実行できる権限が必要です。

既定のシーケンス・キュー・ポンプ・関節フィードバック・手動操作設定は、このチェックアウトの `src/` 内のYAMLです。`sequence_file:=...`、`queue_config:=...`、`pump_config:=...`、`joint_feedback_config:=...`、`manual_config:=...`、`manual_velocity_config:=...`、`joy_source:=web|local`、`listen:=...`、`ipc_socket:=...`、`non_blocking:=...` を変更できます。`./run_production.sh --show-args` は引数を表示するだけで実機ノードを起動しません。起動後は `/current_joints` の実測値が届くことを確認してからUIで操作します。CANのactiveは関節角の受信確認ではありません。

Joy入力は既定で `joy_source:=web` です。USBコントローラーをPCへ接続してWeb UIで機器・割当を選び、送信を有効にします。Raspberry Piの従来のJoy入力を使う場合は `joy_source:=local` とします。完全手動モード・シーケンス内の手動待ち・ジョグ操作の詳細は [UIの手動操作説明](src/catchrobo2026_ui/README.md#pcのusbコントローラーと手動操作) を参照してください。ジョグ距離は [manual.yaml](src/catchrobo2026_ui/config/manual.yaml) の `manual_step_cm` で設定します。

同梱RVizでは、手動操作の指令姿勢を `/target_arm_markers`（Target Arm）、`/current_joints`から求めた実測姿勢を `/current_robot_markers`（Current Arm）で区別して表示します。`automatic.launch.py`だけを起動し実測入力がない場合、Current Armは表示されません。別PCのRVizで見る場合は`ROS_DOMAIN_ID`を合わせ、`ROS_LOCALHOST_ONLY=1`を解除してLAN上のDDS通信を有効にしてください。

手首の実測角の許容幅は [joint_feedback.yaml](src/nav_director/config/joint_feedback.yaml) の `wrist_feedback_tolerance_deg` で指定します。単位は度で、既定の `6.0` は実測値を −366°〜+6°まで受け付けます。有限の `0` 以上 `180` 未満を指定でき、`0` は数値誤差の許容だけを残します。この設定は経路生成・追従・Joyで共有し、指令角の範囲 −360°〜0°、到達判定の許容値、timeoutは変更しません。

設定は起動時に読み込むため、編集後は3ノードを再起動してください。`debug:=true` による動作ごとの再読込は対象外です。本番launchは上記ソースYAML、`automatic.launch.py`・手動launch・navのテストlaunchはインストール済みYAMLを既定で使用します。後者でソース編集を反映する場合は再ビルドするか、`joint_feedback_config:="$PWD/src/nav_director/config/joint_feedback.yaml"` を指定して起動します。

`install/_local_setup_util*.py` が欠損している場合、スクリプトは不完全な環境で起動を続けずエラーを表示します。新しい端末で対象ROS環境を読み込み、上のビルド手順で `install` を再生成してください。起動スクリプトはビルドを自動実行しません。フロントの同梱distを使用するためnpmは不要です。

## UIからの自動操縦

[シーケンスノード](src/catchrobo2026_sequence/README.md) がUIのPICK／PLACEを経路生成・追従と機構操作へ変換します。[設定仕様](src/catchrobo2026_sequence/CONFIG.md) に従い、絶対座標・相対ウェイポイント・共通値・手順のエイリアス／継承をYAMLで指定します。

ワークスペースのルートで、編集するソースYAMLを指定して起動します。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  team:=red \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  debug:=true
```

既定の [sequences.yaml](src/catchrobo2026_sequence/config/sequences.yaml) はCSV座標とデバッグ用手順を記入済みです。相対Z値は全てユーザー指定の仮値+10 mmで、各手順の2回の相対目標はどちらも直近の絶対姿勢のZ+10 mmになります。PLACEは「相対移動→開放→相対移動→オフ」の初期手順です。高さ・移動量・PLACEの操作順は実機確認済みではありません。[sequences.example.yaml](src/catchrobo2026_sequence/config/sequences.example.yaml) は全位置未設定の雛形として残しています。

調整時はYAMLを直接編集します。実行時にCSVは読み込みません。`debug:=true` はPICK／PLACEごとに設定を再読込し、`false` は起動時に1回だけ読み込みます。

## 手動操縦とダミー動作確認

手動操縦は次のコマンドで起動します。実機の `current_joints` を供給するCANブリッジ等は別途起動します。`pump_config:=/path/to/pump.yaml` でポンプ設定を差し替えられます。各並進軸の最大速度は [manual_velocity.yaml](src/catchrobo2026_hand_operated/config/manual_velocity.yaml) の `manual_linear_speed_mm_s`、最大先端回転速度は `manual_angular_speed_rad_s` で設定します。どちらもスティック最大入力時の1秒あたりの速度で、UIとローカルJoyに共通です。同じYAMLの `/**/control_node.ros__parameters` ではWebの軸・ボタン・固定移動距離・差分プリセットを指定します。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py
```

`debug:=true` と編集するファイルのパスを指定すると、以後のYAML編集は再ビルド・再起動なしで反映されます。たとえばワークスペースのルートで次のように起動します。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py \
  manual_velocity_config:="$PWD/src/catchrobo2026_hand_operated/config/manual_velocity.yaml" \
  debug:=true
```

Web UIを使う `automatic.launch.py` にも同じ2引数を指定できます。本番の `./run_production.sh team:=red debug:=true` はこのソースYAMLを既定で読みます。通常のlaunchはインストール済みYAMLが既定なので、ソースを調整するときは上記パス指定を使ってください。

Webの設定はJoy OFF・ジョグ停止中、速度はJoy入力が中立の間に読み直します。操作中はその操作の設定を維持するため、編集後はいったんJoyをOFFにし、スティック・ボタンを離してから再度有効にしてください。プリセットは選択中のIDを引き継ぎ、そのIDが削除された場合は新しい既定値へ戻ります。各ノードは担当する設定を別々の停止・中立境界で適用します。

再読込は毎回コードの既定値から構築し、削除した項目に直前の値を残しません。速度を省略した場合のコード既定値は並進50 mm/s・回転0.5 rad/sです。同梱YAMLの10 mm/s・0.1 rad/sを維持する場合は、その指定を残してください。ROSのread-onlyパラメータ表示は起動時の値で、再読込後のWeb設定はUI、速度はJoyノードの読込ログが有効値を示します。

不正な設定は適用せず、該当ノードの新たなJoy操作を拒否します。Web設定のエラーは「設定・入力確認」、速度設定のエラーはJoyノードのログで確認できます。起動時からWebのゲームパッド割当に誤りがある場合もUIと生入力表示は利用でき、Joy操作を禁止した状態で原因を表示します。ファイルを修正すると再読込で復帰します。`debug:=false` は起動時の設定を固定し、編集の反映には再起動が必要です。この再読込機能を含むコードの初回導入時はビルドと再起動が必要です。

ダミーロボットは既定で無効です。実機の `current_joints` 供給元を起動せずに動作確認する場合は、明示的に有効にします。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py \
  use_dummy:=true initial_pose:='[600.0, 200.0, 200.0, 0.0]'
```

`initial_pose` はダミーの初期手先姿勢 `[x_mm, y_mm, z_mm, yaw_rad]` です。ダミーは目標関節角を即座に現在角へ反映するため、実機の追従性能は再現しません。
