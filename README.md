# catch-robo-2026

キャチロボ2026のロボット制御用ROS 2ワークスペースです。

## 取得とビルド

UIと運動学はサブモジュールです。親リポジトリが記録するコミットも含めて取得します。以下はROS 2 Jazzy、colcon、rosdep、Ninjaを導入済みの環境で、リモートの既定ブランチを取得する一般例です。実機へ使う公開済みブランチ・コミットはチームの配布指示で確認してください。未pushの作業変更はこのcloneには含まれません。

```bash
git clone --recurse-submodules \
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
./run_production.sh
# YAMLを動作ごとに読み直す実機調整時
./run_production.sh debug:=true
# カメラを使用しない場合
./run_production.sh camera_enabled:=false
```

引数なしで、陣営red・Webコントローラー・UI `0.0.0.0:8000`・カメラ有効で起動します。カメラは`auto`で`/dev/video0`を使用し、640×480・20 FPS、映像配信は`0.0.0.0:8081`です。ブラウザーでは`http://<起動した機体のIP>:8000`を開きます。`listen:=...`や`camera_backend:=...`などの明示引数で既定値を上書きできます。

ROSの選択順は `ROBOT_ROS_DISTRO`、現在の `ROS_DISTRO`、`/opt/ros` にある唯一の環境です。明示する場合は `ROBOT_ROS_DISTRO=humble ./run_production.sh team:=red` のように指定します。`sh run_production.sh ...` でも実行できます。スクリプト内部でBashへ切り替え、既存のROS検索パスをクリアしてROS本体と `install/local_setup.bash` を読み込みます。`ROS_DOMAIN_ID` や `RMW_IMPLEMENTATION` などの通信設定は引き継ぎます。

本番launchは `canrpc_bridge/raspi_can.launch.py` でCANを設定し、全3MCUへ世代条件付きSYSTEM_RESETを送ります。新しいBoot IDとREADYを全台で確認してからブリッジをactiveにし、自動操縦launch（UI・シーケンサ・経路生成／追従・Joy・ポンプ）を1回起動します。既定30秒以内に完了しなければ全体を終了し、自動操縦は起動しません。待機時間は `startup_timeout_sec:=60.0` などで変更できます。CAN設定はこのlaunchの1箇所だけで行い、`can0`、1 Mbps／CAN FD 2 Mbpsです。設定変更が必要な場合は `sudo -n ip ...` が実行できる権限が必要です。旧標準IDのcansendと旧nhk2026_bridgeは撤去しました。

既定のシーケンス・ポンプ・関節フィードバック・手動操作設定は、このチェックアウトの `src/` 内のYAMLです。`sequence_file:=...`、`pump_config:=...`、`joint_feedback_config:=...`、`manual_config:=...`、`manual_velocity_config:=...`、`festival_config:=...`、`joy_source:=web`、`listen:=...`、`ipc_socket:=...`、`non_blocking:=...` を変更できます。引数一覧は`ros2 launch launch/production.launch.py --show-args`で確認します。起動後は `/current_joints` の実測値が届くことを確認してからUIで操作します。CANのactiveは関節角の受信確認ではありません。

学祭用の本番・automatic launchは`joy_source:=web`のみを許可します。USBコントローラーをPCへ接続してWeb UIで機器・割当を選び、送信を有効にします。`joy_source:=local`はUIのSTOP状態を迂回するため起動時に拒否します。単独の手動launchでは従来のローカルJoyを利用できます。Webゲームパッドと並進ジョグの操作は[学祭UIの説明](src/catchrobo2026_ui/README.md)を参照してください。シーケンス内の`manual`待ちは学祭UIに完了操作がないため拒否します。ジョグ距離は [manual.yaml](src/catchrobo2026_ui/config/manual.yaml) の `manual_step_cm` で設定します。

同梱RVizでは、手動操作の指令姿勢を `/target_arm_markers`（Target Arm）、`/current_joints`から求めた実測姿勢を `/current_robot_markers`（Current Arm）で区別して表示します。`automatic.launch.py`だけを起動し実測入力がない場合、Current Armは表示されません。別PCのRVizで見る場合は`ROS_DOMAIN_ID`を合わせ、`ROS_LOCALHOST_ONLY=1`を解除してLAN上のDDS通信を有効にしてください。

実測関節角は3値です。[joint_feedback.yaml](src/nav_director/config/joint_feedback.yaml) の `rotation_joint_timeout_sec`（既定1秒）を経路生成・追従で共有します。第4軸の角度範囲や回転制約は運転に使用しません。

設定は起動時に読み込むため、編集後は3ノードを再起動してください。`debug:=true` による動作ごとの再読込は対象外です。本番launchは上記ソースYAML、`automatic.launch.py`・手動launch・navのテストlaunchはインストール済みYAMLを既定で使用します。後者でソース編集を反映する場合は再ビルドするか、`joint_feedback_config:="$PWD/src/nav_director/config/joint_feedback.yaml"` を指定して起動します。

`install/_local_setup_util*.py` が欠損している場合、スクリプトは不完全な環境で起動を続けずエラーを表示します。新しい端末で対象ROS環境を読み込み、上のビルド手順で `install` を再生成してください。起動スクリプトはビルドを自動実行しません。フロントの同梱distを使用するためnpmは不要です。

## 学祭の手動取得

UIでは初期化後にSTARTし、任意の1箇所へ並進操作で移動してワークを手動取得します。競技用PICK／PLACEのキュー・座標・運用シーケンスは削除しました。公開sequence actionもPICK／PLACEを拒否します。START・STOP・停止解除・ENDを使い、第4軸と幅切替機構は撤去しています。

[共通festival.yaml](src/catchrobo2026_pump/config/festival.yaml)の`selected_pump: 1..3`で、使用する吸引を1つ選びます。UI・Joy・sequence・pumpが同じ`festival_config`を読み込みます。1/2/3は既存ROSのPumpControl.left/center/rightというスロット順で、物理L/C/Rの確定値ではありません。既定1は変更可能な仮値です。非選択スロットは起動時と直接service呼出しでもOFFを保持します。変更後は関係する全ノードを再起動してください。

[sequences.yaml](src/catchrobo2026_sequence/config/sequences.yaml)は初期化前後とSTARTが空、ENDが`[675,400,300]`→`[675,-110,220]` mmの2移動です。XYZのみを使い、3要素の有限な実測関節値を1秒以内に受信して経路生成・追従します。起動位置X=670 mmとENDのX=675 mmは維持します。実機での位置・経路・停止性能は未確認です。[シーケンス説明](src/catchrobo2026_sequence/README.md)と[設定仕様](src/catchrobo2026_sequence/CONFIG.md)を参照してください。

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch catchrobo2026_sequence automatic.launch.py \
  sequence_file:="$PWD/src/catchrobo2026_sequence/config/sequences.yaml" \
  festival_config:="$PWD/src/catchrobo2026_pump/config/festival.yaml" \
  debug:=true
```

`debug:=true`はライフサイクル動作前にsequence YAMLを再読込します。ポンプ選択は再読込対象外です。STOPで実行中動作を取り消し、停止解除後のSTART手順再実行は行いません。ENDは明示操作だけで実行します。

## 手動操縦とダミー動作確認

手動操縦は次のコマンドで起動します。実機の `current_joints` を供給するCANブリッジ等は別途起動します。`pump_config:=/path/to/pump.yaml` でポンプ設定を差し替えられます。各並進軸の最大速度は [manual_velocity.yaml](src/catchrobo2026_hand_operated/config/manual_velocity.yaml) の `manual_linear_speed_mm_s` で設定します。スティック最大入力時のmm/sで、UIとローカルJoyに共通です。手先回転の操作と角速度設定は削除しました。同じYAMLの `/**/control_node.ros__parameters` ではWebの軸・ボタン・固定移動距離・差分プリセットを指定します。

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

再読込は毎回コードの既定値から構築し、削除した項目に直前の値を残しません。速度を省略した場合のコード既定値は並進50 mm/sです。同梱YAMLの10 mm/sを維持する場合は、その指定を残してください。ROSのread-onlyパラメータ表示は起動時の値で、再読込後のWeb設定はUI、速度はJoyノードの読込ログが有効値を示します。

不正な設定は適用せず、該当ノードの新たなJoy操作を拒否します。Web設定のエラーは「設定・入力確認」、速度設定のエラーはJoyノードのログで確認できます。起動時からWebのゲームパッド割当に誤りがある場合もUIと生入力表示は利用でき、Joy操作を禁止した状態で原因を表示します。ファイルを修正すると再読込で復帰します。`debug:=false` は起動時の設定を固定し、編集の反映には再起動が必要です。この再読込機能を含むコードの初回導入時はビルドと再起動が必要です。

ダミーロボットは既定で無効です。実機の `current_joints` 供給元を起動せずに動作確認する場合は、明示的に有効にします。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py \
  use_dummy:=true initial_pose:='[600.0, 200.0, 200.0, 0.0]'
```

`initial_pose` はダミーの初期手先姿勢 `[x_mm, y_mm, z_mm, yaw_rad]` です。ダミーは目標関節角を即座に現在角へ反映するため、実機の追従性能は再現しません。

## 学祭向けの経路生成プレビュー

`gakusai2026` ブランチでは、`supplementary/grafen1202/catchrobo_ros` の
`main` / `350721d`（2026-10-21）から次の3パッケージを取り込みました。
既存の実機用経路生成・追従への接続は今後の作業です。

| パッケージ | 内容 |
| --- | --- |
| [robot_arm_description](src/arm/README.md)（`src/arm`） | 4軸絶対角に対応するURDF、衝突形状と姿勢表示 |
| [path_planning](src/path_planning/README.md) | Informed RRT*による幾何経路生成とURDF/FCL自己衝突判定 |
| [path_planner](src/path_planner/README.md) | RRT-Connect、平滑化、動力学制約付き時間付け、軌道検証と表示再生 |

ROS依存は通常の `rosdep install --from-paths src --ignore-src -r -y` で解決します。
追加の主要依存はFCL、urdfdom、xacroです。ROSアダプターは依存不足をビルドエラーにし、
CLIだけへの自動切替は行いません。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to path_planner path_planning --cmake-args -G Ninja
source install/setup.bash
ros2 launch path_planner visualize_trajectory.launch.py
# GUIを省略する場合は末尾に rviz:=false を指定
```

この表示は仮の質量・慣性・トルク上限を使った `simulation_demo` です。
YAMLの実測値を読み込む実機制御器ではなく、生成軌道のオープンループ再生です。
ポンプ・CANブリッジ・Joy・既存シーケンサは起動せず、`target_joint_angles` へも送信しません。
ハンド形状の既定幅492 mmなどは取込元の近似を維持しており、吸引1個の実寸ではありません。
`path_planner` の `model` 引数は表示用URDFだけを変更し、内部の計画・衝突・動力学モデルは変更しません。

プレビューlaunchは既定で `gakusai_preview` 名前空間へトピックとTF通信を分離します。
例えば再計画は次のトピックへ送信します。

```bash
ros2 topic pub --once /gakusai_preview/goal_joint_state sensor_msgs/msg/JointState \
  '{name: [theta1, theta2, theta3, theta4], position: [-1.3699, 0.3804, 2.0976, -0.2009]}'
```

Informed RRT*の幾何経路だけを確認する場合は、上の軌道表示を終了してから次を起動します。
始点・終点・サービス呼出しは [path_planningの説明](src/path_planning/README.md) を参照してください。

```bash
ros2 launch path_planning preview.launch.py
```

各プレビューを同時に使う場合は、それぞれ異なる `namespace:=...` を指定してください。
直接 `ros2 run` する場合も `--ros-args -r __ns:=/gakusai_preview` などを指定します。
将来のカメラ入力、実測関節角からの計画、既存追従器への変換・取消・停止処理は未接続です。
妨害機構はMCU側で扱うため、ROS側での削除・追加対応は不要です。
学祭の吸引1個選択は共通`festival.yaml`へ反映しました。物理チャンネル対応と1吸引ハンドの実寸は未確認です。
