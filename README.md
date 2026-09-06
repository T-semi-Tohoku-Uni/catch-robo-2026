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

既存のチェックアウトでブランチを切り替えた後も、`git submodule update --init --recursive` で記録済みのコミットへ揃えます。UIの配信用ファイルは同梱しているため、通常のビルドにnpmは不要です。Humble向けにも開発していますが、上記手順のローカル検証環境はJazzyです。

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

手動操縦は次のコマンドで起動します。実機の `current_joints` を供給するCANブリッジ等は別途起動します。`pump_config:=/path/to/pump.yaml` でポンプ設定を差し替えられます。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py
```

ダミーロボットは既定で無効です。実機の `current_joints` 供給元を起動せずに動作確認する場合は、明示的に有効にします。

```bash
ros2 launch catchrobo2026_hand_operated handoperated.launch.py \
  use_dummy:=true initial_pose:='[600.0, 200.0, 200.0, 0.0]'
```

`initial_pose` はダミーの初期手先姿勢 `[x_mm, y_mm, z_mm, yaw_rad]` です。ダミーは目標関節角を即座に現在角へ反映するため、実機の追従性能は再現しません。
