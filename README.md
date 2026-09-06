# catch-robo-2026

キャチロボ2026のロボット制御用ROS 2ワークスペースです。

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

## 環境

```bash
sudo apt install ros-humble-rosbridge-suite
```
