# Informed RRT* 経路生成

4軸アームの絶対関節角 `theta1, theta2, theta3, theta4`（rad）を標準とする
経路生成パッケージです。今回は幾何経路生成までを実装しています。
URDF の collision に基づく自己衝突判定を含みます。
TOPP、逆運動学、フィールドとの衝突判定、実機への指令は含みません。

## 構成

- `include/path_planning/informed_rrt_star.hpp` / `src/informed_rrt_star.cpp`:
  ROS 非依存の任意次元 Informed RRT* ライブラリ。
- `include/path_planning/arm_collision_checker.hpp` / `src/arm_collision_checker.cpp`:
  URDF と FCL による自己衝突判定。ROS 通信への依存はありません。
- `include/path_planning/planner_node.hpp` / `src/planner_node.cpp`:
  ROS メッセージ変換とサービス処理。
- `src/main.cpp`: ROS ノードの起動。
- `config/planner.yaml`: 探索設定。パラメータは起動時に読み込みます。

コアの `Path = std::vector<State>` は時間を持ちません。
後段を `Path → 経路の補間・平滑化 → TOPP → 時間付き軌道` と接続できます。
折れ線の角では微分が連続でないため、TOPP に渡す前に経路表現と角の扱いを
決めてください。平滑化で形状を変えた場合は衝突判定も再実施します。

## アルゴリズム

目的関数は状態ベクトルのユークリッド経路長です。最初は探索範囲内を
一様サンプリングし、最初の解が得られた後は始点・終点を焦点とする
超楕円体内を直接サンプリングします。RRT* の親選択と rewiring を行い、
つなぎ替え時には子孫のコストと最良のゴール接続も更新します。
実装の参照: [Gammell et al., Informed RRT* (2014)](https://arxiv.org/abs/1404.2334)。

直線で接続可能なら最短解として即座に返します。それ以外は最大反復数まで
改善します。有限回の探索で最適解・経路の発見を保証するものではありません。
近傍検索は線形走査、近傍半径は固定です。大規模な探索では空間インデックスや
子ノード管理による高速化が必要です。乱数 seed を固定すると再現できます。
角度は有界な実数として扱い、±π の折り返しや関節間の重み付けは行いません。

`StateValidityChecker` は必須です。関節制限に加え、自己衝突・周囲との衝突を
この関数で判定できます。辺は `collision_resolution` 以下の間隔で検査します。
離散判定ではその間隔より薄い障害物を見逃し得ます。必要なら任意引数の
`MotionValidityChecker` で辺全体の連続衝突判定を追加してください。
判定関数は計画中に同じシーンを参照し、状態を変更しない設計にします。

```cpp
#include <path_planning/informed_rrt_star.hpp>
path_planning::PlannerOptions options;
options.lower_bounds = {-3.14, -1.0, -1.0, -3.14};
options.upper_bounds = { 3.14,  1.0,  1.0,  3.14};
path_planning::InformedRrtStar planner(options,
  [](const path_planning::State & q) {
    // Check kinematics and geometry here.
    return q[1] * q[1] + q[2] * q[2] > 0.04;
  });
auto result = planner.plan({0, -0.5, 0, 0}, {0, 0.5, 0, 0});
// result.path, result.cost / 空の path は失敗
```

## ROS 2 で動作確認

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to path_planning
source install/setup.bash
ros2 launch path_planning planner.launch.py
```

launch が `robot_arm_description/urdf/robot_arm.xacro` を展開し、
`robot_description` パラメータでノードへ渡します。別の寸法を使用する場合は
表示と計画で同じモデルを指定してください。

```bash
xacro src/arm/urdf/robot_arm.xacro hand_z:=0.100 > /tmp/planning_arm.urdf
ros2 launch path_planning planner.launch.py model:=/tmp/planning_arm.urdf
```

直接 `ros2 run` する場合も `robot_description` が必須です。
モデル未指定・解析失敗・未対応形状では起動を拒否します。
以前の `allow_collision_unchecked_demo` モードは廃止しました。
仮の ±π 範囲は実機の可動限界ではありません。

別ターミナルで ROS とワークスペースを source して実行します。

```bash
ros2 topic pub --once /gakusai_preview/start_joint_states sensor_msgs/msg/JointState \
  '{name: [theta1, theta2, theta3, theta4], position: [0.0, 0.0, 1.5, 0.0]}'
ros2 topic pub --once /gakusai_preview/goal_joint_states sensor_msgs/msg/JointState \
  '{name: [theta1, theta2, theta3, theta4], position: [0.2, 0.2, 1.5, 0.0]}'
ros2 service call /gakusai_preview/plan_path std_srvs/srv/Trigger '{}'
ros2 topic echo /gakusai_preview/geometric_path --qos-durability transient_local --once
```

| インターフェース | 型 | 用途 |
| --- | --- | --- |
| `start_joint_states` | `sensor_msgs/msg/JointState` | 始点 |
| `goal_joint_states` | `sensor_msgs/msg/JointState` | 終点 |
| `plan_path` | `std_srvs/srv/Trigger` | 最新の始点・終点で計画、成功と説明を返す |
| `geometric_path` | `trajectory_msgs/msg/JointTrajectory` | 関節角列、transient local |

名前があれば `joint_names` 順に並べ替えます。名前なしは設定順です。
不正な状態を受信すると該当の始点・終点を無効化します。
失敗した計画要求では空の経路を配信し、保持されている旧経路を消去します。
出力の速度・加速度は空、`time_from_start` は全点ゼロで、制御器へ直接渡す軌道ではありません。
`/gakusai_preview/joint_states` の URDF 用5関節とは異なる絶対角です。
必要なら `-r start_joint_states:=/gakusai_preview/arm/absolute_joint_states` で既存入力へ接続できます。
計画は同期処理で、計画中の状態更新は完了後に処理します。キャンセルには未対応です。

## RViz で手先の経路だけを表示

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to path_planning
source install/setup.bash
ros2 launch path_planning preview.launch.py
```

既定の名前空間は `gakusai_preview` です。`namespace:=...` で変更できます。

この launch はプランナー・経路表示ノード・RViz を起動します。
アームのモデル・再生・関節状態配信は行わず、手先（`flange_link` 原点）の経路を
緑の線で表示します。プランナーを既に起動している場合は
`ros2 launch path_planning preview.launch.py start_planner:=false` を使用してください。
別ターミナルから上記の始点・終点と `plan_path` 要求を送ると経路が表示されます。

既存 RViz に追加する場合は `Fixed Frame` を `map` にして、`Marker` 表示の
`Topic` を `/gakusai_preview/path_marker` にします。Durability は `Transient Local`
にすると、経路生成後に RViz を開いても最新の経路を受信できます。

経路表示ノードは関節空間の区間を `preview_resolution`（既定0.02 rad）以下の
間隔で補間し、同じ URDF の順運動学で手先の位置へ変換します。
1点だけなら緑の点を表示します。計画失敗による空経路や不正な経路を受信すると
以前の線を消します。`model` 引数でプランナーと表示に同じモデルを指定できます。
このモデルのルートフレームは `map` を想定しています。

## 自己衝突判定

現在の URDF にあるベース円柱、上腕・前腕・手首・ハンドの箱を読み込みます。
寸法・collision の origin xyz/rpy・関節の origin/axis を URDF から取得します。
複数 collision を持つリンクも検査します。対応形状は box/cylinder/sphere です。
mesh や未対応の可動関節、欠落した必須形状は黙って無視せずエラーにします。
FCL の形状判定を使用します（[公式ドキュメント](https://github.com/flexible-collision-library/fcl)）。

4つの絶対角から URDF 関節への変換は既存の表示ノードと同じです。
`[theta1, theta2, theta3-theta2, -theta3, 2*theta1+theta4-pi/2]` を
`base_yaw_joint, shoulder_joint, elbow_joint, wrist_level_joint, flange_yaw_joint`
へ適用します。URDF が revolute の場合はその可動範囲も検査します。
ROS アダプターの `joint_names` は `theta1, theta2, theta3, theta4` の順序に固定します。
入力メッセージ中の名前の順序は任意です。

始点・終点と、経路中の補間姿勢（間隔 `collision_resolution` rad 以下）で
自己衝突を検査します。入力姿勢が衝突する場合はログへリンクの組を出し、
その入力を無効化します。コアでは次のように接続できます。

```cpp
#include <path_planning/arm_collision_checker.hpp>
// urdf_xml は Xacro 展開済みの URDF 文字列
path_planning::ArmCollisionChecker collision(urdf_xml);
path_planning::InformedRrtStar planner(options,
  [&collision](const path_planning::State & q) { return collision.isStateValid(q); });
auto status = collision.check({0.0, 0.0, 1.5, 0.0});
// status.valid / status.links / status.reason
```

衝突判定対象は **flange_link / upper_arm_link の1組だけ**です。
それ以外の組は重なっていても衝突として扱いません。
コアの既定値と `config/planner.yaml` はどちらも同じ1組を指定しています。

```yaml
checked_collision_pairs:
  - flange_link:upper_arm_link
```

`checked_collision_pairs` は検査するペアの一覧です。ペア内の順序は任意です。
以前の除外一覧 `allowed_collision_pairs` は置き換えました。
関節角の範囲チェックと経路途中の補間姿勢の検査は引き続き行います。

フィールドの SDF には mesh 参照がありますが、参照先の mesh ファイルが
このワークスペースにはありません。フィールド・床・把持物との接触は未判定です。
また URDF の形状は近似であり、モータ・配線・平行リンク等の実形状は含みません。
離散姿勢の検査なので、サンプル間の連続的な非衝突を保証するものではありません。

## テスト

```bash
colcon test --packages-select path_planning
colcon test-result --test-result-base build/path_planning --verbose
```

ROS を使わずコアだけビルド・テストする場合:

```bash
cmake -S src/path_planning -B /tmp/path_planning_core -DPATH_PLANNING_WITH_ROS=OFF
cmake --build /tmp/path_planning_core
ctest --test-dir /tmp/path_planning_core --output-on-failure
```

直線最短経路、同一始終点、無効入力、障害物回避、探索時間増加による改善、
rewiring 後の経路コスト、乱数再現性、到達不能、辺判定、4次元探索に加え、
実 URDF の上腕・ハンド間の衝突、対象外ペアの無視、姿勢変換、経路中の判定を確認します。

ROS の入力・サービス・出力も確認する場合（ROS を source したターミナルで実行）:

```bash
python3 src/path_planning/test/test_planner_node.py \
  build/path_planning/path_planner_node build/path_planning/test_robot_arm.urdf
```

このテストは localhost の ROS_DOMAIN_ID=89 を使用します。

経路マーカーの座標・補間・消去を GUI なしで確認する場合:

```bash
python3 src/path_planning/test/test_path_preview.py \
  build/path_planning/path_preview_node build/path_planning/test_robot_arm.urdf
```

このテストは localhost の ROS_DOMAIN_ID=91 を使用します。
