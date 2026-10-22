# 4軸アームの URDF と collision 表示

`robot_arm.xacro` は CatchRobo2026_kinematics の
`homogeneous_transform.h::make_transform_chain()` に合わせたモデルです。
外部リポジトリへの実行時依存や STL はありません。長さの単位は m、角度は rad です。

## 起動

ワークスペースのルートで実行します。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description collision.launch.py
```

初期姿勢を指定できます（IK と同じ絶対角の順序）。

```bash
ros2 launch robot_arm_description collision.launch.py 'initial_angles:=[0.4, -0.2, 0.8, 0.3]'
```

既定の名前空間は `gakusai_preview` で、TF通信も分離します。`namespace:=...` で変更できます。
RViz では collision のみを表示します。`rviz:=false` で GUI を省略できます。

## IK 出力との接続

`arm_joint_state_publisher.py` は `/gakusai_preview/arm/absolute_joint_states`
（`sensor_msgs/msg/JointState`）を購読し、変換した `/gakusai_preview/joint_states` を30 Hzで配信します。
受信するまでは初期姿勢を、受信後は最後の姿勢を表示し続けます。
これは表示用の状態で、実機の最新状態を保証するものではありません。

```bash
ros2 topic pub --once /gakusai_preview/arm/absolute_joint_states sensor_msgs/msg/JointState \
  "{name: [theta1, theta2, theta3, theta4], position: [0.4, -0.2, 0.8, 0.3]}"
```

名前がある場合は順序を入れ替えられます。名前が空なら4要素を上記の順序で渡します。
提示された IK ノードはサービス応答で角度を返すため、呼び出し側からこのトピックに
応答の `joint_angles` を配信してください。サービスの自動呼び出しは行いません。

| URDF 関節 | 配信する角度 |
| --- | --- |
| `base_yaw_joint` | theta1（軸は -Z、初期 yaw は pi/2） |
| `shoulder_joint` | theta2 |
| `elbow_joint` | theta3 - theta2 |
| `wrist_level_joint` | -theta3 |
| `flange_yaw_joint` | 2 * theta1 + theta4 - pi/2 |

入力は4軸ですが、水平維持用の従属関節を含め URDF には5つの回転関節があります。
最後の角度補正により `flange_link` のフィールド座標系 yaw が `theta1 + theta4`
になります。URDF の5関節を独立な5軸として制御・計画しないでください。
複数関節の和を表す依存関係はこの変換ノードで処理します。
外部から変換済みの `/gakusai_preview/joint_states` を配信する場合は `publish_joint_states:=false` を指定します。

## 座標と寸法

| 項目 | 値 |
| --- | --- |
| map → base_link | (0.675, -0.190, 0) m |
| ベース回転系 → 肩 | (-0.040, 0, 0.090) m |
| 上腕 / 前腕 | 各0.480 m、角度ゼロで +Z |
| 水平手首 → flange_link | (+0.040, 0, 0) m |

古いコメントの原点 (675, -130, 228) mm ではなく、現在の変換行列の定数を使っています。
手先姿勢も `make_transform_chain()` の yaw 回転に一致させています。
`forward_kinematics()` が別途返す `THE=-pi/2` を追加の URDF 回転にはしていません。

## 衝突形状の寸法変更

ベースの円柱、上腕・前腕・手首・ハンドの箱に `<collision>` を設定しています。
ベースは半径114 mm、高さ90 mmです。
上腕はローカル X × Y × Z = 191 × 71 × 480 mmで、Z方向が肩から肘への方向です。
前腕はローカル X × Y × Z = 70.25 × 71 × 480 mmで、Z方向が肘から手首への方向です。
ハンドはローカル X × Y × Z = 27 × 492 × 124 mmです。ハンドの取付けはZ軸まわりに90°回転させています。
リンク長と関節位置はコードに基づきますが、手首・ハンド形状は近似です。
学祭で使う吸引1個の形状・寸法への適応は未実施です。

| Xacro 引数 | 初期値 (m) |
| --- | --- |
| `forearm_x`, `forearm_y` | 0.07025, 0.071（前腕の断面寸法） |
| `upper_arm_x`, `upper_arm_y` | 0.191, 0.071（上腕の断面寸法） |
| `base_radius` | 0.114（ベースの半径、高さは0.090） |
| `wrist_width` | 0.050 |
| `hand_x`, `hand_y`, `hand_z` | 0.027, 0.492, 0.124 |

ハンドの箱は flange_link のローカルZ軸まわりに+90°回転させ、-Z 方向に伸ばしています。
この回転は表示・衝突形状の取り付け姿勢に適用しています。実物の取り付け位置・形状が
分かり次第、寸法と collision の origin を更新してください。

例: 太さを変更した URDF を生成して表示する場合:

```bash
xacro src/arm/urdf/robot_arm.xacro forearm_x:=0.080 hand_z:=0.120 > /tmp/robot_arm.urdf
ros2 launch robot_arm_description collision.launch.py model:=/tmp/robot_arm.urdf
```

このパッケージは衝突形状と姿勢を提供します。接触の有無を計算する処理は含みません。
衝突判定の実装では、この関節変換を適用したうえで隣接リンクの接続部の重なりを
適切に除外してください。仮形状には、モータ・配線・平行リンクなどの実形状は含まれません。
実際の可動範囲が未確定なので関節は continuous とし、仮の可動限界は設けていません。
慣性・駆動設定を持たない、可視化と幾何的な衝突判定のためのモデルです。

## 検証

```bash
colcon test --packages-select robot_arm_description
colcon test-result --test-result-base build/robot_arm_description --verbose
```

URDF の関節ツリーを数値評価し、101姿勢の関節位置・手先姿勢・水平維持を検証します。
取込後の既存運動学との独立比較は `path_planner` の
`test_production_kinematics` で行います。
