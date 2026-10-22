# 時間付き軌道の生成とプレビュー

取込元は `supplementary/grafen1202/catchrobo_ros` の `350721d` です。
RRT-Connect、短絡化・C2平滑化、RNEA、TOPP-RA、jerk制限、最終検証を含みます。
ROSノードは仮の動力学値による表示用デモで、実測値による閉ループ制御は未実装です。
`hardware_output_enabled` はコアの検証モード設定であり、CAN出力を追加する機能ではありません。

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to path_planner --cmake-args -G Ninja
source install/setup.bash
ros2 launch path_planner visualize_trajectory.launch.py
```

`rviz:=false` でGUIを省略できます。起動時に内部の `simulation_demo` 設定を使用し、
同梱の `config/robot_dynamics.yaml` / `trajectory_pipeline.yaml` は読み込みません。
YAMLは実測値未設定の雛形で、実機用設定として完成していません。
質量・慣性・モーター特性・摩擦・ペイロード・関節限界の実測確認が必要です。
`model:=...` は表示だけを変更し、内部の計画・衝突・動力学モデルは変更しません。
ハンドの既定形状も吸引1個の実寸へ適応していません。

既定の名前空間は `gakusai_preview` です。トピック・TF通信・RVizの接続を分離し、
既存の `generate_route` / `follow_route` / Joy / CAN には接続していません。
`namespace:=other_preview` で変更できます。`path_planning` との同時起動時は別々の名前空間を使います。

| トピック（既定の接頭辞 `/gakusai_preview/`） | 内容 |
| --- | --- |
| `goal_joint_state` | 再計画の目標4軸絶対角（JointState、配列順theta1〜4） |
| `geometric_path` | 時間付け前の関節列 |
| `timed_trajectory` | 検証済みの時間付き参照軌道 |
| `joint_states` / `arm/absolute_joint_states` | 表示再生中のURDF5関節 / 絶対4軸角 |
| `trajectory_diagnostics` | 計画結果と仮モデルでのトルク等 |
| `trajectory_markers` / `trajectory_flange_path` | 表示マーカー / フランジ経路 |

```bash
ros2 topic pub --once /gakusai_preview/goal_joint_state sensor_msgs/msg/JointState \
  '{position: [-1.3699, 0.3804, 2.0976, -0.2009]}'
```

ROSを使わずコアとCLIデモを確認する場合は明示的に無効化します。

```bash
cmake -S src/path_planner -B /tmp/gakusai_path_planner -G Ninja \
  -DPATH_PLANNER_WITH_ROS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/gakusai_path_planner
ctest --test-dir /tmp/gakusai_path_planner --output-on-failure
/tmp/gakusai_path_planner/trajectory_node --demo
```

ROSビルドでは上流6スイートに加え、既存の `robot_kinematics::forward_kinematics()` と
205姿勢の位置（mm→m）・yaw（phi）を比較する回帰テストを登録します。
これはモデルの数値規約の照合で、実機の安全性・追従性能・衝突形状の実測検証ではありません。
