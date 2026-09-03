# catchrobo2026_debug

`TreeExecutionServer`から`/generate_route` Serviceと`/follow_route` Actionを実行するための構成です。
CAN、rosbridge、UI、`payload`のBlackboard展開は含みません。

## ビルド

```bash
cd ~/catch-robo-2026
source /opt/ros/humble/setup.zsh

colcon build --symlink-install --packages-up-to catchrobo2026_debug
source install/setup.zsh
```

## BTサーバだけを起動する

```bash
ros2 launch catchrobo2026_debug bt_server.launch.py
```

読み込まれたTreeを確認します。

```bash
ros2 service call \
  /get_loaded_trees \
  btcpp_ros2_interfaces/srv/GetTrees \
  "{}"
```

`FollowRouteTask`と`GenerateAndFollowRouteTask`が返れば、プラグインとXMLは正常にロードされています。

この状態では`/generate_route` Serviceと`/follow_route` Actionサーバが存在しないため、
Treeを実行すると接続タイムアウト後に`FAILURE`になります。

```bash
ros2 action send_goal \
  /behavior_server \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'FollowRouteTask', payload: ''}" \
  --feedback
```

## 模擬ノードとまとめて起動する

`nav_director`の経路生成、経路追従、ダミーロボットとBTサーバを起動します。

```bash
ros2 launch catchrobo2026_debug follow_route_debug.launch.py
```

別ターミナルで経路生成と追従をまとめたBTを実行します。

```bash
source /opt/ros/humble/setup.zsh
source ~/catch-robo-2026/install/setup.zsh

ros2 action send_goal \
  /behavior_server \
  btcpp_ros2_interfaces/action/ExecuteTree \
  "{target_tree: 'GenerateAndFollowRouteTask', payload: ''}" \
  --feedback
```

## Cancel

`ExecuteTree`のActionクライアントからCancel要求を送ると、
`TreeExecutionServer`がTreeをhaltし、実行中の`/follow_route` GoalにもCancelが伝播します。

ROS 2 Humbleの標準`ros2 action` CLIには、別プロセスから既存Goalを選んでCancelする
専用サブコマンドがありません。Cancelは今後のUI、または
`rclcpp_action`/`rclpy.action.ActionClient`の`cancel_goal_async()`から確認してください。

## 公開インターフェース

| 種類 | 名前 | 型 |
|---|---|---|
| Action | `/behavior_server` | `btcpp_ros2_interfaces/action/ExecuteTree` |
| Service | `/get_loaded_trees` | `btcpp_ros2_interfaces/srv/GetTrees` |
| Tree ID | `FollowRouteTask` | BehaviorTree.CPP Tree |
| Tree ID | `GenerateAndFollowRouteTask` | 経路生成後に追従するBehaviorTree.CPP Tree |
| BT Node ID | `GenerateRoute` | ROS Service BT plugin |
| BT Node ID | `FollowRoute` | ROS Action BT plugin |
| 内部Service | `/generate_route` | `catchrobo2026_msgs/srv/GenerateRoute` |
| 内部Action | `/follow_route` | `catchrobo2026_msgs/action/FollowRoute` |
