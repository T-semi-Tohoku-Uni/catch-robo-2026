# catchrobo2026_endeffector

エンドエフェクタの指令値を保持・配信するROS 2パッケージ。
`catchrobo2026_hand_operated` から分離し、手動操作や自動操縦から共通のサービスを利用できます。

## 起動

ワークスペースをビルドして `source install/setup.bash` を実行した後、単独起動できます。

```bash
ros2 launch catchrobo2026_endeffector endeffector.launch.py
```

ノード名と実行ファイル名はともに `endeffector_state_node` です。
`catchrobo2026_hand_operated` の `main.launch.py` と `handoperated.launch.py` からも起動します。
手動操作側はサービスクライアントと指令トピックの購読を持ちます。
同じ名前空間でサービスサーバーを重複起動しないでください。

## 操作API

サービス: `/set_endeffector_state`、型: `catchrobo2026_msgs/srv/EndeffectorControl`。

| フィールド | 意味 |
|---|---|
| 要求 `command` | `int32`、受理する値は `0` または `1` |
| 応答 `success` | 指令値の更新を受理したか |

要求の値をそのまま保持して配信します。それ以外の値は `success=false` で拒否し、現在値を維持します。

```bash
ros2 service call /set_endeffector_state catchrobo2026_msgs/srv/EndeffectorControl \
  '{command: 0}'
```

出力は `/endeffector_state`（`std_msgs/msg/Int32MultiArray`、要素1個）、50ms周期（20Hz）です。
`nhk2026_bridge` の [設定](../nhk2026_bridge/config/raspi_canbridge.yml) により、CAN ID `0x300` の4バイトbig-endian整数として送信します。
このトピックは指令値であり、サービスの成功応答も実機の動作完了を意味しません。

Joyは `buttons[2]` の立ち上がりで、購読した現在値が `0` なら `1`、それ以外なら `0` を要求します。

## 初期値と未確認事項

起動時の配信値は、分離前の `main` の実装を維持して **56** です。
サービスで受け付ける `0` / `1` と一致せず、初回の有効なサービス要求を受けるまで56を配信し続けます。
Joyも分離前と同じ初期値を持つため、起動後の最初の操作は通常 `0` の要求になります。

2026-09-06時点で、CAN `0x300` を受けるマイコンの実装はローカルの参照資料から確認できていません。
`0` / `1` / `56` の物理動作、適切な起動値、通信断・停止時の動作は未確認です。
今回の分離では初期値を推測で変更せず、0/1操作と配信周期を保持しています。
