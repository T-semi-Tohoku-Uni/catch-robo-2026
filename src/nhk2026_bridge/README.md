## nhk2026_canbridge 使い方（概要）
ROS 2 Lifecycle Node `nhk2026_canbridge` は、CAN バスと ROS トピックを相互にブリッジします。  
Lifecycle の `configure` → `activate` を行うことで動作を開始します。

### 事前準備（必須）
CAN インターフェースのセットアップを自動で行うため、sudoers に `ip` コマンドの許可を追加します。

`sudo visudo` で sudoers ファイルを開き、以下の行を追加してください。

```
youruser ALL=(root) NOPASSWD: /usr/sbin/ip link set can0 up type can bitrate 1000000 dbitrate 2000000 fd on
youruser ALL=(root) NOPASSWD: /usr/sbin/ip link set can0 up
youruser ALL=(root) NOPASSWD: /usr/sbin/ip -o link show can0
youruser ALL=(root) NOPASSWD: /usr/sbin/ip link set can0 down
```

- `youruser` は実際のユーザ名に置き換えてください。
- `ip` コマンドのパスは `which ip` で確認してください。

### 役割（データの流れ）
- CAN → ROS: 受信した CAN フレームを `pub_*` 設定に従って各トピックへ publish
- ROS → CAN: `sub_*` 設定に従って各トピックを subscribe し、受信メッセージを CAN へ送信

### パラメータ
#### 基本
- `ifname`（string, default: `can0`）

#### ブリッジ設定
- `pub_float_bridge_topic` / `pub_int_bridge_topic` / `pub_bytes_bridge_topic`（string[]）
- `sub_float_bridge_topic` / `sub_int_bridge_topic` / `sub_bytes_bridge_topic`（string[]）
- `pub_float_bridge_canid` / `pub_int_bridge_canid` / `pub_bytes_bridge_canid`（int[]）
- `sub_float_bridge_canid` / `sub_int_bridge_canid` / `sub_bytes_bridge_canid`（int[]）

#### 追加機能
- `add_cmd_vel` (bool) / `cmd_vel_canid` (int) / `cmd_vel_topic_name` (string)
- `add_cmd_vel_feedback` (bool) / `cmd_vel_feedback_canid` (int) / `cmd_vel_topic_feedback_name` (string)

### 必須の対応関係（重要）
- `topic` 配列と `canid` 配列の要素数が一致していないと `configure/activate` が失敗します。
- 対応は「配列の同じインデックス同士」で行われます。

### メッセージ型
- float 系: `std_msgs/msg/Float32MultiArray`
- int 系: `std_msgs/msg/Int32MultiArray`
- bytes 系: `std_msgs/msg/ByteMultiArray`

### 運用上の注意
- Active 状態ではパラメータ変更は拒否されます。変更する場合は `deactivate` してから再設定してください。

### vcanでの起動

`vcan_can.launch.py` は `config/raspi_canbridge.yml` を読み込み、`ifname` だけを上書きします。

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set dev vcan0 mtu 72
sudo ip link set dev vcan0 up
ros2 launch nhk2026_bridge vcan_can.launch.py
```

別名のvcanを使う場合は、launch引数で指定します。

```bash
ros2 launch nhk2026_bridge vcan_can.launch.py ifname:=vcan1
```

### vcanによる簡易テスト

`iproute2` と `can-utils` をインストールし、ワークスペースをビルド・sourceした後に実行します。

```bash
sudo -v
ros2 run nhk2026_bridge test_vcan_bridge.sh
```

テストは専用の `vcan_nhk26` を一時作成し、ROS→CANとCAN→ROSを確認した後に削除します。既存インターフェイスが同名で存在する場合は変更せず失敗します。
