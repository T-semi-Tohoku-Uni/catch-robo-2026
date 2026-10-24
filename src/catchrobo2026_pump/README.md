# ポンプ制御

`set_pump_state`はCAN-RPCの`pump_set`を呼び、MCUの適用結果を待って応答します。`success=true`はremote resultがAPPLIEDかつ適用maskが要求と一致した場合だけです。吸着成功、配線、実電圧の保証ではありません。未応答、拒否、世代変更、結果不一致はfalseで、後続動作へ進めません。executorは待機中も圧力購読と他サービスを処理します。同時ポンプ要求はqueueせず拒否します。

起動時の自動出力はありません。旧`initial_state`、100Hz `pump_state`指令配信は撤去しています。再起動や失敗時にもOFFを推測送信しません。

使用ポンプは[festival.yaml](config/festival.yaml)の`selected_pump: 1..3`です。各PumpControl要求はleft/center/rightの3値（開放=-1、OFF=0、吸引=1）を検証し、選択チャネルだけを採用、非選択はOFFにした絶対maskを送ります。これらのROSスロットと物理L/C/Rの対応は未確認です。

[pump.yaml](config/pump.yaml)の`pump_bits`/`valve_bits`とON極性を保持します。既定の両ON極性falseでは全OFF=63、選択1/2/3の吸引=62/61/59、開放=55/47/31です。MCUは既存のactive-low GPIO規約で適用します。

| パラメータ | 既定 | 意味 |
|---|---|---|
| `canrpc_service` | `canrpc/call` | profileの`rpc.ros_service`へ接続 |
| `pump_rpc_endpoint` | `pump_set` | bridgeで宛先/serviceを解決するendpoint名 |
| `rpc_timeout_sec` | `1.5` | ROS側の応答待ち上限、単調時計 |
| `selected_pump` | `1` | 使用スロット1..3 |
| `pump_bits` / `valve_bits` | `[0,1,2]` / `[3,4,5]` | 6bit内の重複しない割当 |
| `pump_on_level` / `valve_on_level` | `false` / `false` | 論理ONのCAN bit値 |

学祭・手動・pump launchは`canrpc_config`からservice名と待ち期限を伝播します。RPC応答期限+0.5秒をapp、+1秒をJoy/UI、最低3秒をsequenceに用います。直起動する場合は同等の値を明示してください。起動例は `ros2 launch catchrobo2026_pump pump.launch.py canrpc_config:=/absolute/profile.yaml` です。

`pump_state`は指令入力ではなく適用確認の状態出力（Int32MultiArray）です。MCU APPLIED後だけ`[mask]`を1回配信し、要求待ち/不明は空配列を配信します。確認済みの値をheartbeatで再配信しません。UIは「適用確認」「適用不明」「確認期限切れ」を区別し、静的出力でも確認サンプルが既定1秒を超えればstaleになります。継続した実GPIO監視ではありません。bridgeの旧pump PUBLISH TX routeは廃止しています。

## 空気圧による吸引判定

サービス: `/check_suction`、型: `catchrobo2026_msgs/srv/CheckSuction`。
このサービスは空気圧の監視だけを行い、ポンプ・電磁弁の出力を変更しません。
吸引開始は従来どおり `/set_pump_state` へ指示します。
監視中もRPCポンプ操作と圧力購読を継続します。周期出力指令は送信しません。

CAN-RPC Subject `0x406`（Source Node 4）の3個のlittle-endian 32bit整数は、ブリッジで
`pressure_sensor`（`std_msgs/msg/Int32MultiArray`）へ変換済みです。
たとえば `45 00 00 00 45 00 00 00 3F 00 00 00` は `[69, 69, 63]` です。
ポンプノードで再度4バイトずつ組み直す必要はありません。

| 要求 | 意味 |
|---|---|
| `collector_mask` | L/C/Rをbit0/1/2で選択する1〜7のマスク |
| `timeout_sec` | 有限の秒数、`0 < timeout_sec <= 86400` |

| 応答 | 意味 |
|---|---|
| `success` | 選択した全機構が、同じ新規サンプルで閾値条件を満たした |
| `timed_out` | 正常に受理した問い合わせが、成功せず期限に達した |
| `suction_mask` | 最後の有効サンプルで閾値条件を満たした全機構のマスク |
| `pressure` | 要求受理後の最後の有効圧力、L/C/R順の3整数 |
| `message` | 成功・期限切れ・拒否の説明 |

`success=false, timed_out=false` は不正要求、未設定の比較方向、または同時要求数上限による拒否です。
圧力未受信の期限切れと拒否では `pressure` の全要素を `INT32_MIN`（-2147483648）、マスクを0とします。
有効サンプルが一度でも届いた期限切れでは、その要求内の最後のサンプルと判定マスクを返します。
異なる時刻に成功した機構を足し合わせて成功扱いにはしません。

成功条件を満たすと期限前でも応答し、それ以外は期限に応答します。
期限には単調時計を使うため、`use_sim_time` の停止やROS時刻変更に影響されません。
期限監視周期は10 msで、DDS通信とOSの実行遅延は別途加わります。
要求前にノードが受信した圧力は成功判定に使いません。
トピックには計測時刻がないため、新規性は要求受理後の購読コールバックでの受信を基準にします。
配列が3要素でないものや非ゼロoffset、矛盾するlayoutは無視し、未受信として待ちます。
クライアントが待機を中止しても、サービス側の監視は成功または期限まで残ります。

| 起動パラメータ | ノード内既定値 | 意味 |
|---|---|---|
| `pressure_indices` | `[0, 1, 2]` | L/C/Rに対応する受信配列index。0〜2を重複なく指定 |
| `pressure_threshold` | `96`（`0x60`） | 比較の閾値、int32の範囲 |
| `pressure_comparison` | `unconfigured` | `ge`は圧力>=閾値、`le`は圧力<=閾値 |
| `max_pending_suction_checks` | `8` | 同時に監視する問い合わせ数。設定範囲1〜128 |

同梱の [pump.yaml](config/pump.yaml) は `pressure_threshold: 300`、
`pressure_comparison: ge` を指定し、300以上を吸着成功とします。
実機の圧力センサーのL/C/R対応は未確認のため、`pressure_indices` は実機に合わせて確認してください。
設定を省略して `pressure_comparison` が `unconfigured` の場合は、吸引判定サービスだけを拒否します。
ポンプ設定は起動時に読み込みます。変更後はポンプノードを再起動してください。
`debug:=true` によるシーケンス設定の再読込は、ポンプ設定には適用されません。

全機構の吸引を最大2秒待つ例:

```bash
ros2 service call /check_suction catchrobo2026_msgs/srv/CheckSuction \
  '{collector_mask: 7, timeout_sec: 2.0}'
```
