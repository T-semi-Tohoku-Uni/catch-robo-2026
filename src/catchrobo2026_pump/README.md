# catchrobo2026_pump

3つの回収機構のポンプ・電磁弁を操作するROS 2パッケージ。
`pump_controller_node` がL/C/Rそれぞれの開放・オフ・吸引状態を保持し、マイコン向けの6bit指令へ変換します。

## 起動

ワークスペースをビルドして `source install/setup.bash` を実行した後、単独起動できます。

```bash
ros2 launch catchrobo2026_pump pump.launch.py
```

手動パッケージの `main.launch.py` と `handoperated.launch.py` からも、このノードが起動します。
3つのlaunchは共通の [config/pump.yaml](config/pump.yaml) を読み込みます。
実機調整用ファイルを指定する場合は、次のように起動します。

```bash
ros2 launch catchrobo2026_pump pump.launch.py pump_config:=/absolute/path/to/pump.yaml
```

`pump_config` は手動パッケージの両launchでも指定できます。
設定は起動時に読み込むため、変更後はノードを再起動してください。
ソース側のYAMLを編集した場合は再ビルドするか、そのファイルを `pump_config` に直接指定します。

## 操作API

サービス: `/set_pump_state`、型: `catchrobo2026_msgs/srv/PumpControl`。

| フィールド | 意味 |
|---|---|
| `left` / `center` / `right` | L/C/Rそれぞれの状態を `int8` で指定 |
| 応答 `success` | 状態更新を受理したか |

| 値 | 定数 | 動作 | 内部のポンプ出力 | 内部の電磁弁出力 |
|---|---|---|---|---|
| `-1` | `RELEASE` | 開放 | OFF | ON |
| `0` | `OFF` | オフ | OFF | OFF |
| `1` | `SUCTION` | 吸引 | ON | OFF |

1回の要求で3機構を全て更新します。**0はオフ指令であり、前の状態を保持する指定ではありません。**
状態を変えない機構にも、維持したい値を送ってください。
いずれかの値が `-1`・`0`・`1` 以外なら要求全体を拒否し、3機構とも元の状態を保持します。

シーケンサは各ポンプ手順で、UIが選んだ機構に設定された開放・オフ・吸引の値を送り、非選択機構には必ずオフ（0）を送ります。現在の指令状態の読出しは行いません。

Lを吸引、Cをオフ、Rを開放:

```bash
ros2 service call /set_pump_state catchrobo2026_msgs/srv/PumpControl \
  '{left: 1, center: 0, right: -1}'
```

全機構をオフ:

```bash
ros2 service call /set_pump_state catchrobo2026_msgs/srv/PumpControl \
  '{left: 0, center: 0, right: 0}'
```

旧 `command` および `collector_mask`・`pump_on`・`valve_on` のAPIは廃止しました。
外部クライアントも新しい要求型へ更新し、再ビルドしてください。
Joyの〇ボタンは、全3項目に同じ値を送り「開放 → オフ → 吸引」の順に操作します。
YAMLを指定しない直接起動では起動時が吸引56で、ボタンを押すごとに `7 → 63 → 56` と巡回します。
同梱 `pump.yaml` は実機調整により `valve_on_level: true`・`initial_state: 0` に変更されており、起動時はオフ7です。
手動側の巡回位置はサービス成功時に進みます。別クライアントの操作やポンプノードの再起動とは同期しません。
応答が1秒以内に届かなければ巡回位置を進めず、次のボタン操作で同じ要求を再試行します。

## 空気圧による吸引判定

サービス: `/check_suction`、型: `catchrobo2026_msgs/srv/CheckSuction`。
このサービスは空気圧の監視だけを行い、ポンプ・電磁弁の出力を変更しません。
吸引開始は従来どおり `/set_pump_state` へ指示します。
監視中も100 Hzの出力配信とポンプ操作、圧力購読を継続します。

CAN ID `0x406` の3個のbig-endian 32bit整数は、ブリッジで
`pressure_sensor`（`std_msgs/msg/Int32MultiArray`）へ変換済みです。
たとえば `00 00 00 45 00 00 00 45 00 00 00 3F` は `[69, 69, 63]` です。
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

| 起動パラメータ | 値 | 意味 |
|---|---|---|
| `pressure_indices` | `[0, 1, 2]` | L/C/Rに対応する受信配列index。0〜2を重複なく指定 |
| `pressure_threshold` | `96`（`0x60`） | 比較の閾値、int32の範囲 |
| `pressure_comparison` | `unconfigured` | `ge`は圧力>=閾値、`le`は圧力<=閾値 |
| `max_pending_suction_checks` | `8` | 同時に監視する問い合わせ数。設定範囲1〜128 |

比較方向と実機の圧力センサーのL/C/R対応は未確認です。
`pressure_comparison` が `unconfigured` の間は吸引判定サービスだけを拒否します。
確認後に `ge` または `le` と対応indexを設定して、ポンプノードを再起動してください。

全機構の吸引を最大2秒待つ例:

```bash
ros2 service call /check_suction catchrobo2026_msgs/srv/CheckSuction \
  '{collector_mask: 7, timeout_sec: 2.0}'
```

## CAN指令と実機調整

出力は `/pump_state`（`std_msgs/msg/Int32MultiArray`、要素1個）、100 Hz。
`nhk2026_bridge` がCAN ID `0x401` の4バイトbig-endian整数として送ります。

[PumpCommand](include/catchrobo2026_pump/pump_command.hpp) は `union` で32bit整数 `raw` と、
MCU出力番号に対応する `pump1..3` / `valve1..3` のビットフィールドを表します。
上位26bitは0です。L/C/Rの状態は起動設定に従って各フィールドへ割り当てます。
トピックにはホスト順の整数を渡し、ブリッジの `send_int()` で `htobe32()` と
`memcpy()` によりバイト列へ変換します。受信側の `rxdata_to_int()` は `be32toh()` を使います。
たとえば吸引56の送信データは `00 00 00 38` です。

ビットフィールドの配置は対象ABIに依存し、`union` の別メンバからの読出しには
[GCCの拡張](https://gcc.gnu.org/onlinedocs/gcc/Optimize-Options.html#index-fstrict-aliasing)を使用します。
宣言順はホストのエンディアンで切り替え、サイズはコンパイル時に確認します。
コンパイラやCPUを変更した場合は、6ビット全64通りの整数値との対応も確認してください。

2026-09-06に確認したマイコンソース:
`../supplementary/catchrobo2026_pump/Core/Src/main.c`（ワークスペースルートからの相対パス）。
受信コールバックと `setPumpState()` / `setSolenoState()` の対応は以下です。

| CANビット | マイコン機能 | GPIO |
|---|---|---|
| bit0 / bit1 / bit2 | Pump1 / Pump2 / Pump3 | PC7 / PC8 / PC9 |
| bit3 / bit4 / bit5 | 電磁弁1 / 電磁弁2 / 電磁弁3 | PC10 / PC11 / PC12 |

マイコンは受信bit=1をGPIO Low、bit=0をGPIO Highへ変換します。
2026-09-06の指定により、GPIO Highでポンプ・電磁弁が通電する解釈を採用しています。
実機のL/C/Rとの配線・配管対応や実動作は未検証です。

| 起動パラメータ | ノード内既定値 | 調整内容 |
|---|---|---|
| `pump_bits` | `[0, 1, 2]` | L/C/Rの順に対応するポンプのCANビット番号 |
| `valve_bits` | `[3, 4, 5]` | L/C/Rの順に対応する電磁弁のCANビット番号 |
| `pump_on_level` | `false` | ポンプの論理ONをCAN bit=0（GPIO High）とする |
| `valve_on_level` | `false` | 電磁弁の論理ONをCAN bit=0（GPIO High）とする |
| `initial_state` | `1` | 全機構の起動時状態。開放 `-1`、オフ `0`、吸引 `1` |

ビット割当は各3要素、全体で0〜5を重複なく指定します。
たとえばLとRに割り当てるポンプのCANビットを入れ替えるなら `pump_bits: [2, 1, 0]` に変更できます。
極性設定はCANビットの値を指し、GPIOのHigh/Lowを直接指定するものではありません。
旧 `initial_pump_on` / `initial_valve_on` は `initial_state` へ置き換えています。

ノード内既定設定での起動状態は全機構吸引、指令は56（`0b111000`）です。
同梱YAMLの設定はこれと異なり、`valve_on_level=true`・`initial_state=0` により全機構オフの7を送ります。
一方、確認したマイコンは起動時に全6出力をstate=1（GPIO Low、`0b111111`相当）に設定します。
ROSノードが起動する前の状態は、このYAMLでは変更できません。

`/pump_state` は指令値であり、`set_pump_state` の成功は実機動作や吸着の成功確認ではありません。
空気圧による吸引成功の確認は上記の `check_suction` を使います。
通信断時の実機出力変更・保持・停止動作は、この監視サービスでは扱いません。

## main側の旧ポンプ実装との対応

2026-09-06に確認した `main`（`1b2c19d`）は、旧APIの `command=1/2/3` を
それぞれ生指令 `63/56/7` に変換します。起動値は56で、Joyの巡回は `56→7→63→56` です。
このパッケージではL/C/Rの3状態APIを使い、ノード内既定の生指令を当時のmainに合わせています。

既定ビット割当での設定と生指令の関係は以下です。

| 論理状態 | 両ON極性 `true` | 両ON極性 `false`（ノード内既定） |
|---|---|---|
| オフ `0` | 0 | 63 |
| 吸引 `1` | 7 | 56 |
| 開放 `-1` | 56 | 7 |

両ON極性を `false`、`initial_state: 1`、Joyの初回要求を開放に設定し、
mainの起動値56と操作順 `56→7→63→56` に対応します。
YAMLを指定せず直接ノードを起動した場合は、この既定設定になります。
この対応は採用した制御仕様であり、実機での通電・吸着・開放の検証結果ではありません。

## 検証

`test/test_check_suction.py` は隔離ROS domainと専用namespaceで実ポンプノードへ模擬圧力を送ります。
未受信・過去サンプル・閾値境界・部分成功・対応index・不正要求とフレーム・同時要求上限、
監視中の配信と操作継続、ROS時刻停止中の期限、終了時の保留要求を確認します。
CANや実機のセンサー・吸着動作を検証する試験ではありません。

```bash
colcon test --packages-select catchrobo2026_pump \
  --ctest-args -R test_check_suction --output-on-failure
```
