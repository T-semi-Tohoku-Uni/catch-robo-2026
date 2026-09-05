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
Joyの〇ボタンは、全3項目に同じ値を送り「オフ → 吸引 → 開放」の順に操作します。
手動側の巡回位置はサービス成功時に進みます。別クライアントの操作やポンプノードの再起動とは同期しません。
応答が1秒以内に届かなければ巡回位置を進めず、次のボタン操作で同じ要求を再試行します。

## CAN指令と実機調整

出力は `/pump_state`（`std_msgs/msg/Int32MultiArray`、要素1個）、100 Hz。
`nhk2026_bridge` がCAN ID `0x401` の4バイトbig-endian整数として送ります。

2026-09-06に確認したマイコンソース:
`../supplementary/catchrobo2026_pump/Core/Src/main.c`（ワークスペースルートからの相対パス）。
受信コールバックと `setPumpState()` / `setSolenoState()` の対応は以下です。

| CANビット | マイコン機能 | GPIO |
|---|---|---|
| bit0 / bit1 / bit2 | Pump1 / Pump2 / Pump3 | PC7 / PC8 / PC9 |
| bit3 / bit4 / bit5 | 電磁弁1 / 電磁弁2 / 電磁弁3 | PC10 / PC11 / PC12 |

マイコンは受信bit=1をGPIO Low、bit=0をGPIO Highへ変換します。
実機のL/C/Rとの配線・配管対応、およびどちらのビット値で通電するかは未確認です。

| 起動パラメータ | 既定値 | 調整内容 |
|---|---|---|
| `pump_bits` | `[0, 1, 2]` | L/C/Rの順に対応するポンプのCANビット番号 |
| `valve_bits` | `[3, 4, 5]` | L/C/Rの順に対応する電磁弁のCANビット番号 |
| `pump_on_level` | `true` | ポンプの論理ONをCAN bit=1とする。ONがbit=0なら `false` |
| `valve_on_level` | `true` | 電磁弁の論理ONをCAN bit=1とする。ONがbit=0なら `false` |
| `initial_state` | `-1` | 全機構の起動時状態。開放 `-1`、オフ `0`、吸引 `1` |

ビット割当は各3要素、全体で0〜5を重複なく指定します。
たとえばLとRに割り当てるポンプのCANビットを入れ替えるなら `pump_bits: [2, 1, 0]` に変更できます。
極性設定はCANビットの値を指し、GPIOのHigh/Lowを直接指定するものではありません。
旧 `initial_pump_on` / `initial_valve_on` は `initial_state` へ置き換えています。

既定設定での起動状態は全機構開放、指令は `0b111000` で、従来ROSノードの初期値を維持しています。
一方、確認したマイコンは起動時に全6出力をstate=1（GPIO Low、`0b111111`相当）に設定します。
ROSノードが起動する前の状態は、このYAMLでは変更できません。

マイコンには現時点で `0x404` の返信、吸着センサの報告、通信断時の出力変更処理はありません。
`/pump_state` は指令値であり、サービス成功は実機動作や吸着の成功確認ではありません。
