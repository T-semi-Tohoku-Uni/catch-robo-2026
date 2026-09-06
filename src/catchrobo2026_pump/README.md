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
既定設定では起動時が吸引56で、ボタンを押すごとに `7 → 63 → 56` と巡回します。
手動側の巡回位置はサービス成功時に進みます。別クライアントの操作やポンプノードの再起動とは同期しません。
応答が1秒以内に届かなければ巡回位置を進めず、次のボタン操作で同じ要求を再試行します。

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

| 起動パラメータ | 既定値 | 調整内容 |
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

既定設定での起動状態は全機構吸引、指令は56（`0b111000`）で、mainの初期値を維持しています。
一方、確認したマイコンは起動時に全6出力をstate=1（GPIO Low、`0b111111`相当）に設定します。
ROSノードが起動する前の状態は、このYAMLでは変更できません。

マイコンには現時点で `0x404` の返信、吸着センサの報告、通信断時の出力変更処理はありません。
`/pump_state` は指令値であり、サービス成功は実機動作や吸着の成功確認ではありません。

## main側の旧ポンプ実装との対応

2026-09-06に確認した `main`（`1b2c19d`）は、旧APIの `command=1/2/3` を
それぞれ生指令 `63/56/7` に変換します。起動値は56で、Joyの巡回は `56→7→63→56` です。
このパッケージではL/C/Rの3状態APIを使い、全機構を同じ状態にしたときの生指令をmainに合わせています。

既定ビット割当での設定と生指令の関係は以下です。

| 論理状態 | 両ON極性 `true` | 両ON極性 `false`（現在の既定） |
|---|---|---|
| オフ `0` | 0 | 63 |
| 吸引 `1` | 7 | 56 |
| 開放 `-1` | 56 | 7 |

両ON極性を `false`、`initial_state: 1`、Joyの初回要求を開放に設定し、
mainの起動値56と操作順 `56→7→63→56` を維持しています。
YAMLを指定せず直接ノードを起動した場合も、同じ既定設定になります。
この対応は採用した制御仕様であり、実機での通電・吸着・開放の検証結果ではありません。
