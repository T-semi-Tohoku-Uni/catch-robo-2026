# 設定と再利用

`config/gakusai.yaml` が学祭構成の設定正本です。ROS nodeは `config_file` をconfigure時に読み、launchとMCU生成ツールは同じprofileを検証します。設定変更は各ROS process・必要なMCUを再起動して反映します。`config_file`、`reset_on_startup`、`startup_timeout_sec` は起動時だけ指定できるread-only ROS parameterです。

## 起動

```bash
ros2 launch canrpc_bridge raspi_can.launch.py config_file:=/absolute/project.yaml
```

`reset_on_startup` は既定false。本番 `production.launch.py` ではtrueを渡します。本番のprofile差替え引数は `canrpc_config:=/absolute/project.yaml` です。`startup_timeout_sec` は既定0でprofile値を使い、正の値を明示した場合だけ全体期限を上書きします。他のbus・通信・route設定をlaunch既定値で上書きしません。

CAN設定を行うのは `bus.configure: true` の場合だけです。設定済みのUP／FD／bitrate／dbitrateが一致すればdown/upしません。falseの場合は既存SocketCANインターフェースを使い、ipコマンドを実行しません。全profileとlaunchの明示overrideを検証してから設定を行います。`--show-args` とscriptのhelpはCANを操作しません。

## Schema version 1

未知key・重複YAML key・欠損必須値・不正な値を拒否します。整数・boolean・stringの型を区別します。数値を文字列として引用しないでください。完全な例は [gakusai.yaml](../config/gakusai.yaml) と [reuse_example.yaml](../config/reuse_example.yaml) です。

| root項目 | 内容 |
| --- | --- |
| schema_version | 整数1 |
| project | 空でないプロジェクト名。表示・記録用 |
| local_node | ROS bridgeのNode ID、1〜126 |
| bus | SocketCANと送信設定 |
| timing | 管理優先度・周期・peer監視 |
| reset | 起動RESET対象と期限 |
| nodes | セグメントのNode定義とreset許可元 |
| routes | ROSとPUBLISHの接続定義list |
| rpc／rpc_endpoints | 普通RPC client設定と名前付きendpoint |
| application | 機構固有設定map。ROS汎用loaderは内容を実行しない |

### bus

全項目必須です。

| 項目 | 内容 |
| --- | --- |
| interface | 1〜15文字、英数字・`_ . -` のインターフェース名 |
| configure | launchでインターフェースを設定するboolean |
| nominal_bitrate | 1〜1,000,000 bit/s |
| data_bitrate | nominal以上、8,000,000 bit/s以下 |
| brs | CAN FD送信BRSのboolean |
| non_blocking | SocketCAN送信のnonblocking設定 |
| tx_retry_timeout_ms | 1〜1000msの有限TX再試行予算 |

CAN FDとExtended IDはプロトコル要件として固定です。nonblocking=falseではカーネルwrite自体がブロックする場合があり、TX再試行予算とは別です。

### timing

全項目必須。`priority` と `announce_priority` は0〜7、他は正の整数です。

| 項目 | 内容 |
| --- | --- |
| priority | 管理フレームとroute未指定時の送信優先度 |
| announce_priority | BOOTの既定優先度。nodeごとのoverride可 |
| announce_period_ms | BOOT通知周期 |
| peer_timeout_ms | READYと見なす最終通知からの期限 |
| poll_period_ms | ROS受信pollと起動初期化loopの周期 |
| rx_budget | poll1回の最大受信数、1〜65536 |

`poll_period_ms <= announce_period_ms < peer_timeout_ms` を要求します。各時間は2^31ms未満です。

### reset

全項目必須です。

| 項目 | 内容 |
| --- | --- |
| targets | 起動時にresetするNode IDのlist。重複不可、local_node以外のnodes宣言済みID |
| timeout_sec | 起動初期化の全体期限、正の有限秒 |
| request_window_ms | 対象uptime軸に加算する受付窓 |
| max_window_ms | serverが許す受付窓の上限 |
| retry_interval_ms | 同一RESET_REQUESTの再送間隔 |
| retry_window_ms | 初回送信から再送可能な期間 |
| execution_timeout_ms | MCUの安全化・ACCEPTED実送信・reset移行の総期限 |

`retry_interval <= retry_window < request_window <= max_window`、`execution_timeout <= request_window < timeout_sec×1000` を要求します。時間は2^31ms未満です。targets空はreset無効起動なら利用できますが、`reset_on_startup: true` では拒否します。明示のstartup timeout overrideは障害試験や起動期限短縮にも利用でき、profileの受付窓との大小は強制しません。

### nodes

任意のrole名から `{id, reset_sources, announce_priority?}` へのmapです。role名は英字または`_`で始まる英数字・`_`。IDは1〜126で全role間一意、local_nodeを含めます。

`reset_sources` はSYSTEM_RESETを許可する他Node IDの重複なしlistです。空ならreset無効／capability 0。すべてnodesに宣言します。ROSのlocal_nodeはSYSTEM_RESET serverを実装しないため、該当nodeのreset_sourcesは空である必要があります。`announce_priority` は省略するとtiming値を使います。

profileのlocal_nodeはROS側の送信元です。MCUの自身のIDは生成対象roleのidを使うため、全MCUにlocal_nodeの値を自身のIDとして設定しません。

### routes

各routeの必須値は `name, direction, topic, subject, type, count` です。

| 項目 | 内容 |
| --- | --- |
| name | 空でない一意のroute名 |
| direction | `tx`: ROS→CAN、`rx`: CAN→ROS |
| topic | ROS topic名。絶対／相対／`~/`を使用可能、置換式なし |
| subject | 0〜4095 |
| type | `float32`、`int32`、`uint8` |
| count | float32/int32は1〜16、uint8は1〜64 |
| priority | 任意、0〜7。省略時timing.priority |
| source | rx必須、宣言済みremote Node ID。txでは指定せずlocal_nodeを使用 |
| require_peer | 任意。rx既定true、txはfalseのみ |
| finite | float32だけの任意boolean、既定true。falseでは非finiteを型として許可 |
| min／max | 任意の全要素共通値域。有限値、min<=max |

float32は `std_msgs/msg/Float32MultiArray`、int32は `Int32MultiArray`、uint8は `ByteMultiArray` です。txはdata要素数がcountと一致し、layout.data_offset=0であることを要求します。layoutのshape／labelはwireへ載せません。rxもcount値だけの配列を発行します。

同Subjectは全routeで同type/countを要求します。TX Subject重複と同source+subjectのRX重複、同方向topic重複は拒否します。int32/uint8のmin/maxは型範囲内の整数に限定します。NaNはfinite=falseでもmin/maxとの比較を満たしません。rxのrequire_peer=falseはBOOT未受信でもそのrouteを受け付けますが、起動RESETの新READY確認は別途必須です。

## 学祭profileのアプリケーション契約

既定 `gakusai.yaml` は次の既存wire値・単位・並びを維持します。

| Subject | topic／方向 | Data |
| --- | --- | --- |
| service 1 | arm_target／REQUEST、Node 2 | q0／q1／q2、float32×3、rad、finite必須 |
| 0x210 | current_joints／rx、Source 2 | q0／q1／q2、float32×3、rad、finite必須 |
| RPC pump_set | Node 3／Service 0 | 既存ポンプ／弁絶対bitmask、u32 LE、0〜63 |
| 0x406 | pressure_sensor／rx、Source 4 | 既存センサ順の値、int32×3 |

オフセット・座標変換や圧力値の正規化をbridgeで追加しません。圧力の物理単位・L/C/R配線はこのprofileで確定せず、ポンプの論理対応・極性・圧力indexは従来のpump設定が担当します。関節の第4軸・phiや幅操作はwireへ追加しません。

## 別プロジェクトでの実証例

`reuse_example.yaml` はproject=conveyor_demo、local=17、reset対象42/77、異なるtopic・Subject・優先度・bitrate・BRSを使います。float32×2、int32×4、uint8×9を定義し、uint8×9はwire12byte・末尾3byte=0になります。

```bash
ros2 launch canrpc_bridge raspi_can.launch.py \
  config_file:=/absolute/reuse_example.yaml reset_on_startup:=true
```

この例の `bus.configure: false` は既存インターフェースを使う設定であり、インターフェースや対向ノードを作成しません。Node 42/77も同じwire・profileの役割に適合する必要があります。ROSのpackage・ソース・message定義を変更する必要はありません。

## 普通RPCとROS Call

`rpc`／`rpc_endpoints`は両方まとめて省略でき、その場合はPUBLISH／起動RESETだけを使い普通RPCのROS serviceを作りません。片方だけの指定は拒否します。

指定時は`rpc.ros_service`（学祭設定`canrpc/call`）に `canrpc_bridge/srv/Call` を作成します。要求は `string endpoint` と論理payloadの `uint8[] data`、応答は `uint8 outcome`、論理responseの `uint8[] data`、`string message` です。アプリケーションはNode／Serviceを保持せずendpoint名を指定します。

| outcome | 意味 |
| --- | --- |
| 0 OK | remoteの最終RESPONSEを受信。application resultはdata内で別途解釈 |
| 1 REJECTED | 明示CONTROL拒否 |
| 2 TIMEOUT_UNKNOWN | 応答期限・通知失効・UNKNOWN_OUTCOME・送信エラーなどで実行結果不明 |
| 3 PEER_CHANGED | 相手Boot世代変更、実行結果不明 |
| 4 UNAVAILABLE | peer条件未成立またはguard済みTagを確保できず未送信 |
| 5 BUSY | 同一peerの別RPCがinflightで未送信 |
| 6 INVALID_REQUEST | endpoint名・論理request長不正で未送信 |

ROS callbackは最終状態まで応答を保留し、executorを待機でblockしません。ROS caller側のtimeoutを延ばしてもwireの期限／cache期間は変わりません。callerが待機をやめても操作の取消・未実行を意味しません。上位の待機期限はprofileのresponse_timeout_msより長く設定します。

`rpc`の全項目は `ros_service, retry_interval_ms, retry_window_ms, query_interval_ms, response_timeout_ms, result_retention_ms, tag_guard_ms, max_frame_age_ms, cache_capacity`。時間は正・2^31ms未満、cache_capacityは1〜32です。`retry_interval <= retry_window < response_timeout`、`query_interval < response_timeout`、`result_retention >= response_timeout + max_frame_age`、`tag_guard >= response_timeout + result_retention + max_frame_age` を要求します。

`rpc_endpoints` は `{name, destination, service, request_bytes, response_bytes, require_ready?, startup_diagnostic?}` のlist。nameとdestination＋serviceの組を一意にし、destinationは宣言済みremote Node、serviceは0〜31、各論理長は0〜64。require_readyの既定はtrue。falseでもpeer Bootが既知でfreshである必要があります。長さ・padding不一致のRESPONSEは無視し、別操作の応答を当てません。

学祭profileはpump_setを絶対mask設定、arm_readyを非作動の準備状態確認として使います。旧pump_stateの100Hz PUBLISHを併用してRPC結果を上書きすることはありません。INIT／BOOTの上位状態遷移は、ROS serviceの受付だけでなく返却されたapplication resultとflagsで判断します。

`application.motor_initialization` は学祭STM32アプリのbounded boot手順用で、step_timeout_ms／homing_timeout_ms／feedback_timeout_ms／feedback_max_age_ms／inter_command_ms／home_settle_msを定義します。汎用ROS bridgeやportable coreがモータ制御を行う設定ではありません。

## 共通coreとMCU生成

`core/include/canrpc_core.h` はC99 header-onlyでROS／HALに依存せず、ID・codec・padding・peer管理・普通RPC client/serverのTag／cache／期限判定・RESET client/server判定を共通化しています。OS乱数、SocketCAN、ROS publisher/subscriber、HAL送受信、機構停止・reset実行はアダプターの責務です。

主なAPIは `canrpc_id/publish_id`、`put/get_u32/u64/float`、`wire_length/padding_valid`、`peer_announce/peer_ready`、`reset_client_begin/status/complete/should_retry`、`reset_server_request/can_execute`、普通RPCの `rpc_client_begin/next/receive/finish` と `rpc_server_request/query/complete`。優先度・時間・Node IDは呼出側から渡します。プロトコル定数と固定wire構造はcoreに置き、プロジェクト値をcoreへ追加しません。

```bash
cmake -S core -B /tmp/canrpc-core-build
cmake --build /tmp/canrpc-core-build
ctest --test-dir /tmp/canrpc-core-build --output-on-failure
python3 tools/canrpc_profile.py config/gakusai.yaml \
  --mcu-root /path/to/mcu --check
```

`--check`を外すと正本core・STM32アダプター・profileから各独立MCU repoのvendor/config headerを更新します。MCU repo単独でbuildでき、外部repoやsubmoduleの追加は不要です。更新後はMCUもbuild・反映が必要です。現STM32学祭アプリの生成はros/arm/pump/pressure role・既存の3軸／ポンプ／圧力payloadへ限定し、80MHz／20tqで表現できないbitrateや未対応アプリ構成を明示拒否します。別機構のアプリ処理は別アダプターへ実装し、共通coreをそのまま利用します。

launch/Python generator/C++ runtime loaderは同じprofile制約を検証します。インストール後もshare/canrpc_bridgeのconfig・docs・core・adapters・toolsとincludeのcoreが同梱され、外部supplementary資料を必要としません。

STM32学祭アダプターの機構保護はprofileのroute検査とは別です。ROS側のfinite/min/max/require_peerを変更しても、MCU側の実機固有のfinite検査・ポンプ0〜63・fresh controller gateを解除しません。別アプリでのpayloadと機構保護は、そのアプリのアダプターで定義します。

CatchRobo用generatorを使わない新MCUでも、coreへ任意のNode・allowlist・時間を直接渡せます。例えばNode 42がcontroller 17からのresetだけを許可する構成は次のように記述できます。

```c
#include "canrpc_core.h"

static const uint8_t allowed[] = {17};
static const canrpc_reset_server_config_t policy = {
  1, allowed, 1, 500, 1500, 200
}; /* enabled, sources/count, peer timeout, max deadline, execution timeout */
static canrpc_peer_t controller;
static canrpc_reset_server_t reset_operation;

/* Receive validated Extended CAN FD bytes from the platform adapter. */
void receive_reset(const uint8_t own_boot[16], const uint8_t request[48],
                   uint8_t tag, uint32_t now_ms) {
  uint8_t status;
  if (canrpc_reset_server_request(&reset_operation, &policy, own_boot,
        &controller, CANRPC_READY, 17, tag, request, 48, now_ms, &status)) {
    /* Start application safety handling; confirm status TX before reset. */
  }
}
```

この例ではcontrollerのBOOTを `canrpc_peer_announce` へ渡し、返信IDに `canrpc_id(3, CANRPC_CONTROL, 17, CANRPC_RESET_STATUS, 42, tag)` を使います。乱数生成、受信source／destinationの照合、時刻、送受信、安全化、MCU resetはプラットフォーム実装が担当します。特定のrole名やROSを必要としません。

`startup_diagnostic`は既定false。trueはrequest_bytes=0かつrequire_ready=falseに限定し、起動失敗時にも非作動の診断だけを有限時間で照会します。学祭arm_readyの応答は48byte/version1で、形式は[protocol](protocol.md#arm-readiness-diagnostic-version-1)を参照してください。

Nodeごとに`nodes.<role>.rpc`へ8つのpolicy値（ROS service名以外）をすべて指定すると、global rpcをそのpeerだけ上書きします。Tagは同じpeerの全serviceで共有するためendpoint別policyにはしません。cache_capacity上限は32。学祭armはresponse 100ms、retry interval 10ms/window 20ms、query 10ms、retention 120ms、guard 240ms、max frame age 20ms、cache 32です。pumpは既定1000ms/guard5000ms/cache8を維持します。

学祭`arm_target`はNode2 service1、request float32LE q0/q1/q2(rad)の12byte、response result u32LE＋目標echo12byte＋axis confirmation mask u32LEの20byteです。旧target PUBLISH routeは削除し、連続入力のcoalescing/送信上限/unknown時の停止は専用ROS target adapterが担当します。generic bridgeはrobot固有のresultをdecodeしません。

明示INITではROS専用`ArmTargetGate`で目標送信をpauseし、既存inflightが終端した後にarm_readyを照会します。READYのときだけ同じgate tokenと全体deadlineでresumeします。失敗時はpauseを維持します。deadlineは同一hostのCLOCK_MONOTONIC nsであり、古いresume/期限超過はadapterが拒否します。resumeは物理停止や再homeを意味しません。

startup diagnosticは同じdestinationに1endpointまでです。resumeの応答喪失・期限超過で成功を確認できない場合は、stateが新しい期限を付けたbest-effort PAUSEを送ります。これは元の失敗を変更せず、PAUSE受信・物理停止の確認も意味しません。期限内resumeが適用済みで応答だけ失われた場合はUNKNOWNとして扱い、後の明示INITで再確認します。
