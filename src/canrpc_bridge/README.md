# canrpc_bridge

YAMLのプロジェクト設定からROSトピックとCAN-RPC PUBLISHを接続し、名前付きendpointの普通RPCをROS serviceへ公開するCAN FD用lifecycle bridgeです。Node ID、Subject、topic、型、要素数、値域、通信設定、起動RESET対象と時間設定をコードから分離しています。ROSとMCUは同じROS非依存C99 coreを使います。

- [実装プロトコル仕様](docs/protocol.md)
- [設定・他プロジェクトへの再利用](docs/configuration.md)
- [共通C99 core](core/README.md)

通常のROS起動:

```bash
ros2 launch canrpc_bridge raspi_can.launch.py config_file:=/absolute/project.yaml
```

bridge単体を手動でconfigure/activateする場合も設定ファイルを必ず指定します。

```bash
ros2 run canrpc_bridge canrpc_bridge --ros-args -p config_file:=/absolute/project.yaml
```

同梱の `config/gakusai.yaml` がこのロボットの設定正本です。起動RESETはbridge単体では無効、本番 `run_production.sh` / `production.launch.py` では有効です。本番に別設定を渡す場合は `canrpc_config:=/absolute/project.yaml` を指定します。

普通RPCは `canrpc_bridge/srv/Call` のendpoint名で呼び、最終RESPONSEまたは結果不明までROS応答を保留します。実測feedbackはPUBLISH、連続関節目標・ポンプの絶対状態変更・準備状態照会はRPCです。

UI・自動制御プロセスはCANの成否と独立して起動します。起動RESETは全対象の現在READYを確認してから世代条件付きSYSTEM_RESETを送り、異なるBoot IDのREADYを全対象で確認するまで実機操作を許可しません。CAN/初期化失敗ではDEGRADEDとして診断を続け、全体を終了しません。CAN設定はlaunchへ集約し、help/`--show-args`では実行しません。

学祭armは起動後、通信READY／モーターWAITINGで待機し、明示UI INITの`arm_initialize`要求までnative設定・STOP・原点復帰を送りません。startup SYSTEM_RESETは有効のままで、未開始ならnative送信なし、開始済みなら既存q0 STOPの実TX確認後にresetします。手順内・FAULT時のSTOPは維持します。ポンプは6出力OFF（既存ROS極性に対応するGPIO Low／wire mask63）で起動します。UI INITは開始応答のBoot IDと一致する64byte/version2の`arm_ready`で完了を有限時間確認します。[wireと運用](docs/protocol.md#明示的なモーター初期化)。


### CAN未接続・駆動電源なしでの診断

`canrpc/status`（canrpc_bridge/msg/BridgeStatus、reliable/transient-local、1Hzと状態変化時即時）はSTARTING/DEGRADED/READY、commands_enabled、socket_open、session_id、理由を表示します。session_idはactivateごとのホストBoot IDです。target adapterはstatus失効/DEGRADED/世代変更でowner/gateを閉じ、復帰だけで古いJoy目標を再開しません。Lifecycle activeはROS bridge処理の稼働であり、MCU初期化完了ではありません。STATUS READYは通信操作の許可であり、モーター準備完了は明示UI INITの成功で確認してください。ホストBOOTのREADYは通信準備を意味し、MCUやアームのREADYを代替しません。

STARTINGは元の全体期限（学祭30秒）内で非同期にRESETを進め、その間に処理したCallはUNAVAILABLEです。READY/activate/deactivateの境界ではCall窓口と世代を更新し、旧未処理要求はUNAVAILABLEまたは呼出側の有限timeoutとなってwireへ再演しません。すべてのROS要求への即応答を保証するものではありません。timeout/RESET拒否はDEGRADEDへ移ります。DEGRADEDでは一般RPC/目標PUBLISHを拒否し、`startup_diagnostic: true`だけを照会できます。診断wireがREADYでもROS outcomeはUNAVAILABLEのまま実dataを返すため、UI INITを偽成功にしません。CAN RX/TX/controller error・required peerのREADY失効/世代変更はpendingをUNKNOWN/PEER_CHANGEDで一度だけ終端し、古い目標を破棄してlatchします。

電源やBOOTが戻っただけではRESETや運動を再開しません。原因を直し、profile通りにinterfaceを準備した後、`ros2 lifecycle set /canrpc_bridge deactivate`、`ros2 lifecycle set /canrpc_bridge activate`を明示して初期化を再試行してください。unconfiguredなら先にconfigureが必要です。launchのip設定失敗ではinactiveのまま残り、ip設定はlifecycle操作では再実行されません。bridgeプロセスが終了した場合は明示して再起動してください。復旧後もUIで明示INITが必要で、arm boot FAULT時のMCU resetは手動という既存制約を維持します。

`reset_on_startup=false`は開発・再利用用で、READYはtransport準備のみです。全reset.targets必須監視は導入せず、RPCは各endpointのpeer条件で拒否します。設定不正・他ノード自体の故障まで隠す契約ではありません。

通常はMCU起動→駆動電源投入→UI INITの順です。駆動電源を維持した外部MCU reset／MCU電源断では、モーター側に前回の電流・位置指令が残る可能性があります。開始済みCANRPC resetのSTOP実TX確認は維持しますが、外部resetまで安全化する保証ではありません。FAULT後は駆動電源OFF→MCU reset→駆動電源ON→bridge明示復旧→UI INITを基本手順として新しい試行を開始します。UART printfは診断であり、駆動電源の検出や物理停止の証明ではありません。

## 圧力基板の省略と再追加

既定profileはpressure routeを`enabled: false`、`reset.targets`を`[2,3]`とし、圧力基板を起動条件から外します。圧力Node/wire定義とfirmwareは保持し、圧力topic配信・吸引判定・UI圧力表示は使いません。再追加は同profileでrouteをtrue、targetsを`[2,3,4]`へ戻してROS一式を再起動してください。[設定手順と保証境界](docs/configuration.md#圧力基板を省略する既定構成)。
