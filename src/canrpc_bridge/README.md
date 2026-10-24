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

起動RESETは全対象の現在READYを確認してから世代条件付きSYSTEM_RESETを送り、異なるBoot IDのREADYを全対象で確認するまで指令購読・UI・自動制御を起動しません。失敗時は子を終了してlaunchを非ゼロで終了します。CAN設定はlaunchへ集約し、help/`--show-args`では実行しません。

学祭armの第1軸は既存STOP命令の実送信確認後にMCUをリセットし、第2・3軸とポンプは既存出力を保持します。物理的な静止feedbackの確認ではありません。再起動で既存のbase原点復帰とポンプのboot出力が再実行されます。
