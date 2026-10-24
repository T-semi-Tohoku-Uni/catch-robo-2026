# canrpc_bridge

学祭用の3軸CAN-RPC bridge。`nhk2026_bridge` のSocketCAN FD送受信・有限TX再試行を基に、専用lifecycleノードへ絞った。基本仕様は `supplementary/canrpc/spec.md`、アプリ共通定義は `include/canrpc_bridge/canrpc.h`。

| Node | ID |
| --- | --- |
| ROS bridge | 1 |
| Arm | 2 |
| Pump | 3 |
| Pressure | 4 |

| Subject | ROS topic | Source | Payload |
| --- | --- | --- | --- |
| 0x200 | target_joint_angles | 1 | q0/q1/q2, float32 × 3, rad |
| 0x210 | current_joints | 2 | q0/q1/q2, float32 × 3, rad |
| 0x401 | pump_state | 1 | existing pump/valve bitmask, int32 × 1 |
| 0x406 | pressure_sensor | 4 | existing sensor order, int32 × 3 |

全payloadはlittle-endian。Extended ID・CAN FDデータフレームのみ受信し、両方向の送信BRSはOFF。標準ID・Classic CAN・不正長・不正source・非finite関節値を破棄する。ROSの関節配列も厳密に3値、poseのphiは関節配列に含めない。init/end/幅の旧CAN送信は廃止。

BOOT_ANNOUNCEは通信開始直後と200ms周期。Linux getrandom／MCU hardware RNGから非ゼロ128bit Boot IDを生成し、通信世代内で維持する。ANNOUNCE_QUERYへ応答する。通常RPC serviceは未実装でREJECT_SERVICEを返す。ROS自身のSYSTEM_RESETは無効・capability 0、ROS宛RESET_REQUESTには権限なしを返す。MCUはNode 1からの世代条件付きSYSTEM_RESETに対応する（capability 1）。

アプリ契約として、arm/pumpはROSのBOOT、ROSはarm/pressureのBOOTを監視する。未知世代、READY以外、750msの通知失効中は対応PUBLISHを破棄する。同世代のuptimeが進んだ通知だけ生存時刻を更新し、直近8世代の古いBoot IDへの後退を無視する。BOOTは計測値の鮮度を更新しない。再接続時の指令再生はしない。通知断時に既存の物理出力をどう停止するかは変更せず、既存出力を保持する。

MCU各repoのCore/Inc/canrpc.hはこの正本の同一コピーを同梱する。共通定義の変更時は3コピーを同期する。`test/test_protocol.c` はROS不要のC99 wire検証。


## 本番起動時の初期化

`reset_on_startup` のbridge既定値はfalse、本番launchのみtrueを渡す。初期化中もROS自身のBOOT READYを200ms周期で配信する。このREADYはROSの通信準備状態を意味する。

全3MCU（Node 2/3/4）の現在世代READY・SYSTEM_RESET capabilityを確認してから、旧Boot IDを固定した48byte RESET_REQUESTを各1操作送る。受付締切は対象uptime＋観測後の経過＋1000ms。同じ操作の再送はTag・nonce・payloadを保持し、200ms間隔で最大900ms未満まで行う。ACCEPTEDまたは新世代を観測したら再送を止める。新世代に合わせたRESET_REQUESTの作り直しはしない。

ACCEPTEDは完了と扱わず、各MCUの異なるBoot IDとREADYを全台で確認して初めてactiveへ遷移する。STATUSが失われても新世代READYで完了できる。初期化完了までtarget/pump購読を作成せず、自動操縦も起動しない。拒否、通信エラー、既定30秒の全体期限超過は起動失敗として子を終了し、launchを非ゼロで終了する。timeoutは未実行・停止済みを意味しない。`startup_timeout_sec` は0より大きく600以下で設定する。

CAN設定は `raspi_can.launch.py` に集約し、正しい設定で既にUPならdown/upしない。`run_production.sh`は環境を確認してlaunchへ渡すだけで、helpや`--show-args`の表示でCAN設定・初期化を実行しない。

armの第1軸は、既存STOP命令の実送信を確認してからMCUをリセットする。第2・3軸とポンプは既存出力を保持する。この確認は物理的な静止feedbackの確認ではない。MCU再起動により既存のbase原点復帰とポンプのboot出力が再実行される。
