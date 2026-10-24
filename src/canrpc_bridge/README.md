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

BOOT_ANNOUNCEは通信開始直後と200ms周期。Linux getrandom／MCU hardware RNGから非ゼロ128bit Boot IDを生成し、通信世代内で維持する。ANNOUNCE_QUERYへ応答する。通常RPC serviceは未実装でREJECT_SERVICEを返す。SYSTEM_RESETは無効・capability 0、RESET_REQUESTには権限なしを返しリセットしない。

アプリ契約として、arm/pumpはROSのBOOT、ROSはarm/pressureのBOOTを監視する。未知世代、READY以外、750msの通知失効中は対応PUBLISHを破棄する。同世代のuptimeが進んだ通知だけ生存時刻を更新し、直近8世代の古いBoot IDへの後退を無視する。BOOTは計測値の鮮度を更新しない。再接続時の指令再生はしない。通知断時に既存の物理出力をどう停止するかは変更せず、既存出力を保持する。

MCU各repoのCore/Inc/canrpc.hはこの正本の同一コピーを同梱する。共通定義の変更時は3コピーを同期する。`test/test_protocol.c` はROS不要のC99 wire検証。
