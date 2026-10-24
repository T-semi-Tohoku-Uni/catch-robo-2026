# Profile validation and STM32 synchronization

`canrpc_profile.py` validates the version-1 YAML profile without ROS or hardware access. It exports `validate_profile(path)` for launch-time validation before any CAN interface changes. Python 3 and PyYAML are required.

```sh
python3 tools/canrpc_profile.py config/gakusai.yaml
python3 tools/canrpc_profile.py config/reuse_example.yaml
python3 tools/canrpc_profile.py config/gakusai.yaml --mcu-root /path/to/mcu
python3 tools/canrpc_profile.py config/gakusai.yaml --mcu-root /path/to/mcu --check
```

The generic schema covers Node IDs, routes/types/counts/priorities, CAN settings, BOOT/peer timing, and RESET policy. Duplicate YAML keys, unknown fields, invalid scalar types, ambiguous routes, inconsistent subject schemas, illegal IDs, lengths exceeding one CAN FD frame, and invalid timing relationships are rejected. The ROS local node does not offer a reset server; its `reset_sources` must be empty. RESET sources on MCU nodes are explicit allowlists; an empty list disables SYSTEM_RESET and its capability.

Without `--mcu-root`, validation places no STM32 assumptions on a reusable ROS profile. `reuse_example.yaml` demonstrates different node identities and payload lengths. Hardware generation specifically targets the existing CatchRobo three-axis arm, pump, and pressure applications: role names `ros/arm/pump/pressure`, current float32[3] and pressure int32[3] PUBLISH routes, plus pump_set request4/response8, arm_ready request0/response64, arm_initialize request0/response64, and arm_target request12/response20 RPC endpoints remain their physical application contract. Changing that hardware contract requires changing the application, not just the profile.

The synchronization command vendors the exact portable `core/include/canrpc_core.h`, the canonical `adapters/stm32g4/canrpc_stm32.h`, a small application include wrapper, and generated `canrpc_project.h` into each repository's `Core/Inc`. The project header holds all project Node IDs/Subjects, priorities, BOOT/peer and reset timings, reset source policy, BRS flag, and CAN prescalers. Each MCU repository builds independently with Python 3 (standard library only), without ROS, PyYAML, or a sibling checkout. The exact standalone checker and canrpc_board.h startup guard are also vendored.

The current STM32 adapter uses an 80 MHz FDCAN clock and 20 time quanta (seg1=15, seg2=4). Generation accepts nominal/data rates only when integer prescalers exactly represent them (nominal 1..512, data 1..32). It updates FDCAN1 prescalers and filter counts in `.ioc`, checks the expected 80 MHz metadata and generated numeric initialization, and leaves motor-native FDCAN3 untouched. BRS comes from the generated header at transmission. The execution timeout must exceed the MCU application's 10 ms main poll. Unsupported timing fails explicitly, without rounding or silent defaults.

`--check` writes nothing and fails for vendor/header/`.ioc` drift or changed FDCAN1 initialization. First synchronize the profile/header/.ioc, then regenerate with CubeMX, run --check, and rebuild. Synchronization deliberately permits stale generated C so this workflow cannot deadlock. A successful check does not prove that firmware installed on hardware matches the files or that physical CAN communication works.

Ordinary RPC profiles add `rpc` finite timeout/cache/Tag policy and `rpc_endpoints` with named destination/Service and logical request/response lengths. The current MCU applications accept the profile's ROS node as the ordinary RPC requester. The arm readonly endpoint permits a fresh known server in STARTING/FAULT; mutating pump calls require READY. `application.motor_initialization` is optional application data in generic profiles and required only for this STM32 arm generation, with finite command/homing/feedback waits and existing ADRC freshness limits. `initialization_timeout_ms` bounds the whole explicit initialization attempt (default 20000 ms), must cover the bounded FSM plus 10 ms polling margins, and does not change ordinary RPC deadlines. The arm starts in application WAITING with transport READY; only `arm_initialize` starts native setup, once per boot. Both `arm_ready` and `arm_initialize` return the 64-byte version-2 diagnosis including the MCU Boot ID.

Node `rpc` overrides select a peer-specific ordinary RPC policy. The core/server cache maximum is 32; the arm profile uses a shorter guarded window and larger cache for latest-target calls, while the pump retains its slower policy. Optional `startup_diagnostic` endpoints must be readonly, require zero request bytes, allow non-READY peers, and be unique per destination. `application.arm_control.native_ack_timeout_ms` is checked against the existing 50ms ADRC step bound and the arm RPC deadline including main-poll margin. `target_period_ms` configures the ROS sender; it does not alter native motor timing.

## CubeMX生成設定の単独検査

各MCU repoから `python3 tools/check_canrpc.py --repo . --role arm`（pump/pressureではroleを変更）を実行します。Python標準ライブラリだけを使い、最上位CMakeのALL target依存から毎build実行します。初回configure後に`.ioc`だけを変更しても検査を再実行し、不一致をbuild失敗にします。

検査範囲はprofileheader/.ioc/生成数値のFDCAN1整合、80MHz clock、GPIO/IRQとhandler dispatch、TIM6、USER起動hook、各基板のnative CAN/ポンプGPIO/ADC-DMA前提です。無関係なUART表示変更等は許容し、全.ioc snapshot一致を要求しません。USER CODE保持を有効にし、非生成headerと最上位CMakeを維持してください。

`canrpc_board_valid`はUSER2でHAL初期化後とCAN filter/start後の二回だけ、実clock mux/kernel周波数/divider、handle＋NBTP/DBTP/CCCR、PA11/12 AF、filter RAM/FIFO1/IRQを読み取り検査します。設定上書き・二重HAL初期化・周期割込中の繰返し検査はしません。不一致はCANRPC/app開始を拒否しますが、MX_GPIO_Initの既存GPIO初期出力は既に生じています。ポンプ物理出力や停止の保証を追加する変更ではありません。

実CubeMX未導入の環境ではUSER保持を模擬したfixtureと設定破壊負例・cross buildを検証しました。実CubeMXの版別再生成や生成方式変更まで検証済みとは扱いません。

## MCU UART diagnostics

The three applications send diagnostics on their existing USART2 (115200 baud,
8N1, PA2 TX / PA3 RX). This is UART output; USB CDC and SWO are not enabled by
this change. A USB serial adapter or an existing board debugger VCP requires the
actual board wiring to match. A disconnected receiver never stalls control.

`canrpc_debug.h` is canonical here and is vendored by `canrpc_profile.py`.
`_write` queues bytes into a fixed 2048-byte ring only from main context with
interrupts enabled. IRQ/masked calls and overflow are dropped and counted by
`drop`; no formatting is called from periodic IRQs. Each main-loop drain writes
at most 32 bytes and exits immediately when UART TXE is not ready. No UART IRQ,
DMA, HAL transmit wait, float formatting, or reset-time flush is introduced.
Arm and pump service the UART during their existing 10 ms tick wait. Fatal
handlers may drain already queued bytes with interrupts disabled and do not
wait on HAL ticks. Fatal errors before USER2/UART readiness cannot print.

Boot lines report the node, actual UART baud rate, Boot ID, and startup stage.
Arm lines report RPC state, boot step, boot completion/failure reason and failing axis;
per-axis lines report motor ID, mode, fault, freshness flags, feedback age,
command and response. Axis data is captured with a short IRQ mask and formatted
after interrupts resume; on failure the original fault snapshot is retained.
Numeric stage/reason/command definitions remain in `arm_boot.h` and the arm
README. Arm summaries occur on boot-step/state changes and otherwise at 1 Hz;
pressure raw/filtered values occur at 1 Hz from main; pump GPIO applied-mask
changes produce an event. These values do not prove motor power, motion,
pneumatic actuation, or sensor validity.

FDCAN1 (bus=rpc), plus arm FDCAN3 (bus=motor), PSR/ECR changes are sampled at most once per second and report raw values,
BUSOFF, TEC and REC. Reading PSR acknowledges LEC/DLEC history (read-to-clear);
this is diagnostic sampling, not a recovery operation. Legacy CAN transmit
failures are counted in IRQ context and reported from main at most once per
second. High-rate targets and individual successful CAN frames are not logged.
Output is best effort: overflow, IRQ failures and immediate resets can omit
lines; no reset/stop acknowledgement depends on log delivery. Actual UART
wiring, runtime timing and physical hardware behavior require board testing.
