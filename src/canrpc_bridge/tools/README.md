# Profile validation and STM32 synchronization

`canrpc_profile.py` validates the version-1 YAML profile without ROS or hardware access. It exports `validate_profile(path)` for launch-time validation before any CAN interface changes. Python 3 and PyYAML are required.

```sh
python3 tools/canrpc_profile.py config/gakusai.yaml
python3 tools/canrpc_profile.py config/reuse_example.yaml
python3 tools/canrpc_profile.py config/gakusai.yaml --mcu-root /path/to/mcu
python3 tools/canrpc_profile.py config/gakusai.yaml --mcu-root /path/to/mcu --check
```

The generic schema covers Node IDs, routes/types/counts/priorities, CAN settings, BOOT/peer timing, and RESET policy. Duplicate YAML keys, unknown fields, invalid scalar types, ambiguous routes, inconsistent subject schemas, illegal IDs, lengths exceeding one CAN FD frame, and invalid timing relationships are rejected. The ROS local node does not offer a reset server; its `reset_sources` must be empty. RESET sources on MCU nodes are explicit allowlists; an empty list disables SYSTEM_RESET and its capability.

Without `--mcu-root`, validation places no STM32 assumptions on a reusable ROS profile. `reuse_example.yaml` demonstrates different node identities and payload lengths. Hardware generation specifically targets the existing CatchRobo three-axis arm, pump, and pressure applications: role names `ros/arm/pump/pressure`, current float32[3] and pressure int32[3] PUBLISH routes, plus pump_set request4/response8, arm_ready request0/response48, and arm_target request12/response20 RPC endpoints remain their physical application contract. Changing that hardware contract requires changing the application, not just the profile.

The synchronization command vendors the exact portable `core/include/canrpc_core.h`, the canonical `adapters/stm32g4/canrpc_stm32.h`, a small application include wrapper, and generated `canrpc_project.h` into each repository's `Core/Inc`. The project header holds all project Node IDs/Subjects, priorities, BOOT/peer and reset timings, reset source policy, BRS flag, and CAN prescalers. Each MCU repository can then build independently without Python, ROS, or a sibling checkout.

The current STM32 adapter uses an 80 MHz FDCAN clock and 20 time quanta (seg1=15, seg2=4). Generation accepts nominal/data rates only when integer prescalers exactly represent them (nominal 1..512, data 1..32). It updates FDCAN1 prescalers and filter counts in `.ioc`, checks the expected 80 MHz metadata and compiled initialization expressions, and leaves motor-native FDCAN3 untouched. BRS comes from the generated header at transmission. The execution timeout must exceed the MCU application's 10 ms main poll. Unsupported timing fails explicitly, without rounding or silent defaults.

`--check` writes nothing and fails for vendor/header/`.ioc` drift or changed FDCAN1 initialization. Run it after editing a profile or the core, then regenerate and rebuild all affected MCU firmware. A successful check does not prove that firmware installed on hardware matches the files or that physical CAN communication works.

Ordinary RPC profiles add `rpc` finite timeout/cache/Tag policy and `rpc_endpoints` with named destination/Service and logical request/response lengths. The current MCU applications accept the profile's ROS node as the ordinary RPC requester. The arm readonly endpoint permits a fresh known server in STARTING/FAULT; mutating pump calls require READY. `application.motor_initialization` is optional application data in generic profiles and required only for this STM32 arm generation, with finite command/homing/feedback waits and existing ADRC freshness limits.

Node `rpc` overrides select a peer-specific ordinary RPC policy. The core/server cache maximum is 32; the arm profile uses a shorter guarded window and larger cache for latest-target calls, while the pump retains its slower policy. Optional `startup_diagnostic` endpoints must be readonly, require zero request bytes, allow non-READY peers, and be unique per destination. `application.arm_control.native_ack_timeout_ms` is checked against the existing 50ms ADRC step bound and the arm RPC deadline including main-poll margin. `target_period_ms` configures the ROS sender; it does not alter native motor timing.
