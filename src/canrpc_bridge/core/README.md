# Portable CANRPC core

`include/canrpc_core.h` is a header-only C99 library. Copy this directory into another project, include the header, or link the CMake interface target `canrpc_core`. No ROS, STM32 HAL, allocator, clock, socket, or filesystem is required.

```sh
cmake -S core -B core-build
cmake --build core-build
ctest --test-dir core-build --output-on-failure
```

The core owns the wire ID fields, little-endian codecs, legal CAN FD lengths and zero-padding validation, BOOT payloads and peer generations, RESET server validation/deduplication, and RESET client status/retry/completion tracking. Wire constants remain here; application Node IDs, Subjects, priorities, timeouts, routes, and hardware behavior are supplied by the caller.

- `canrpc_publish_id(subject, source, seq, priority)` and `canrpc_boot_payload(data, boot, uptime, state, capabilities)` require explicit application values.
- `canrpc_peer_announce()` accepts forward uptime only within one generation and remembers eight retired Boot IDs. `canrpc_peer_ready(peer, now_ms, timeout_ms)` never updates sensor freshness.
- `canrpc_reset_server_request()` takes a policy, local generation/state, the selected requester's peer state, and the current clock. It returns 1 only for a new accepted operation. Duplicate requests return the saved status; callers must not repeat hardware actions.
- `canrpc_reset_server_expired()` / `canrpc_reset_server_can_execute()` enforce the configured execution bound. Drivers must confirm actuator preparation and actual ACCEPTED transmission, and recheck the deadline immediately before resetting. The core never resets hardware.
- `canrpc_reset_client_begin()` freezes the operation's target generation, nonce, Tag, and payload. `status()` validates the complete response identity; `observe()` / `complete()` detect a new READY generation; `should_retry()` suppresses retransmission after acceptance or generation change.

Zero-initialize peer and operation structures. All durations used with wrapping 32-bit millisecond clocks must be positive and shorter than `2^31` ms. Access from concurrent contexts requires synchronization by the adapter. The core does not authenticate peers or guarantee physical exactly-once execution. Retired history is bounded; ordinary frames contain no Boot ID.

The application profile and STM32 synchronization tool live in `../config/` and `../tools/canrpc_profile.py`. The STM32 adapter retains entropy generation, interrupt synchronization, FDCAN transmission confirmation, and the arm's STOP preparation; those are intentionally outside the portable core. See the package's documentation for the complete implemented contract.

## Ordinary RPC

`canrpc_rpc_pair_t` reserves Tags across every Service of one directed peer pair and allows one in-flight call. `canrpc_rpc_client_begin()` freezes payload/Service/Tag/generations; `next()` chooses an identical REQUEST before acceptance or QUERY_STATE after acceptance. `receive()` validates final responses and controls. Expiry or peer loss/change terminates UNKNOWN, without rollback or automatic execution in a new generation. The local `control` field distinguishes wire UNKNOWN from peer change/expiry and timeout.

`canrpc_rpc_server_t` is a bounded cache (maximum 32 entries, profile-selected active capacity). `server_request()` checks generations and duplicates before gating new execution on readiness. The same source/Tag with different Service/payload is CONFLICT even across peer generations; guarded entries are never evicted for new calls. `server_query()` replays retained results or returns UNKNOWN. `server_complete()` requires completion within the configured response deadline. The adapter must recheck generation/deadline immediately before invoking the application.

The dispatcher supports synchronous handlers and bounded deferred completion. Deferred handlers carry the allocation token and call `server_complete_checked()`; a reused slot rejects an old token. Running/deferred entries return RUNNING until the configured deadline, then UNKNOWN. The adapter must cancel stale work on generation change or expiry and never convert a late hardware event into RPC success. Long operations beyond the RPC deadline still require a separate application operation ID and progress/result/cancel contract. A slot index alone is not a completion token.

Profiles configure finite request retries, query intervals, response deadlines, result retention, frame-residence assumption, cache capacity, and Tag guard. Guarding is bounded protection; ordinary frames contain no Boot IDs, so delayed frames before observing a generation change cannot be completely excluded.
