# Changes, rationale and outstanding issues

Scope: changes since repository parent revision
`def2aebdd1175422397a749621a7a4a6ae90f622`, plus the finalized handoff documentation.
The source revision used for the selected hardware logs is preserved by each
run's snapshot and SHA-256 metadata. Packaging does not alter hardware behavior.

## Implemented during this reference-tool workflow

| File | Change and rationale |
| --- | --- |
| `PrepareTorqueHold.py` (new) | Per-OFF-joint hold-current-pose preparation, speed preload and checked activation; full model/configuration/health/pose guards before the next joint |
| `CenterizedReference.py` | Added explicit `--direct`: prepares torque and removes only the 128-tick start-proximity guard and interactive prompt; support/exclusive-port acknowledgements and feedback guards remain |
| `CenterizedReference.py` | Replaced per-goal unicast ramp with Protocol-1 Sync Write and every-ID register/encoder verification before advancement |
| `CaptureEncoderReference.py` | Validated contiguous register block reads to reduce USB round-trips; model-specific address70 read only on supported MX-64 |
| Shared read helpers | At most three READ attempts for receive timeout/corrupt response, 20 ms between attempts; contextual errors and retry logs; no WRITE retries |
| Torque preparation | A hold-goal write may already activate torque; record possible activation before writing, inspect feedback, only explicitly enable if still OFF |
| Torque preparation | Before preload, old-goal changes on an OFF joint are logged rather than automatically diagnosed as a competing writer; pose/configuration/health remain checked |
| Torque preparation | Newly activated joints get bounded READ-only settling: default 2 s, 50 ms polling, three consecutive stationary readings; only this phase defers stationarity |
| Logging | Added pre-guard feedback, expected/observed register mismatch details, activation distinctions, frame timing and read-only USB latency reporting |
| `setup.py` | Added `PrepareTorqueHold` ROS entry point |
| Six offline test files | Synthetic transport, direct mode, activation, settling, invalid-data, fail/no-more-write and guard regression coverage |
| Documentation | Added unified torque/reference workflow and explicit result interpretation; archived selected runs and reproducible offline reports |

`--hold-settle-timeout` accepts 0.1–10 s. Each newly activated joint must finish
its checks before the next. All thresholds are engineering choices, not universal
safe motion limits. Normal `--execute` retains its conservative near-reference
guard and exact interactive confirmation. Default/`--help` are offline.

## Unchanged configuration and safety-relevant behavior

- Final raw targets: IDs1–20=2048 except ID8=1946 and ID12=2081.
- Expected models: legacy MX-28 (29) for IDs1–6/19–20; legacy MX-64 (310) for IDs7–18.
- Protocol1, default `/dev/ttyUSB0`, 1 Mbps; MX(2.0)/X-series tables are not supported.
- No changes to bridge `ComsROS2U2D2.py`, `Centerized.py`, servo controller, URDF,
  demo zero offsets, firmware, EEPROM, gains or torque limits.
- Capture remains strictly read-only, including on error or Ctrl+C.
- No mass torque-OFF reset, no automatic rollback/torque shutdown, no blind
  write retries and no skipping failed IDs. Existing/last goals may stay active.
- Initially ON joints' hold-preparation goals/speeds remain unchanged.
- Centerized ramp uses 4 ticks/frame, nominal 0.2 s, speed cap20, tracking error
  limit24, final ±8 ticks/Moving=0 for three consecutive reads within 10 s.
- Goal writes and torque enable can cause physical motion. Ctrl+C is not an
  emergency stop; configured register limits do not prove a collision-free path.
- USB latency was observed at 1 ms in selected runs. Tools only report it;
  no udev/SDK timeout edits were made. It may revert after reconnect/reboot.
- Register and joint reads are sequential, not synchronized/atomic observations.

## Outstanding issues — deliberately not hidden by handoff finalization

1. **Already-target shortcut:** `ALREADY_AT_REFERENCE_GOALS_NO_WRITES` returns
   before repeated final settling checks. Completion does not prove ±8-tick
   accuracy; the latest goal run takes this path. A separate tested revision is needed.
2. **Capture completion label:** `completed=True` means the loop ended; historical
   incomplete captures can still have that label. Always check `read_failures`,
   error rows and expected per-ID sample counts. Included captures are complete.
3. **Static offsets/repeatability:** some stationary joints are outside final
   tolerance and positions change across runs with identical goals. No gain,
   torque-limit, zero-offset or tolerance change was made to manufacture PASS.
4. **Intermittent communication:** earlier receive timeouts/corrupt packets remain
   unexplained. Block reads and retry logs improve turnaround/observability,
   not proof that the physical/SDK source is fixed. Spare candidates discussed
   were IDs16,15,18, with ID13 also monitored; no servo fault or swap is proven.
5. No independent geometric pose measurements, dynamics tests or RL certification.

## Verification performed during packaging

Only offline mock tests, offline default preview/`--help`, Git diff checks and
archived-file integrity/consistency checks. Exact output is retained in
`reports/offline_validation.txt`. No live `--inspect`, `--execute`, `--direct`,
capture, Wizard connection or motor action was run by the assistant.
Original local run folders, backup directories and dirty bytecode are preserved;
unrelated files are not included in the commit.
