# Darnet encoder-reference handoff — 4 October 2026

This is the finalized **handoff package**, not a certified final calibration.
It covers legacy MX-28/MX-64, IDs 1–20, Protocol 1.0, U2D2 at 1 Mbps.
The newest capture is the primary observed encoder dataset; earlier captures
are comparisons, not alternative zero-offset configurations to apply blindly.

## Outcome

- Primary capture: `encoder_capture_20261004_180445_688090` (18:04:45 WIB).
- 100/100 valid readings, five per joint, no recorded read failures/retries.
- All goal registers match the requested raw targets; all sampled joints have
  torque ON, Moving=0, present-speed raw=0. Within-capture ranges are 0–1 tick.
- IDs 14, 15, 16 and 19 exceed the script's ±8-tick final tolerance.
- Latest Centerized result is `ALREADY_AT_REFERENCE_GOALS_NO_WRITES`;
  **zero final settling samples were taken on that shortcut**. `completed=True`
  is not a near-target pose certificate.
- Between latest goal feedback and primary capture, ID11 changed +17 ticks
  and ID15 −16 ticks with unchanged goals. Cause/operator handling is unknown.
- `--pose-confirmed` was supplied by the operator. No CAD/geometric pose
  measurement or independently verified pose photo is included.

Read [RESULTS.md](RESULTS.md) for the selected results and limitations,
[CHANGES.md](CHANGES.md) for software changes, and
[docs/TORQUE_REFERENCE_WORKFLOW.md](docs/TORQUE_REFERENCE_WORKFLOW.md) for operation.

## Contents

| Path | Purpose |
| --- | --- |
| `reports/primary_encoder_summary.csv` | Actual encoder mean/min/max and goal error for all 20 IDs |
| `reports/capture_comparison.csv` | Same statistics for all three selected captures |
| `reports/summary.json` | Machine-readable provenance, checks, known limitations and goal results |
| `reports/offline_validation.txt` | Offline test/preview verification performed during packaging |
| `evidence/` | Six complete, byte-preserved original run folders |
| `sources/` | Current three reference tools, setup.py and six offline test files |
| `docs/` | Current reference/capture/torque workflow documentation |
| `tools/build_report.py` | Offline regeneration and integrity/consistency checks; no motor access |
| `manifest.json` | SHA-256 and byte size of every other file in this package |

CSV values are raw ticks, not robot joint angles. Model direction, mechanical
zero, offsets and safe ranges are not derived/applied here. Sample/register
reads are sequential, not simultaneous across registers or motors.
Full run folders retain their original source/configuration snapshots; older
`zero_offsets.json` snapshots are provenance only, **not approved calibration**.
Package-local `.gitattributes` prevents Git line-ending normalization and
whitespace checks on archived evidence/source copies, preserving their hashes.
Active source and newly authored reports/documentation remain checked normally.

## Reading or reproducing this handoff

No robot or SDK is needed to inspect the files or regenerate the reports:

```bash
python3 -B handoff/encoder_reference_20261004/tools/build_report.py
```

This only reads archived files and writes derived reports/manifest within this
folder. For offline tests from repository root:

```bash
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_*reference.py'
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_encoder_capture.py'
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_torque_hold.py'
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_direct_center.py'
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_block_sync_transport.py'
python3 -B -m unittest discover -s src/darnet_description/test -p 'test_hold_settling.py'
```

## What remains before adopting a final reference

1. Document robot support/load, approach direction and any handling between runs;
   independently confirm the intended geometric pose against the robot model.
2. Resolve/justify the persistent offsets and measure repeatability across
   separately positioned captures; do not silently widen tolerance to claim PASS.
3. Fix the already-target reporting shortcut and clarify incomplete-capture
   completion status in a separate behavioral revision with offline regression tests.
4. Investigate intermittent timeout/corrupt packets separately from static offset.
   Latest clean capture does not certify communications, dynamics or RL readiness.

No motor was operated by the assistant while preparing this package. No servo
replacement is assumed; the three spare-servo models and any swaps remain unconfirmed.
No URDF, bridge/controller, zero offsets, PID gains, torque limits or firmware
were changed for this handoff. Local backups and bytecode caches are excluded.
