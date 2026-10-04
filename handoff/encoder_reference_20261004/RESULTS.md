# Selected results and interpretation

## Primary dataset

Use `evidence/encoder_capture_20261004_180445_688090/` as the **latest observed
encoder candidate**. It is complete and internally stable, but not a certified
geometric zero. The full numerical result for every ID is in
`reports/primary_encoder_summary.csv`; no mean is substituted into a raw file.

| Primary capture joint | Target | Observed encoder | Error from target |
| --- | ---: | ---: | ---: |
| ID14 — Lutut Kiri | 2048 | 2058 | +10 |
| ID15 — Kaki Kanan Atas | 2048 | 2038 | −10 |
| ID16 — Kaki Kiri Atas | 2048 | 2037–2038 (mean 2037.6) | −11 to −10 |
| ID19 — Leher Putar | 2048 | 2057 | +9 |

The other IDs are within ±8 ticks in this capture. All 100 readings are valid;
no read retries/failures are recorded. All sampled torque states are ON,
Moving flags are 0 and present-speed values are 0. Encoder ranges over the five
samples are at most one tick. Temperatures are 37–48 C; sampled voltage values
are 106–121 raw (10.6–12.1 V). Sparse voltage reads cannot exclude brief supply dips.

## Goal/capture provenance

Run folder timestamps for Centerized are UTC; capture folder timestamps are
local WIB (UTC+7). Metadata timestamps are UTC and used for comparison.

| Evidence | UTC time | Interpretation |
| --- | --- | --- |
| Goal `20261004T110103_691892Z` | 11:01:03–11:01:23 | Actual ramp followed by 33 settling reads; timeout, ID15 −10 / ID19 +9 |
| Capture `20261004_180130_557636` | 11:01:30–11:01:33 | 100 valid reads, no retries; matches preceding final feedback within one tick |
| Capture `20261004_180349_734853` | 11:03:49–11:03:52 | Complete comparison capture; predates latest goal run, not its post-motion result |
| Goal `20261004T110409_969079Z` | 11:04:09–11:04:10 | Already-target shortcut; no ramp, no final settling reads |
| Capture `20261004_180445_688090` | 11:04:45–11:04:48 | Primary capture, about 35 s after latest goal exits; current pose differs |
| Goal `20261004T110043_458436Z` | 11:00:43 | Context: ID01 model read failed after three corrupt-packet attempts |

All goal targets are 2048 except ID8=1946 and ID12=2081. The goal/capture
source SHA-256 values agree for the archived shared scripts/configuration.
Matching software and temporal order do not prove an unchanged physical pose.

## Repeatability remains unresolved

Relative to latest goal feedback, the primary capture's mean changed +17 ticks
on ID11 and −16 on ID15, while goal registers remained unchanged. Between the
two newest captures, ID11 changed +16.2 ticks and ID15 −16. This is much larger
than their 0–1-tick within-capture ranges. Robot handling, support/load, direction
of approach, mechanical state or another writer are not documented well enough
to identify a cause. Do not label it electrical noise or a failed servo solely
from these differences. The latest capture is not better certified than the
earlier well-paired 18:01 capture just because it is newer.

## Decision

- **Ready to share:** source revision, raw observations, summaries and diagnostic context.
- **Not certified:** final geometric zero, inter-run repeatability, reliable
  communication under all conditions, safe mechanical range, dynamics or RL readiness.
- `pose_confirmed_by_operator=True` records a human assertion only.
- `ALREADY_AT_REFERENCE_GOALS_NO_WRITES` and capture `completed=True` are not
  sufficient evidence of the three-sample ±8-tick settling condition.
- Keep all original CSV/JSON/event logs unchanged. Do not apply these ticks
  directly to `zero_offsets.json`, a controller or URDF.
