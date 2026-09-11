# Driving quality review — 2026-09-10

## Status

This is a review snapshot of pre-existing uncommitted changes, not a deployment or a road-validated tune. No running controls, vehicle settings, model weights, or safety limits were changed for this review. The user reports the car is parked; the persistent `IsOnroad` flag still reads `1`. Investigate that mismatch before relying on it to permit settings writes.

The existing repository is `magus-warrior/ToadPilot`. Its default branch is `FrogPilot`, while the installed checkout is on `ToadPilot` at `b1e039fa0ef3c5b08042c15919b0ddfab3aba53c`. Use the ToadPilot branch as the development baseline. Review snapshots should use separate branches without changing the installed branch.

## Evidence and current settings

July passenger notes in `/data/toadpilot/live_drive_trials/drive-notes.md` report that removing experimental centering and straight-line smoothing substantially reduced curve response delay. They also record an EPS fault at a 1525-unit command. Do not revive higher steering authority or introduce a permanent left/right bias to address the current complaints.

The August 31 observations in `TOADPILOT_LIVE_DRIVE_NOTES.md` describe right-biased model paths with weak lane evidence and modest steering demand. This supports investigating path placement and timing; it does not establish the cause of today's symptoms. A midpoint calculated from an unreliable boundary is unsuitable as a steering target.

The September 10 saved configuration has version 8, Observe profile, centering and straight-line smoothing disabled, automatic traffic mode enabled with 30 mph / 1 second thresholds, and improved stop-and-go enabled. Saved Params additionally show:

| Setting | Saved value |
| --- | --- |
| StartAccelRamp | 1.00 m/s² |
| TrafficFollow | 0.50 seconds |
| ForceAutoTuneOff | 0 |
| AlwaysOnLateral / AlwaysOnLateralMain | 1 / 1 |
| MinimumLaneChangeSpeed | 1 mph |

These are saved values, not proof that running processes loaded them. Observe does not disable every longitudinal feature, and earlier notes show that its auto-tune checkbox is not a reliable substitute for checking effective live tuning. No new drive trace was collected in this review.

## Pending changes preserved in this snapshot

- Accept schema version 8 and migrate older valid documents; preserve Traffic Mode thresholds through shared toggles and Fleet Manager saves. Previously the version-8 saved config was rejected by the version-7 decoder, causing fail-safe fallback.
- Apply configured Traffic Mode speed and entry-time thresholds. Permit its free-flow exit timer to accumulate when a nearby lead is moving freely.
- Pass Traffic Mode state into LongControl and cap a configured launch ramp at the existing 0.55 m/s² breakaway floor while Traffic Mode is active. This reduces the possible floor relative to the current 1.00 setting; it is not proof that launches will be comfortable, and the planner/actuator interactions still need measurement.
- Preserve the associated pre-existing tests and August observations.

## Validation

The five focused unittest modules passed: `test_auto_traffic_mode`, `test_start_accel_ramp`, `test_toadpilot`, `test_toadpilot_safety_regressions`, and `test_toadpilot_fleet`: 161 tests, 2.173 seconds. `git diff --check` passed. Tests used reduced scheduling priority and disabled Python bytecode writes. These checks do not validate actual road behavior or certify a deployment.

## Next controlled work

1. Resolve the parked/on-road signal mismatch and verify driver monitoring and bounded telemetry availability. Earlier notes report missing driver monitoring and normal route logging; their current health was not established here.
2. Establish an off-road, backed-up baseline with zero launch ramp, ordinary following distance, a sensible lane-change speed, and cruise disengagement also disengaging lateral control. Treat these as separate recorded settings changes, not a silent batch tune. Keep steering authority at stock limits.
3. Measure stop approach and launch separately: planner acceleration, stop flags, LongControl state/output, Toyota target/final acceleration, measured acceleration/jerk, lead distance, and driver intervention. Compare zero-ramp behavior before adding launch assistance.
4. Compare left/right curves with similar speeds and curvature. Separate raw model path, requested curvature, actual curvature, actuator clipping, and driver input. Keep centering experiments in observation mode until reliable lane evidence and direction-balanced data support them.
5. Use lightweight, bounded diagnostics and offline comparisons on this existing device. Avoid adding inference workloads or bringing back previously leaking recorders without resource measurements.

Current upstream Toyota work also addresses acceleration response and overshoot. Useful references are the [official 0.9.8 explanation](https://blog.comma.ai/098release/) and [current Toyota controller](https://github.com/commaai/opendbc/blob/master/opendbc/car/toyota/carcontroller.py). The local fork already contains adaptations, so compare behavior and dependencies before importing further changes; an upstream feature's existence is not evidence that it is missing here.
