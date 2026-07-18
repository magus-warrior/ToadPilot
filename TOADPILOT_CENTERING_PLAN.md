# TOADPILOT Lane Centering Implementation Notes

Implementation checkpoint: 2026-07-18. Initial Adaptive/Full road testing improved curve tracking but exposed lane-crossing risk while slowing or stopping before a curve and inconsistent centering activation. The follow-up revisions fail back to stock lateral behavior during hard transitions and monotonically fade existing trim during a short list of soft-evidence gaps; more controlled-road validation is required before strength is increased.

## Problem being addressed

The reported failure is an unexplained lean within a well-read lane: the inner lane boundaries appear reliable and the model path supports returning to center, but normal lateral tracking permits a persistent offset. This is lane-center tracking, not simply a request for more steering torque and not the same problem as curve-exit steering unwind.

TOADPILOT therefore corrects the curvature setpoint seen by the feedback controller. It does not inject a blind opposite feed-forward torque, raise PID gains globally, increase Toyota's command ceiling, or change Panda safety.

## Implemented signal path

1. Read inner model lane boundaries `laneLines[1]` and `laneLines[2]` only when both have strong probability and low standard deviation.
2. Sample their midpoint over the near field and fit a line; use its intercept plus the branch's existing camera offset as the vehicle-to-lane-center estimate. This avoids trusting a single noisy first model point.
3. Check lane width throughout the sample and at speed-dependent lookahead. Reject non-finite geometry, widths outside 2.6–4.6 m, and absolute lane-center offsets beyond 1.20 m.
4. Compare the model path with the lane midpoint at lookahead. Use this only as corroboration: a path more than 0.20 m from lane center or aimed away from the return direction pauses assistance.
5. Filter the offset and count persistence only on new 20 Hz model frames. Entry requires eight confirming frames and at least 0.10 m offset; the active state releases below 0.05 m.
6. Convert the confirmed offset into a small, speed-bounded curvature delta and add it to the model-requested curvature before stock `clip_curvature` processing.
7. Let the existing lateral controller track the adjusted setpoint. Toyota controller rate/measured-torque checks and Panda remain downstream and unchanged.

## Activation and immediate pause gates

Lane centering requires all of the following:

- exact supported Camry TSS2 torque-control capability;
- TOADPILOT enabled with Adaptive or Full selected;
- separate `lane_centering_assist=true` opt-in;
- active lateral control and speed of at least 5 m/s, with authority blending from zero at 5 m/s to full at 12 m/s;
- no brake press, planned/forced stop, or deceleration below -0.75 m/s²;
- a fixed 15 m/s denominator floor prevents the allowed curvature correction from increasing as speed falls;
- base desired lateral acceleration no greater than 0.65 m/s², limiting this first version to straights and gentle curves;
- the existing requested curve, when meaningful, not opposing the lane-recovery direction;
- two valid inner lane lines with probability at least 0.65 and standard deviation no more than 0.35;
- plausible lane width, persistent offset, and a corroborating model path;
- no lane-change state, no left or right blinker, no driver steering input, no steering-limit event, and steering rate below 75 deg/s; and
- Experimental Mode off unless the existing explicit Experimental assist opt-in is enabled.

Hard-gate failure clears the active correction and publishes a diagnostic reason. With `smooth_lane_transitions=true`, brief low-confidence/uncertainty, invalid-width, and planner-direction failures may only decay an already-active same-direction correction for at most 0.25 cumulative seconds. Alternating valid/soft evidence cannot reset that budget or increase the correction. Invalid evidence cannot create a correction. Curve-direction disagreement, excessive planner bias, the gentle-curve boundary, direction reversal, and every dynamic safety gate above remain immediate. Invalid or stale model data never extends confirmation; confirmation advances only once per new model `frameId`.

## Initial profile bounds

| Profile | Gain | Maximum added lateral acceleration | Rise smoothing | Fall smoothing |
| --- | ---: | ---: | ---: | ---: |
| Observe | 0.00 | 0.00 m/s² | 0.000 | 0.20 |
| Adaptive | 0.14 | 0.10 m/s² | 0.025 | 0.10 |
| Full Existing Authority | 0.22 | 0.16 m/s² | 0.035 | 0.12 |

The acceleration cap is converted to curvature using vehicle speed, so high-speed correction remains small. These are conservative starting values, not validated final tuning.

## Configuration and migration

`ToadPilotConfig` schema version 3 adds the strict default-on boolean `smooth_lane_transitions` alongside `lane_centering_assist`. New or missing configurations default to enabled/Adaptive with Auto-tune, lane centering, smooth transitions, smart warning, and telemetry on; the Experimental curve override remains off. Exact version-1 documents retain their special enabled/Adaptive migration and add both lane toggles on. Exact version-2 documents preserve every saved choice and add only smooth transitions on. Malformed, incomplete, and unsupported configurations still use a distinct disabled/Observe fail-safe with both lane features off. Explicit version-3 settings remain authoritative.

Fleet settings remain writable with no connected car or positive road-state signal, so the device can be preconfigured offline. A positive onroad signal blocks and rechecks configuration writes. Controller output still requires the supported Camry fingerprint, and learned-tune reset still requires confirmed offroad state.

The FrogPilot runtime snapshot publishes `toadpilot_lane_centering_assist` and `toadpilot_smooth_lane_transitions`. Fleet Manager exposes both checkboxes, accurate Experimental interaction wording, and live lane-centering state without changing or activating the stored device configuration during development.

## Telemetry contract

Lane centering is distinct from the curve feed-forward residual. Runtime status uses separate fields so the two cannot be mistaken for each other:

- `lane_centering_active`;
- `lane_centering_gate_reason`;
- `lane_offset_m`;
- `path_lane_bias_m`;
- `lane_width_m` and signal confidence;
- confirmation count;
- bounded soft-gate fade count; and
- `lane_centering_curvature_delta`.

Inactive and invalid states must publish current/reset values rather than leaving a stale active correction visible. Fleet Manager also applies the existing status freshness check.

## Deliberate exclusions

- No Toyota `1500/15/25/350` limit changes.
- No Panda policy or safety-limit changes.
- No global PID-gain increase.
- No use of road edges or a single lane line in the first version.
- No attempt to fight a path that is intentionally offset from lane center.
- No dedicated curve-exit overtracking/unwind residual. That remains a separate future experiment.
- No output on an unsupported fingerprint and no reboot or tune-data reset.

## Required validation before use

- Exact version-1/version-2 migrations, missing/mistyped toggle fail-safe behavior, and strict version-3 Fleet payload tests.
- Synthetic lane geometry: centered zero, left/right symmetry, camera-offset correction, malformed arrays, low confidence, excessive standard deviation, bad width, and path disagreement.
- Controller transitions: eight-frame persistence, entry/release hysteresis, no duplicate-frame confirmation, no direction carryover, bounded/smoothed correction, and immediate zero on every gate.
- Integration: correction is applied before stock curvature clipping; final normalized output and Toyota/Panda limits remain unchanged.
- Replay: straight and gentle curves, lane-width changes, broken/merging markings, road crown, construction, S-curves, lane changes, blinkers, and planner offsets.
- Controlled-road A/B: begin with Observe, establish Adaptive with centering off, then enable centering separately. Compare lane-offset distribution, oscillation/overshoot, interventions, saturation, and left/right balance.

Rollback is the separate checkbox first, then the TOADPILOT master switch. Neither rollback requires deleting learned torque data.

## Initial road finding and correction

The first drive showed that extra curve authority was useful at steady road speed, but the prototype had not modeled longitudinal transitions. Curve entry assistance could remain eligible while the model anticipated a curve during a stop approach, centering curvature authority increased as speed fell, and the learned lateral-acceleration offset was applied as constant feed-forward even below the learner's 15 m/s collection floor. Together these were plausible lane-crossing mechanisms.

The follow-up revision gates curve assistance below 12 m/s and blends centering assistance from zero at 5 m/s to full at 12 m/s. Both remain immediately gated during a brake press, planned/forced stop, or deceleration below -0.75 m/s². It floors the centering-cap speed at 15 m/s and keeps the learned offset diagnostic-only. Factor and friction Auto Tune remain active. These changes improve transition safety and consistency; they deliberately do not increase centering gain.

## Current verification checkpoint

As of 2026-07-18:

- all 81 focused TOADPILOT configuration, Fleet, learner, torque-controller, lane-centering, migration, transition-fade, and save-persistence tests pass;
- modified Python files pass bytecode compilation;
- the Git diff passes whitespace/error checking;
- lane-centering coverage includes schema migration, strict toggle defaults, signal confidence/geometry, camera-frame handling, offset sanity, left/right symmetry, immediate direction-reversal clearing, new-frame persistence, planner agreement, all dynamic gates, Experimental opt-in, and profile caps; and
- no direct live-Params write or learner reset was performed, and there was no reboot or commit; the saved legacy v1 document adopts the new defaults through the tested in-memory migration when the updated process loads it.

The repository image does not include the ruff executable, so that optional lint command could not run. Unit, compilation, and diff checks remain green.
