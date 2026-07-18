# ToadPilot Driving Improvement TODO

Updated 2026-07-18 from passenger-marked live monitoring. Keep these items evidence-driven and preserve Toyota/Panda torque safety limits.

## P0 — Curve and transition quality

- [x] Add a default-on, separately switchable bounded fade for brief soft-evidence gate changes. It can only monotonically remove an existing lane-centering trim for at most 0.25 seconds; every hard safety gate and raw direction reversal remains immediate. Synthetic regressions cover fade bounds, toggle-off behavior, invalid-evidence non-activation, and hard-gate behavior. Controlled-road validation remains required.
- [ ] Add a regression for the passenger-marked long curve near 50–58 mph. Most of the curve had high lane confidence and very small tracking error, but centering repeatedly alternated among `active`, `centered`, `planner_direction`, and `curve_direction`. Near the exit, lane confidence fell, command/output gap reached ~0.77, curvature error reached ~0.0024 1/m, and the driver intervened. Add hysteresis/continuity for soft evidence gates while preserving immediate hard safety gates.
- [ ] Investigate the passenger-marked persistent left lean at highway speed. Live status showed high lane confidence (~0.993), plausible 3.04 m width, ~0.103 m measured lane offset, small model path bias (~-0.027 m), no controller saturation, and zero centering correction because `curve_direction` rejected it. Determine whether this is model-path placement, road crown/camera calibration, or an over-conservative direction gate before changing output.
- [ ] Add direction-balanced observation and validation. The current high-authority observer had 50 accepted left samples and zero right samples; never infer a permanent asymmetric tune until comparable right-turn evidence exists.
- [ ] Add confidence-aware model-path continuity when lane confidence collapses or detected width becomes implausible. Hold only a short, bounded last-reliable direction estimate; never directly center from invalid geometry.
- [ ] Model the Camry steering actuator delay and rate response. Begin valid curve demand earlier and constrain requested curvature transitions to what the actuator can follow.
- [ ] Treat curve entry, steady-state tracking, and curve exit separately. Avoid both late entry and delayed unwind/overshoot.
- [ ] Add regression scenarios for the observed 29–34 mph turn: lane width jumped near 5.3 m, curvature tracking error reached about 0.0031 1/m, steering command/output gap reached about 0.64, and driver override reached about 41%.
- [ ] Add regression scenarios for the observed low-speed turn/start: stopped model curvature was unstable, assistance began late after launch, and lane confidence fell during acceleration.

## P0 — Lane-change transitions

- [ ] Review the passenger-marked lane-change/turn event around 60–66 mph. ToadPilot correctly gated on lane change, but command/output gap reached about 0.60 while lane confidence collapsed; ensure entry and completion transitions are smooth.
- [ ] Preserve zero ToadPilot centering during an active lane change and driver steering, then require fresh stable geometry before reactivation.
- [ ] Prevent stale pre-lane-change centering state or direction from returning after lane-change completion.

## P1 — Overall driving quality

- [ ] Replace abrupt confidence/gate handoffs with bounded fades where safe; braking, planned stops, driver override, invalid input, and direction reversal remain immediate hard gates.
- [ ] Distinguish actuator rate limiting, genuine saturation, undertracking, and unwind overshoot in diagnostics and alerts.
- [ ] Evaluate map/navigation only for anticipation and speed planning, never precise lane steering.
- [ ] Tune longitudinal-to-lateral coordination so deceleration before curves does not create path or steering discontinuities.
- [ ] Score revisions on centering consistency, curve entry/hold/exit, launch timing, intervention rate, steering smoothness, and alert quality—not raw steering authority.

## Upstream modernization and stop/go

- [x] Reset inactive desired curvature toward measured wheel curvature and add the roll-aware 3 m/s^2 curvature cap with distinct saturation semantics.
- [x] Make Toyota Hybrid standstill release explicit while preserving non-Hybrid lurch protection, legacy PCM behavior, and the FrogPilot SNG override.
- [x] Remove the false LongControl stop hold caused by treating numeric `forcingStopLength < 1` as a boolean. The release path now uses only planner `shouldStop` and FrogPilot `forcingStop`, with a call-site regression.
- [x] Add a Camry-only start acceleration ramp with a strict zero default, 0.00–1.00 m/s² / 0.05 m/s² Fleet Manager slider, positive-plan requirement, brake/stop/standstill gates, low-speed fade, timeout, and fail-neutral tests.
- [x] Adapt bounded delayed feedback/current feed-forward, jerk-aware friction, and KP-independent low-speed correction to the Camry Adaptive/Full linear path without changing controller/Panda authority.
- [x] Fix the Fleet Manager form rollback/save verification path. Keep the experimental response request unavailable/effective=false until matched controller and Panda safety sources/tests exist.
- [x] Align Toyota radar-distance DBC source with the already-current prebuilt runtime; add temporary ACC fault, thermal fan, throttle-boundary, and planner-validity ports.
- [x] During a supervised passenger session, observe the launch chain live with bounded cereal/Params only. The marked delay showed real `forcingStop=false` while numeric model trajectory length hovered below 1 m; the old call site could therefore keep LongControl in `stopping`. The Camry also has `startingState=false` and `startAccel=0`, so there was no separate launch kick after release.
- [ ] Validate one controlled stop/go drive after restart with `StartAccelRamp=0.00` so the stop-release correction is isolated. Record bounded `shouldStop`, `forcingStop`, LongControl state, requested/applied acceleration, standstill release, and measured response.
- [ ] Only if the zero-ramp drive still needs assistance, test one 0.05 m/s² ramp increment at a time and verify immediate cancellation for brake, planned/forced stop, cruise standstill, and inactive longitudinal control.
- [ ] Restore a coherent cereal/msgq source pipeline before retrying loggerd or driver monitoring; measure queue drops and RSS with bounded IPC buffers.
- [ ] Restore Panda safety source/build/tests before any effective max-torque or rate-limit experiment. A saved UI research request is not authority.

## Safety and validation

- [ ] Make one bounded change per supervised drive and retain a regression for every marked failure.
- [ ] Never increase Toyota/Panda torque limits to hide timing, perception, or controller problems.
- [ ] Keep leaking `loggerd` disabled; use lightweight in-memory monitoring until a bounded telemetry recorder is designed.
- [ ] Do not port Toyota radar `STATUS_MSG`, wheel-speed fault bits, or excessive-actuation lockouts piecemeal; each currently requires missing matched runtime/schema/Panda components.
