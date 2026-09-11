# TOADPILOT

**2026-09-10 longitudinal update:** schema 9 replaces the old stop-and-go
switch and start-ramp slider with default-on Responsive takeoff and Smooth
stop finish. See [current behavior and validation](TOADPILOT_RESPONSIVE_STOP_GO.md).
The independently configured start boost is restored through its existing API
and saved setting. The start-ramp and configuration sections below describe
earlier UI revisions.

Project notes and safety contract for the TOADPILOT lateral-control fork.

**Status:** local development build, updated 2026-07-19. New configurations default to enabled with the `Adaptive` profile, lane centering, smooth lane transitions, and guarded Camry stop-and-go on. Effective controller output remains limited to the exact `TOYOTA_CAMRY_TSS2` torque-control fingerprint. It is not a general Toyota tune.

## Project identity

TOADPILOT is the successor to the one-off `ToyotaPerformanceMode` experiment. Its goal is to use the electronic steering authority that the existing Toyota controller already permits more effectively, while making every behavior explicit, measurable, and reversible.

It does **not** increase Toyota CAN torque limits, bypass Panda safety, infer that driver-applied mechanical torque is available to software, or treat an on-screen saturation warning as permission to command more torque. “Full” means **Full Existing Authority**, not a higher authority envelope.

The old `ToyotaPerformanceMode` parameter may remain on the device as a legacy artifact, but it is no longer the controller switch. TOADPILOT uses one versioned `ToadPilotConfig` document so that related choices are applied together.

## Configuration

The version 4 configuration contains:

| Setting | Default | Meaning |
| --- | --- | --- |
| `enabled` | `true` | Master switch. Disabled returns the controller to the branch's normal behavior. |
| `profile` | `adaptive` | `observe`, `adaptive`, or `full`. |
| `auto_tune` | `true` | In Adaptive/Full, select filtered live instead of offline torque parameters. |
| `lane_centering_assist` | `true` | Confidence-gated lane-center tracking in Adaptive/Full. |
| `smooth_lane_transitions` | `true` | Monotonically fade an existing centering trim during brief soft-evidence gaps. |
| `improved_stop_and_go` | `true` | Permit a bounded, planner-confirmed Camry launch handoff through stale Toyota cruise-standstill status. |
| `curve_assist_in_experimental` | `false` | Permit curve feed-forward assistance while Experimental Mode is active; lane centering is independent. |
| `smart_saturation_warning` | `true` | Debounce transient steering-limit warnings without suppressing a sustained limit. |
| `telemetry` | `true` | Publish local controller diagnostics. |

The parser is strict: missing or unknown keys, wrong types, unsupported versions, and unknown profiles are rejected. An exact valid version-1 document is upgraded to the requested enabled/Adaptive defaults with lane centering, smooth transitions, and guarded stop-and-go on; its Auto-tune, Experimental, warning, and telemetry choices are preserved. An exact version-2 document preserves every saved choice and adds `smooth_lane_transitions=true` and `improved_stop_and_go=true`. An exact version-3 document preserves every saved choice and adds only `improved_stop_and_go=true`. A missing configuration uses the same defaults. A malformed document, corrupt JSON, or unsupported future version uses a separate fail-safe configuration: disabled and Observe, with both lane features and improved stop-and-go off. Once saved as version 4, every explicit choice is preserved.

Capability and enablement are separate. Even if the stored master switch is on, the effective switch remains off unless the detected vehicle is Toyota `TOYOTA_CAMRY_TSS2` with torque lateral control.

Fleet Manager allows settings to be prepared and saved with no vehicle connected or road-state signal available. A positive onroad signal still locks configuration writes, and learned-tune reset still requires a confirmed offroad state.

Fleet Manager also displays an **Experimental Camry 1700 authority** research request. It defaults off, can only be changed with confirmed offroad state, and is latched for reboot. It becomes effective only when the exact Camry torque fingerprint, EPS firmware, controller flag, boot latch, and paired debug Panda signature all match. Normal commands retain the stock 1500-unit mapping. The extra envelope requires a 0.4-second full-saturation handshake, a 5–15 m/s speed window, same-direction calibrated-yaw undertracking, meaningful curve demand, no driver/brake/stop/blinker state, and steering rate below 75 deg/s.

## Profiles

TOADPILOT adds a smoothed, signed feed-forward residual only while entering or undertracking a real curve. Curve strength begins above 0.25 m/s² desired lateral acceleration and reaches full weighting at 2.0 m/s². Assistance drops immediately while unwinding, during driver steering input, at a steering-limit event, or at steering rates of 75 deg/s and above.

| Profile | Entry target | Tracking target | Maximum combined target | Purpose |
| --- | ---: | ---: | ---: | --- |
| Observe | 0.000 | 0.000 | 0.000 | Stock output with diagnostics only. |
| Adaptive | 0.035 | 0.080 | 0.115 | Modest curve-entry and tracking support. |
| Full Existing Authority | 0.060 | 0.150 | 0.210 | More assertive demand shaping inside the same output envelope. |

The values are normalized controller feed-forward terms, not raw Toyota torque units. They are targets before smoothing and before the PID's existing output clamp. Actual CAN output can be lower because of smoothing, PID response, rate limiting, driver-torque checks, and downstream safety enforcement.

The residual never carries old-direction assistance across a direction reversal. Disablement and Observe produce exactly zero residual.

## Lane centering assist

The lane-centering implementation defaults on as a separately controllable Adaptive/Full feature. Initial road testing found unsafe behavior while slowing or stopping before a curve; the follow-up revision therefore removes unvalidated learned-offset feed-forward and gates both assists during braking and planned/forced stops. A later passenger-observed drive found a hard, late steering handoff after launches, so centering now blends conservatively from 5 to 12 m/s instead of switching on at 12 m/s. Another observed highway curve repeatedly alternated among planner, curve-direction, and lane-confidence gates. The default-on smooth-transition option now fades only an already-applied centering trim during confidence/width or safe planner-direction gaps. Replay showed that carrying trim through `curve_direction` could reverse a small model request, so that disagreement deliberately remains immediate-off.

The assist estimates lateral offset from the model's two inner lane boundaries. A near-field linear fit makes the estimate less sensitive to a noisy first point, and the existing camera offset corrects its intercept. The model path is used only as corroboration: TOADPILOT acts when the path is already aimed back toward the lane center and pauses when the path is materially biased away, since that bias may be intentional obstacle, merge, or road-edge avoidance.

After a persistent offset is confirmed, TOADPILOT adds a small target-curvature correction before the existing `clip_curvature` rate limiter. The normal lateral controller then tracks that adjusted setpoint. This placement gives the feedback controller one consistent target instead of making a feed-forward term fight an unchanged planner setpoint.

The initial activation contract is deliberately narrow:

- TOADPILOT must be enabled in Adaptive or Full and `lane_centering_assist` must be explicitly on;
- both inner lane lines must have probability at least 0.65, standard deviation no more than 0.35, finite geometry, and a plausible 2.6–4.6 m lane width;
- speed must be at least 5 m/s, assistance blends from zero at 5 m/s to its existing authority at 12 m/s, base desired lateral acceleration must be no more than 0.65 m/s², and the offset must persist for eight new 20 Hz model frames;
- a brake press, a planned/forced stop, or deceleration below -0.75 m/s² immediately pauses centering and curve assistance;
- entry begins beyond 0.10 m and releases below 0.05 m to avoid chatter; lane-center estimates beyond 1.20 m are rejected as implausible;
- the model path must agree with the return direction and remain within 0.20 m of the lane center at lookahead;
- lane changes, either blinker, driver steering, genuine high-authority steering saturation, steering rate of 75 deg/s or more, inactive lateral control, and invalid inputs immediately pause the assist. Ordinary Toyota actuator rate limiting does not masquerade as saturation;

With smooth transitions enabled, low lane confidence/uncertainty, implausible width, and planner-direction disagreement are soft gates. They can only monotonically remove a same-direction trim that was already active; they cannot create or increase one from invalid evidence. Once started, valid/invalid alternation cannot reset the cumulative 25-control-update (0.25-second) budget or increase the correction; fresh confirmation is required after it reaches zero. Curve-direction disagreement, excessive planner bias, a curve beyond the centering boundary, raw lane-offset direction reversal, stale/model-invalid data, lane changes, blinkers, driver input, braking/stops, excessive deceleration, high steering rate, genuine saturation, inactive control, disable/Observe, and the lane-centering toggle remain immediate zero-output gates. Turning smooth transitions off restores immediate removal for soft gates too.

| Profile | Centering gain | Maximum added lateral acceleration |
| --- | ---: | ---: |
| Observe | 0.00 | 0.00 m/s² |
| Adaptive | 0.14 | 0.10 m/s² |
| Full Existing Authority | 0.22 | 0.16 m/s² |

These are small path-tracking limits, not additional electronic steering authority. Below 15 m/s the curvature cap no longer grows as speed falls; the correction is otherwise speed-bounded, independently smoothed, processed by the stock curvature-rate limiter, and ultimately remains inside the unchanged normalized controller, Toyota, and Panda limits.

This feature addresses unexplained lane lean when lane geometry is strong and the model path already supports returning to center. A distinct curve-exit unwind/overtracking residual is still not implemented; planned-curvature unwind continues to remove the existing curve residual and use the normal feedback controller.

## Safety boundary

The ordinary electronic steering path is intentionally unchanged:

- normalized lateral-controller output remains capped at `-1.0 ... +1.0`;
- Toyota's normalized command mapping remains 1500 command units;
- torque-mode rise remains 15 units per 100 Hz command tick;
- torque-mode reduction remains 25 units per 100 Hz command tick; and
- maximum commanded-versus-measured EPS torque error remains 350 units.

On a specifically authorized research boot, only the absolute ceiling can extend to 1700 after the handshake above. The 15-unit rise, 25-unit reduction, 350-unit measured-torque error, and 450-unit real-time delta guards remain identical in the Toyota controller and Panda. Stable steering rates through 55 deg/s retain full extra headroom. From 55–75 deg/s only the extra 200 units taper linearly; an increase of at least 20 deg/s within 100 ms while already above 45 deg/s pauses only that extra headroom without disarming it. Stock 1500 authority remains available. A Toyota temporary EPS fault removes the extension immediately and holds only the extra headroom for one second after the fault clears before the normal 0.4-second handshake can run again.

The controller reports `stock`, `arming`, `armed`, `extended`, `returning_stock`, `eps_temporary_fault`, and `eps_cooldown` with exact gate reasons, target/final output commands, request-bit state, arm progress, effective headroom scale, rate rise, cooldown, and raw Toyota EPS state. The live status also records Panda's actual safety parameter and route-relative `safetyTxBlocked` increments. That counter covers every transmit address, so a correlated increment is useful evidence of a Panda safety block but is not, by itself, proof that the steering frame was rejected. EPS acceptance still comes from Toyota's raw fault state and observed response rather than the controller command alone.

This distinction matters: a person turning the wheel can add mechanical torque beyond what the EPS motor supplies. Doing that safely in one drive does not demonstrate that an electronic command above 1500 is accepted by the EPS or safe for Panda to transmit.

Likewise, “Turn Exceeds Steering Limit” describes a controller saturation/tracking condition. It is useful evidence that the requested path was not achieved, but it is not a safety certification for raising command limits. Smart warning mode uses the car's existing `steerLimitTimer` to ignore brief transients and still raises the warning for sustained saturation.

Model-curvature limiting is reported separately from downstream Toyota/Panda steering clipping. A sustained roll-aware lateral-acceleration cap can therefore raise a tracking warning from the raw pre-cap request without masquerading as extra electronic authority or feeding TOADPILOT's assist gate.

## Upstream modernization

The 2026-07-18 modernization pass adapted the current behavior that maps cleanly onto this controller instead of importing incompatible units or source-only assumptions:

- inactive control resets toward measured curvature before engagement;
- requested curvature is bounded by a roll-aware +/-3 m/s^2 lateral-acceleration envelope;
- Camry Hybrid standstill release is explicit, while non-Hybrid lurch protection and legacy paths remain intact;
- Adaptive/Full linear control uses delay-aligned feedback, current-demand feed-forward, bounded jerk-aware friction, and KP-independent low-speed correction;
- invalid localization samples cannot enter torque learning;
- the model throttle-probability threshold is 0.4 above the unchanged 2.5 m/s unconditional launch region;
- temporary Toyota ACC faults, thermal-block cooling, planner input validity, and Toyota radar-distance source/runtime agreement are updated.

Measured lateral-acceleration-rate feedback from openpilot #36291 was not stacked onto this work because current #36619 removed/superseded it. The exact #37704 embedded sensor-timestamp fix remains deferred because the old `liveLocationKalman` schema has no equivalent timestamp. Toyota standstill steering also remains disabled: upstream never enabled Toyota for it and explicitly noted EPS-fault risk.

The complete status, source links, and architectural blockers are in [TOADPILOT_UPSTREAM_PORTS.md](TOADPILOT_UPSTREAM_PORTS.md).

## Longitudinal stop release and start ramp

The passenger-marked 2026-07-18 launch delay was not an ACC failure. Live status showed FrogPilot's real `forcingStop` flag clear while `forcingStopLength` repeatedly sat near 0.7–1.0 m. Despite its name, that numeric field carries tracked model-trajectory length. `controlsd` was treating every value below 1 m as another stop boolean, so LongControl could remain in `stopping` after the explicit forced-stop state had cleared. The release path now combines only `longitudinalPlan.shouldStop` and `frogpilotPlan.forcingStop`.

The default-on `improved_stop_and_go` correction is independent of the optional start ramp. It is effective only when the TOADPILOT master switch, exact Camry TSS2 torque fingerprint, openpilot longitudinal control, and auto-resume capability all match. After a real LongControl stopping state, finite positive planner acceleration starts a handoff only while controls are active, no stop is requested, and the brake is not pressed. During that handoff, stale Toyota cruise-standstill status cannot send LongControl back through the launch-cancellation path. A contradictory planner-only stop with a still-positive finite acceleration request may preserve the handoff lifecycle for at most 0.10 seconds, but LongControl remains in `stopping` and commands no positive acceleration for every stop frame. A forced stop, brake, disengagement, non-positive/invalid acceleration, invalid speed, disablement, another vehicle, or a longer planner stop discards the handoff; it also ends by 2.0 m/s or two seconds. Transition events and exact end reasons are logged as `toadpilot_stop_and_go` diagnostics.

On the same exact, enabled Camry release, Toyota's inner acceleration controller clears the braking-approach integral and unwinds only the remaining negative acceleration request at 8 m/s3 until it reaches zero. Positive acceleration still uses the existing 4 m/s3 slew. This targets the observed brake-release dead time without increasing the post-zero positive-command jerk. Live stop/go status exposes the planner request, LongControl output, Toyota controller target/final output, and whether this negative-command release is active.

The `StartAccelRamp` setting defaults to `0.00 m/s²`, which preserves the ordinary planner/PID acceleration command exactly while the stop-release correction still works. Its Fleet Manager slider spans 0.00–1.00 m/s² in 0.05 m/s² steps and is effective only on the exact Camry TSS2 fingerprint with openpilot longitudinal control.

When configured above zero, the ramp can arm only on a real `stopping`-to-`pid`/`starting` transition. It still requires active longitudinal control, a positive planner acceleration request, no brake input, and no planned or forced stop. Toyota cruise standstill also blocks or clears it except while the guarded stop-and-go handoff is actively filtering the known stale status. The bounded breakaway target is applied immediately after a confirmed release, then later increases remain limited to 2.0 m/s³; Toyota's downstream positive command limit remains 4.0 m/s³. The floor fades to zero by 2.0 m/s (about 4.5 mph) and expires after two seconds. The 0.10-second planner consistency hold preserves ramp state but cannot command positive acceleration. Any confirmed stop, brake, unfiltered standstill, inactive-control, non-positive post-launch planner request, invalid input, or speed cutoff clears it without a delayed re-trigger.

This stripped checkout has no native FrogPilot Qt settings source, so the functional slider lives in Fleet Manager's Tools page and uses the normal persistent Params/toggle-refresh path. Writes are blocked while the device reports onroad. A full source fork should register the key natively and add the same control to the native longitudinal settings UI.

## Auto-tune semantics

The auto-tune switch selects which torque model TOADPILOT uses; it does not raise the steering envelope.

- **On:** use the filtered `LiveTorqueParameters` factor and friction. The learned lateral-acceleration offset remains visible in diagnostics but is no longer applied to feed-forward: its learner observes only above 15 m/s, and initial road testing showed that extrapolating the intercept as constant low-speed steering bias can cause drift while slowing.
- **Off:** in Adaptive and Full, restore the offline `CarParams` factor, offset, and friction. The learner may continue observing in the background, but its result is not applied by the controller.
- **TOADPILOT disabled:** retain the branch's normal live-tune behavior; this toggle does not silently redefine stock behavior outside TOADPILOT.

Observe deliberately retains the branch's normal live-tune behavior regardless of this switch, so its steering output remains a trustworthy A/B baseline. Smart saturation warning is independent of the residual and can still change the timing of a steering-limit alert. Disable `smart_saturation_warning` as well when an A/B comparison must preserve both controller output and alert timing.

In Adaptive and Full's linear Auto Tune path, the live learner is the single source of factor, offset, and friction. Legacy manual factor/friction overrides are suppressed so a saved manual value cannot silently produce a mixed tune. Observe and TOADPILOT-disabled operation continue to follow the branch baseline, including its normal manual/live-tune behavior.

Torque-cache format v2 persists pure learner values rather than manual overrides. Its factor/friction bounds use one conservative contract for every torque car regardless of full or decimated estimator cadence; restore accepts only float32 wire-rounding tolerance and clips values back to the exact mathematical bounds.

The learned lateral-acceleration offset is bounded independently from the steering-output clamp. It must be finite and within `-0.5 ... +0.5` m/s² to be accepted; an invalid fit is rejected and the learner dataset is reset to its offline starting state. The value remains cached and reported for diagnosis but is not currently applied to feed-forward.

For the linear torque formula, a **smaller** lateral-acceleration factor generally produces a stronger feed-forward command for the same requested acceleration. A larger number is not “more torque.” UI labels and tune reviews must make that direction clear.

The existing learner is intentionally conservative: its primary fit uses low-authority steering data, modest lateral acceleration, sufficient speed, and no driver steering input. High-authority data should be observed and reported separately. It must not be fed into the linear learner until enough clean, balanced data supports a reviewed model of nonlinearity and left/right behavior.

Neural feed-forward is a separate tuning path. Full NNFF replaces the linear feed-forward calculation, so TOADPILOT suspends linear Auto Tune while full NNFF is active. NNFF Lite augments the linear path and is compatible with linear Auto Tune. Even when the combination is supported, validate one change at a time so results remain interpretable.

## Experimental Mode interaction

TOADPILOT does not write `ExperimentalMode`, `ConditionalExperimental`, or curve-control parameters. The live Experimental Mode state is passed directly to the lateral controller.

Lane centering is independent of Experimental Mode because it adjusts the shared model setpoint using the same confidence and safety gates. Curve feed-forward assistance remains gated off by default in Experimental Mode; the explicit `curve_assist_in_experimental` switch opts only that curve residual in. Observe remains output-neutral in either mode.

## Telemetry

Configuration is persistent in `ToadPilotConfig`. Runtime status is local and ephemeral:

- `ToadPilotStatus` contains the effective configuration, curve residual, desired and actual lateral acceleration, tracking error, feed-forward before and after assistance, requested normalized steer, saturation/limit flags, active torque parameters, Experimental Mode state, and a monotonic timestamp. Lane centering is reported separately with its active state, gate reason, measured lane offset, path-to-lane bias, confirmation count/confidence, soft-gate fade count, lane width, and applied curvature delta.
- `ToadPilotSteeringStatus` contains the live 100 Hz-observed controller state/reason, target and output command, arm progress, extra-headroom scale, steering-rate rise, cooldown, and raw Toyota EPS state. `ToadPilotTorqueStatus` remains the lower-rate route evidence and explicitly labels its extended-output count as 20 Hz diagnostic samples rather than CAN frames.
- `ToadPilotStopGoStatus` contains the live handoff/ramp state, planner/output/measured acceleration, exact transition reason, and reason counters. Manager preserves one compact `ToadPilotStopGoLastRoute` summary at the offroad transition.
- The linear Adaptive/Full status also reports whether the upstream-response adaptation is active, its delay-aligned lateral-acceleration setpoint, and its filtered desired lateral jerk. These are fixed-size live diagnostics, not a route recorder.
- `ToadPilotTuneStatus` is the separate slot for versioned, observe-only high-authority summaries. It reports accepted/candidate/rejected counts, left/right response estimates, and separate 0.50–0.65, 0.65–0.80, and 0.80–1.00 normalized-steer bins.

The observer accepts only samples above 15 m/s after a two-second active/no-driver-input buffer, then requires matching steer/lateral-acceleration direction and 0.25–4.0 m/s² absolute lateral acceleration. These estimates are never fed into controller tuning or the stock linear learner in this phase.

This observer is preliminary diagnostics, not a production-quality dataset. It does not yet label or reject lane-change state, poor model/lane context, or steering-limit context. Interpret its bins only as leads for later review; use replay or separately labeled logs before drawing tuning conclusions.

Controller status has a 1 Hz heartbeat and transition publishing capped at 10 Hz in shared-memory Params. The route identifier is cached before the control loop so transition reporting does not add persistent Params reads to the 100 Hz path. Consumers must inspect the timestamp because disabling telemetry can leave an older payload visible. This JSON status path is appropriate for this stripped prototype; a production source fork should publish typed messaging outside the real-time controller instead.

Fleet Manager may read configuration and status while the car is onroad, but configuration writes and tune reset are accepted only while offroad. Reset also requires an explicit confirmation payload. A confirmed reset removes the saved torque cache immediately, posts a durable reset request for the live learner to consume, restores its in-memory factor/friction/offset state to the offline baseline, and clears stale TOADPILOT status. This remains a destructive tune-data operation; normal rollback only requires disabling TOADPILOT.

Useful comparison metrics are median and high-percentile absolute tracking error, time at normalized saturation, steering-limit warnings, driver interventions, EPS command rejections/faults, and left/right balance. A higher requested torque number by itself is not a success metric.

## Current device observations

These numbers are a snapshot, not a recommended tune:

- detected car: `TOYOTA_CAMRY_TSS2`, Toyota torque lateral control, openpilot longitudinal enabled;
- cached learner sample count: 12,000 points;
- raw combined lateral-acceleration factor: approximately 1.958943;
- filtered live factor: 1.9560784;
- offline stock factor: 2.354832;
- filtered live offset: 0.1396728;
- filtered live friction: 0.1284044 versus offline stock friction 0.118436; and
- current decay/reset metadata: decay 250, reset 1.

In the linear model, the live factor represents roughly 20.4% stronger feed-forward than the offline factor before friction, feedback, and output limiting. A previously saved manual factor of 2.37 is inactive and would actually be slightly weaker than the offline factor, not stronger.

An exploratory direction-split fit of cached points produced materially different slopes (roughly 1.64 in one direction and 2.08 in the other, versus about 1.96 combined). That is a reason to collect and inspect cleaner balanced data—not a reason to immediately install asymmetric gains. Road crown, alignment, wind, sample imbalance, sensor bias, and model nonlinearity are all plausible contributors.

At the time of this snapshot, Experimental Mode, Conditional Experimental, and curve handling were enabled, while NNFF and NNFF Lite were disabled. The legacy `ToyotaPerformanceMode` value was still present; new controller code ignores it.

## Validation checklist

Do not jump from unit tests directly to Full on public roads. Validate in stages, with a ready driver and an environment where an immediate disengagement is safe.

### 1. Static and bench checks

- Run the focused TOADPILOT unit tests and Python compilation checks.
- Confirm invalid configuration falls back to disabled/Observe.
- Confirm exact version-1 configuration adopts enabled/Adaptive with both lane features and improved stop-and-go on while preserving its other toggles, exact version 2 preserves every saved choice while adding smooth transitions and improved stop-and-go on, and exact version 3 adds only improved stop-and-go on.
- Confirm unsupported fingerprints cannot become effectively enabled.
- Confirm the Toyota and Panda safety constants are unchanged in the final diff.
- Exercise enable, disable, profile changes, auto-tune selection, and process restart.
- Confirm Fleet Manager rejects configuration changes and tune reset while onroad, and rejects reset without explicit confirmation.
- Confirm a confirmed offroad reset survives a learner-process boundary and restores offline factor/friction with zero learned offset.
- Confirm stale telemetry is detectable from its timestamp.
- Confirm the experimental steering-response request remains unavailable/effective=false and cannot mutate stock Toyota/Panda constants.
- Confirm invalid response inputs clear the fixed history and retain the established controller until delay history is warm again.
- Confirm disabled, Observe, NNFF, and NNFF Lite outputs do not consume the adapted delayed-response path.
- Confirm a short model trajectory cannot hold LongControl in `stopping` unless a real planner/forced-stop boolean remains true.
- First validate stop/go with `StartAccelRamp=0.00` to isolate the default-on guarded release correction. If a nonzero ramp is later tested, change one 0.05 m/s² step at a time in a controlled setting and verify every brake/stop/disengagement/unfiltered-standstill cancellation gate.

### 2. Observe baseline

- Start with `enabled=true`, `profile=observe`.
- Verify controller output matches the branch baseline and residual stays exactly zero.
- Disable smart saturation warning too if the comparison must preserve baseline alert timing as well as steering output.
- Collect representative left/right curves at multiple speeds and lateral accelerations.
- Label driver input, lane changes, poor lane/model confidence, wind, road crown, and steering-limit periods so contaminated samples can be excluded.
- Establish baseline tracking error, saturation time, warnings, and interventions before judging another profile.

### 3. Adaptive evaluation

- Use a controlled route or closed course in good conditions.
- Keep Experimental assistance off for the first comparison.
- Verify immediate gating during driver input, high steering rate, and steering-limit events.
- Compare repeated runs against the Observe baseline; investigate regressions and direction imbalance.
- Disable TOADPILOT immediately for oscillation, delayed unwinding, steering faults, unexpected direction, or increased interventions.
- For a clean Adaptive curve-only baseline, manually turn lane centering off first. Then restore it separately and verify its telemetry gate reason before judging its feel.
- Repeat straight and gentle-curve passes in both lane-offset directions; include road crown, imperfect markings, merges, lane changes, blinkers, and S-curves.
- Verify the assist releases without overshoot or left/right hunting when the offset crosses its release deadband.

### 4. Full Existing Authority

- Consider Full only after Adaptive has repeatable evidence across both turn directions.
- Repeat the same route and metrics instead of relying on a single difficult corner.
- Confirm normalized output never exceeds ±1 and downstream applied torque never exceeds existing limits.
- Treat frequent saturation as a planner/model/tune signal, not as justification to raise Toyota or Panda limits.

### 5. Regression and rollback

- Recheck stock/disabled behavior after every controller change.
- Recheck Experimental Mode both with its opt-in off and on.
- Preserve the pre-test torque snapshot before resetting learned data.
- Roll back by disabling TOADPILOT; do not require deletion of learner data for a normal rollback.

## Migration to an unstripped source fork

This checkout is a compiled FrogPilot release with important native UI, Params registry, and Panda safety sources removed by the release workflow. The prototype therefore uses narrowly scoped Python and Fleet Manager integration. A maintainable TOADPILOT fork needs the full source tree.

Migration work:

1. Keep an unstripped source branch and reproduce the exact device build from source.
2. Register `ToadPilotConfig`, `ToadPilotStatus`, `ToadPilotSteeringStatus`, `ToadPilotStopGoStatus`, `ToadPilotStopGoLastRoute`, `ToadPilotTuneStatus`, and `StartAccelRamp` in the native Params key registry, then remove the temporary Python allowlist.
3. Add the versioned profile controls to the native FrogPilot lateral-tuning UI and the zero-default start-ramp slider to the native longitudinal UI, including dependencies, translations, safe defaults, and clear “smaller factor is stronger” wording. Fleet Manager can remain a diagnostic/remote surface.
4. Restore Panda safety source and tests. Prove that Toyota max torque, rate limits, measured-torque checks, request-bit behavior, and violation rejection are unchanged.
5. Replace real-time JSON Params writes with a typed message or a non-real-time collector, including schema version and freshness.
6. Rebase the branch's torque controller and learner against current upstream behavior, then remove any compatibility-only offset workaround once the base implementation consumes it correctly.
7. Add controller integration tests, replay tests, Toyota/Panda safety tests, and hardware-in-the-loop checks in CI.
8. Version telemetry and tune-data formats before experimenting with high-authority or direction-specific learning.
9. Keep high-authority observation isolated until reviewable data, rejection rules, minimum sample counts, confidence bounds, and rollback behavior are defined.

## File map

- `selfdrive/frogpilot/toadpilot.py`: strict config, profiles, assist gating/smoothing, observer aggregation, and runtime status.
- `selfdrive/locationd/torqued.py`: isolated high-authority sampling and 1 Hz observer status publication.
- `selfdrive/controls/lib/latcontrol_torque.py`: torque-controller integration and stock/live tune selection.
- `selfdrive/controls/controlsd.py`: live Experimental state, lane-centering setpoint integration, and smart saturation-warning integration.
- `selfdrive/controls/lib/longcontrol.py`: real stop-flag merge and stopped-to-go ramp integration.
- `selfdrive/frogpilot/controls/lib/start_accel_ramp.py`: strict setting bounds and fail-neutral launch-ramp state machine.
- `selfdrive/frogpilot/frogpilot_variables.py`: fingerprint-gated effective configuration.
- `selfdrive/frogpilot/fleetmanager/`: prototype configuration and diagnostics surface.
- `selfdrive/frogpilot/tests/test_toadpilot.py`: focused config, safety-gate, transition, observer, telemetry, and Params tests.
- `selfdrive/frogpilot/tests/test_toadpilot_fleet.py`: offroad mutation, reset-confirmation, status-freshness, route, and template tests.
- `selfdrive/frogpilot/tests/test_toadpilot_safety_regressions.py`: live-tune bounds, cache restore/reset, cross-cadence, and exact-authority regression tests.
- `selfdrive/frogpilot/tests/test_controlsd_curvature.py`: engagement reset, curvature limits, saturation semantics, and raw-request cap-warning tests.
- `selfdrive/frogpilot/tests/test_toyota_standstill.py`: Hybrid/non-Hybrid/legacy Toyota standstill state-machine tests.
- `selfdrive/frogpilot/tests/test_start_accel_ramp.py`: stop-flag, zero-default, launch-gate, fade, expiry, and LongControl integration tests.
- `selfdrive/frogpilot/tests/test_toadpilot_upstream_torque_ports.py`: bounded response history, profile neutrality, low-speed KP behavior, and learner-input validity tests.
- `selfdrive/frogpilot/tests/test_small_upstream_ports.py`: throttle boundary, Toyota temporary fault, thermal fan, and mode-aware planner-validity tests.
- `selfdrive/frogpilot/tests/test_toyota_radar_dbc.py`: checked-in/runtime Toyota radar-distance agreement.

The design rule for future work is simple: improve how well TOADPILOT uses the authority already available, measure the result, and keep the hard electronic safety envelope independent and unchanged.
