# TOADPILOT upstream port inventory

Audit date: 2026-07-18. This is the Camry Hybrid and device-relevant inventory, not a claim that every upstream commit should be copied.

The latest published openpilot release is [v0.11.1](https://github.com/commaai/openpilot/releases/tag/v0.11.1). The current-master snapshot audited here, [`c21b082`](https://github.com/commaai/openpilot/commit/c21b0821daffaa529f2a210ffce67e5166a07b60), identifies itself as 0.11.2, but its release-notes heading is still empty, so this document calls those changes **post-v0.11.1/current master**, not released 0.11.2 features.

This fork has roughly openpilot 0.9.7/0.9.8-era process and schema architecture plus substantial FrogPilot/TOADPILOT divergence. It also lacks the source/build pipeline for Panda safety, cereal/msgq C++, native Params, Qt UI, and the embedded DBC runtime. Those boundaries make a semantic adaptation safer than a commit-by-commit rebase.

## Implemented or adapted

| Upstream change | Local result | Boundary |
| --- | --- | --- |
| [openpilot #34943](https://github.com/commaai/openpilot/pull/34943), reset curvature while inactive | Implemented around the custom lane-centering request. Inactive lateral control follows measured wheel curvature through the existing rate limiter, so re-engagement does not start from a stale model/TOAD target. | No authority change. |
| [openpilot #34651](https://github.com/commaai/openpilot/pull/34651), curvature limited by lateral acceleration | Ported the current roll-aware +/-3 m/s^2 cap and kept curvature limiting distinct from downstream Toyota/Panda clipping. Normal jerk-rate limiting is not reported as saturation. Raw pre-cap demand can still raise a correctly debounced tracking warning. | FrogPilot's planner-only static curvature cap is preserved. |
| [opendbc #3009](https://github.com/commaai/opendbc/pull/3009) via [openpilot #36996](https://github.com/commaai/openpilot/pull/36996), Toyota Hybrid standstill release | Made the Hybrid direct-release path explicit and added non-Hybrid lurch protection. Legacy/gas-interceptor and FrogPilot SNG behavior are preserved. | This removes a possible hold; it does not add launch acceleration. |
| [openpilot #36335](https://github.com/commaai/openpilot/pull/36335), low-speed torque behavior | Adapted only the low-speed error contribution so changing effective KP no longer silently retunes it. | Camry TOAD Adaptive/Full linear path only; no global gain change. |
| [openpilot #36619](https://github.com/commaai/openpilot/pull/36619), current jerk-aware torque descendant | Adapted delayed feedback setpoint, current feed-forward demand, a 0.19 s jerk lookahead, 1.2 Hz filtering, 0.3 jerk/friction input, and 0.2 friction transition. History is two fixed one-second/100-sample deques with a fail-neutral warm-up gate. | Does not import the incompatible acceleration-space PID/inverse-torque rewrite or upstream gain retune. Disabled, Observe, NNFF, and NNFF Lite retain their established paths. |
| [openpilot #37704](https://github.com/commaai/openpilot/pull/37704), torque-learning timestamp correctness | Added strict validity and finite-value checks before the local torque learner/observer consumes localization. | **Partial only:** this schema has no embedded monotonic measurement timestamp equivalent to `livePose.timestamp`; `logMonoTime` remains in use and the real timestamp migration is deferred. |
| [openpilot #35928](https://github.com/commaai/openpilot/pull/35928), throttle probability threshold | Updated 0.5 to 0.4 with an isolated boundary helper/test. | At or below 2.5 m/s throttle is already unconditionally allowed, so this cannot speed a launch from zero. |
| [opendbc #2282](https://github.com/commaai/opendbc/pull/2282), temporary Toyota ACC fault | Maps `TEMP_ACC_FAULTED` to `carFaultedNonCritical`; Camry TSS2 parser support is tested. | Permanent `ACC_FAULTED` behavior is unchanged. |
| [openpilot #37804](https://github.com/commaai/openpilot/pull/37804), thermal-block cooling | A thermally blocked, ignition-on device requests 100% fan. | The upstream 1 Hz status logging was omitted to avoid adding logger/memory pressure. |
| [openpilot #38341](https://github.com/commaai/openpilot/pull/38341), planner input validity | Both longitudinal and UI plans require every service consumed by this fork's active mode. Radar and live-parameter requirements are mode-aware. | Adapted service set; a blind `all_checks()` would be wrong for FrogPilot's optional paths. |
| [opendbc #3460](https://github.com/commaai/opendbc/pull/3460), Toyota radar distance precision | Aligned both checked-in Toyota radar DBCs to 15-bit/0.01 m and tested all 32 tracks. The prebuilt runtime already round-tripped 1.23 m correctly, so this fixes stale source/runtime disagreement. | No new live actuator behavior. |
| Local Fleet Manager save/reset defect | Polling no longer repaints the form; saves verify the returned configuration and steering-request value; explicit false choices round-trip; failed writes restore the last confirmed state. | This is a local web-UI fix, not an upstream Qt port. |
| Local Camry launch delay | Removed the false `forcingStopLength < 1` LongControl hold and added a strictly zero-default, Camry-gated start-ramp slider. The ramp requires a real stop release plus positive planner acceleration and clears on every brake/stop/standstill/inactive gate. | The release fix adds no acceleration. A nonzero ramp is opt-in, capped at 1.00 m/s², fades by 2.0 m/s, and remains inside existing acceleration and Toyota command-rate limits. |

The user-visible experimental steering-response checkbox currently stores only an immutable, offroad/reboot-latched research request. It remains unavailable and ineffective because this checkout has no matched controller plus Panda safety source/test bundle. Stock limits remain 1500 / 15 / 25 / 350.

## Deliberately not ported yet

| Candidate | Decision and prerequisite |
| --- | --- |
| [openpilot #34918](https://github.com/commaai/openpilot/pull/34918), steering at standstill | Do not force on Toyota. Upstream warned that some EPS units fault, Toyota was never opted into `steerAtStandstill`, and this old schema lacks that contract. Preserve the Camry's currently working low-speed cutoff. |
| [openpilot #36291](https://github.com/commaai/openpilot/pull/36291), measured lateral-acceleration-rate feedback | Superseded and removed by #36619/current torque control. Stacking its D term onto the descendant would not be an upstream port. |
| [openpilot #35790](https://github.com/commaai/openpilot/pull/35790) and the full current torque rewrite | Shadow/A-B as one cohesive controller later. It changes controller units, inverse torque mappings, common PID, gains, and dependencies and conflicts with this fork's NNFF/custom torque path. |
| [openpilot #34975](https://github.com/commaai/openpilot/pull/34975), online steering-delay estimation | High-value diagnostics, but local cereal has no calibrated `livePose` service/schema and there is no `lagd`. First publish an old-localizer shadow estimate; never feed actuation until route comparisons and confidence gates are proven. |
| [opendbc #2466](https://github.com/commaai/opendbc/pull/2466), Toyota pitch transient compensation | Shadow old/new acceleration requests first. It may help hill entry/exit, but is not evidence for a faster flat stop/go launch. |
| [openpilot #36075](https://github.com/commaai/openpilot/pull/36075), learned torque offset application | Keep diagnostic-only until direction-balanced routes explain the observed left lean and road-crown/calibration effects. |
| [opendbc #3452](https://github.com/commaai/opendbc/pull/3452), Toyota radar status validation | Blocked: prebuilt `libdbc.so` has no `STATUS_MSG`, this cereal lacks `radarUnavailableTemporary`, and mapping temporary status to the legacy generic fault would change disengagement semantics. Requires matched DBC runtime, schema, interface, and control event work. |
| [opendbc #3380](https://github.com/commaai/opendbc/pull/3380), Toyota wheel-speed fault validation | High-priority safety work, but must land as DBC + CarState + Panda safety masks/checks/tests. Panda safety source/tests are absent here; never half-port it. |
| [openpilot #36884](https://github.com/commaai/openpilot/pull/36884), bounded IPC queue memory | Potentially important for the loggerd/DM memory failure, but cereal/msgq C++ sources and service buffer classes are stripped. Restore and rebuild every producer/consumer before evaluating drops and RSS. |
| Current excessive-actuation lockouts | Worthwhile companion to any future authority experiment, but upstream depends on `selfdrived`, calibrated `livePose`, persistent offroad lockout, and current UI/schema. Adapt only after trustworthy pose and full Params/UI sources exist. |
| Current driver-monitoring/camera stack | A platform migration: local DM processes are disabled, the model/camera/metadata pipeline is old, and AGNOS/runtime assumptions differ. Restore as a matched safety stack, not isolated model files. |
| Current model/planner weights and tinygrad pipeline | Model blob, metadata, preprocessing, parser, planner semantics, and hardware runtime are inseparable. Never copy a weight alone. |
| Camera exposure, CPU-cap, and cruise-fault-priority changes | Exposure and CPU changes need shadow image/latency/thermal benchmarks. Cruise-fault priority is worthwhile but requires an old-controlsd event adaptation. |

## Stop/go conclusion

Official openpilot/opendbc through current master contains no evidence-backed Camry Hybrid actuator-authority or “faster go” tune. A supervised passenger session found a local release bug instead: `forcingStop` was false while numeric `forcingStopLength` hovered below 1 m, and the old call site treated that model length as a stop request. The Camry configuration also has no separate starting state or nonzero `startAccel` kick.

The local release path now uses only real stop booleans. The optional ramp remains zero by default and should be evaluated only after an isolated zero-ramp drive. The next controlled session should verify the live chain directly:

1. model/planner `shouldStop` clears;
2. planned and controller-requested acceleration become positive;
3. Toyota controller sends the allowed positive command and releases standstill;
4. PCM/GVC and measured vehicle acceleration respond.

That live session should use bounded in-memory values and no route retention. Raising Toyota or Panda limits remains a separate experiment that cannot be made effective in this stripped checkout.
