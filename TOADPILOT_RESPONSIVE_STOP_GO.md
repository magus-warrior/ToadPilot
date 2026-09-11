# Responsive takeoff and smooth stop finish

Implementation date: 2026-09-10. This branch is an isolated development build;
it has not been installed into the running car or validated on a drive.

The requested behavior is prompt takeoff behind a moving lead and a gentler
final stop. Following distance, earlier braking, steering, model weights,
Toyota acceleration limits, and Panda safety are outside this change.

## Responsive takeoff

- Traffic Mode retains the selected acceleration profile and existing Human
  Acceleration shaping. Previously its blended ceiling reduced Sport from
  3.0 to 1.6 m/s² at zero speed, before further Human Acceleration adjustments.
  This restores a ceiling, not a minimum command: the existing lead-aware
  planner still controls acceleration and stopping.
- The Toyota brake-release correction now remembers a real stopping state
  while waiting up to two seconds for the first positive request below 2 m/s.
  Previously a transition into PID at zero acceleration lost the one-frame
  opportunity to clear the braking integral and accelerate negative-command
  release. Stops, brake input, inactivity, invalid inputs, and speed/timeout
  boundaries cannot authorize a launch. A pending positive acceleration while
  the controller is still stopping cannot trigger release.
- Existing faster release applies only to negative commands returning to zero;
  positive requests retain the existing slew and controller limits.
- The old `StartAccelRamp` floor is retired for the supported Camry. Its stored
  value is preserved for rollback but is no longer applied, including when
  Responsive takeoff is turned off. This removes the immediate 1.0 m/s² kick
  and abrupt two-second removal.

The recorded September start/stop oscillations remain honored as stop requests.
This change does not suppress the planner's stop flag to force movement. Road
logs do not establish that the zero-acceleration edge case caused all reported
delays; the regression test proves that the edge case exists and is corrected.

## Smooth stop finish

For an active, supported Camry stopping gently between 0.05 and 0.5 m/s, with
measured deceleration between 0.15 and 1.2 m/s² and a non-positive planner demand
gentler than -0.8 m/s², the controller may ease the final approach. Relief is
limited to 0.3 m/s² relative to an independently maintained stock stopping ramp,
requested braking remains at least 0.5 m/s², and easing is limited to 0.5 m/s³.
It never adds relief against a stronger planner braking request.

Driver braking, forced stops, invalid inputs, stronger measured deceleration,
and stronger planner braking use the stock request. At confirmed standstill
following relief, the normal stopping ramp resumes from the applied request
without a relief-removal step and reaches the original -2.0 m/s² hold target.
The feature is not a global brake limit. Its numerical bounds require actual
stop-distance and comfort validation before calling it road-proven.

## Settings and compatibility

Version 9 adds `responsive_takeoff` and `smooth_stop_finish`. Both default on
and migrate on from exact, valid older configurations as requested. Every
existing choice, including the master switch, remains saved. Once explicitly
disabled in version 9, either choice remains off. Malformed or incomplete
documents fail with both new features off. They become effective only through
the existing exact Camry TSS2 torque/openpilot-longitudinal capability gate.
They are independent of the lateral profile, including Observe.

Fleet Manager's ToadPilot Driving tab replaces the old stop-and-go toggle and
separate start-ramp slider with the two new toggles. The old configuration key
and ramp API remain readable for compatibility; the ramp API reports retired
and effective zero. The persistent ramp value has not been deleted.

Existing shared-memory stop/go status includes the new effective toggle values
and current stop-finish relief. This uses the existing publisher and does not
create a recording process or additional disk logging. The helpers use a few
scalar state values; a local 100,000-iteration microbenchmark measured roughly
10 microseconds for both helpers together (not a full control-loop benchmark).

## Validation and activation

Focused tests cover schema migration, explicit off choices, UI/API round trips,
Traffic Mode's restored ceiling, delayed-positive Toyota release, stop/brake/
invalid/timeout gates, braking bounds, normal hold recovery, and other vehicles.
JavaScript syntax and template rendering are checked separately. Synthetic
tests establish software behavior, not vehicle response or stopping distance.

Before installation, preserve current settings and the installed checkout,
and use a coherent offroad restart so old processes cannot consume a mixture
of configuration versions. No live Params, model, hardware, or safety limits
were changed during implementation. The first controlled evaluation should
measure planner release, Toyota target/output, first movement, driver overrides,
and final-stop acceleration at the existing following distance. Compare each
new toggle separately and retain the original branch for rollback.
