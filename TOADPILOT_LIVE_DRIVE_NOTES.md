# ToadPilot Live Drive Notes

Date: 2026-08-31 (America/Los_Angeles)

## Safety boundary

- These are observations from one drive, not proof of road readiness.
- No runtime code, torque limit, or vehicle parameter was changed while on-road.
- Do not increase steering torque based on this drive.
- Make one bounded change set off-road, run focused tests, then repeat a controlled A/B drive with the driver ready to take over.

## Configuration found before/during the drive

- Vehicle: Toyota Camry TSS2 Hybrid with openpilot longitudinal control.
- Persistent ToadPilot JSON is schema version 8, but the checked-in decoder accepts only exact schema version 7. The running shared toggles therefore fell back to failsafe values (`toadpilot_enabled=false`, Observe profile, automatic traffic mode off). This explains why several added features appeared to do nothing.
- `AlwaysOnLateral=1` and `AlwaysOnLateralMain=1`.
- `StartAccelRamp=1.00` (maximum).
- `ForceAutoTuneOff=0`; live torque tuning remained active even though the unsupported saved JSON requested `auto_tune=false`.
- `MinimumLaneChangeSpeed=1 mph`.
- `TrafficFollow=0.61 s`.
- Driver monitoring and `loggerd` were not running, so this drive was not persisted as a normal route log.

## Drive observations

- No CAN, EPS, steering-fault, or thermal fault was observed. Temperatures stayed roughly 69-78 C.
- The reported right-lean incident occurred around 34-35 mph with only a modest right steering request (about +0.10 to +0.13 normalized / +156 to +185 CAN units) and low lateral tracking error (about 0.05-0.07 m/s^2). This does not look like insufficient steering authority.
- A short model sample showed asymmetric lane confidence. Across about 200 frames, one lane boundary was usually weak; the model path was a median ~0.39 m right of the two-line midpoint, with a maximum ~0.94 m. Because one boundary was weak, the geometric midpoint is not trustworthy enough for automatic correction, but the direction matches the reported lean.
- Live calibration was calibrated and did not show an obvious gross yaw error. The learned steering offset was stable near -3.1 degrees and roughly matched the physical steering sensor offset at rest.
- The learned lateral torque factor fell to about 1.77 versus the stock 2.35 value, close to its estimator lower bound. That makes the live-tuned feed-forward stronger than stock and is a useful A/B variable.
- Cruise disengagement repeatedly left lateral control active because Always On Lateral was enabled. Examples included steering requests around +510 to +738 units while the controls state was disabled.
- At about 10 mph, lateral reached +1.00 / +1500 while controls were disabled during a likely sharp right turn. During parking, it repeatedly reached the low-speed actuator limit (+/-1.00 / +/-350) while the driver sometimes applied opposing torque. This proves retained steering authority while cruise was disabled; it does not by itself prove an unintended turn.
- Lane-change behavior could activate at very low speeds because the minimum lane-change speed was 1 mph.
- Longitudinal control generally tracked without faults. Many stop releases were immediately overridden with accelerator input. One clean release rose to roughly 2.4 mph in 2.7 seconds. The maximum `StartAccelRamp` setting should not be evaluated together with other changes.

## Highest-priority off-road changes for the next controlled A/B

1. Disable `AlwaysOnLateral` and `AlwaysOnLateralMain` so cruise disengagement also removes lateral authority for the baseline test.
2. Reset `StartAccelRamp` to `0.00`.
3. Set `ForceAutoTuneOff=1` to compare the stock torque factor against the aggressive learned value.
4. Raise `MinimumLaneChangeSpeed` to 20 mph.
5. Replace the unsupported version-8 ToadPilot JSON with a validated version-7 config; enable only automatic traffic mode for the first isolated test and keep lane-centering assistance off.
6. Change the automatic traffic-mode low-speed boundary from 13.0 m/s (~29.1 mph) to exactly 30 mph (13.4112 m/s), with a boundary unit test and UI text.
7. Keep the maximum steering torque at 1500. More authority could amplify the observed path bias.
8. Restore/verify driver monitoring and route logging before further development drives.

## Follow-up engineering

- Analyze the last persisted July route logs off-road for interventions and faults, while noting they do not contain this drive.
- Treat confidence-aware path continuity as a future bounded change; do not steer directly from a single weak lane line.
- Update the upstream-port audit to current openpilot and separate code/driver-monitoring improvements from excluded model upgrades.
