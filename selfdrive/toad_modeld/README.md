# ToadPilot on-device model integration

Supported models use the comma device's existing QCOM GPU:

| Model | Generation | Matching StarPilot runtime |
| --- | --- | --- |
| Regret Driven Framework V4 (`toad-rdf4`) | v15 | `4a553da9ac22a14e34513c011aed04c1e93b4619` |
| Pop Model V2 (`toad-pop-v2`) | v11 | `6bd48c28e8feea277cfdd3916c313dfec6236a93` |
| South Carolina (`toad-south-carolina`) | v11 | `6bd48c28e8feea277cfdd3916c313dfec6236a93` |

The legacy v1–v7 runners remain available. Each artifact has a pinned SHA-256
and its own installed runtime; serialized GPU programs cannot be mixed with an
arbitrary tinygrad revision. Pop v2 and South Carolina use separate vision and
policy outputs, with 25-frame sampled feature history. RDF4 is a supercombo with
its own history and direct action output. The parser/constants are identical
between the two pinned revisions. No Chestnut or external GPU is required.

`runner.py` verifies the bundled artifact before loading it and checks camera
dimensions, stride, UV offset, input/output shapes and finite values. The camera
warp and inference functions come from the compiled upstream artifact. Queue
construction, parser, constants, message filling and action conversion are adapted
from the pinned StarPilot source. Runtime and upstream licenses are installed
beside the artifact.

The v15 action's first component is divided by `max(1, vEgo)^2` to obtain
curvature. Lateral and longitudinal output smoothing use 0.1 and 0.3 seconds,
respectively. v11 derives acceleration and curvature from the plan at the
configured action horizon, following the pinned upstream mlsim path. A v15 stop uses the raw acceleration and upstream 0.3 m/s and 0.1 m/s²
thresholds. This fork has no live steering-delay estimator; its configured
`CarParams.steerActuatorDelay` supplies the actuator delay. Smoothing and the
0.075-second frame/action horizon are added explicitly.

Normal cruise retains ToadPilot's existing longitudinal planner. Experimental
Mode takes the smaller of the MPC and model acceleration, combines their stop
requests, and retains the planner's acceleration limits. No special freeway speed
policy is introduced. The additive cereal Action fields use upstream ordinals;
`longitudinalValid` is a local presence flag so an old model publisher cannot be
mistaken for a valid new-model acceleration command.

Install from the pinned reference checkout while offroad:

```sh
python -m selfdrive.toad_modeld.install --reference /path/to/StarPilot
python -m selfdrive.toad_modeld.validate --frames 100 --output /tmp/rdf4-validation.json
```

The reference must contain `tinygrad_repo/tinygrad`, both licenses and all three
bundled `selfdrive/modeld/models/driving_tinygrad.pkl` chunks. Installation checks
the revision, working-tree modifications and model SHA-256. It installs into
`/data/toadpilot/model-runtime/<revision>` without replacing legacy tinygrad.

The model manager adds installed models to its catalog and creates its model
file link. The randomizer excludes these new models; select one explicitly in the
model selector. Restart after installing the schema/process changes before
selecting it. Returning to North Dakota selects the original runner.

`python -m selfdrive.toad_modeld.modeld --shadow --max-frames 100` checks an
already-running camera stream without publishing model or odometry messages.
It uses stored vehicle parameters. Offroad shadow messages may be invalid because
there is no live calibration service; this check does not validate road behavior.

Validation covers unit/regression tests, synthetic inference and serialization,
and a real-camera shadow run. These checks establish software compatibility, not
driving quality, sustained onroad timing, or freedom from unexpected slowing in
Experimental Mode.

For Pop v2 / South Carolina, use the corresponding pinned reference and supply
`--model toad-pop-v2 --artifact /path/to/pop223_driving_tinygrad.pkl` or
`--model toad-south-carolina --artifact /path/to/sc23_driving_tinygrad.pkl` to the
installer. Both validation commands accept `--model MODEL_ID` as well.

The Tools page's default-on **Smooth braking** setting is separate from model
output conversion. It eases mild brake release by limiting each increase to
0.8 m/s³, with at most 0.15 m/s² additional braking. Stronger requested braking
passes immediately. It bypasses low speed, stopping, positive acceleration,
manual brake/gas, inactive controls and invalid input. **Smooth stop finish**
remains the separate near-standstill feature. Both are Camry TSS2 features and
can be disabled in Tools; they do not change model weights or actuator limits.

## Optional software backports (configuration v11)

Tools → Research exposes five independent settings, all initially off. Existing
v10 choices migrate unchanged. Smooth braking retains its saved setting (default
on). The new settings remain behind the existing supported-Camry/master gates.

* **Live steering-delay learning**: adapts comma v0.11.2's masked correlation and
  block averaging to this fork's calibrated `liveLocationKalman` yaw. Requires
  calibrated, healthy localization, active unsaturated steering without override,
  speed above 50 mph, useful curvature variation, and recovery after interruptions.
  Modern models use only fresh, confident estimates in 0.15–0.65 seconds; invalid
  or unavailable estimates return gradually to the original vehicle delay.
  Delay changes are bounded to 0.005 seconds per model frame. Cached values are
  keyed by vehicle parameters and selected model and need new route evidence.
  The confidence-window peak index includes the minimum-lag offset; flat/nonfinite
  inputs and sampling gaps are rejected. This differs from the upstream adapter.
* **Modern longitudinal coordination**: Pop V2, South Carolina and RDF4 in
  Experimental Mode use separate model, cruise and lead-plan candidates, choosing
  the lowest acceleration and retaining every stop request. Cruise jerk follows
  comma v0.11.2. The fork's current compiled MPC, cruise obstacle, following policy,
  acceleration limits, throttle gating and controller remain in use. It is not a
  full replacement with upstream openpilot's planner. Turning it off restores the
  previous blended-MPC path. Missing/invalid modern actions invalidate the plan.
* **Learn from accelerator overrides**: optional with modern coordination. Uses
  upstream v0.11.2's bounded boost (0.025 m/s² per second, 0.05 per override, 0.2
  total); requires prior evidence that the model was limiting acceleration. Clears
  on disengagement, disabled operation, braking or a model stop. Lead/cruise
  candidates still constrain acceleration; no freeway speed floor is introduced.
* **Toyota hill-response compensation**: ports the transient pitch feedforward
  from opendbc `d1bd15ecb18e273e4af740dbbfb7def555828668`. Adds at most ±1.5 m/s²
  to PID feedforward with existing final command limits. Inactive during stopping,
  pedal overrides or missing/invalid attitude. Filters initialize from the first
  valid attitude to avoid a startup pulse. Steering authority is unchanged.
* **Camera timing checks**: a local guard for modern models, not a camera-driver
  port. Rejects repeated frames, timestamps older than 200 ms or in the future,
  and road/wide synchronization error above 10 ms. Invalid input marks model and
  odometry publications invalid; normal validity requirements remain in place.

Sources (MIT; copyright/license in repository LICENSE):

* https://github.com/commaai/openpilot/blob/v0.11.2/openpilot/selfdrive/locationd/lagd.py
* https://github.com/commaai/openpilot/blob/v0.11.2/openpilot/selfdrive/controls/lib/longitudinal_planner.py
* https://github.com/commaai/openpilot/blob/v0.11.2/openpilot/selfdrive/controls/lib/accel_boost.py
* https://github.com/commaai/opendbc/blob/d1bd15ecb18e273e4af740dbbfb7def555828668/opendbc/car/toyota/carcontroller.py

### Hardware scope and validation limits

The installed C3 Lite still uses AGNOS 10.1, its matched Panda firmware and native
camera executable. This release checkout omits the camera C++ implementation and
uses a customized Toyota/Panda pairing. A newer OS/firmware/camera migration needs
matching build artifacts and hardware integration tests; runtime toggles cannot
provide that. No other vehicle fingerprints or firmware acceptance rules were
changed. Driver-monitoring processes/configuration are unchanged, as requested.

Tests exercise settings migration, bounded learning, estimator recovery, real
longitudinal solver updates and Toyota controller calls, and camera rejection.
Camera shadow runs publish no control/model/odometry messages and are offroad.
Passing these checks does not establish road behavior, eliminate Experimental
Mode slowdowns or certify equivalence to the latest complete upstream release.
