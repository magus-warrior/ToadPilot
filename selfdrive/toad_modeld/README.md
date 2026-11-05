# ToadPilot on-device RDF4 integration

This module supports the pinned Regret Driven Framework V4 (`toad-rdf4`, generation
v15) bundled with StarPilot commit
`4a553da9ac22a14e34513c011aed04c1e93b4619`. It uses the comma device's QCOM GPU.
The legacy v1–v7 runners and model selections remain available.

The artifact, tinygrad implementation, input history, image warps, metadata and
parser are a matched set. Other downloaded pickles are **not** interchangeable.
In particular, the public Pop v2 resource tested during this port did not load
with this tinygrad revision. This integration does not advertise that model.

`runner.py` verifies the bundled artifact before loading it and checks camera
dimensions, stride, UV offset, input/output shapes and finite values. The camera
warp and inference functions come from the compiled upstream artifact. Queue
construction, parser, constants, message filling and action conversion are adapted
from the pinned StarPilot source. Runtime and upstream licenses are installed
beside the artifact.

The v15 action's first component is divided by `max(1, vEgo)^2` to obtain
curvature. Lateral and longitudinal output smoothing use 0.1 and 0.3 seconds,
respectively. A stop uses the raw acceleration and upstream 0.3 m/s and 0.1 m/s²
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

The model manager adds the installed model to its catalog and creates its model
file link. The randomizer excludes this new model; select it explicitly in the
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
