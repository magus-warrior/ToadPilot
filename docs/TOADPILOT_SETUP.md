# ToadPilot setup

This guide is for owners comfortable using SSH on a comma device. The reference setup is a **2022 Toyota Camry Hybrid, comma C3 Lite, and AGNOS 10.1**. Related **2021–2024 Camry Hybrids** share the vehicle platform but need individual verification. A clean-device installer and other hardware/OS combinations have not been tested.

## Before changing software

- Park and turn the ignition off. Back up your current software and toggle settings.
- Keep a working rollback copy. Do not delete `/data/params`, `/data/models`, or `/data/toadpilot/model-runtime` when updating an existing ToadPilot installation.
- Preserve `/data/models/north-dakota.thneed` as the fallback. If missing, download North Dakota through the model settings before relying on it. Do not rename another model to North Dakota.
- Use this repository and branch, not the upstream FrogPilot install URL:

  ```text
  Repository: https://github.com/magus-warrior/ToadPilot.git
  Branch:     ToadPilot
  ```

## Update an existing ToadPilot checkout

From an SSH terminal, while offroad:

```sh
cd /data/openpilot
git status --short
```

If there are local changes, save them before proceeding. With a clean checkout on branch `ToadPilot`:

```sh
git pull --ff-only origin ToadPilot
```

Install any missing modern models using the next section, then reboot from the device settings while the ignition is off. Updating source does not download modern model binaries.

For a first installation, prepare a separate checkout of this branch and retain the working installation until the device's OS, prebuilt applications, Panda firmware, and model files are confirmed compatible. This repository does not provide a tested one-click replacement procedure. Do not apply the development car's steering firmware to another vehicle merely because its model year is listed.

## Install a modern model

The model installer verifies the model checksum and copies its matching tinygrad runtime. Run it from this branch in `/data/openpilot`, with the ignition off. It does not change the selected model.

For example, to install **South Carolina**, first obtain the pinned runtime source:

```sh
git clone --filter=blob:none --no-checkout https://github.com/StarPilotHQ/StarPilot.git /data/starpilot-runtime-source
git -C /data/starpilot-runtime-source fetch --depth=1 origin 6bd48c28e8feea277cfdd3916c313dfec6236a93
git -C /data/starpilot-runtime-source sparse-checkout set --no-cone '/tinygrad_repo/tinygrad/' '/tinygrad_repo/LICENSE' '/LICENSE'
git -C /data/starpilot-runtime-source checkout --detach 6bd48c28e8feea277cfdd3916c313dfec6236a93
```

Use an unused destination directory for the source checkout. Download the QCOM model, then install it:

```sh
curl --fail --location --retry 3 --output /data/sc23_driving_tinygrad.pkl \
  https://raw.githubusercontent.com/firestar5683/StarPilot-Resources/b141c7dc3cc421ca68fa308d22343b7d2f897fa0/sc23_driving_tinygrad.pkl
cd /data/openpilot
python -m selfdrive.toad_modeld.install \
  --reference /data/starpilot-runtime-source \
  --model toad-south-carolina \
  --artifact /data/sc23_driving_tinygrad.pkl
```

If verification fails, stop and check the download and reference revision. Do not bypass the checksum. Optional synthetic inference check:

```sh
python -m selfdrive.toad_modeld.validate --model toad-south-carolina --frames 100 --output /data/south-carolina-validation.json
```

Reboot while offroad, then select the installed model in the device's model selector. Keep North Dakota available. Change one model or setting at a time so comparisons are meaningful.

Other models, pinned checksums, and runtime differences are documented in the [model guide](../selfdrive/toad_modeld/README.md). **Deep RL 3 V2 requires a different runtime**; the South Carolina runtime cannot be reused for it. The large model files and installed runtimes live outside the Git checkout and are not included by cloning or pulling this repository.

## First use and rollback

1. Confirm the detected vehicle is correct. Check the device for errors before engaging.
2. On the same network, open `http://<device-IP>:8082/tools` to review ToadPilot settings. New installations use code defaults; saved settings are preserved on updates.
3. Start with familiar settings. Research features and a successful offroad inference test do not establish road behavior.
4. To compare behavior, select another installed model while offroad and reboot. To roll back software, restore your saved working checkout while offroad, with its compatible OS, model runtime, and firmware.

For help, open an issue with your vehicle year/trim, device/OS, commit, model, and relevant logs. Remove private locations and tokens first.
