# ToadPilot 🌈

**Built and tuned for the 2022 Toyota Camry Hybrid.** ToadPilot is a personal fork of [FrogPilot](https://github.com/FrogAi/FrogPilot) and [openpilot](https://github.com/commaai/openpilot), focused on smoother braking, better stop-and-go behavior, and more model choices.

## Which Camrys?

| Vehicle | Status |
| --- | --- |
| **2022 Camry Hybrid** | Primary development and testing vehicle |
| **2021–2024 Camry Hybrid** | Related years using the same `TOYOTA_CAMRY_TSS2` platform; each vehicle still needs verification |
| 2018–2020 Camry Hybrid, 2025+ Camry, other Toyota/Lexus models | Outside this build's validated scope |

The reference device is a **comma C3 Lite running AGNOS 10.1**. Sharing a vehicle fingerprint does not prove that another year's steering firmware or behavior matches. This fork also contains steering research tied to specific EPS and Panda firmware; those settings are not a general upgrade for every Camry. See the [vehicle definitions](selfdrive/car/toyota/values.py) and [safety information](docs/SAFETY.md).

## What you get

- **Smoother braking:** releases mild braking gradually to reduce abrupt changes. Stronger requested braking takes effect immediately.
- **Smoother stop finish:** softens the last part of a gentle stop while preserving the stop request and standstill hold.
- **Stop-and-go improvements:** responsive takeoff, restart handling, and optional automatic Traffic Mode.
- **Optional research controls:** steering-delay learning, longitudinal coordination, accelerator-override learning, hill compensation, and camera timing checks.
- **Model choices:** North Dakota as the legacy fallback, plus ten integrated modern models.
- **Rainbow Road styling:** vivid rainbow accents, dark purple surfaces, and a matching web interface. Rainbow Path is a separate display option.

## Getting started

This is a device-specific research branch, not a fully packaged clean-device installer. **Start with a working, compatible comma installation and keep a backup.** Model downloads and their matching runtimes are separate from the Git checkout.

1. Use repository **`magus-warrior/ToadPilot`**, branch **`ToadPilot`**. The original `frogpilot.download` URL installs upstream FrogPilot, not this fork.
2. Follow the [manual setup guide](docs/TOADPILOT_SETUP.md). Keep the car off while installing or changing models.
3. Confirm that the device correctly recognizes your Camry. Do not force a fingerprint to bypass a mismatch.
4. On the same Wi-Fi network, open **`http://<device-IP>:8082/tools`** on your phone or computer. Find the device IP in its network settings.
5. Review the ToadPilot controls there. Keep unfamiliar research settings off until you understand them. Change one setting or model at a time.

Existing installations keep their saved driving settings. A new user's defaults can differ from the development car's settings.

## Models

**North Dakota** is the retained legacy fallback. The modern choices are:

| Model family | Choices |
| --- | --- |
| Regret Driven Framework | V4, V5, V6 |
| Other modern models | Pop Model V2, South Carolina, MacroStiff, Watermelon Indigo V11, Terrific Super Fantastic Do Over, Rebellious Hope, Deep RL 3 V2 |

Only installed modern models appear in the selector. Each needs its exact model file and matching runtime; copying a `.pkl` into `/data/models` is not enough. Follow the [model installation guide](selfdrive/toad_modeld/README.md). Select modern models explicitly; they are excluded from random selection. North Dakota remains available when its fallback file is installed.

The catalog excludes older model downloads. On the development car, **South Carolina** is currently selected. This is a comparison starting point, not a claim that it is best for every driver.

## Testing and limits

The seven latest model additions passed synthetic inference, message serialization, and offroad camera checks on the reference device. The relevant model/control tests passed **60 tests**, and the web interface passed **41 tests**. Short camera runs do not establish road quality or sustained driving performance.

This software assists a responsible, attentive driver. It does not make the car autonomous. Experimental Mode can still slow unexpectedly, and the driver must be ready to take over. Read the [limitations](docs/LIMITATIONS.md) before use.

## Help, credits, and license

Report problems in [this repository's issues](https://github.com/magus-warrior/ToadPilot/issues), including your vehicle year/trim, device, selected model, settings, and relevant logs. Do not post account tokens or private routes.

Thanks to the openpilot, FrogPilot, StarPilot, tinygrad, and contributing communities. The [original FrogPilot README](README.frogpilot.md) preserves upstream information and credits. See [LICENSE](LICENSE) and individual component licenses. This is research software supplied without warranty.
