# CHIRP driver for the TCA-152A

An out-of-tree [CHIRP](https://chirpmyradio.com) clone-mode driver for the
**TCA-152A** (a.k.a. TCA PRC-152A) handheld, built from the codeplug format
reverse-engineered in the [OpenTCA](../OpenTCA) project.

> **Target variant:** developed and validated on a **2023+ generation, GPS
> model (DOM 2025)** unit (MCU: TA3782F / SinOne SC95F761x). TCA-152 models
> differ internally — **pre-2023 units are quite different**, and there are
> separate **GPS / non-GPS** 2023+ variants. The codeplug format below is not
> assumed valid for other variants; treat this driver as scoped to the 2023+
> GPS model until a dump from another variant is checked.

> **⚠️ Experimental.** Reading (download) is validated against real radio
> dumps. **Writing (upload) is UNVERIFIED** — the write framing is inferred,
> not captured from the official software. Always back up your config with the
> official programmer first, and upload at your own risk. A bad write corrupts
> only the codeplug (recoverable by re-writing with the official tool), not the
> firmware — but treat it as untested until proven on hardware.

## Status

| Area | State |
|---|---|
| Download (radio → CHIRP) | ✅ works; validated against known dumps |
| Channel decode: freq, tones (CTCSS/DTCS), name, power, bandwidth, duplex, scan skip | ✅ |
| Per-channel extras: Busy Lock, SPEC, Special QT/DQT, Code | ✅ (exposed under "Extra") |
| Settings: squelch, timeout, step, work mode, priority TX, channel group, VOX gain/delay, beep, roger, LEDs, auto-lock, battery save, dynamic mic, earphone, channel-name display | ✅ |
| Settings: repeater (relay, RPT-SPK/PTT/MD/RL) and GPS (GPS ON, RX/TX GPS) | ✅ |
| Settings: FM channel / work mode / forbid RX | ✅ |
| Settings: Light Control, Scan Mode, Boot-Info Type (3-state enums) | 🟡 need 1 more dump each |
| FM tuner channels / VFO | mapped in the format; not yet surfaced as editable memories |
| Upload (CHIRP → radio) | 🟡 implemented, **UNVERIFIED** |
| Squelch, backlight, timeout, scan mode, boot-info, checkboxes, DTCS polarity | ⬜ not yet mapped |

## Install

This is an external module — no need to patch a CHIRP checkout.

1. In CHIRP, enable developer functions: **Help → Enable Developer Functions**
   (some builds: **File → …**), restart if prompted.
2. **File → Load Module…** and pick `tca152a.py`.
3. The radio appears as **TCA 152A 2023+ GPS** under Radio → Download From
   Radio (VENDOR `TCA`, MODEL `152A`, VARIANT `2023+ GPS`).

The module must be reloaded each CHIRP session (that's how Load Module works).
Alternatively, drop `tca152a.py` into `chirp/drivers/` of a CHIRP source
checkout to have it always available.

## Usage

1. Connect the programming cable, turn the radio on.
2. **Radio → Download From Radio**, choose the port and **TCA / 152A**.
3. Edit channels/settings.
4. Save the `.img`, or (experimental) **Radio → Upload To Radio**.

## Memory map

See [`MEMORY_MAP.md`](MEMORY_MAP.md) for the address layout and encodings, and
the full authoritative map in the OpenTCA repo's `CONFIG_MAP.md`.

## Serial protocol

Handshake `50 53 4F 4A 55 49 4D` → `06`; `02` → 8-byte ident → `06`; then
`52 <addr16 BE> 10` reads 16-byte blocks (`57 …` response). Upload mirrors this
with a `W` opcode and a trailing additive checksum (unverified).

## License

GPL-3.0-or-later (CHIRP is GPLv3). See [`LICENSE`](LICENSE).
