# Somfy IO Bridge (ESPHome)

Local, cloud-free control of Somfy IO (io-homecontrol) covers from Home Assistant - no TaHoma/Connexoon box required.

> [!IMPORTANT]
> Everything already paired to a motor keeps working exactly as before. This bridge just adds itself as one more virtual remote alongside whatever's already there - existing physical remotes, wall switches, sensors, and TaHoma/Connexoon all keep working unchanged; nothing is removed or replaced. This matters in practice because a motor's memory for how many controllers it can remember is small and finite: Somfy doesn't publish a maximum for IO motors. The protocol itself does track a paired-controller count internally, but no public maximum could be found for this project - if you're already close to whatever your specific motor's limit turns out to be, pairing one more (including this bridge) could fail or push out an existing one. **TaHoma/Connexoon is just another paired controller here too**, no special exemption from whatever the limit turns out to be.

> [!CAUTION]
> Personal use only, on devices you own. Interacting with IO (io-homecontrol) devices that aren't yours may be illegal in your jurisdiction. Provided as-is, with no guarantees of any kind - see the licenses of the upstream projects this is built on.

Drives the board's onboard SX1276 radio directly at the register level (no ESPHome core radio component covers IO's (io-homecontrol's) FSK framing), via a vendored port of a real working implementation rather than a from-scratch reimplementation.

"IO" here is Somfy's own branding for their io-homecontrol devices - io-homecontrol is a wider interoperability standard also used by other manufacturers (Velux, Honeywell, etc.), not something Somfy-specific. This bridge has only been built and tested against Somfy IO hardware, even though the underlying protocol layer (`components/iohc/`) is standard io-homecontrol and should, in principle, apply to other manufacturers' io-homecontrol devices too.

Somfy IO (io-homecontrol) is a different protocol on a different frequency from Somfy RTS (one-way, rolling code). If your motor/remote actually speaks RTS instead, see [`somfy-rts-bridge`](https://github.com/danielpetrovic/somfy-rts-bridge) - check your motor/remote's own documentation (or the frequency printed on it) to know which protocol it actually speaks.

**Status**: Stable for 1W (one-way) control - real-world daily use across 14 physical shutters. 2W (two-way) control with real position feedback works on mains-powered Somfy io motors paired to this bridge (tested on two), ported from [`laberning/home_io_control`](https://github.com/laberning/home_io_control) - see [Two-Way (2W)](#two-way-2w) below.

### Cover device classes

Every cover takes a standard ESPHome/Home Assistant `device_class`, set per-cover in YAML to match whatever's physically installed (the motor/protocol has no way to know this itself):

| `device_class` | Home Assistant meaning |
|---|---|
| `awning` | Awning |
| `blind` | Blind (slats that tilt, e.g. venetian) |
| `curtain` | Curtain |
| `damper` | Damper (HVAC airflow) |
| `door` | Generic door |
| `garage` | Garage door |
| `gate` | Gate |
| `shade` | Shade / roller shade (no tilt) |
| `shutter` | Shutter |
| `window` | Window |

The examples below use `shutter` - substitute whichever matches your own devices.

---

## Before you start

**Requires ESPHome 2026.11.0 or newer.** ESPHome 2026.11 made the `relation` option mandatory for every entry under `esphome: devices:`, and older versions reject it, so the YAML in this repository only builds on 2026.11.0 or newer (the configuration stops with a clear version message otherwise). On older ESPHome, use the files from release `v1.0.1` instead.

This is an **ESPHome** project, driven from Home Assistant's own **ESPHome app** (called an "add-on" before Home Assistant 2026.2 renamed these to "apps" - you'll still see the older name in older screenshots/guides), not a separate program you install on your PC. ESPHome itself is maintained by Nabu Casa, the same company behind Home Assistant, which is why it integrates this smoothly (auto-discovery, native API, OTA updates from within Home Assistant).

Install it in Home Assistant first: Settings → Apps → search "ESPHome" → Install → Start (turning on "Show in sidebar" is convenient). This app is what compiles the `.yaml` files described in Setup below into firmware and flashes it onto the board - no PlatformIO, Arduino IDE, or separate `esphome` command-line install needed on your own computer.

---

## Setup

1. **Hardware**: LilyGO TTGO T3 LoRa32 **868MHz** V1.6.1 - nothing else needed. Pin mapping (confirmed against the official schematic) lives in `components/iohc/iohc_board_config.h`.
2. **Place this bridge's files** - `somfy-io-bridge.yaml` and `somfy-io-cover.yaml` - under `/config/esphome/` on your Home Assistant instance. Any app that lets you browse/edit files under `/config` works for this - Studio Code Server, File Editor, or a Samba/SSH share to your PC are all fine. `components/iohc/` doesn't need copying anywhere - `somfy-io-bridge.yaml`'s `external_components:` points straight at this repo on GitHub, so ESPHome fetches it automatically at compile time.
3. **Secrets**: add `wifi_ssid`, `wifi_password`, and `io_bridge_api_key` to `secrets.yaml` under `/config/esphome/`. OTA updates use this same key (`ota: encryption:` with no key of its own inherits the `api:` one) rather than a separate password - no extra secret needed for that. Two ways to get the values:

   #### Normal flow (recommended)

   Entirely inside the ESPHome app, no terminal needed:
   - Open the ESPHome app → the "⋮" (three-dot) menu, top right → **Secrets**. Edits `secrets.yaml` directly inside the app, creating it if it doesn't exist yet.
   - For `io_bridge_api_key` (a base64-encoded 32-byte value): ESPHome's own site has a generator built into its docs, entirely client-side (nothing sent anywhere) - open [esphome.io/components/api](https://esphome.io/components/api/#configuration-variables), find the encryption key generator, and use its **Copy** button.
   - This flow **doesn't** cover the optional fixed `node`/`key` device identity in step 4 below - that's not an ESPHome-recognized value, so it has no dedicated generator page; it always needs the Advanced flow.

   #### Advanced flow

   A terminal instead - useful if you'd rather script it, or need step 4's optional `node`/`key` values. Home Assistant's own "Terminal & SSH" or "Studio Code Server" app always has both `openssl` and Python 3 available regardless of your own computer's OS (on Windows, neither is usually available unless you have Git for Windows - use its "Git Bash"). No terminal at all? Your browser's own Developer Tools (`F12` → Console) works too: `Array.from(crypto.getRandomValues(new Uint8Array(32)), b => b.toString(16).padStart(2, '0')).join('')` prints hex, which works fine as the base64 field's value too - it's raw random bytes either way.

   ```yaml
   wifi_ssid: "your_wifi_name"
   wifi_password: "your_wifi_password"
   io_bridge_api_key: "base64-encoded-32-byte-key"     # openssl rand -base64 32
   ```
4. **(Optional) Generate a fixed identity for the new cover** - via the Advanced flow above (see step 3), since this value has no ESPHome-provided generator. Not required - leave `node`/`key` as empty strings (the default) and the board generates a random identity itself on first boot, which works completely fine. The only reason to do this step is so that identity **survives a board replacement**: with the default random identity, a replacement board generates a *different* one and every motor needs re-pairing; with a fixed identity from `secrets.yaml`, a replacement board flashed with the same config reproduces the exact same identity, no re-pairing needed. Worth doing *before* pairing a new cover if you want this, since switching an already-paired cover from random to fixed later needs an unpair/re-pair cycle anyway - see [Board-independent identity](#board-independent-identity-surviving-a-board-replacement) below for the command.
5. **Add an area + device entry** for each physical cover under `esphome:` in `somfy-io-bridge.yaml` - this is what makes the cover show up as its own HA device. Every device entry needs `relation: via` (required since ESPHome 2026.11):
   ```yaml
   esphome:
     areas:
       - id: my_room_area
         name: "My Room" # must match an existing HA area name exactly, or HA creates a new one
     devices:
       - id: my_new_cover_io_device
         name: "My Room Shutter" # include the room name here
         area_id: my_room_area
         relation: via
   ```
6. **Add a cover entry** for each physical cover under `packages:` in `somfy-io-bridge.yaml` (`cover_id` + `_device` must match the device `id` from step 5), including the `node`/`key` from step 4 if you generated them (otherwise leave both as empty strings):
   ```yaml
   packages:
     my_new_cover_io: !include
       file: somfy-io-cover.yaml
       vars:
         cover_id: my_new_cover_io
         cover_name: "My New Cover IO"
         node: !secret io_my_new_cover_io_node
         key: !secret io_my_new_cover_io_key
   ```
   This creates the cover plus its "Program" button (pairs and unpairs, like the physical remote's own PROG button), a "My" button, three Identify buttons (see [Identify](#identify)), and "Mode" select. See `somfy-io-cover.yaml` for the full set of optional vars (`device_class`, defaults to `shutter` - see [Cover device classes](#cover-device-classes); `broadcast_type` - rarely needs overriding). `node`/`key` can be left as empty strings instead, in which case a random identity is auto-generated and stored on the board's own flash on first boot - not recommended, see step 4.
7. **Flash** `somfy-io-bridge.yaml` via the ESPHome app - open the app, find this file's card, and click **Install**:
   - **First flash**: choose "Plug into this computer", connect the board via USB, and pick the serial port when your browser prompts for it (only works from a browser tab open on the machine physically connected to the board).
   - **Every flash after that** (updates): choose "Wirelessly" instead - no USB needed, once the board is already on your Wi-Fi.
   - After a successful first flash, the board announces itself to Home Assistant automatically - go to Settings → Devices & Services, an "ESPHome" discovery card should appear; click **Configure** and enter the `io_bridge_api_key` value from `secrets.yaml` the one time it's needed.
8. **Pair each cover to its motor** (see below).
9. **Test**: Open/Close/Stop from the cover's card in Home Assistant. The mode `select` defaults to `One-Way`, which supports real percentage targets - see [Modes](#modes).

## Pairing (and unpairing) a cover to its motor

Single `Program` button. `Add` (cmd `0x30`) and `Remove` (cmd `0x39`) are two structurally distinct commands - checked directly against upstream's own reference implementation (`rspaargaren/iohomecontrol`) - and the Program button picks between them by reading this bridge's own persisted pairing record (was the last successful Add/Remove for this cover an add or a remove?) and sending the matching command.

**To add** (pair a new cover to a motor that already has at least one working control):
1. If the already-paired remote you're using to open programming mode has multiple channels, select the correct channel on it first.
2. Press and hold that remote's PROG button until the motor jogs (can take a few seconds, don't give up too early). The motor then stays in programming mode for about **10 minutes** (confirmed via a real capture - the motor jogs a second time, unprompted, when the window closes on its own; no RF traffic accompanies that closing jog, it's a purely local/mechanical indication).
3. Within that window, press the new cover's `<Cover Name> Program` button in Home Assistant (transmits `Add`, cmd `0x30` - the actual key-exchange command; despite the label, this is *not* the same as `RemoteButton::Pair`/cmd `0x2e`, which only applies once a key is already shared). The motor jogs again, confirming the change - the new virtual remote is now bonded, in addition to (not instead of) any existing remote (e.g. TaHoma, or a physical Situo).

**To remove** (e.g. before migrating to a fixed node/key, or decommissioning a cover) - the two presses in this order:
1. **First**, press and hold PROG on the control you want to **keep** (a physical remote or TaHoma - not the cover being removed; select the correct channel first if it's multi-channel). Hold until the motor jogs.
2. **Then**, within the same programming window, press the `<Cover Name> Program` button in Home Assistant for the cover you want to **remove** (transmits `Remove`, cmd `0x39`, per this bridge's own persisted record).

> [!WARNING]
> Per Somfy's own instructions: a control can only be removed if **another** control is available to open the motor's programming window first. If this board's Program button is the *only* paired control for a motor and you remove it, you lose the ability to open that motor's programming window at all (nothing left to press PROG on) - you'd need the motor manufacturer's own hard-reset procedure to recover, see [Factory-resetting a motor](#factory-resetting-a-motor-double-power-cut) below. Always keep at least one other working control (TaHoma, a physical Situo/Telis, etc.) paired before removing this board's identity.

There's no acknowledgment in this protocol, so this bridge's record of whether a cover is currently bonded is an assumption, not a verified fact - if an Add/Remove is sent while the motor wasn't actually in its programming window, that record and the motor's real state fall out of sync (the frame goes nowhere, but the record still updates as if it worked). Confirming the motor stops responding (after a removal) or starts responding (after an add) is the only way to verify a press actually took; if it doesn't match what you expected, the Program button may need pressing twice to get back in sync.

## Factory-resetting a motor (Double Power Cut)

Documented in Somfy's own motor manuals as one option in the motor's troubleshooting flow - this applies to both RTS and IO motors. This section covers the mechanics of the reset itself, once you've decided it's needed, not when to reach for it over a lighter remedy. Both paths below start with the same Double Power Cut; only the second half differs depending on whether you already have a working paired remote.

**Step 1 (both paths) - Double Power Cut:**
1. Motor powered.
2. Cut power - **2-3 seconds** (the manual's own diagram shows a 2s minimum; TaHoma's own "advanced reset" flow uses 3s, and 3-8-3 timing has been confirmed working in practice).
3. Restore power - **8 seconds**.
4. Cut power again - **2-3 seconds**.
5. Restore power - the motor moves for ~5 seconds (or a short up/down movement if it was sitting at an end-limit), confirming the cut registered.

> [!WARNING]
> Per the manual: only power the specific motor being reset - anything else sharing the same circuit gets reset too. Do not use a 2-way remote/box (TaHoma, Connexoon) during this procedure.

Timing by hand is error-prone - a smart plug driven by a short HA script gets it exact:
```yaml
script:
  reset_shutter_motor_dpc:
    alias: "Reset Shutter Motor (Double Power Cut)"
    sequence:
      - service: switch.turn_off
        target: {entity_id: switch.YOUR_SMART_PLUG}
      - delay: "00:00:02"
      - service: switch.turn_on
        target: {entity_id: switch.YOUR_SMART_PLUG}
      - delay: "00:00:08"
      - service: switch.turn_off
        target: {entity_id: switch.YOUR_SMART_PLUG}
      - delay: "00:00:02"
      - service: switch.turn_on
        target: {entity_id: switch.YOUR_SMART_PLUG}
```

**Step 2 depends on your situation:**
- **You have a working, currently-paired remote**: hold its PROG button for **more than 7 seconds**, until the motor does **2 short movements → OK**. This erases every *other* 1-way control, but the remote you used stays paired - you can go straight into calibration with it afterward without pressing PROG again.
- **You have no working remote** (lost or defective): using the **new** replacement remote you intend to pair, press PROG **briefly - under 1 second** - until the motor does **1 short movement → OK**. This erases every *other* 1-way control, and pairs the new remote immediately as part of the same step.

**What gets reset either way**: all settings (except the retraction/back-impulse function) return to factory defaults. **What's genuinely unclear**: whether an existing 2-way bond (TaHoma, Connexoon) survives - the manual doesn't say either way, and it can't be tested in isolation in practice (post-reset calibration and any 2-way box's own onboarding both require a fresh 1-way pairing ceremony, which confounds any attempt to observe 2-way state on its own).

**After either path, before the motor is usable with this bridge**:
1. Make sure you have one working physical remote paired (per whichever path above applied) and use it to program travel limits - this bridge has no travel-limit/calibration feature of its own.
2. Check rotation direction while in calibration mode: long-press UP and DOWN together to enter it, then press and hold UP or DOWN to see which way the motor actually turns. If reversed, press **MY for 2 seconds** to flip it.
3. Only after travel limits are calibrated, add this bridge as an additional control via its own Program button (see [Pairing](#pairing-and-unpairing-a-cover-to-its-motor) above).
4. If a 2-way box was previously bonded, re-add the device through its normal app flow - it will ask for a PROG press on a paired 1-way remote as part of its own search.

## Identify

Three extra buttons per cover - `<Cover Name> Identify`, `Start Identify`, `Stop Identify` - jog the motor without changing its position, useful for visually confirming which physical motor a Home Assistant entity actually maps to. `Identify` is a one-shot jog (matches TaHoma's own "locate device" action); `Start`/`Stop Identify` are for a manual continuous blink. Confirmed working against real hardware. Always transmits over 1W, regardless of the cover's own `Mode` selection - unlike the cover's own Open/Close/Stop/Position controls, Identify was never wired to route through 2W.

No physical Somfy remote has an Identify button, so there was no confirmed 1W frame format to copy going in - what's implemented is a best-guess signed 1W frame built by matching the command byte (`cmd=0x1E`) and payload bytes seen in a real captured 2W TaHoma-to-motor exchange, wrapped in the same self-signed `data+sequence+hmac` structure this bridge's own Pair/Remove frames use (since a 1W broadcast has no live challenge/response round trip to lean on the way 2W does). That guess turned out correct - it works over 1W despite Identify itself only being documented/observed as a 2W (TaHoma) feature.

## My and Set My

`My` moves the cover to its motor-stored favorite position - height, and tilt where the motor supports it - the same as pressing My on a physical Situo remote. Two independently real-hardware-confirmed wire patterns exist, selected per-cover via the [Tilt Support switch](#tilt-support-switch) below:

- **Simple** (off, default for shutters/shades) - a single frame, `main=0xd8`. Matches two independent real reference implementations ([`laberning/home_io_control`](https://github.com/laberning/home_io_control), [`nicolas5000/io-rts-esp32`](https://github.com/nicolas5000/io-rts-esp32)), and is what a plain roller shutter or shade (no tilt) needs.
- **Extended** (on, default for blinds) - a 16-byte trigger frame followed by a two-frame confirmation burst, confirmed against a real Situo capture from a tilt-capable venetian blind - needed to reproduce tilt, which the simple pattern alone cannot. Confirmed *not* working on plain shutters already 2W-bonded to a TaHoma/Connexoon box, which is why this is a per-cover toggle rather than a single fixed pattern.

Since no position value is ever transmitted for this command - the motor recalls whatever it has stored internally - there's no way to know in advance which direction the cover will move, or where it will end up. The displayed position and movement state are deliberately left unchanged after a My press rather than guessing: an incorrect guess would risk firing the wrong opening/closing automation trigger in Home Assistant, which is worse than showing no movement at all. Move the cover with `Position`/`Open`/`Close` first if you need the displayed position to stay accurate.

`Set My` reprograms the motor's own stored favorite position to wherever the cover is currently physically sitting - the same real-world effect as holding My on a physical Situo remote for about 5 seconds. Move the cover to the desired position (and tilt, if applicable, via the physical remote) first, then press `Set My`. No position is transmitted here either, for the same reason - the motor samples its own current physical state, not a value sent to it. Both patterns are confirmed working end-to-end on real hardware (reprogram, then re-recall of the newly stored position).

### Tilt Support switch

Each cover has a **Tilt Support** switch (config entity) selecting which of the two `My`/`Set My` patterns above it uses - on for the extended/tilt-capable pattern, off for the simple pattern. Controlled by the `my_pattern` YAML substitution (`auto` (default) / `simple` / `extended`) on first-ever boot only - `auto` resolves from the cover's `device_class` (`blind` → on, everything else → off). Once toggled from Home Assistant, the choice persists on the device (survives reboots) and the YAML default no longer applies. Override it directly if a specific motor needs the other pattern despite its configured device class.

## Board-independent identity (surviving a board replacement)

By default, each cover generates a **random** virtual remote identity (a 3-byte node address + 16-byte AES key) on first boot, stored only in that specific board's flash (NVS). If the board ever dies, a replacement would generate a *different* random identity, requiring every motor to be re-paired.

Do this **before** pairing a new cover, not after - switching an already-paired cover from random to fixed needs an unpair/re-pair cycle (see step 3 in Setup above). Generate a fixed identity for the cover - this is a one-off, offline step that just produces two random hex strings, it does **not** run on Home Assistant, the ESPHome app, or the board itself. Run it via the Advanced flow from step 3 in Setup above (Home Assistant's Terminal & SSH/Studio Code Server, your own computer's terminal, or the browser console fallback):
```shell
python3 -c "import secrets; print('node:', secrets.token_hex(3)); print('key:', secrets.token_hex(16))"
```
No Python available where you're running this? Two separate `openssl` calls work just as well: `openssl rand -hex 3` (node) and `openssl rand -hex 16` (key).

Then add the printed values to `secrets.yaml`, named after the cover:
```yaml
io_<cover_id>_node: "<the printed node>"
io_<cover_id>_key: "<the printed key>"
```
and reference them in the cover's `!include` vars:
```yaml
node: !secret io_<cover_id>_node
key: !secret io_<cover_id>_key
```
A replacement board flashed with the same config reproduces the exact same identity - no re-pairing needed. If a cover is already paired with a random identity, unpair it first before switching to a fixed one, then pair again.

**The same idea applies to this bridge's own 2W controller identity** (the box/TaHoma-like role, see [Modes](#modes) below) - one shared identity for the whole bridge, not per-cover, set on the `iohc:` hub itself rather than per-cover:
```yaml
iohc:
  id: iohc_hub
  controller_address: !secret io_bridge_2w_controller_address
  system_key: !secret io_bridge_2w_system_key
```
Generated the same way (`secrets.token_hex(3)`/`secrets.token_hex(16)`, or the two `openssl rand -hex` calls above), stored under whatever names you like in `secrets.yaml`. Leave both unset (the default) to keep the random-generate-and-persist-on-device behavior. As of this writing no real 2W bond has ever succeeded on any install this bridge has been tested against (see [Modes](#modes) below), so there is nothing to lose by fixing this now rather than after a real bond exists.

## Modes

Each cover has a `select:` entity to switch between:
- **`One-Way`** (default) - the only mode that can send arbitrary percentage targets to the motor. An explicit percentage (confirmed via passive 2W read-back across multiple physical devices/percentages) or a full Open/Close reaching the motor's own mechanical end-stop both land exactly where expected regardless of the travel-time animation's accuracy along the way - see [Travel time](#travel-time) below. A manual **Stop** partway through a move is the one exception: this bridge never gets real position feedback over 1W, so whatever's displayed and persisted afterward is exactly wherever the travel-time estimate happened to be at that instant, not a real reading - see [Travel time](#travel-time) for why that estimate's accuracy still matters here.
- **`Two-Way`** - real 2W commands to a motor paired to this bridge: every command is confirmed by the motor, and Home Assistant shows the motor's real position and direction. See [Two-Way (2W)](#two-way-2w) below for pairing. A motor that is not paired to this bridge refuses its commands (the log says `refused with 0x80`).

The other entities' `(1W)`/`(2W)` suffixes (`Program (1W)`, `Travel Time Open (1W)`, `Program (2W)`, ...) name the mode they belong to. The options were called `Position (1W)`/`Two-Way (2W)` before; the mode is stored as a number, so every cover keeps its mode, but an automation that selects an option by its text needs the new names.

A third mode, `Open / My / Close` (three discrete states, no time tracking, no arbitrary percentages, ported over from the RTS bridge's own default model), was removed 2026-11-27 - unused, with no install ever confirmed running a cover in it. A cover whose saved mode was still set to it migrates to `One-Way` the next time it boots.

## Travel time

Each cover has **Travel Time Open**/**Travel Time Close** number entities (config entities, seconds) controlling how long the local position estimate takes to run from one extreme to the other. One fixed 25s value couldn't fit every installation (GitHub issue #4 - real stopwatch data showed one axis taking 45s against a 25s model, and another taking 12-15s, on the same board, errors running in opposite directions at once).

This matters differently depending on [mode](#modes) and how a move actually ends. In `One-Way`, an explicit percentage or a full Open/Close that reaches the real end-stop both land exactly where expected no matter how accurate the animation was along the way, since the motor itself is the source of truth there - purely cosmetic in that case. But a manual **Stop** partway through a move never gets a real position back from the motor at all (no feedback over 1W), so the value shown and persisted afterward is exactly whatever this estimate computed at that instant - a mistuned value means that's a genuinely wrong number, not just a wrong animation.

Starts from the `travel_time_open`/`travel_time_close` YAML substitutions (default 25s each) on first-ever boot; once changed from Home Assistant, the choice persists on the device and the YAML default no longer applies, same mechanism as [Tilt Support](#tilt-support-switch). Both values are always in Home Assistant's own space (what "open" means to HA) regardless of [Direction inversion](#direction-inversion) below - the component handles mapping that onto the motor's real physical direction internally, so a value entered here never needs to be swapped by hand when Invert Direction is toggled.

## Direction inversion

Some installations have both motor axes wired or configured so this bridge's own Open/Close convention comes out physically backwards from Home Assistant's - an awning where `Open` retracts it and `Close` extends it, when HA's own convention is that an awning is open when extended. `device_class` doesn't affect this - it only changes labels/icons, not behavior.

Each cover has an **Invert Direction** switch (config entity) for exactly this - on swaps which command actually gets sent for Open/Close, and mirrors the local position estimate and any real 2W position feedback back to Home Assistant's own convention, so the entity reports the same orientation it accepts. Starts from the `invert` YAML substitution (default `false`) on first-ever boot; once toggled from Home Assistant, the choice persists on the device and the YAML default no longer applies, same mechanism as [Tilt Support](#tilt-support-switch). `Stop` and `My`/`Set My` are unaffected either way - both are direction-agnostic commands. [Travel time](#travel-time)'s open/close values are also unaffected from your point of view - they stay in Home Assistant's own space either way, the component remaps them internally.

## Two-Way (2W)

In `Two-Way` mode this bridge is the motor's 2W controller, the role a TaHoma box plays: it commands the motor under its own system key, the motor confirms every command, and the bridge reads the motor's real position. The 2W engine is a port of [`laberning/home_io_control`](https://github.com/laberning/home_io_control)'s (exchange, retries, channel access, pairing, status polling).

### Pairing a motor

Requirements:
- The cover's `motor_address`: the motor's own 3-byte address. With a TaHoma or Connexoon it is the Overkiz device's address in Home Assistant; it also shows in this bridge's log whenever the motor answers (`... to XXXXXX refused with 0x80` before pairing).
- A battery or solar motor also needs `low_power: "true"` (it is woken with the long preamble).

Steps:
1. Press **PROG once** on the motor's own remote (select its channel first on a multi-channel remote) and hold until the motor jogs. Its pairing window stays open for about 10 minutes. Do not press PROG a second time: that can close the window or remove the remote.
2. Press the cover's **`Program (2W)`** button. With a `motor_address` it pairs that motor by its address (laberning's `pair_device`); the log ends with `Device XXXXXX took the key and is paired to this bridge`. The `pair_device` action does the same from Home Assistant.
3. Set the cover's **Mode** to `Two-Way`.

What pairing changes:
- The motor now holds this bridge's system key instead of its previous hub's. That hub (TaHoma, Connexoon, ...) can no longer command or read it; remove the device from the hub's app if you want, the motor ignores it. Re-adding it in the hub's app gives the hub its key back, and this bridge then needs the cover set back to `One-Way`.
- The motor's physical 1W remotes keep working.
- `Scan Paired Devices (2W)` lists every motor that holds this bridge's key.

### In use

- A command shows opening or closing at once; while the motor moves, its position is read every 3 s until it reports itself at rest. A command that gets no answer is retried (as in laberning's component) and logged as a warning if it still fails.
- Each Two-Way cover is read 5 s after boot, so Home Assistant shows its real position after a restart.
- `status_refresh_interval` (per cover, off by default, at least `1min`) reads the motor whenever it has reported nothing for that long, so a move made with a physical remote shows up within the interval. `5min` suits a mains motor; a battery motor is woken by every read, so use an hour or more, or leave it off.
- The `Try ... (heard X preamble(s), Y frame(s))` log lines say whether a missing reply arrived corrupted (a preamble, no frame) or not at all.

### Known limitations

- **Discover & Pair** (`Program (2W)` without a `motor_address`): the Somfy io motors tested here never answer a discovery broadcast, even with their pairing window open, so pairing needs the `motor_address`.
- **Recover System Key** (taking over an existing hub's key, so motors need no re-pairing): implemented, but against a TaHoma Switch the hub has not yet gone past the device announcement. Not working yet.
- A move made with a physical remote shows in Home Assistant only at the next read; reading the motor right after a remote press (laberning's linked remotes) is not ported yet.
- While a 2W exchange runs the bridge does nothing else, up to 2.5 s for a command (the same as laberning's component).
- Pairing a battery (low-power) motor has not been tested yet.

## Broadcast type caveat

`broadcast_type` (per cover, default `0` = "All") controls which device-class group the `Add`/`Pair` broadcast targets - see `sDevicesType` in `components/iohc/iohc_utils.h` for the full list. The default is confirmed working against real shutter motors; there should be no need to change it.

## Scope and status

**Implemented (1W, one-way commands):**
- Real-time passive reception/decode of IO traffic (RSSI per source address, logged).
- Pairing and unpairing via a single `Program` button, dispatching to `Add` (cmd `0x30`) or `Remove` (cmd `0x39`) based on this bridge's own persisted pairing status - matches how real Somfy remotes and Somfy's own official add/remove instructions actually work, see [Pairing](#pairing-and-unpairing-a-cover-to-its-motor) above.
- Open / Close / Stop / My / absolute Position (0-100%) commands, confirmed working against real hardware. My and Stop are distinct commands - see [My and Set My](#my-and-set-my) above for My's two real, per-cover-selectable frame shapes ([Tilt Support switch](#tilt-support-switch)).
- `Set My`, reprogramming the motor's own stored favorite position to the cover's current physical position - both patterns confirmed working end-to-end on real hardware, see [My and Set My](#my-and-set-my) above.
- Identify / Start Identify / Stop Identify (`cmd=0x1E`) - best-guess frames, confirmed working against real hardware, see [Identify](#identify) above.
- Per-cover selectable mode (`One-Way` / `Two-Way`, see [Modes](#modes) above).
- Per-cover Invert Direction switch, for installations where this bridge's own Open/Close convention comes out physically backwards from Home Assistant's - see [Direction inversion](#direction-inversion) above.
- Per-cover configurable travel time (Travel Time Open/Close number entities), replacing a single fixed 25s value that couldn't fit every installation - see [Travel time](#travel-time) above.
- 1W TX now recovers on its own if a transmission's completion is never confirmed (missed interrupt, stuck radio flag), instead of leaving the radio keyed indefinitely until power-cycled.

All of the above confirmed working against real motors, including pairing/unpairing multiple physical shutters end-to-end.

**Implemented (2W, two-way):**
- Pairing a motor to this bridge by its address, confirmed 2W commands, real position and direction, the read after boot and the optional status refresh, ported from `laberning/home_io_control` and confirmed on two mains-powered Somfy io motors - see [Two-Way (2W)](#two-way-2w) above, including its known limitations.

**Removed:**
- **Target Closure sensor** - removed: a Two-Way cover reports its real position itself, and Confirm And Resend takes the overheard current position over into a Position (1W) cover. The `Debug Passive Decode (2W)` and `Debug Channel Hop (2W)` switches are removed too: received frames are always decoded, and the receiver always rotates over the three channels.
- **`Open / My / Close` mode** - a third select option (three discrete states, no time tracking) ported over from the RTS bridge's own default model. Removed 2026-11-27 as unused; no install was ever confirmed running a cover in it. A cover whose saved mode was still set to it migrates to `One-Way` at its next boot.

## Files

- `somfy-io-bridge.yaml`: the device config (radio setup, Wi-Fi/API/OTA, OLED display, diagnostic entities (WiFi Signal, Uptime, Loop Time, Restart Reason, Restart), configuration entities (Display, Display Brightness, Display Page Interval), Debug Logging control switch, and one `packages:` entry per physical cover).
- `somfy-io-cover.yaml`: reusable package template (cover + Program button + My button + Set My button + Identify/Start/Stop Identify buttons + Mode select + Tilt Support switch + Invert Direction switch + Travel Time Open/Close numbers), instantiated per cover via substitution variables (`cover_id`, `cover_name`, `device_class`, `node`, `key`, `broadcast_type`, `motor_address` - required for Program (2W)/Two-Way mode, see [Pairing](#pairing-and-unpairing-a-cover-to-its-motor), `my_pattern` - see [Tilt Support switch](#tilt-support-switch), `invert` - see [Direction inversion](#direction-inversion), `travel_time_open`/`travel_time_close` - see [Travel time](#travel-time)).
- `components/iohc/`: this repo's own `external_component` - fetched automatically via `external_components: type: git` in `somfy-io-bridge.yaml` (see Setup above), no manual copying needed.
  - Flat directory (no subdirectories except `cover/`, `button/`, `select/`, `switch/`, `number/`) - matches both git-source's auto-detection (`components/` at the repo root) and, historically, the only structure ESPHome's local-component loader supports, if you ever switch back to `type: local` for local development - see the comment in `iohc.h` for why.
  - `iohcRadio.*`, `iohcPacket.*`, `SX1276Helpers.*`, `sx1276Regs-Fsk.h`, `TickerUsESP32.*`, `Delegate.h`: vendored radio/protocol layer, near-verbatim from upstream.
  - `iohc_remote1w.*`: the command/pairing layer (Add/Remove/Open/Close/Stop/Vent/SetMy/Position/Identify), rewritten around ESPHome's `Preferences`-backed persistence instead of upstream's JSON-file + MQTT model.
  - `iohc_blind_position.*`: the local travel-time position estimator, per-cover configurable (default 25s open/close, see [Travel time](#travel-time)), used for the cosmetic "still moving" animation in `One-Way` mode (see [Modes](#modes)).
  - `cover/`, `button/`, `select/`, `switch/`, `number/`: the ESPHome platform integration. `button/`, `switch/`, and `number/` each dispatch multiple entity types via their own `type:` field (switch: Tilt Support/Invert Direction; number: Travel Time Open/Close); `select/` implements one entity type (Mode).

## OLED display

The onboard SSD1306 cycles through a page per physical cover (name + current position) plus a shared status page (device IP, WiFi signal), switching automatically every 3 seconds.

## Board-specific notes

Tested and working on the LilyGO TTGO T3 LoRa32 **868MHz** V1.6.1 specifically. Requires the `arduino` framework (not `esp-idf`) since the vendored radio code uses Arduino's `SPI`/`Preferences` libraries directly - both are declared under `esphome: libraries:` in `somfy-io-bridge.yaml` since PlatformIO's dependency finder doesn't pick them up automatically from a local `external_component`.

---

## Credits & Attribution

Not a from-scratch reimplementation - built on other people's protocol reverse-engineering and radio-level work, adapted into an ESPHome component.

This bridge's own SX1276 driver and 1W layer (`iohcRadio.*`, `iohcPacket.*`, `SX1276Helpers.*`, `iohc_remote1w.*`) were ported near-verbatim from one source; its 2W bonding/control logic (`iohc_controller2w.*`) was independently corrected and verified against two different, separate sources. These are genuinely different lineages, not "based on rspaargaren" as a whole - see the table below for how the four projects actually relate.

- **Radio driver and 1W layer** ported from [`rspaargaren/iohomecontrol`](https://github.com/rspaargaren/iohomecontrol) (Apache-2.0) - a standalone Arduino/PlatformIO firmware for the same SX1276 radio chip, with real working 1W pairing/commands (its own 2W path is explicitly disclaimed by its author as no longer stable). That project itself builds on [`Velocet/iown-homecontrol`](https://github.com/Velocet/iown-homecontrol) (protocol documentation and reverse-engineering, largely reconstructed from decompiled TaHoma/Cozytouch firmware) and [`cridp/iown-homecontrol-esp32sx1276`](https://github.com/cridp/iown-homecontrol-esp32sx1276) (SX1276 register handling for this exact radio chip).
- **2W engine** (`iohc_controller2w.*`, the 2W parts of `iohc.cpp`, `iohc_key_extraction.*`, the frame log hold in `iohc_log_buffer.*`, and the Two-Way cover state in `cover/iohc_cover.*`) is ported from [`laberning/home_io_control`](https://github.com/laberning/home_io_control) (MIT, Copyright (c) 2026-present Lars Berning and other contributors): its exchange engine (send-and-receive, attempt plans, retries, channel access with a learned busy limit, the low-power wake belief), its pairing engine and roll-call, its idle listening, its status polling and refresh, its key extraction responder and its frame log hold, rewritten for this bridge's radio stack.
- Also cross-checked against [`nicolas5000/io-rts-esp32`](https://github.com/nicolas5000/io-rts-esp32) (Apache-2.0) - a second, independent working 2W implementation (and itself the project `laberning/home_io_control` is built on, per its own Acknowledgments), used to add the required `DISCOVER_CONFIRMATION` (`0x2C`→`0x2D`) bonding step neither this project nor `laberning/home_io_control` had, add Listen-Before-Talk RF collision avoidance, and fix a second crypto bug in the key-transfer IV construction. Again, no code copied verbatim - only the confirmed protocol/algorithm details.
- The vendored/ported files live in `components/iohc/` with their original copyright headers intact - see the comment at the top of each file for what was ported near-verbatim versus rewritten for ESPHome (also summarized under [Files](#files) above).

### How this repository relates to other io-homecontrol implementations

If you've come across any of these separately and are wondering how they compare - and specifically, how much of this repository actually comes from `rspaargaren` versus the other two:

| | rspaargaren | nicolas5000 | laberning | This repo |
|---|---|---|---|---|
| What it is | Standalone firmware | Standalone firmware | ESPHome component | ESPHome component |
| Install | Compile yourself | Compile yourself | ESPHome | HA's ESPHome app |
| Talks to HA via | MQTT | MQTT/CLI | Native API | Native API |
| Radio driver | Original | Original | Built on nicolas5000 | Ported from rspaargaren¹ |
| 1W bonding | Implemented, unsupported | Passive-only² | Passive-only² | **Implemented, stable** |
| 1W control | Implemented | Not implemented | Not implemented | **Implemented, stable** |
| 2W bonding | Implemented, "unstable" | **Confirmed working** | **Confirmed working** | **Confirmed working** (by address) |
| 2W control | Implemented | **Confirmed, w/ position** | **Confirmed, w/ position** | **Confirmed, w/ position** |
| Scope | Broad - 1W+2W, OLED, web | Broad - 2W, CLI, Ethernet | Broad - multi-device, tilt | 1W stable; 2W ported from laberning³ |

¹ Same "layered origin" pattern as laberning building on nicolas5000 - see above, not a claim this repo copies rspaargaren wholesale.
² Only passively overhears 1W traffic (nicolas5000: to extract remote IDs/site keys; laberning: to fire HA automation events) - neither actively bonds/controls over 1W.
³ This repo's modes are `One-Way` (1W) and `Two-Way` (2W) - see [Modes](#modes) and [Two-Way (2W)](#two-way-2w).

The practical upshot: none of the three reference projects offer stable 1W bonding/control the way this one does (rspaargaren has it but disclaims support for it; the other two skip it entirely and only listen passively) - that's this project's own strongest, most-tested feature. Its 2W side is a port of `laberning/home_io_control`'s, so for 2W features beyond covers (heaters, sensors, linked remotes and more), that component is the reference.

Only the protocol/radio-level pieces were ported from rspaargaren/iohomecontrol specifically (packet framing, CRC/AES handling, position estimation, the 1W command/pairing layer); its MQTT layer, web server, and Arduino `main.cpp` structure were not, and its 2W logic was independently rebuilt against the other two sources instead of being ported from rspaargaren at all - this repository's bridge runs entirely inside ESPHome's own component/lifecycle model.

## License

Apache License 2.0 - see [LICENSE](LICENSE).

The 2W engine contains code ported from [`laberning/home_io_control`](https://github.com/laberning/home_io_control), MIT License, Copyright (c) 2026-present Lars Berning and other contributors - see [LICENSES/MIT-home_io_control.txt](LICENSES/MIT-home_io_control.txt). The files it applies to say so in their header.
