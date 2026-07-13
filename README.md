# Somfy IO Bridge (ESPHome)

Local, cloud-free control of Somfy IO (io-homecontrol) covers from Home Assistant - no TaHoma/Connexoon box required.

> [!IMPORTANT]
> Everything already paired to a motor keeps working exactly as before. This bridge just adds itself as one more virtual remote alongside whatever's already there - existing physical remotes, wall switches, sensors, and TaHoma/Connexoon all keep working unchanged; nothing is removed or replaced. This matters in practice because a motor's memory for how many controllers it can remember is small and finite: Somfy doesn't publish a maximum for IO motors. The protocol itself does track a paired-controller count internally, but no public maximum could be found for this project - if you're already close to whatever your specific motor's limit turns out to be, pairing one more (including this bridge) could fail or push out an existing one. **TaHoma/Connexoon is just another paired controller here too**, no special exemption from whatever the limit turns out to be.

> [!CAUTION]
> Personal use only, on devices you own. Interacting with IO (io-homecontrol) devices that aren't yours may be illegal in your jurisdiction. Provided as-is, with no guarantees of any kind - see the licenses of the upstream projects this is built on.

Drives the board's onboard SX1276 radio directly at the register level (no ESPHome core radio component covers IO's (io-homecontrol's) FSK framing), via a vendored port of a real working implementation rather than a from-scratch reimplementation.

"IO" here is Somfy's own branding for their io-homecontrol devices - io-homecontrol is a wider interoperability standard also used by other manufacturers (Velux, Honeywell, etc.), not something Somfy-specific. This bridge has only been built and tested against Somfy IO hardware, even though the underlying protocol layer (`components/iohc/`) is standard io-homecontrol and should, in principle, apply to other manufacturers' io-homecontrol devices too.

Somfy IO (io-homecontrol) is a different protocol on a different frequency from Somfy RTS (one-way, rolling code). If your motor/remote actually speaks RTS instead, see [`somfy-rts-bridge`](https://github.com/danielpetrovic/somfy-rts-bridge) - check your motor/remote's own documentation (or the frequency printed on it) to know which protocol it actually speaks.

**Status**: Stable for 1W (one-way) control - real-world daily use across 14 physical shutters. 2W (two-way) control/position-feedback is implemented but this bridge's own 2W bonding has never yet succeeded against real hardware - see [2W bonding: current status and open problem](#2w-bonding-current-status-and-open-problem) below.

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

This is an **ESPHome** project, driven from Home Assistant's own **ESPHome app** (called an "add-on" before Home Assistant 2026.2 renamed these to "apps" - you'll still see the older name in older screenshots/guides), not a separate program you install on your PC. ESPHome itself is maintained by Nabu Casa, the same company behind Home Assistant, which is why it integrates this smoothly (auto-discovery, native API, OTA updates from within Home Assistant).

Install it in Home Assistant first: Settings → Apps → search "ESPHome" → Install → Start (turning on "Show in sidebar" is convenient). This app is what compiles the `.yaml` files described in Setup below into firmware and flashes it onto the board - no PlatformIO, Arduino IDE, or separate `esphome` command-line install needed on your own computer.

---

## Setup

1. **Hardware**: LilyGO TTGO T3 LoRa32 **868MHz** V1.6.1 - nothing else needed. Pin mapping (confirmed against the official schematic) lives in `components/iohc/iohc_board_config.h`.
2. **Place this bridge's files** - `somfy-io-bridge.yaml` and `somfy-io-cover.yaml` - under `/config/esphome/` on your Home Assistant instance. Any app that lets you browse/edit files under `/config` works for this - Studio Code Server, File Editor, or a Samba/SSH share to your PC are all fine. `components/iohc/` doesn't need copying anywhere - `somfy-io-bridge.yaml`'s `external_components:` points straight at this repo on GitHub, so ESPHome fetches it automatically at compile time.
3. **Secrets**: add `wifi_ssid`, `wifi_password`, `io_bridge_api_key`, and `io_bridge_ota_password` to `secrets.yaml` under `/config/esphome/`. Two ways to get the values:

   #### Normal flow (recommended)

   Entirely inside the ESPHome app, no terminal needed:
   - Open the ESPHome app → the "⋮" (three-dot) menu, top right → **Secrets**. Edits `secrets.yaml` directly inside the app, creating it if it doesn't exist yet.
   - For `io_bridge_api_key` (a base64-encoded 32-byte value): ESPHome's own site has a generator built into its docs, entirely client-side (nothing sent anywhere) - open [esphome.io/components/api](https://esphome.io/components/api/#configuration-variables), find the encryption key generator, and use its **Copy** button.
   - For `io_bridge_ota_password`: no format requirement, just needs to exist - any password works, typed directly into the Secrets editor. For a random one instead of hand-picked, click **Regenerate** then **Copy** on that same generator page a second time.
   - This flow **doesn't** cover the optional fixed `node`/`key` device identity in step 4 below - that's not an ESPHome-recognized value, so it has no dedicated generator page; it always needs the Advanced flow.

   #### Advanced flow

   A terminal instead - useful if you'd rather script it, or need step 4's optional `node`/`key` values. Home Assistant's own "Terminal & SSH" or "Studio Code Server" app always has both `openssl` and Python 3 available regardless of your own computer's OS (on Windows, neither is usually available unless you have Git for Windows - use its "Git Bash"). No terminal at all? Your browser's own Developer Tools (`F12` → Console) works too: `Array.from(crypto.getRandomValues(new Uint8Array(32)), b => b.toString(16).padStart(2, '0')).join('')` prints hex, which works fine as the base64 field's value too - it's raw random bytes either way.

   ```yaml
   wifi_ssid: "your_wifi_name"
   wifi_password: "your_wifi_password"
   io_bridge_api_key: "base64-encoded-32-byte-key"     # openssl rand -base64 32
   io_bridge_ota_password: "some-password"             # openssl rand -hex 16
   ```
4. **(Optional) Generate a fixed identity for the new cover** - via the Advanced flow above (see step 3), since this value has no ESPHome-provided generator. Not required - leave `node`/`key` as empty strings (the default) and the board generates a random identity itself on first boot, which works completely fine. The only reason to do this step is so that identity **survives a board replacement**: with the default random identity, a replacement board generates a *different* one and every motor needs re-pairing; with a fixed identity from `secrets.yaml`, a replacement board flashed with the same config reproduces the exact same identity, no re-pairing needed. Worth doing *before* pairing a new cover if you want this, since switching an already-paired cover from random to fixed later needs an unpair/re-pair cycle anyway - see [Board-independent identity](#board-independent-identity-surviving-a-board-replacement) below for the command.
5. **Add a cover entry** for each physical cover under `packages:` in `somfy-io-bridge.yaml`, including the `node`/`key` from step 4 if you generated them (otherwise leave both as empty strings):
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
6. **Flash** `somfy-io-bridge.yaml` via the ESPHome app - open the app, find this file's card, and click **Install**:
   - **First flash**: choose "Plug into this computer", connect the board via USB, and pick the serial port when your browser prompts for it (only works from a browser tab open on the machine physically connected to the board).
   - **Every flash after that** (updates): choose "Wirelessly" instead - no USB needed, once the board is already on your Wi-Fi.
   - After a successful first flash, the board announces itself to Home Assistant automatically - go to Settings → Devices & Services, an "ESPHome" discovery card should appear; click **Configure** and enter the `io_bridge_api_key` value from `secrets.yaml` the one time it's needed.
7. **Pair each cover to its motor** (see below).
8. **Test**: Open/Close/Stop from the cover's card in Home Assistant. The mode `select` defaults to `Position`, which supports real percentage targets - see [Modes](#modes).

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
- **`Position`** (default) - the only mode that sends arbitrary percentage targets to the motor. The motor itself executes these accurately (confirmed via passive 2W read-back across multiple physical devices/percentages); the displayed position while a move is in progress is a fixed 25s travel-time animation, purely cosmetic (not user-configurable - there's nothing to tune since it never affects where the motor actually ends up, only how long the UI shows "still moving").
- **`Open / My / Close`** - three discrete states (0% closed / 50% "My" / 100% open), no time tracking, no arbitrary percentages - any position request strictly between 0 and 100 collapses to `Vent` (`main=0xd8`), the real My/favorite-position command, confirmed via a live capture of a real TaHoma "My" press. My and Stop are **not** the same command: the dedicated Stop action still sends `Stop` (`main=0xd2`), which (also confirmed via live capture) only has an effect while the motor is actively moving.
- **`Two-Way (Experimental)`** - real 2W commands to an actually-bonded motor. The control path (per-command challenge/response, position feedback) is implemented and validated against real captured frames. This bridge's own bonding has never yet succeeded against real hardware, though - see [2W bonding: current status and open problem](#2w-bonding-current-status-and-open-problem) below for the full picture. Selecting this mode on an unbonded motor just logs a warning and ignores commands.

## 2W bonding: current status and open problem

This bridge's own 2W bonding (`Program (2W)` - becoming a real independent controller for a motor, rather than just overhearing an existing box's traffic) has never yet succeeded against real hardware, despite substantial engineering effort across many sessions. This section documents what's been built, what's been ruled out, and why the underlying obstacle is still unresolved.

**What's implemented**: a full bonding state machine (`iohc_controller2w.*`) that actively broadcasts `DISCOVER` (`0x28`), and on a response, drives the real key-exchange sequence (`KEY_INIT`/`0x31` → `CHALLENGE_REQUEST`/`0x3C` → `KEY_TRANSFER`/`0x32` → `CHALLENGE_ANSWER`/`0x3D`), plus a required `DISCOVER_CONFIRMATION` (`0x2C`→`0x2D`) step. The frame construction and crypto (AES-ECB IV/challenge-response) were independently verified byte-for-byte against two separate, real working implementations - [`laberning/home_io_control`](https://github.com/laberning/home_io_control) and [`nicolas5000/io-rts-esp32`](https://github.com/nicolas5000/io-rts-esp32) - not just against upstream's own more exploratory reference code. Listen-Before-Talk RF collision avoidance and a fast (~900µs/channel) 3-channel RX hop were also added, matching those references' own documented behavior. A passive key-sniffer (watching for `0x31`→`0x3C`→`0x32` from any source, not just this bridge's own attempts) was built to try to recover an already-established key from an existing box's own bonding traffic instead.

**The obstacle**: every real bonding attempt against real hardware - including tests with an independently-confirmed-open pairing window, correct channel, working collision avoidance, and byte-identical frames to a genuine TaHoma broadcast captured on the same install - gets zero response to the initial `DISCOVER` broadcast.

**This isn't unique to this project.** Both reference implementations document the same real-world requirement: a device already bonded to another box (TaHoma, Connexoon, etc.) will not answer a new controller's discovery/bonding attempt while that bond exists. `nicolas5000/io-rts-esp32`'s own README states this explicitly as the intended workflow for its own key-extraction feature - remove the device from the box's app, **fully reset the device** (forgetting all previous pairings), re-pair a physical remote, *then* re-pair the box - only that genuine, fresh re-pairing event produces a capturable key transfer. `laberning/home_io_control`'s own config also expects to be handed an existing hub's Node ID/System Key directly if a device was previously paired elsewhere, rather than attempting to derive it independently.

**What this project has tried, following that same recipe, without success**: removing a shutter from TaHoma, a genuine factory reset (Double Power Cut - see [Factory-resetting a motor](#factory-resetting-a-motor-double-power-cut) above), re-pairing a 1-way remote, and re-adding the device through TaHoma's own app - the same sequence the reference project documents as sufficient. TaHoma **has** successfully re-bonded to a freshly-reset motor this way, confirmed multiple times, but the actual key-transfer exchange (`0x31`/`0x32`/`0x33`) was never captured in any of those attempts, despite the passive key-sniffer running throughout. Whether that's a timing/channel gap in the sniffer, or something about this specific motor generation/firmware that differs from what the reference authors tested against, is unresolved.

**Bottom line**: this bridge can only reliably control a motor over 1W. Becoming a real independent 2W controller - bonding with a motor itself, without relying on TaHoma/Connexoon at all - remains unsolved. There is currently no passive-decode fallback either (see "Removed" under [Scope and status](#scope-and-status) below) - real position feedback for this bridge is blocked entirely on 2W bonding actually working.

If you've made progress on this, or have a working real bonding capture from a different motor generation/firmware, please open an issue - this is the single biggest open question in this project.

## Broadcast type caveat

`broadcast_type` (per cover, default `0` = "All") controls which device-class group the `Add`/`Pair` broadcast targets - see `sDevicesType` in `components/iohc/iohc_utils.h` for the full list. The default is confirmed working against real shutter motors; there should be no need to change it.

## Scope and status

**Implemented (1W, one-way commands):**
- Real-time passive reception/decode of IO traffic (RSSI per source address, logged).
- Pairing and unpairing via a single `Program` button, dispatching to `Add` (cmd `0x30`) or `Remove` (cmd `0x39`) based on this bridge's own persisted pairing status - matches how real Somfy remotes and Somfy's own official add/remove instructions actually work, see [Pairing](#pairing-and-unpairing-a-cover-to-its-motor) above.
- Open / Close / Stop / My / absolute Position (0-100%) commands, confirmed working against real hardware. My and Stop are distinct commands (`Vent`/`main=0xd8` vs `Stop`/`main=0xd2`), confirmed via live captures of real TaHoma traffic.
- Identify / Start Identify / Stop Identify (`cmd=0x1E`) - best-guess frames, confirmed working against real hardware, see [Identify](#identify) above.
- Per-cover selectable mode (`Position` / `Open / My / Close` / `Two-Way (Experimental)` - control path implemented, but this bridge's own 2W bonding has not yet succeeded against real hardware, see [Modes](#modes) above).

All of the above confirmed working against real motors, including pairing/unpairing multiple physical shutters end-to-end.

**Implemented but not yet working (2W bonding):**
- **This bridge's own 2W bonding and control.** The real AES challenge/response + key-transfer bonding ceremony, and per-command 2W control, are both implemented and byte-verified against two independent working reference implementations - but bonding has never yet succeeded against real hardware. See [2W bonding: current status and open problem](#2w-bonding-current-status-and-open-problem) above for the full picture.

**Removed:**
- **Target Closure and Last RSSI sensors** (passive 2W position/signal decode of an *existing* 2W-bonded controller's own traffic, e.g. TaHoma) - previously implemented and confirmed accurate against real hardware, but pulled out entirely (not just disabled) since there's no point half-shipping diagnostic sensors that depend on continuous receive-side decoding ahead of real 2W control actually working - see the `Debug Passive Decode (2W)` switch's own comment in `somfy-io-bridge.yaml` for the 2026-07-13 incident that prompted this.

## Files

- `somfy-io-bridge.yaml`: the device config (radio setup, Wi-Fi/API/OTA, OLED display, diagnostic entities (WiFi Signal, Uptime, Loop Time, Restart Reason, Restart), configuration entities (Display, Display Brightness, Display Page Interval), Debug Logging / Debug Channel Hop (2W) / Debug Passive Decode (2W) control switches, and one `packages:` entry per physical cover).
- `somfy-io-cover.yaml`: reusable package template (cover + Program button + My button + Identify/Start/Stop Identify buttons + Mode select), instantiated per cover via substitution variables (`cover_id`, `cover_name`, `device_class`, `node`, `key`, `broadcast_type`, `motor_address` - required for Program (2W)/Two-Way mode, see [Pairing](#pairing-and-unpairing-a-cover-to-its-motor)).
- `components/iohc/`: this repo's own `external_component` - fetched automatically via `external_components: type: git` in `somfy-io-bridge.yaml` (see Setup above), no manual copying needed.
  - Flat directory (no subdirectories except `cover/`, `button/`, `select/`) - matches both git-source's auto-detection (`components/` at the repo root) and, historically, the only structure ESPHome's local-component loader supports, if you ever switch back to `type: local` for local development - see the comment in `iohc.h` for why.
  - `iohcRadio.*`, `iohcPacket.*`, `SX1276Helpers.*`, `sx1276Regs-Fsk.h`, `TickerUsESP32.*`, `Delegate.h`: vendored radio/protocol layer, near-verbatim from upstream.
  - `iohc_remote1w.*`: the command/pairing layer (Add/Remove/Open/Close/Stop/Vent/Position/Identify), rewritten around ESPHome's `Preferences`-backed persistence instead of upstream's JSON-file + MQTT model.
  - `iohc_blind_position.*`: the local travel-time position estimator, fixed 25s open/close, used only for the cosmetic "still moving" animation in `Position` mode (see [Modes](#modes)).
  - `cover/`, `button/`, `select/`: the ESPHome platform integration, selected via `type:`.

## OLED display

The onboard SSD1306 cycles through a page per physical cover (name + current position) plus a shared status page (device IP, WiFi signal), switching automatically every 3 seconds.

## Board-specific notes

Tested and working on the LilyGO TTGO T3 LoRa32 **868MHz** V1.6.1 specifically. Requires the `arduino` framework (not `esp-idf`) since the vendored radio code uses Arduino's `SPI`/`Preferences` libraries directly - both are declared under `esphome: libraries:` in `somfy-io-bridge.yaml` since PlatformIO's dependency finder doesn't pick them up automatically from a local `external_component`.

---

## Credits & Attribution

Not a from-scratch reimplementation - built on other people's protocol reverse-engineering and radio-level work, adapted into an ESPHome component.

- Built on [`rspaargaren/iohomecontrol`](https://github.com/rspaargaren/iohomecontrol) (Apache-2.0) - a standalone Arduino/PlatformIO firmware for the same SX1276 radio chip, with real working pairing, 1W/2W commands, and AES/CRC handling.
- That project itself builds on [`Velocet/iown-homecontrol`](https://github.com/Velocet/iown-homecontrol) (protocol documentation and reverse-engineering, largely reconstructed from decompiled TaHoma/Cozytouch firmware) and [`cridp/iown-homecontrol-esp32sx1276`](https://github.com/cridp/iown-homecontrol-esp32sx1276) (SX1276 register handling for this exact radio chip).
- The bridge's own 2W (two-way) bonding/control logic (`iohc_controller2w.h`/`.cpp`) was corrected against [`laberning/home_io_control`](https://github.com/laberning/home_io_control) (Apache-2.0) - a mature, independently-tested ESPHome IO-Homecontrol component with confirmed real 2W pairing on the same SX1276 chip family. Its documented protocol sequence (controller-initiated discovery/key-exchange, and the two-key system_key/transfer_key model) corrected a wrong directionality assumption and a crypto bug this project's own earlier bonding attempts carried over from `rspaargaren`'s more exploratory reference code - no code was copied verbatim, only the confirmed protocol/algorithm understanding.
- Also cross-checked against [`nicolas5000/io-rts-esp32`](https://github.com/nicolas5000/io-rts-esp32) - a second, independent working implementation, used to add the required `DISCOVER_CONFIRMATION` (`0x2C`→`0x2D`) bonding step neither this project nor `laberning/home_io_control` had, add Listen-Before-Talk RF collision avoidance, and fix a second crypto bug in the key-transfer IV construction. Again, no code copied verbatim - only the confirmed protocol/algorithm details.
- The vendored/ported files live in `components/iohc/` with their original copyright headers intact - see the comment at the top of each file for what was ported near-verbatim versus rewritten for ESPHome (also summarized under [Files](#files) above).

### How this repository relates to other io-homecontrol implementations

If you've come across `rspaargaren/iohomecontrol` or `laberning/home_io_control` separately and are wondering how they compare:

| | rspaargaren/iohomecontrol | laberning/home_io_control | This repository |
|---|---|---|---|
| What it is | Standalone firmware | ESPHome component | ESPHome component |
| Install/flash | Compile yourself (Arduino/PlatformIO) | ESPHome (HA app or standalone) | Home Assistant's ESPHome app |
| Talks to HA via | MQTT | ESPHome native API | ESPHome native API |
| Radio layer | Original | Original | Ported from rspaargaren |
| 2W bonding | Not confirmed working here | **Confirmed working** on real hardware | Implemented, never succeeded - see [status](#2w-bonding-current-status-and-open-problem) |
| Scope | Broad - 1W+2W, OLED, web server | Broad - multiple device types, real 2W | 1W stable/daily use; 2W control implemented, bonding unresolved |

`laberning/home_io_control` is itself built on a third project, [`nicolas5000/io-rts-esp32`](https://github.com/nicolas5000/io-rts-esp32) (per its own Acknowledgments) - not an independent peer implementation, so it isn't listed as its own column here. This repo separately cross-checked its own crypto and bonding sequence against `nicolas5000/io-rts-esp32` directly too (see [Credits](#credits--attribution)) - if working 2W bonding matters to you today, either of those two has actually achieved it, this repo hasn't yet.

Only the protocol/radio-level pieces were ported from rspaargaren/iohomecontrol specifically (packet framing, CRC/AES handling, position estimation); its MQTT layer, web server, and Arduino `main.cpp` structure were not - this repository's bridge runs entirely inside ESPHome's own component/lifecycle model instead.

## License

Apache License 2.0 - see [LICENSE](LICENSE).
