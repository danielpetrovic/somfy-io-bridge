/*
   Copyright (c) 2024. CRIDP https://github.com/cridp

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

           http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
 */

// Trimmed port of upstream's iohcRemote1W (renamed iohc_remote1w to fit this
// component's flat-directory naming convention - see iohc.h). Upstream's
// class is a singleton that owns a JSON-loaded (LittleFS + ArduinoJson) list
// of many virtual remotes, plus MQTT/webserver publishing hooks. None of
// that fits ESPHome: here, each ESPHome `cover:` entry gets its own
// IOHCRemote1W instance (one virtual remote identity per physical motor,
// exactly like upstream's own one-entry-per-bonded-device model - see
// addRemote() upstream), configured from YAML instead of a JSON file, with
// its bonded identity (node address + key) and sequence number persisted via
// Arduino's Preferences (the same NVS wrapper upstream's own
// nvs_helpers.cpp uses) instead of a JSON file + separate NVS calls.
//
// Frame-building logic (forgePacket, the Open/Close/Stop/Position/Add/Pair/
// Remove cases) is ported near-verbatim from iohcRemote1W::cmd() - only the
// persistence and multi-remote bookkeeping around it changed.

#ifndef IOHC_REMOTE1W_H
#define IOHC_REMOTE1W_H

#include <Preferences.h>
#include <string>
#include "iohcRadio.h"
#include "iohcPacket.h"
#include "iohc_blind_position.h"

namespace IOHC {

    enum class RemoteButton {
        Pair,
        Add,
        Remove,
        Open,
        Close,
        Stop,
        Position,
        // "My"/favorite position while idle - NOT the same as Stop. Two
        // confirmed real-hardware patterns exist, selected per-cover via
        // my_pattern_extended_ below (see IOHCRemote1W::set_my_pattern_extended()):
        // - simple (main=0xd8, single 14-byte p0x00_14 frame, no companion):
        //   what this bridge originally sent, matches two independent real
        //   reference implementations (laberning/home_io_control,
        //   nicolas5000/io-rts-esp32) byte-for-byte, and is what a plain
        //   roller shutter (no tilt) needs.
        // - extended (16-byte main=0xD200 trigger + CMD 0x20 companion burst,
        //   _p0x00_16/_p0x20_16): captured from a real physical Situo against
        //   a tilt-capable blind (GitHub issue #1) - reproduces height AND
        //   tilt, which the simple pattern alone does not.
        // Both are real, confirmed-working captures for their respective
        // device type - this is not a case of one being right and one being
        // wrong. The simple pattern was confirmed NOT to work on plain
        // shutters already 2W-bonded to a TaHoma/Connexoon box when sent as
        // the extended pattern (root cause still not understood - see
        // git-workflow.md/MEMORY.md for the v2026.08.1 rollback history) -
        // exactly the case the default (device_class-based) resolution is
        // meant to route back to the pattern it always worked with.
        Vent,
        // 0x1E: identify/locate. No physical remote has this button - only
        // seen via a real captured TaHoma/2W frame. Overkiz exposes it as 3
        // separate high-level actions (identify/startIdentify/stopIdentify),
        // confirmed via pyoverkiz's own command enum - matched 1:1 here.
        Identify,
        StartIdentify,
        StopIdentify,
        // The single "Prog" button's actual entry point. Add (0x30) and
        // Remove (0x39) are NOT a motor-side toggle - checked directly
        // against upstream's own reference: they're two structurally
        // distinct commands, each unconditionally doing what its name says
        // (Add never removes, Remove never adds). A real physical remote
        // decides locally, from its own remembered pairing status, which one
        // to actually send when its PROG button is pressed - it has no way
        // to query the motor's live state. Prog reproduces that: it reads
        // our own persisted paired_ flag and dispatches to Add or Remove
        // accordingly, same as a real remote would.
        Prog,
        // "Program (2W)" button's entry point (Phase 3) - NOT a 1W wire
        // command at all (there is no such frame), reuses this enum purely
        // as the existing button-type-selection mechanism, same way Vent
        // already triggers extra cover-state logic beyond a raw command.
        // Named to match Prog above (same 1:1 relationship as "Program (2W)"
        // is to "Program") rather than introducing new vocabulary at this
        // layer - see IOHCCover::press_prog2w() / IOHC::IOHCController2W::arm_bonding().
        Prog2W,
        // "Set My" button's entry point (GitHub issue #1) - reprograms the
        // motor's own stored My/favorite position (height, and tilt where
        // applicable) to wherever the shutter is currently physically
        // sitting. Somfy's own terminology: their consumer FAQ titles this
        // "set the my favourite position"; their installer guides also use
        // "programmed"/"record"/"modify" for the same action - "Set" was
        // chosen as the closest match to their most prominent official
        // wording. No "(1W)" suffix - unlike Program, which needs it to
        // distinguish the real 1W pairing ceremony from the still-unbuilt
        // 2W bonding counterpart, every other button here (including this
        // one) is 1W-only right now, so the suffix would be redundant. No
        // position argument is sent - matches protocol reality (the motor
        // samples its own current physical state, nothing is transmitted) -
        // so the desired position must already be commanded via the normal
        // controls before pressing this. Transmit-only, no cover state side
        // effects. Two patterns, same my_pattern_extended_ toggle as Vent:
        // - extended: same trigger + start-marker frames as Vent, then a
        //   sustained CMD 0x20 stepping burst matching a real
        //   confirmed-successful reprogram capture exactly (steps 0x02-0x16)
        //   before the release - matches a tilt-capable blind's own real
        //   captured reprogram (GitHub issue #1).
        // - simple: same start-marker/stepping/release companion burst as
        //   extended above (steps 0x02-0x16), but the trigger is main=0xd2
        //   - the same value as Stop - not main=0xd8 (Vent/My's own value).
        //   Confirmed byte-for-byte against a real Situo capture (Office
        //   Shutter, remote 75B4CB, 2026-08-14 - passively overheard while
        //   physically reprogramming My on the real remote). Two earlier
        //   guesses at the trigger/hold mechanism itself (repeating
        //   independently-forged frames; retransmitting one unchanged
        //   frame via repeat/repeatTime) failed to reprogram anything, and
        //   a third (this companion burst paired with main=0xd8 instead of
        //   0xd2) actively cleared the motor's existing stored My position
        //   - real hardware showed that combination is actively harmful,
        //   not just ineffective, before this real capture settled the
        //   trigger value.
        //
        // Both patterns share one more real-capture-derived detail: every
        // frame is sent 4 times total (repeat=3), not forge_packet()'s
        // default 5 (repeat=4) - confirmed against real captures'
        // own observed transmission count. With the wrong count (5), Set
        // My silently failed on some real, already-paired, battery-powered
        // shutters (Bedroom Middle Shutter) while working on others (Office
        // Shutter) using the exact same trigger/burst content - the
        // over-the-air transmission pattern itself mattered, not just the
        // decoded field values. Fixed 2026-08-14; confirmed end-to-end
        // (reprogram, then correct re-recall of the newly stored position,
        // matching the physical Situo's own stored value) on both Office
        // Shutter and Bedroom Middle Shutter afterward.
        SetMy,
    };

    // One virtual remote identity, bonded 1:1 with one physical motor.
    // Broadcast "type" is the device-class group the motor listens on (see
    // sDevicesType in iohc_utils.h) - default 0 ("All") matches upstream's
    // own addRemote() default, confirmed working for Add/Remove against
    // real shutter motors. Deliberately YAML-configurable (not hardcoded)
    // in case a different device type ever needs a different group.
    class IOHCRemote1W {
    public:
        // pref_namespace must be unique per cover (e.g. the cover's object_id)
        // so each motor's bonded identity/sequence persists independently.
        //
        // fixed_node_hex/fixed_key_hex (6/32 hex chars) are optional. If both
        // are given, this identity comes straight from YAML/secrets.yaml
        // instead of being randomly generated into this board's own flash -
        // the point being that a replacement board flashed with the same
        // config reproduces the exact same bonded identity, so motors still
        // recognize it and nothing needs re-pairing. If left empty (the
        // original behavior), a random identity is generated on first boot
        // and persists only in this board's NVS - fine until the board dies.
        void begin(iohcRadio *radio, const std::string &pref_namespace, const std::string &fixed_node_hex = "",
                   const std::string &fixed_key_hex = "");

        void set_travel_time_open(uint32_t seconds) { position_tracker_.setTravelTimeOpen(seconds); }
        void set_travel_time_close(uint32_t seconds) { position_tracker_.setTravelTimeClose(seconds); }
        void set_type(uint8_t type) { type_ = type; }
        void set_manufacturer(uint8_t manufacturer) { manufacturer_ = manufacturer; }
        // Selects which real-hardware-confirmed My/Set My wire pattern this
        // motor needs - see RemoteButton::Vent/SetMy's own comments above
        // for what each does and why both are real. Resolved by the cover
        // config layer (device_class, or an explicit override) before this
        // is ever called - see cover/__init__.py's my_pattern option.
        void set_my_pattern_extended(bool extended) { my_pattern_extended_ = extended; }

        bool is_paired() const { return paired_; }
        BlindPosition &position_tracker() { return position_tracker_; }

        // percent is only used for RemoteButton::Position (0-100).
        void cmd(RemoteButton button, int percent = -1);

    private:
        void forge_packet(iohcPacket *packet);
        void load_or_generate_identity();
        void bump_and_persist_sequence();

        iohcRadio *radio_{};
        Preferences prefs_;
        std::string pref_namespace_;
        bool has_fixed_identity_{false};

        address node_{};      // our virtual remote's own address (Source)
        uint8_t key_[16]{};    // shared session key, established at Add time
        uint16_t sequence_{1};
        uint8_t type_{0};
        uint8_t manufacturer_{2}; // 2 = Somfy, matches upstream's own default
        bool paired_{false};
        // true (extended) is only a fallback if set_my_pattern_extended() is
        // never called - the cover config layer always resolves and sets an
        // explicit value (see set_my_pattern_extended() above), so this
        // default should never actually matter in practice.
        bool my_pattern_extended_{true};

        BlindPosition position_tracker_{};
    };
}
#endif
