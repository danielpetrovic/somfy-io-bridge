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

#ifndef IOHC_KEY_EXTRACTION_H
#define IOHC_KEY_EXTRACTION_H

// "Recover System Key" - device-role responder, ported from
// github.com/laberning/home_io_control's KeyExtractionResponder/
// pairing_responder (MIT licensed). Where that project emulates an
// unpaired device so a user's OWN existing hub (TaHoma, Connexoon, a Velux
// box) runs its normal "add a device" wizard against it, and hands over
// its node_id/system_key as part of that pairing - with nothing on the
// hub's own motors touched, and no reset of anything.
//
// This bridge's own 2W bonding (IOHCController2W) never completed against
// real hardware. The system key this class can recover is the SAME key
// already shared between the hub and every motor it has ever bonded, so
// holding it is what IOHCController2W's own "Program (2W)" attempt has
// been missing for every already-TaHoma-paired motor - no per-motor
// re-bonding needed for those, unlike a brand-new motor nobody has paired.
//
// Scope narrowed from the reference for this bridge's own use (Somfy
// TaHoma, not a general multi-vendor hub): no typed-discovery handling
// (always answers as a Somfy roller shutter, type word 0x0080, matching a
// real captured Somfy RS100), no key-pull direction (TaHoma only pushes
// its key, never asks for ours), no per-attempt timing scheduler (this
// bridge's own loop()-polled millis() comparisons, matching
// IOHCController2W's own style, rather than a named-timeout service this
// bridge doesn't have). The node-verification round (0x36/0x37/0x3C/0x3D,
// used by some Velux hubs) is still ported in full since it costs little
// and nothing here is Somfy-specific enough to assume TaHoma never does
// it. NOT yet hardware-tested: nothing here has recovered a real key from
// a real TaHoma box.
//
// Does NOT touch IOHCController2W::system_key_ automatically on a
// successful extraction - only logs the recovered key. Adopting it as
// this bridge's own system key (so it could then command every motor
// TaHoma already trusts, with no further bonding) is a deliberate,
// separate decision: it would also mean this bridge's own key no longer
// matches whatever it may have already bonded with under its own
// previously-generated key.

#include <cstdint>
#include "iohcPacket.h"
#include "iohcRadio.h"

namespace esphome {
namespace iohc {
class IOHCComponent;
}  // namespace iohc
}  // namespace esphome

namespace IOHC {

class IOHCKeyExtractor {
 public:
    void begin(iohcRadio *radio, esphome::iohc::IOHCComponent *owner);

    // Arm (generates a fresh throwaway node ID, listens for a hub's
    // discovery) or disarm (stops answering, does not touch anything
    // already recovered - the recovered key/hub id stay logged, not
    // erased, until the next arm cycle overwrites them).
    void set_armed(bool armed);
    bool armed() const { return state_ != ResponderState::DISARMED; }

    // Per-cycle timeouts and the deferred slotted discovery answer - see
    // the .cpp's loop() for the three timers this drives (10 minute hard
    // cap, 1 minute post-extraction grace, the discovery answer's own slot
    // delay). Called from esphome::iohc::IOHCComponent::loop().
    void loop();

    // Dispatches a received 2W frame if it is one of this responder's own
    // (0x28/0x2C/0x31/0x32/0x36/0x3C) and the responder is armed. Returns
    // true if it was consumed. Safe to call unconditionally for every
    // frame, like IOHCController2W::handle_frame() - a cheap no-op while
    // disarmed or for a frame not addressed to it.
    bool handle_frame(iohcPacket *packet);

 private:
    enum class ResponderState : uint8_t {
        DISARMED,
        ARMED_IDLE,             // listening for a hub's discovery (0x28)
        SENT_DISCOVER_RESP,     // our 0x29 is queued or sent; waiting for 0x2C or 0x31
        SENT_CONFIRM_ACK,       // sent our 0x2D; waiting for 0x31
        SENT_CHALLENGE,         // sent our 0x3C; waiting for the hub's 0x32
        EXTRACTED,              // key recovered from a 0x32; some hubs stop here
        SENT_NODE_VERIFY_RESP,  // sent our 0x37; waiting for the hub's 0x36 retry or its 0x3C
    };
    static const char *stage_name_(ResponderState state);

    void generate_throwaway_id_();
    void handle_discover_(iohcPacket *packet);
    void handle_discover_confirm_(const IOHC::address &hub_src);
    void handle_key_init_(const IOHC::address &hub_src);
    void handle_key_transfer_(iohcPacket *packet);
    void handle_node_verify_req_(const IOHC::address &hub_src);
    void handle_node_verify_challenge_(iohcPacket *packet);

    void send_discover_resp_now_();
    void send_reply_(iohcPacket *packet);
    void arm_grace_();
    void log_result_();
    void listen_before_talk_();

    iohcRadio *radio_{nullptr};
    esphome::iohc::IOHCComponent *owner_{nullptr};

    ResponderState state_{ResponderState::DISARMED};
    IOHC::address throwaway_id_{};
    uint8_t challenge_[6]{};
    IOHC::address hub_node_id_{};
    uint8_t recovered_key_[16]{};

    uint32_t armed_until_ms_{0};
    uint32_t grace_until_ms_{0};

    // The 0x29 answers a broadcast search in a random slot, not at once -
    // see send_discover_resp_now_()'s doc. Deferred here rather than sent
    // from handle_discover_() itself.
    bool discover_reply_pending_{false};
    uint32_t discover_reply_due_ms_{0};
    IOHC::address discover_reply_dst_{};
    uint32_t discover_reply_freq_{CHANNEL2};

    // Channel this session's replies go out/are held on, once the first
    // one (the 0x29) has actually been sent.
    uint32_t session_freq_{CHANNEL2};
};

}  // namespace IOHC

#endif
