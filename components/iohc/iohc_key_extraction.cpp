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

#include "iohc_key_extraction.h"
#include "iohc_2w_frame.h"
#include "iohcCryptoHelpers.h"
#include "iohc.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <esp_system.h>
#include <cstring>
#include <vector>

/// See iohc_key_extraction.h for what this file ports and what it
/// deliberately narrows from github.com/laberning/home_io_control's own
/// KeyExtractionResponder/pairing_responder (MIT), and the real-hardware
/// warning below carries over unchanged: nothing here has recovered a key
/// from a real TaHoma box yet. The crypto (recovering the key is the exact
/// same XOR primitive, run in reverse, this bridge's own passive key-sniffer
/// already uses - see IOHCController2W::handle_frame()'s SniffState handling)
/// is the one piece independently corroborated by that other, already-working
/// feature; the frame shapes and timing below are new to this bridge and
/// unverified against a real hub.

namespace IOHC {

static const char *const TAG = "iohc.key_extraction";

namespace {

constexpr uint32_t AUTO_OFF_MS = 10UL * 60UL * 1000UL;        // 10 minute arm window.
constexpr uint32_t POST_EXTRACT_GRACE_MS = 60UL * 1000UL;     // 1 minute after a successful extraction.
constexpr uint8_t ID_GEN_MAX_ATTEMPTS = 16;                   // Collision-retry budget for the throwaway ID.
constexpr uint16_t DISCOVER_RESP_PREAMBLE_SYMBOLS = 32;       // Peer is cold-scanning; needs the longer start preamble.
constexpr uint16_t REPLY_PREAMBLE_SYMBOLS = 16;               // Peer already holds this channel open.
constexpr uint32_t DISCOVERY_SLOT_MS = 41;                    // One answer slot's length (laberning's measured value).
constexpr uint8_t ADDRESS_SUFFIX_MASK = 0x3F;
constexpr uint8_t ADDRESS_SUFFIX_DISCOVERY_64 = 0x3C;
constexpr uint8_t ADDRESS_SUFFIX_DISCOVERY_128 = 0x3D;
constexpr uint8_t ADDRESS_SUFFIX_DISCOVERY_255 = 0x3E;
constexpr uint8_t DISCOVERY_SLOTS_32 = 32;
constexpr uint8_t DISCOVERY_SLOTS_64 = 64;
constexpr uint8_t DISCOVERY_SLOTS_128 = 128;
constexpr uint8_t DISCOVERY_SLOTS_255 = 255;

// How many answer slots a discovery broadcast opened, from its own
// destination address suffix - a searcher collects answers for the whole
// window before it confirms one, so the answer is a timed transmission,
// not a reply at once. See laberning/home_io_control's proto_codecs.cpp
// discovery_slot_count(), ported unchanged.
uint8_t discovery_slot_count(const IOHC::address &dst) {
    switch (dst[2] & ADDRESS_SUFFIX_MASK) {
        case ADDRESS_SUFFIX_DISCOVERY_64: return DISCOVERY_SLOTS_64;
        case ADDRESS_SUFFIX_DISCOVERY_128: return DISCOVERY_SLOTS_128;
        case ADDRESS_SUFFIX_DISCOVERY_255: return DISCOVERY_SLOTS_255;
        default: return DISCOVERY_SLOTS_32;  // the plain discovery suffix, and anything unrecognised.
    }
}

}  // namespace

const char *IOHCKeyExtractor::stage_name_(ResponderState state) {
    switch (state) {
        case ResponderState::DISARMED: return "disarmed";
        case ResponderState::ARMED_IDLE: return "armed_idle";
        case ResponderState::SENT_DISCOVER_RESP: return "sent_discover_resp";
        case ResponderState::SENT_CONFIRM_ACK: return "sent_confirm_ack";
        case ResponderState::SENT_CHALLENGE: return "sent_challenge";
        case ResponderState::EXTRACTED: return "extracted";
        case ResponderState::SENT_NODE_VERIFY_RESP: return "sent_node_verify_resp";
    }
    return "disarmed";
}

void IOHCKeyExtractor::begin(iohcRadio *radio, esphome::iohc::IOHCComponent *owner) {
    radio_ = radio;
    owner_ = owner;
}

void IOHCKeyExtractor::generate_throwaway_id_() {
    for (uint8_t attempt = 0; attempt < ID_GEN_MAX_ATTEMPTS; attempt++) {
        for (uint8_t i = 0; i < 3; i++)
            throwaway_id_[i] = static_cast<uint8_t>(esp_random() & 0xFF);
        // Avoid the two broadcast-discovery addresses and an all-zero ID -
        // this bridge has no device registry to check for a real collision
        // against (unlike the reference, which also checks its own
        // already-registered devices), so this is a best-effort check only.
        if ((throwaway_id_[0] == 0 && throwaway_id_[1] == 0 &&
             (throwaway_id_[2] == 0x3B || throwaway_id_[2] == 0x3F || throwaway_id_[2] == 0x00)))
            continue;
        return;
    }
    // Every attempt collided (astronomically unlikely) - the last-generated
    // candidate is used anyway rather than leaving the buffer stale.
}

void IOHCKeyExtractor::set_armed(bool armed) {
    if (!armed) {
        if (state_ == ResponderState::DISARMED)
            return;
        state_ = ResponderState::DISARMED;
        discover_reply_pending_ = false;
        if (owner_ != nullptr) {
            owner_->set_responder_hop_wanted(false);
        }
        ESP_LOGI(TAG, "Recover System Key: disarmed");
        return;
    }

    generate_throwaway_id_();
    memset(challenge_, 0, sizeof(challenge_));
    memset(hub_node_id_, 0, sizeof(hub_node_id_));
    discover_reply_pending_ = false;
    state_ = ResponderState::ARMED_IDLE;
    armed_until_ms_ = esphome::millis() + AUTO_OFF_MS;
    session_freq_ = CHANNEL2;

    if (owner_ != nullptr) {
        // Idle wait: stay on CH2, same as this bridge's own discovery
        // broadcasts and TaHoma's own are expected to use (Findings
        // this project already relies on elsewhere: "commands are sent on
        // CH2"). No hop needed to catch it.
        owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::HOLD_REQUEST);
        owner_->set_responder_hop_wanted(true);
    }

    ESP_LOGW(TAG,
             "Recover System Key: ARMED for 10 minutes as throwaway device %02X%02X%02X. Start adding a new "
             "device on your existing hub (TaHoma: Settings > Devices > add a Somfy device) now. Remove the "
             "phantom device it adds afterward.",
             throwaway_id_[0], throwaway_id_[1], throwaway_id_[2]);
}

void IOHCKeyExtractor::listen_before_talk_() {
    // Same ETSI EN 300 220-style check IOHCController2W::listen_before_talk()
    // uses before every 2W transmission - duplicated here rather than shared,
    // since that one is a private method of a different class.
    for (uint8_t i = 0; i < 5; i++) {
        float rssi = Radio::readRssi();
        if (rssi < -90.0f) return;
        esphome::delay(5);
    }
}

void IOHCKeyExtractor::send_reply_(iohcPacket *packet) {
    radio_->retune(session_freq_);
    listen_before_talk_();
    radio_->send(packet);
}

bool IOHCKeyExtractor::handle_frame(iohcPacket *packet) {
    if (state_ == ResponderState::DISARMED)
        return false;
    if (packet->payload.packet.header.CtrlByte1.asStruct.Protocol)
        return false;  // 1W frame, not ours.

    uint8_t cmd = packet->payload.packet.header.cmd;

    if (cmd == 0x28) {
        handle_discover_(packet);
        return true;
    }

    const auto &dst = packet->payload.packet.header.target;
    if (memcmp(dst, throwaway_id_, sizeof(IOHC::address)) != 0)
        return false;
    const auto &src = packet->payload.packet.header.source;

    if (cmd == 0x2C) {
        handle_discover_confirm_(src);
        return true;
    }
    if (cmd == 0x31) {
        handle_key_init_(src);
        return true;
    }
    if (cmd == 0x32) {
        handle_key_transfer_(packet);
        return true;
    }
    if (cmd == 0x36) {
        handle_node_verify_req_(src);
        return true;
    }
    if (cmd == 0x3C) {
        handle_node_verify_challenge_(packet);
        return true;
    }
    return false;
}

void IOHCKeyExtractor::handle_discover_(iohcPacket *packet) {
    const auto &hub_src = packet->payload.packet.header.source;

    bool accept = false;
    if (state_ == ResponderState::ARMED_IDLE || state_ == ResponderState::SENT_DISCOVER_RESP) {
        accept = true;
    } else if (state_ == ResponderState::EXTRACTED || state_ == ResponderState::SENT_NODE_VERIFY_RESP) {
        // A completed attempt's hub may legitimately start a fresh one (its own
        // retry, or the user running the wizard a second time); a DIFFERENT
        // hub's unrelated 0x28 must not knock a live post-extraction round back
        // to SENT_DISCOVER_RESP.
        accept = memcmp(hub_src, hub_node_id_, sizeof(IOHC::address)) == 0;
    }
    if (!accept)
        return;
    state_ = ResponderState::SENT_DISCOVER_RESP;

    // A device answers a broadcast search once, in a random slot from 1 to
    // N (never slot 0), on one of the two channels the search did NOT come
    // in on - the searcher listens there for answers (a discovery reply on
    // the request channel is rare). Deferred, not sent at once.
    const uint32_t slots = discovery_slot_count(packet->payload.packet.header.target);
    const uint32_t slot = 1 + (esp_random() % slots);
    const uint32_t delay_ms = slot * DISCOVERY_SLOT_MS;

    static constexpr uint32_t kChannels[3] = {CHANNEL2, CHANNEL1, CHANNEL3};
    size_t arrival_index = 0;
    for (size_t i = 0; i < 3; i++) {
        if (kChannels[i] == packet->frequency)
            arrival_index = i;
    }
    const size_t step = 1 + (esp_random() % 2);
    const uint32_t freq = kChannels[(arrival_index + step) % 3];

    memcpy(discover_reply_dst_, hub_src, sizeof(IOHC::address));
    discover_reply_freq_ = freq;
    discover_reply_pending_ = true;
    discover_reply_due_ms_ = esphome::millis() + delay_ms;
    ESP_LOGI(TAG, "Recover System Key: hub %02X%02X%02X is searching - answering in slot %u/%u (%u ms) on %lu Hz",
             hub_src[0], hub_src[1], hub_src[2], static_cast<unsigned>(slot), static_cast<unsigned>(slots),
             static_cast<unsigned>(delay_ms), static_cast<unsigned long>(freq));
}

void IOHCKeyExtractor::send_discover_resp_now_() {
    auto *packet = new iohcPacket;
    forge_2w_frame(packet, throwaway_id_, discover_reply_dst_, true, true, false);
    packet->payload.packet.header.cmd = 0x29;
    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x29_resp);
    auto &resp = packet->payload.packet.msg.p0x29_resp;
    // Type/subtype word 0x0080 - the word a real Somfy RS100 io reports
    // (laberning/home_io_control's own KEY_EXTRACTION_DISCOVER_RESP_TYPE_WORD).
    // No typed-discovery handling here - always answers as this one device
    // kind, which is all this bridge's own motors are.
    resp.type_subtype[0] = 0x00;
    resp.type_subtype[1] = 0x80;
    // A real captured Somfy 0x29 reports its own node ID as its backbone
    // address here.
    memcpy(resp.backbone, throwaway_id_, sizeof(IOHC::address));
    resp.manufacturer = 2;  // Somfy.
    // Multi Information Byte 0xDD: turnaround class 3 (40ms) and the
    // low-power bit set - the byte a real Somfy Izymo dimmer sends, with
    // the power-save bits forced low-power (this bridge scans on the
    // ordinary ESPHome loop, not instantly, so advertising always-alive
    // would be optimistic; low-power is also what every real shutter this
    // bridge actually has would honestly report).
    resp.flags = 0xDD;
    resp.timestamp[0] = 0x00;
    resp.timestamp[1] = 0x0E;
    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    packet->start_preamble = DISCOVER_RESP_PREAMBLE_SYMBOLS;

    session_freq_ = discover_reply_freq_;
    radio_->retune(session_freq_);
    listen_before_talk_();
    radio_->send(packet);
    // Hold this channel for the rest of the session - the hub's follow-up
    // frames (0x2C, then 0x31...) are expected back on it, matching every
    // unicast pairing reply this project has measured elsewhere.
    if (owner_ != nullptr) {
        owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::HOLD_FREQ, session_freq_);
    }
    ESP_LOGI(TAG, "Recover System Key: sent DISCOVER_RESP (0x29) to %02X%02X%02X on %lu Hz", discover_reply_dst_[0],
             discover_reply_dst_[1], discover_reply_dst_[2], static_cast<unsigned long>(session_freq_));
}

void IOHCKeyExtractor::handle_discover_confirm_(const IOHC::address &hub_src) {
    if (state_ != ResponderState::SENT_DISCOVER_RESP && state_ != ResponderState::SENT_CONFIRM_ACK)
        return;
    state_ = ResponderState::SENT_CONFIRM_ACK;

    auto *packet = new iohcPacket;
    forge_2w_frame(packet, throwaway_id_, hub_src, false, true, false);
    packet->payload.packet.header.cmd = 0x2D;
    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    packet->start_preamble = REPLY_PREAMBLE_SYMBOLS;
    send_reply_(packet);
    ESP_LOGI(TAG, "Recover System Key: acknowledged DISCOVER_CONFIRM (0x2C) from %02X%02X%02X", hub_src[0],
             hub_src[1], hub_src[2]);
}

void IOHCKeyExtractor::handle_key_init_(const IOHC::address &hub_src) {
    uint8_t new_challenge[6];
    for (uint8_t &b : new_challenge) b = static_cast<uint8_t>(esp_random() & 0xFF);

    bool resend_only = false;
    if (state_ == ResponderState::SENT_DISCOVER_RESP || state_ == ResponderState::SENT_CONFIRM_ACK) {
        memcpy(challenge_, new_challenge, sizeof(challenge_));
        memcpy(hub_node_id_, hub_src, sizeof(IOHC::address));
        state_ = ResponderState::SENT_CHALLENGE;
    } else if (state_ == ResponderState::SENT_CHALLENGE) {
        // A retry: the hub missed our 0x3C. Resend the SAME challenge -
        // the eventual 0x32 must be decrypted with the exact bytes it
        // actually received, not a freshly generated one.
        resend_only = true;
    } else {
        return;
    }

    auto *packet = new iohcPacket;
    forge_2w_frame(packet, throwaway_id_, hub_src, false, false, false);
    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x3c);
    packet->payload.packet.header.cmd = 0x3C;
    memcpy(packet->payload.packet.msg.p0x3c.challenge, challenge_, sizeof(challenge_));
    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    packet->start_preamble = REPLY_PREAMBLE_SYMBOLS;
    send_reply_(packet);
    ESP_LOGI(TAG, "Recover System Key: sent CHALLENGE_REQ (0x3C) to %02X%02X%02X%s", hub_src[0], hub_src[1],
             hub_src[2], resend_only ? " (retry)" : "");
}

void IOHCKeyExtractor::handle_key_transfer_(iohcPacket *packet) {
    if (state_ != ResponderState::SENT_CHALLENGE)
        return;
    if (packet->buffer_length < sizeof(_header) + sizeof(_p0x32))
        return;
    const auto &src = packet->payload.packet.header.source;
    const uint8_t *transfer16 = packet->payload.packet.msg.p0x32.encrypted_key;

    // Recovering the key is the SAME xor_with_encrypted_iv() primitive this
    // bridge's own 2W passive key-sniffer already uses in reverse
    // (IOHCController2W::handle_frame()'s SniffState handling): the
    // transfer payload is AES(IV, TRANSFER_KEY) XOR real_system_key, and
    // XOR is its own inverse, so calling build_key_transfert_response()
    // with the HUB'S transfer payload in place of "system_key" recovers
    // the hub's real key instead of building a new transfer.
    // iv_from_key_init=true matches the IV laberning/home_io_control's own
    // recover_system_key_from_transfer() derives (the single 0x31 command
    // byte) - the form a Somfy/TaHoma hub is expected to use.
    uint8_t recovered[16];
    iohcCrypto::build_key_transfert_response(challenge_, transfer16, recovered, /*iv_from_key_init=*/true);
    memcpy(recovered_key_, recovered, sizeof(recovered_key_));
    state_ = ResponderState::EXTRACTED;

    auto *resp = new iohcPacket;
    forge_2w_frame(resp, throwaway_id_, src, false, true, false);
    resp->payload.packet.header.cmd = 0x33;
    resp->buffer_length = resp->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    resp->start_preamble = REPLY_PREAMBLE_SYMBOLS;
    send_reply_(resp);

    log_result_();
    ESP_LOGI(TAG,
             "Recover System Key: still listening for up to %u more seconds in case the hub verifies this "
             "device (CMD_NODE_VERIFY_REQ/0x36) - leave the switch on until it turns off on its own",
             static_cast<unsigned>(POST_EXTRACT_GRACE_MS / 1000));
    arm_grace_();
}

void IOHCKeyExtractor::handle_node_verify_req_(const IOHC::address &hub_src) {
    // Our throwaway ID went out in clear in our own 0x29/0x37, so the dst
    // check alone doesn't prove this frame came from the hub we actually
    // exchanged keys with - require the hub address captured at key-init.
    if (memcmp(hub_src, hub_node_id_, sizeof(IOHC::address)) != 0)
        return;
    if (state_ != ResponderState::EXTRACTED && state_ != ResponderState::SENT_NODE_VERIFY_RESP)
        return;
    state_ = ResponderState::SENT_NODE_VERIFY_RESP;

    auto *packet = new iohcPacket;
    forge_2w_frame(packet, throwaway_id_, hub_src, false, false, false);
    packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(IOHC::address);
    packet->payload.packet.header.cmd = 0x37;
    uint8_t *data = packet->payload.buffer + sizeof(_header);
    memcpy(data, throwaway_id_, sizeof(IOHC::address));
    packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    packet->start_preamble = REPLY_PREAMBLE_SYMBOLS;
    send_reply_(packet);
    arm_grace_();  // the hub is still progressing - push the disarm back out.
    ESP_LOGI(TAG, "Recover System Key: answered NODE_VERIFY_REQ (0x36) from %02X%02X%02X", hub_src[0], hub_src[1],
             hub_src[2]);
}

void IOHCKeyExtractor::handle_node_verify_challenge_(iohcPacket *packet) {
    const auto &src = packet->payload.packet.header.source;
    if (memcmp(src, hub_node_id_, sizeof(IOHC::address)) != 0)
        return;
    if (state_ != ResponderState::SENT_NODE_VERIFY_RESP)
        return;
    if (packet->buffer_length < sizeof(_header) + sizeof(_p0x3c))
        return;

    // Authenticate our own 0x37 (cmd + our 3-byte node ID) against the
    // hub's challenge, under the recovered key - the same primitive this
    // bridge's own 2W command path uses to answer a motor's 0x3C.
    std::vector<uint8_t> our_0x37_data(throwaway_id_, throwaway_id_ + sizeof(IOHC::address));
    uint8_t answer[6];
    iohcCrypto::build_challenge_answer(0x37, our_0x37_data, packet->payload.packet.msg.p0x3c.challenge,
                                       recovered_key_, answer);

    auto *resp = new iohcPacket;
    forge_2w_frame(resp, throwaway_id_, src, false, true, false);
    resp->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x3d);
    resp->payload.packet.header.cmd = 0x3D;
    memcpy(resp->payload.packet.msg.p0x3d.response, answer, sizeof(answer));
    resp->buffer_length = resp->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
    resp->start_preamble = REPLY_PREAMBLE_SYMBOLS;
    send_reply_(resp);
    // Deliberately stays in SENT_NODE_VERIFY_RESP (never advances) so a
    // retried 0x3C is answered the same way.
    arm_grace_();
    ESP_LOGI(TAG, "Recover System Key: answered CHALLENGE_REQ (0x3C) against our address from %02X%02X%02X",
             src[0], src[1], src[2]);
}

void IOHCKeyExtractor::arm_grace_() { grace_until_ms_ = esphome::millis() + POST_EXTRACT_GRACE_MS; }

void IOHCKeyExtractor::log_result_() {
    // Deliberate exception to this bridge's usual practice of never logging
    // key material in the clear (see IOHCController2W's own passive
    // key-sniff log line for the only other one) - the whole point of this
    // feature is to show the user this value.
    ESP_LOGW(TAG, "========================================");
    ESP_LOGW(TAG, "SYSTEM KEY RECOVERED FROM HUB %02X%02X%02X - DO NOT SHARE THIS KEY", hub_node_id_[0],
             hub_node_id_[1], hub_node_id_[2]);
    ESP_LOGW(TAG, "Anyone with this key and the hub's node id can command every device it has ever bonded.");
    ESP_LOGW(TAG, "This has not been tested against a real hub - verify it (e.g. by controlling a device");
    ESP_LOGW(TAG, "with it) before relying on it. It is logged only; nothing in this bridge adopts it on");
    ESP_LOGW(TAG, "its own.");
    ESP_LOGW(TAG, "  hub_node_id: \"%s\"", bytesToHexString(hub_node_id_, 3).c_str());
    ESP_LOGW(TAG, "  system_key:  \"%s\"", bytesToHexString(recovered_key_, 16).c_str());
    ESP_LOGW(TAG, "========================================");
}

void IOHCKeyExtractor::loop() {
    if (state_ == ResponderState::DISARMED)
        return;
    const uint32_t now = esphome::millis();

    if ((state_ == ResponderState::EXTRACTED || state_ == ResponderState::SENT_NODE_VERIFY_RESP) &&
        now >= grace_until_ms_) {
        ESP_LOGI(TAG, "Recover System Key: no further hub activity for %u s after extraction. Disarming.",
                 static_cast<unsigned>(POST_EXTRACT_GRACE_MS / 1000));
        set_armed(false);
        return;
    }
    if (now >= armed_until_ms_) {
        ESP_LOGW(TAG, "Recover System Key: 10 minute window expired (stage=%s). Disarming.", stage_name_(state_));
        set_armed(false);
        return;
    }
    if (discover_reply_pending_ && now >= discover_reply_due_ms_) {
        discover_reply_pending_ = false;
        if (state_ == ResponderState::SENT_DISCOVER_RESP) {
            send_discover_resp_now_();
        }
    }
}

}  // namespace IOHC
