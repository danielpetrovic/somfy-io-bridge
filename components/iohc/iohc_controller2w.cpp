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

#include "iohc_controller2w.h"
#include "iohc_2w_frame.h"
#include "iohcCryptoHelpers.h"
#include "cover/iohc_cover.h"
#include "iohc.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <esp_system.h>
#include <cstring>

namespace IOHC {

    static const char *const TAG = "iohc.controller2w";

    // Timeouts. 60s overall for discovery matches the real, user-facing "go
    // press the motor's pairing button now" window (same shape as the
    // existing 1W Program-button UX). The rest are generous relative to the
    // ~1-2s response times seen across every real 2W exchange captured this
    // session (io-2w-protocol.md Findings 5-14).
    static constexpr uint32_t TIMEOUT_ARMED_MS = 60000;
    static constexpr uint32_t TIMEOUT_STEP_MS = 5000;
    static constexpr uint32_t TIMEOUT_CMD_MS = 10000;

    // DISCOVER-phase resend timing (Finding 22). RX coverage across
    // CH1/CH2/CH3 is now the bridge-wide cooperative hop's job (Finding 31,
    // esphome::iohc::IOHCComponent::maybe_hop_()) - this class only manages
    // when to resend the 0x28 itself, always on CH2. DISCOVER_RESEND_MS
    // matches home_io_control/proto_timing.h's own
    // PAIRING_DISCOVERY_WAIT_MS default (2000ms) reasonably closely.
    static constexpr uint32_t DISCOVER_RESEND_MS = 2000;

    // Start preamble (SX1276 bytes) of the 0x28 broadcast. Real controllers
    // send the discovery behind about 52 preamble bytes (Somfy patent
    // EP 2 401 729 B1, confirmed by SDR measurements in laberning/
    // home_io_control's ADR 0078) and some receivers ignore it behind the
    // long wake-up preamble every other send of this bridge starts with.
    // Directed pairing start frames (0x2C, 0x31, 0x6F) use the same short start
    // preamble as the discovery. laberning/home_io_control caps every directed
    // pairing frame to the discovery preamble (pairing_start_preamble()), also
    // for low-power targets, and its default is 32 bytes on this radio (ADR
    // 0078). A frame that continues an exchange (0x32) uses the short response
    // preamble instead, about 3.5 ms (proto_timing.h RESPONSE_PREAMBLE_MS).
    static constexpr uint16_t PAIRING_START_PREAMBLE_SYMBOLS = 32;
    static constexpr uint16_t PAIRING_CONTINUATION_PREAMBLE_SYMBOLS = 16;

    // Sequence timing, from laberning/home_io_control's pairing_engine.h and
    // proto_timing.h: 1.5 s per discover-confirm try (3 tries, silence is not
    // an error), 300 ms pause before the key-init, 500 ms windows for the 0x3C
    // challenge and the 0x33 confirm, 3 tries per step, 3 rounds in total.
    static constexpr uint32_t CONFIRM_WINDOW_MS = 1500;
    static constexpr uint8_t CONFIRM_TRIES = 3;
    static constexpr uint32_t KEY_INIT_DELAY_MS = 300;
    static constexpr uint32_t KEY_WINDOW_MS = 500;
    static constexpr uint8_t KEY_STEP_TRIES = 3;
    static constexpr uint8_t KEY_ROUNDS = 3;
    static constexpr uint32_t ATTEMPT_CAP_MS = 90000;

    // Fixed SET_CONFIG1 payload - asks the device to auto-broadcast status
    // updates. Matches github.com/laberning/home_io_control's own working
    // controller captures (Finding 20). Best-effort only, not required for
    // BONDED status.
    static constexpr uint8_t SET_CONFIG1_PAYLOAD[5] = {0xE0, 0x10, 0x0A, 0x08, 0x00};

    // Listen-Before-Talk (Finding 22) - matches home_io_control's
    // proto_timing.h ETSI EN 300 220 constants exactly (-90dBm threshold,
    // 5 retries, 5ms backoff). This bridge never had any LBT before this -
    // a real gap given this session directly captured genuine RF
    // contention (real TaHoma<->motor traffic landing mid-attempt during a
    // bonding test).
    static constexpr int16_t LBT_RSSI_THRESHOLD_DBM = -90;
    static constexpr uint8_t LBT_MAX_RETRIES = 5;
    static constexpr uint32_t LBT_RETRY_DELAY_MS = 5;

    uint32_t IOHCController2W::pack_address(const IOHC::address &addr) {
        return (static_cast<uint32_t>(addr[0]) << 16) | (static_cast<uint32_t>(addr[1]) << 8) |
               static_cast<uint32_t>(addr[2]);
    }

    void IOHCController2W::begin(iohcRadio *radio, esphome::iohc::IOHCComponent *owner,
                                  const std::string &fixed_controller_hex, const std::string &fixed_system_key_hex) {
        radio_ = radio;
        owner_ = owner;
        load_or_generate_identity(fixed_controller_hex);
        load_or_generate_system_key(fixed_system_key_hex);
    }

    void IOHCController2W::load_or_generate_system_key(const std::string &fixed_system_key_hex) {
        // Same "iohc2wgw" namespace as the controller address - one shared
        // controller identity, one shared secret, both persisted together.
        if (!fixed_system_key_hex.empty()) {
            hexStringToBytes(fixed_system_key_hex, system_key_);
            ESP_LOGCONFIG(TAG, "Using fixed 2W system key from YAML");
            return;
        }

        if (identity_prefs_.isKey("syskey") && identity_prefs_.getBytesLength("syskey") == sizeof(system_key_)) {
            identity_prefs_.getBytes("syskey", system_key_, sizeof(system_key_));
            ESP_LOGCONFIG(TAG, "Loaded 2W system key");
            return;
        }

        for (uint8_t &b : system_key_) b = esp_random() & 0xff;
        identity_prefs_.putBytes("syskey", system_key_, sizeof(system_key_));
        ESP_LOGCONFIG(TAG, "Generated new 2W system key");
    }

    void IOHCController2W::load_or_generate_identity(const std::string &fixed_controller_hex) {
        // Fixed namespace, not hashed - there is only ever one controller
        // identity for the whole bridge (shared across every cover's 2W
        // bonding), unlike IOHCRemote1W's per-cover "io"+hash namespaces.
        identity_prefs_.begin("iohc2wgw", false);

        if (!fixed_controller_hex.empty()) {
            hexStringToBytes(fixed_controller_hex, controller_address_);
            ESP_LOGCONFIG(TAG, "Using fixed 2W controller identity %02X%02X%02X from YAML",
                          controller_address_[0], controller_address_[1], controller_address_[2]);
            return;
        }

        if (identity_prefs_.isKey("addr") && identity_prefs_.getBytesLength("addr") == sizeof(controller_address_)) {
            identity_prefs_.getBytes("addr", controller_address_, sizeof(controller_address_));
            ESP_LOGCONFIG(TAG, "Loaded 2W controller identity %02X%02X%02X", controller_address_[0],
                          controller_address_[1], controller_address_[2]);
            return;
        }

        for (uint8_t &b : controller_address_) b = esp_random() & 0xff;
        identity_prefs_.putBytes("addr", controller_address_, sizeof(controller_address_));
        ESP_LOGCONFIG(TAG, "Generated new 2W controller identity %02X%02X%02X", controller_address_[0],
                      controller_address_[1], controller_address_[2]);
    }

    void IOHCController2W::arm_bonding(const IOHC::address &motor_address, esphome::iohc::IOHCCover *cover) {
        static const IOHC::address zero{0, 0, 0};
        if (memcmp(motor_address, zero, sizeof(IOHC::address)) == 0) {
            ESP_LOGW(TAG, "Program (2W) pressed but this cover has no motor_address configured - refusing to arm "
                          "(see README's Real position feedback section for how to look it up). Will not listen "
                          "for just any DISCOVER broadcast.");
            return;
        }
        if (attempt_.state != BondState::IDLE) {
            ESP_LOGW(TAG, "Program (2W) already in progress for %02X%02X%02X - ignoring new request until it finishes "
                          "or times out",
                     attempt_.target_motor[0], attempt_.target_motor[1], attempt_.target_motor[2]);
            return;
        }
        memcpy(attempt_.target_motor, motor_address, sizeof(IOHC::address));
        attempt_.armed_cover = cover;
        attempt_.state = BondState::WAITING_DISCOVER_RESP;
        attempt_.state_entered_ms = esphome::millis();
        attempt_.discover_retries = 0;
        attempt_.started_ms = attempt_.state_entered_ms;
        attempt_.last_tx_ms = 0;
        attempt_.key_init_due_ms = 0;
        attempt_.target_low_power = false;
        attempt_.confirm_tries = 0;
        attempt_.key_rounds = 0;
        attempt_.key_init_sends = 0;
        attempt_.transfer_tries = 0;
        if (owner_ != nullptr) {
            // The reply to a broadcast almost never comes back on the channel
            // it was sent on, so listen on CH1 and CH3 only until a 0x29 shows up.
            owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::DISCOVERY);
            owner_->set_bonding_hop_wanted(true);
        }
        send_discover();
        attempt_.last_discover_sent_ms = esphome::millis();
        ESP_LOGI(TAG, "2W bonding with %02X%02X%02X: sent DISCOVER (0x28) on CH2, waiting up to 60s (resending "
                      "every %lus) - RX coverage across CH1/CH2/CH3 is enabled for the duration of this attempt - "
                      "make sure the motor's pairing window is already open (hold PROG on an existing paired "
                      "remote first, same as before)",
                 motor_address[0], motor_address[1], motor_address[2], DISCOVER_RESEND_MS / 1000);
    }

    void IOHCController2W::reset_bonding(BondState to_state, const char *reason) {
        ESP_LOGW(TAG, "2W bonding with %02X%02X%02X: %s", attempt_.target_motor[0], attempt_.target_motor[1],
                 attempt_.target_motor[2], reason);
        attempt_.state = to_state;
        attempt_.armed_cover = nullptr;
        if (to_state != BondState::BONDED) {
            // Explicitly does not touch sessions_ for any OTHER motor -
            // see the plan file's "never let one motor's bonding attempt
            // touch another's persisted state" principle.
            attempt_.state = BondState::IDLE;
        }
        // Release this attempt's own hop request (Finding 32) - if Debug
        // Logging is still separately requesting it, the hop stays active;
        // otherwise this drops back to CH2-only immediately.
        if (owner_ != nullptr)
            owner_->set_bonding_hop_wanted(false);
    }

    void IOHCController2W::finalize_bonding(const IOHC::address &motor) {
        Session2W &s = sessions_[pack_address(motor)];
        s.bonded = true;
        memcpy(s.key, system_key_, sizeof(system_key_));
        ESP_LOGI(TAG, "2W bonding with %02X%02X%02X succeeded - system key now shared", motor[0], motor[1],
                 motor[2]);
        send_set_config1(motor);
        attempt_.state = BondState::BONDED;
        attempt_.armed_cover = nullptr;
        if (owner_ != nullptr)
            owner_->set_bonding_hop_wanted(false);
    }

    void IOHCController2W::loop() {
        // In-flight per-command timeouts, independent of the bonding FSM
        // below - see Session2W::cmd_state. Deliberately placed BEFORE the
        // attempt_.state early-return below: this ran only while a bonding
        // attempt happened to be active until this fix (2026-07-11), since
        // attempt_.state is IDLE almost all the time (no bonding attempt in
        // progress is the normal resting state) - meaning a command's
        // cmd_state could get stuck at SENT_WAITING_CHALLENGE/
        // ANSWERED_WAITING_ACK forever, permanently refusing every further
        // command to that motor until a reboot. Confirmed on real hardware:
        // a Force Test Bond (2W) Stop command that never got a 0x3C answer
        // never timed out and blocked all further commands to that motor.
        for (auto &kv : sessions_) {
            Session2W &s = kv.second;
            if (s.cmd_state != CmdState::IDLE && (esphome::millis() - s.cmd_sent_ms) > TIMEOUT_CMD_MS) {
                ESP_LOGW(TAG, "2W command to a bonded motor timed out waiting for challenge/answer - giving up "
                              "on this attempt");
                s.cmd_state = CmdState::IDLE;
            }
        }

        // Key sniffing (Finding 31) no longer needs any loop()-driven
        // hop/timeout management - it's a pure passive watcher inside
        // handle_frame() now. RX coverage across all 3 channels comes from
        // the bridge-wide cooperative hop (esphome::iohc::IOHCComponent::maybe_hop_()).

        if (attempt_.state == BondState::IDLE || attempt_.state == BondState::BONDED) return;

        // WAITING_DISCOVER_RESP: resend 0x28 (always on CH2, via
        // send_discover() which retunes itself) every DISCOVER_RESEND_MS -
        // RX coverage across CH1/CH2/CH3 in between comes from the
        // bridge-wide cooperative hop, not anything managed here. A single
        // 0x28 could also easily be sent before the user's paired remote
        // has actually opened the motor's pairing window, hence the resend.
        if (attempt_.state == BondState::WAITING_DISCOVER_RESP) {
            uint32_t now = esphome::millis();
            if ((now - attempt_.last_discover_sent_ms) > DISCOVER_RESEND_MS) {
                attempt_.discover_retries++;
                send_discover();
                attempt_.last_discover_sent_ms = esphome::millis();
                ESP_LOGD(TAG, "2W bonding: resent DISCOVER (0x28) on CH2, attempt %u", attempt_.discover_retries + 1);
            }
        }

        const uint32_t now = esphome::millis();

        // Overall cap for the whole attempt.
        if ((now - attempt_.started_ms) > ATTEMPT_CAP_MS) {
            reset_bonding(BondState::FAILED, "timed out, resetting - press Program (2W) again to retry");
            return;
        }

        // 0x2C: up to CONFIRM_TRIES sends, CONFIRM_WINDOW_MS apart. A motor that
        // never answers with a 0x2D is not fatal, real controllers carry on.
        if (attempt_.state == BondState::WAITING_CONFIRM_ACK && (now - attempt_.last_tx_ms) >= CONFIRM_WINDOW_MS) {
            if (attempt_.confirm_tries < CONFIRM_TRIES) {
                attempt_.confirm_tries++;
                send_discover_confirmation(attempt_.target_motor, attempt_.target_low_power);
                attempt_.last_tx_ms = now;
            } else {
                ESP_LOGI(TAG, "2W bonding: no 0x2D after %u tries, continuing with the key exchange", CONFIRM_TRIES);
                attempt_.key_init_due_ms = now + KEY_INIT_DELAY_MS;
                attempt_.state = BondState::KEY_INIT_PENDING;
                attempt_.state_entered_ms = now;
            }
            return;
        }

        // Pause before a key-init round, then send the 0x31.
        if (attempt_.state == BondState::KEY_INIT_PENDING && now >= attempt_.key_init_due_ms) {
            if (attempt_.key_rounds >= KEY_ROUNDS) {
                reset_bonding(BondState::FAILED, "the key exchange did not complete in 3 rounds, resetting");
                return;
            }
            attempt_.key_rounds++;
            attempt_.key_init_sends = 1;
            send_key_init(attempt_.target_motor);
            attempt_.last_tx_ms = now;
            attempt_.state = BondState::SENT_KEY_INIT;
            attempt_.state_entered_ms = now;
            return;
        }

        // 0x31 sent: wait for the 0x3C challenge, resend up to KEY_STEP_TRIES.
        if (attempt_.state == BondState::SENT_KEY_INIT && (now - attempt_.last_tx_ms) >= KEY_WINDOW_MS) {
            if (attempt_.key_init_sends < KEY_STEP_TRIES) {
                attempt_.key_init_sends++;
                send_key_init(attempt_.target_motor);
                attempt_.last_tx_ms = now;
            } else {
                ESP_LOGW(TAG, "2W bonding: no 0x3C challenge after %u key-inits in round %u", KEY_STEP_TRIES,
                         attempt_.key_rounds);
                attempt_.key_init_due_ms = now + KEY_INIT_DELAY_MS;
                attempt_.state = BondState::KEY_INIT_PENDING;
                attempt_.state_entered_ms = now;
            }
            return;
        }

        // 0x32 sent: wait for the 0x33 confirm, resend up to KEY_STEP_TRIES, then
        // start the next round (which also switches the key-wrap IV derivation).
        if (attempt_.state == BondState::SENT_KEY_TRANSFER && (now - attempt_.last_tx_ms) >= KEY_WINDOW_MS) {
            if (attempt_.transfer_tries < KEY_STEP_TRIES) {
                attempt_.transfer_tries++;
                send_key_transfer(attempt_.target_motor, attempt_.challenge);
                attempt_.last_tx_ms = now;
            } else {
                ESP_LOGW(TAG, "2W bonding: no 0x33 after %u key transfers in round %u", KEY_STEP_TRIES,
                         attempt_.key_rounds);
                attempt_.key_init_due_ms = now + KEY_INIT_DELAY_MS;
                attempt_.state = BondState::KEY_INIT_PENDING;
                attempt_.state_entered_ms = now;
            }
            return;
        }

        // Waiting for the discovery reply: 60 s armed window.
        if (attempt_.state == BondState::WAITING_DISCOVER_RESP &&
            (now - attempt_.state_entered_ms) > TIMEOUT_ARMED_MS) {
            reset_bonding(BondState::FAILED, "timed out, resetting - press Program (2W) again to retry");
        }
    }

    bool IOHCController2W::is_bonded(const IOHC::address &motor_address) const {
        auto it = sessions_.find(pack_address(motor_address));
        return it != sessions_.end() && it->second.bonded;
    }

    void IOHCController2W::reset_sniff_state() {
        // Called after a successful capture so the state machine is ready
        // to catch another bonding sequence - no "disarm" concept anymore,
        // this watcher just always runs (Finding 31).
        sniff_.state = SniffState::IDLE;
    }

    void IOHCController2W::listen_before_talk() {
        for (uint8_t i = 0; i < LBT_MAX_RETRIES; i++) {
            float rssi = Radio::readRssi();
            if (rssi < LBT_RSSI_THRESHOLD_DBM) return;
            ESP_LOGD(TAG, "LBT: channel busy (RSSI %.1f dBm), retry %u/%u", rssi, i + 1, LBT_MAX_RETRIES);
            esphome::delay(LBT_RETRY_DELAY_MS);
        }
        // Transmit anyway after LBT_MAX_RETRIES - never block a
        // user-triggered action forever waiting for a clear channel.
    }

    // --- Frame builders ---
    // forge_2w_packet()/forge_2w_frame() now live in iohc_2w_frame.h, shared
    // with the "Recover System Key" responder (iohc_key_extraction.cpp),
    // which builds the same frame shapes from the device side.

    void IOHCController2W::send_discover() {
        // Controller-initiated broadcast to 0x00003B, 0-byte payload, a single
        // start+end frame behind the short start preamble.
        static const IOHC::address kBroadcastDiscover = {0x00, 0x00, 0x3B};
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, kBroadcastDiscover, true, true, false);
        packet->payload.packet.header.cmd = 0x28;
        packet->start_preamble = PAIRING_START_PREAMBLE_SYMBOLS;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        // Always CH2 for TX; the reply comes back on CH1 or CH3 almost every time.
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);
    }

    void IOHCController2W::send_discover_confirmation(const IOHC::address &to, bool low_power) {
        // 0-byte payload, start frame, low-power flag follows the target's
        // power class from its discovery reply, no ACK bit.
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, to, true, false, low_power);
        packet->payload.packet.header.cmd = 0x2C;
        packet->start_preamble = PAIRING_START_PREAMBLE_SYMBOLS;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);
        ESP_LOGI(TAG, "Sent DISCOVER_CONFIRMATION (0x2C) to %02X%02X%02X (low-power flag %u, try %u/%u)", to[0],
                 to[1], to[2], low_power ? 1 : 0, attempt_.confirm_tries, CONFIRM_TRIES);
    }

    void IOHCController2W::send_key_init(const IOHC::address &to) {
        // Start frame with the low-power flag set (control bytes 48 20), the
        // fixed shape real controllers use for the pairing key-init.
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, to, true, false, true);
        packet->payload.packet.header.cmd = 0x31;
        packet->start_preamble = PAIRING_START_PREAMBLE_SYMBOLS;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);
        ESP_LOGI(TAG, "Sent KEY_INIT (0x31) to %02X%02X%02X (round %u/%u, send %u/%u)", to[0], to[1], to[2],
                 attempt_.key_rounds, KEY_ROUNDS, attempt_.key_init_sends, KEY_STEP_TRIES);
    }

    void IOHCController2W::send_key_transfer(const IOHC::address &to, const uint8_t *challenge6) {
        // The IV of the key wrap is derived from the KEY_INIT byte in the first
        // round and from the challenge frame in the next one (see
        // build_key_transfert_response()), so one test covers both.
        const bool iv_from_key_init = (attempt_.key_rounds % 2) == 1;
        uint8_t response[16];
        iohcCrypto::build_key_transfert_response(challenge6, system_key_, response, iv_from_key_init);

        // Continuation frame (neither start nor end) with the low-power flag set (18 20),
        // behind the short response preamble.
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, to, false, false, true);
        packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x32);
        packet->payload.packet.header.cmd = 0x32;
        memcpy(packet->payload.packet.msg.p0x32.encrypted_key, response, 16);
        packet->start_preamble = PAIRING_CONTINUATION_PREAMBLE_SYMBOLS;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);
        ESP_LOGI(TAG, "Sent KEY_TRANSFER (0x32) to %02X%02X%02X (IV from %s, send %u/%u)", to[0], to[1], to[2],
                 iv_from_key_init ? "the key-init byte" : "the challenge frame", attempt_.transfer_tries,
                 KEY_STEP_TRIES);
    }

    void IOHCController2W::send_set_config1(const IOHC::address &to) {
        // Best-effort, no response waited for. Start frame, low-power set.
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, to, true, false, true);
        packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x6f);
        packet->payload.packet.header.cmd = 0x6F;
        memcpy(packet->payload.packet.msg.p0x6f.data, SET_CONFIG1_PAYLOAD, sizeof(SET_CONFIG1_PAYLOAD));
        packet->start_preamble = PAIRING_START_PREAMBLE_SYMBOLS;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);
        ESP_LOGI(TAG, "Sent SET_CONFIG1 (0x6F) to %02X%02X%02X (best-effort)", to[0], to[1], to[2]);
    }

    void IOHCController2W::send_command(const IOHC::address &to, uint8_t main0, uint8_t main1,
                                         esphome::iohc::IOHCCover *cover) {
        Session2W &s = sessions_[pack_address(to)];
        if (!s.bonded) {
            ESP_LOGW(TAG, "2W command to %02X%02X%02X refused - not bonded yet (Program (2W) first)", to[0], to[1],
                      to[2]);
            return;
        }
        if (s.cmd_state != CmdState::IDLE) {
            ESP_LOGW(TAG, "2W command to %02X%02X%02X still waiting on a previous command - ignoring", to[0], to[1],
                      to[2]);
            return;
        }

        auto *packet = new iohcPacket;
        forge_2w_packet(packet, controller_address_, to);
        packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x00_2w);
        packet->payload.packet.header.cmd = 0x00;
        packet->payload.packet.msg.p0x00_2w.origin = 0x01;
        setAcei(packet->payload.packet.msg.p0x00_2w.acei, 0xE7);
        packet->payload.packet.msg.p0x00_2w.main[0] = main0;
        packet->payload.packet.msg.p0x00_2w.main[1] = main1;
        packet->payload.packet.msg.p0x00_2w.reserved[0] = 0x00;
        packet->payload.packet.msg.p0x00_2w.reserved[1] = 0x00;
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        radio_->retune(CHANNEL2);
        listen_before_talk();
        radio_->send(packet);

        s.last_sent_cmd = 0x00;
        s.last_sent_data = {0x01, 0xE7, main0, main1, 0x00, 0x00};
        s.cmd_state = CmdState::SENT_WAITING_CHALLENGE;
        s.cmd_sent_ms = esphome::millis();
        s.pending_cmd_cover = cover;
        ESP_LOGI(TAG, "Sent 2W command (main=%02X%02X) to %02X%02X%02X - waiting for challenge/response", main0,
                 main1, to[0], to[1], to[2]);
    }

    bool IOHCController2W::handle_frame(IOHC::iohcPacket *packet) {
        // Passive key-sniff (Findings 27/28) - observes only, never
        // consumes/returns true, so it can never interfere with anything
        // below or with iohc.cpp's own passive-decode path, both of which
        // must keep seeing every frame regardless of whether a sniff is
        // armed. Deliberately placed first/unconditional so it sees 0x31
        // even for a source/destination pair this bridge has no session or
        // bonding attempt for at all (the whole point - this watches OTHER
        // controllers' bonding exchanges, not this bridge's own). Always
        // runs (Finding 31) - no arm/disarm anymore.
        if (!packet->payload.packet.header.CtrlByte1.asStruct.Protocol) {
            const auto &src = packet->payload.packet.header.source;
            const auto &dst = packet->payload.packet.header.target;
            uint8_t cmd = packet->payload.packet.header.cmd;

            if (sniff_.state == SniffState::IDLE && cmd == 0x31) {
                memcpy(sniff_.box, src, sizeof(IOHC::address));
                memcpy(sniff_.device, dst, sizeof(IOHC::address));
                sniff_.state = SniffState::SAW_KEY_INIT;
                ESP_LOGI(TAG, "2W key sniff: saw KEY_INIT (0x31) %02X%02X%02X -> %02X%02X%02X, watching for its "
                              "CHALLENGE_REQUEST",
                         src[0], src[1], src[2], dst[0], dst[1], dst[2]);
            } else if (sniff_.state == SniffState::SAW_KEY_INIT && cmd == 0x3C &&
                       memcmp(src, sniff_.device, sizeof(IOHC::address)) == 0 &&
                       memcmp(dst, sniff_.box, sizeof(IOHC::address)) == 0) {
                memcpy(sniff_.challenge, packet->payload.packet.msg.p0x3c.challenge, sizeof(sniff_.challenge));
                sniff_.state = SniffState::SAW_CHALLENGE;
                ESP_LOGI(TAG, "2W key sniff: saw matching CHALLENGE_REQUEST (0x3C), watching for KEY_TRANSFER "
                              "(0x32)");
            } else if (sniff_.state == SniffState::SAW_CHALLENGE && cmd == 0x32 &&
                       memcmp(src, sniff_.box, sizeof(IOHC::address)) == 0 &&
                       memcmp(dst, sniff_.device, sizeof(IOHC::address)) == 0 &&
                       packet->buffer_length >= sizeof(_header) + sizeof(_p0x32)) {
                // Same primitive used to BUILD a real KEY_TRANSFER payload
                // (build_key_transfert_response()) - XOR is symmetric, so
                // calling it with the captured ciphertext in place of "our
                // own system_key" recovers the real system_key instead. See
                // that function's own doc comment (Finding 28).
                uint8_t derived_key[16];
                iohcCrypto::build_key_transfert_response(sniff_.challenge, packet->payload.packet.msg.p0x32.encrypted_key,
                                                          derived_key);
                std::string hex = bytesToHexString(derived_key, 16);
                ESP_LOGW(TAG, "2W key sniff SUCCESS: derived real system_key from %02X%02X%02X's bonding with "
                              "%02X%02X%02X: %s",
                         sniff_.box[0], sniff_.box[1], sniff_.box[2], sniff_.device[0], sniff_.device[1],
                         sniff_.device[2], hex.c_str());
                reset_sniff_state();
            }
        }

        // Per-command challenge/response for already-bonded motors (Phase
        // 3d) - independent of the bonding attempt_ FSM below, since this
        // fires for ordinary Open/Close/Position/Stop commands sent long
        // after bonding completed, not during an active bonding attempt.
        {
            const auto &src = packet->payload.packet.header.source;
            auto it = sessions_.find(pack_address(src));
            if (it != sessions_.end() && it->second.bonded && it->second.cmd_state != CmdState::IDLE) {
                Session2W &s = it->second;
                uint8_t cmd = packet->payload.packet.header.cmd;

                if (s.cmd_state == CmdState::SENT_WAITING_CHALLENGE && cmd == 0x3C) {
                    uint8_t answer[6];
                    iohcCrypto::build_challenge_answer(s.last_sent_cmd, s.last_sent_data,
                                                        packet->payload.packet.msg.p0x3c.challenge, s.key, answer);
                    auto *reply = new iohcPacket;
                    forge_2w_packet(reply, controller_address_, src);
                    reply->payload.packet.header.CtrlByte1.asStruct.MsgLen += sizeof(_p0x3d);
                    reply->payload.packet.header.cmd = 0x3D;
                    memcpy(reply->payload.packet.msg.p0x3d.response, answer, 6);
                    reply->buffer_length = reply->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
                    radio_->retune(CHANNEL2);
                    listen_before_talk();
                    radio_->send(reply);
                    s.cmd_state = CmdState::ANSWERED_WAITING_ACK;
                    s.cmd_sent_ms = esphome::millis();
                    ESP_LOGD(TAG, "Sent CHALLENGE_ANSWER (0x3D) for in-flight command to %02X%02X%02X", src[0],
                              src[1], src[2]);
                    return true;
                }

                if (s.cmd_state == CmdState::ANSWERED_WAITING_ACK && s.last_sent_cmd == 0x00 && cmd == 0x04 &&
                    packet->buffer_length >= 13 && packet->buffer_length <= MAX_FRAME_LEN) {
                    s.cmd_state = CmdState::IDLE;
                    uint16_t raw = (static_cast<uint16_t>(packet->payload.buffer[11]) << 8) |
                                   packet->payload.buffer[12];
                    esphome::iohc::IOHCCover *cover = s.pending_cmd_cover;
                    s.pending_cmd_cover = nullptr;
                    // Only 0x0000-0xC800 is a real closure percentage - a
                    // Stop command (main=0xD200) echoes back above that
                    // range and is not a position (same bug/fix as iohc.cpp's
                    // passive decode path, see its comment for the full
                    // story). Confirm the command completed either way, just
                    // don't feed a bogus value into the cover's real state.
                    ESP_LOGI(TAG, "2W command to %02X%02X%02X confirmed", src[0], src[1], src[2]);
                    if (raw <= 0xC800 && cover != nullptr) {
                        float closure_percent = raw / 512.0f;
                        ESP_LOGI(TAG, "  real closure=%.0f%%", closure_percent);
                        cover->update_real_position_authoritative(closure_percent);
                    }
                    return true;
                }
            }
        }

        // Only ever consume frames while a bonding attempt is in progress,
        // and only from the exact motor that attempt is scoped to - never
        // touch anything else, so this never interferes with the existing,
        // unrelated passive-decode Tier 1 feature (iohc.cpp's on_receive()
        // still gets first look at everything, this is called after/instead
        // only for the bonding-family commands it explicitly routes here).
        if (!bonding_active()) return false;

        const auto &source = packet->payload.packet.header.source;
        const auto &target = packet->payload.packet.header.target;
        uint8_t cmd = packet->payload.packet.header.cmd;

        // While a bonding attempt waits, every frame heard is logged with its
        // addresses, so a reply that arrives from an unexpected address or on
        // an unexpected channel is visible instead of silently dropped.
        ESP_LOGI(TAG, "Bonding RX: cmd=0x%02X %02X%02X%02X -> %02X%02X%02X, %s, len=%u, %.1f dBm, %lu Hz", cmd,
                 source[0], source[1], source[2], target[0], target[1], target[2],
                 packet->payload.packet.header.CtrlByte1.asStruct.Protocol ? "1W" : "2W", packet->buffer_length,
                 packet->rssi, static_cast<unsigned long>(packet->frequency));

        // The discovery reply is accepted from whichever motor answers: the
        // configured motor address may not be what the motor reports after a
        // reset, and a mismatch used to drop the reply without a trace. The
        // rest of the exchange then uses the address that actually answered.
        // Only a motor in its own pairing window answers a 0x28, so this
        // cannot pick up an unrelated shutter.
        if (attempt_.state == BondState::WAITING_DISCOVER_RESP && cmd == 0x29 &&
            !packet->payload.packet.header.CtrlByte1.asStruct.Protocol &&
            memcmp(source, attempt_.target_motor, sizeof(IOHC::address)) != 0) {
            ESP_LOGW(TAG, "DISCOVER_RESP (0x29) came from %02X%02X%02X, not the configured %02X%02X%02X - "
                          "continuing with the address that answered",
                     source[0], source[1], source[2], attempt_.target_motor[0], attempt_.target_motor[1],
                     attempt_.target_motor[2]);
            memcpy(attempt_.target_motor, source, sizeof(IOHC::address));
        }

        if (memcmp(source, attempt_.target_motor, sizeof(IOHC::address)) != 0) return false;

        const uint8_t *data = packet->payload.buffer + sizeof(_header);
        const uint8_t data_len = packet->buffer_length > sizeof(_header)
                                     ? static_cast<uint8_t>(packet->buffer_length - sizeof(_header))
                                     : 0;
        const uint32_t now = esphome::millis();

        // A refusal from the motor ends the attempt with its reason.
        if (cmd == 0xFE && attempt_.state != BondState::WAITING_DISCOVER_RESP) {
            ESP_LOGW(TAG, "2W bonding: the motor answered with an error 0x%02X in state %u", data_len > 0 ? data[0] : 0,
                     static_cast<unsigned>(attempt_.state));
            return true;
        }

        switch (attempt_.state) {
            case BondState::WAITING_DISCOVER_RESP:
                if (cmd != 0x29) return false;
                // The reply carries type, manufacturer and the Multi Information
                // Byte whose low bits give the power class (1 = low power).
                // Power class decides the low-power flag of the 0x2C.
                if (data_len >= 7) {
                    const uint8_t flags = data[6];
                    attempt_.target_low_power = (flags & 0x03) == 1;
                    ESP_LOGI(TAG, "DISCOVER_RESP (0x29): manufacturer=%u flags=0x%02X power class=%s, addressed to "
                                  "%02X%02X%02X (this controller is %02X%02X%02X)",
                             data[5], flags, attempt_.target_low_power ? "low power" : "always listening", target[0],
                             target[1], target[2], controller_address_[0], controller_address_[1],
                             controller_address_[2]);
                }
                ESP_LOGI(TAG, "Received DISCOVER_RESP (0x29) from %02X%02X%02X - sending DISCOVER_CONFIRMATION",
                         source[0], source[1], source[2]);
                // Every unicast pairing reply comes back on the request channel (CH2).
                if (owner_ != nullptr)
                    owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::HOLD_REQUEST);
                attempt_.confirm_tries = 1;
                send_discover_confirmation(source, attempt_.target_low_power);
                attempt_.last_tx_ms = now;
                attempt_.state = BondState::WAITING_CONFIRM_ACK;
                attempt_.state_entered_ms = now;
                return true;

            case BondState::WAITING_CONFIRM_ACK:
                if (cmd != 0x2D) return false;
                ESP_LOGI(TAG, "Received DISCOVER_CONFIRMATION_ACK (0x2D) from %02X%02X%02X - key-init follows after "
                              "%u ms",
                         source[0], source[1], source[2], static_cast<unsigned>(KEY_INIT_DELAY_MS));
                attempt_.key_init_due_ms = now + KEY_INIT_DELAY_MS;
                attempt_.state = BondState::KEY_INIT_PENDING;
                attempt_.state_entered_ms = now;
                return true;

            case BondState::KEY_INIT_PENDING:
                // A late 0x2D (or a 0x33 that crossed our 0x32) in the pause is harmless.
                if (cmd == 0x33) {
                    finalize_bonding(source);
                    return true;
                }
                return false;

            case BondState::SENT_KEY_INIT:
                // Most devices challenge with 0x3C; some accept immediately
                // with a direct 0x33. Accept either.
                if (cmd == 0x33) {
                    finalize_bonding(source);
                    return true;
                }
                if (cmd != 0x3C || data_len != 6) return false;
                ESP_LOGI(TAG, "Received CHALLENGE_REQ (0x3C) - sending KEY_TRANSFER");
                memcpy(attempt_.challenge, data, sizeof(attempt_.challenge));
                attempt_.transfer_tries = 1;
                send_key_transfer(source, attempt_.challenge);
                attempt_.last_tx_ms = now;
                attempt_.state = BondState::SENT_KEY_TRANSFER;
                attempt_.state_entered_ms = now;
                return true;

            case BondState::SENT_KEY_TRANSFER:
                if (cmd == 0x3C) {
                    ESP_LOGW(TAG, "2W bonding: the motor challenged the key transfer with a second 0x3C (a 0x3D "
                                  "answer is not implemented for bonding yet)");
                    return true;
                }
                if (cmd != 0x33) return false;
                finalize_bonding(source);
                return true;

            default:
                return false;
        }
    }
}
