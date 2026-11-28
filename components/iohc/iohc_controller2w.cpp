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
#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <cstring>

namespace IOHC {

    static const char *const TAG = "iohc.controller2w";

    // --- laberning/home_io_control constants (proto_timing.h, pairing_engine.h, roll_call.h) ---

    // Pairing.
    static constexpr uint32_t PAIRING_DISCOVERY_INITIAL_DWELL_MS = 300;
    static constexpr uint32_t PAIRING_DISCOVERY_WAIT_MS = 2000;
    static constexpr uint8_t PAIRING_DISCOVERY_MAX_ATTEMPTS = 3;
    static constexpr uint8_t PAIRING_DISCOVER_CONFIRM_TRIES = 3;
    static constexpr uint32_t PAIRING_DISCOVER_CONFIRM_ACK_TIMEOUT_MS = 1500;
    static constexpr uint32_t PAIRING_KEY_INIT_DELAY_MS = 300;
    static constexpr uint32_t PAIRING_KEY_CHALLENGE_TIMEOUT_MS = 500;
    static constexpr uint8_t PAIRING_KEY_EXCHANGE_RETRIES = 3;
    static constexpr uint8_t PAIRING_KEY_VERIFY_MAX_TRIES = 2;
    static constexpr uint8_t PAIRING_SET_CONFIG1_MAX_TRIES = 1;

    // Exchange engine.
    static constexpr uint8_t EXCHANGE_RETRY_COUNT = 3;
    static constexpr uint8_t EXCHANGE_MAX_ATTEMPTS = 9;
    static constexpr uint32_t RESPONSE_START_WAIT_MS = 400;   // first reply to a START frame
    static constexpr uint32_t RESPONSE_WAIT_MS = 500;         // first reply to a continuation; the final reply
    static constexpr uint32_t EXCHANGE_EARLY_RESPONSE_WAIT_MS = 200;
    // A blocking listen returns as soon as a frame arrives; with none, it
    // wakes this often to run the receiver's hop (5 ms slices on a rotating
    // listen), queued sends and the watchdog.
    static constexpr uint32_t EXCHANGE_LISTEN_SLICE_MS = 1;
    // Upper bound for one 0x72 copy to leave the radio (about 8 ms of air time).
    static constexpr uint32_t STATUS_UPDATE_ACK_TX_WAIT_MS = 100;
    static constexpr uint32_t EXCHANGE_RETRY_DELAY_MS = 250;
    static constexpr uint32_t EXCHANGE_TOTAL_BUDGET_MS = 2500;
    static constexpr uint32_t EXCHANGE_ATTEMPT_OVERHEAD_MS = 10;
    static constexpr uint32_t LOW_POWER_RESPONSE_WAIT_MS = 550;
    static constexpr uint32_t LOW_POWER_SHORT_LEAD_WINDOW_MS = 100;
    static constexpr uint8_t LOW_POWER_SETTLING_ATTEMPTS = 3;
    static constexpr uint32_t LOW_POWER_MAX_TRAVEL_MS = 120000;
    static constexpr uint8_t UNCONFIRMED_EXECUTE_MAX_RESENDS = 1;
    static constexpr uint32_t UNCONFIRMED_EXECUTE_RESEND_DELAY_MS = 750;
    static constexpr uint8_t SCHEDULED_POLL_MAX_TRIES = 1;
    static constexpr uint8_t SCHEDULED_POLL_RETRY_GRACE_FIRST_FAILURE = 1;
    static constexpr uint8_t SCHEDULED_POLL_RETRY_GRACE_LAST_FAILURE = 3;

    // Status polling (status_poll_policy.h).
    static constexpr uint32_t DEFAULT_SETTLE_POLL_DELAY_MS = 3000;
    static constexpr uint32_t STOP_SETTLE_POLL_CAP_MS = 1000;
    static constexpr uint32_t MAX_TRACKED_STATUS_POLL_WINDOW_MS = 240000;
    // The entity's initial status request (INITIAL_STATUS_REQUEST_DELAY_MS).
    static constexpr uint32_t INITIAL_STATUS_REQUEST_DELAY_MS = 5000;
    // Status refresh at rest (periodic_poll_policy.h): first refresh 15 to 45 s
    // after boot, then the interval with up to 10 % either way.
    static constexpr uint32_t PERIODIC_POLL_INITIAL_DELAY_MS = 15000;
    static constexpr uint32_t PERIODIC_POLL_INITIAL_SPREAD_MS = 30000;
    static constexpr uint32_t PERIODIC_POLL_JITTER_PERCENT = 10;
    static constexpr uint32_t PREDICTED_END_MARGIN_MS = 1000;
    static constexpr uint32_t PRIVATE_RESPONSE_REMAINING_TIME_BIAS_MS = 1000;
    static constexpr std::array<uint32_t, 5> SILENT_RETRY_LADDER_MS{5000, 15000, 30000, 60000, 300000};
    static constexpr std::array<uint32_t, 4> AUTH_RETRY_LADDER_MS{5000, 30000, 120000, 300000};

    // Channel access (decisions_channel.h).
    static constexpr int16_t CHANNEL_LIMIT_UNTIL_QUIET_LEVEL_DBM = -85;
    static constexpr int16_t BUSY_LIMIT_FLOOR_DBM = -90;
    static constexpr int16_t BUSY_LIMIT_CEILING_DBM = -75;
    static constexpr int16_t QUIET_LEVEL_MARGIN_DB = 19;
    static constexpr int16_t BUSY_LIMIT_LOG_STEP_DB = 3;
    static constexpr uint16_t QUIET_LEVEL_MIN_READINGS = 200;
    static constexpr uint16_t QUIET_LEVEL_HALVING_READINGS = 4096;
    static constexpr uint32_t CHANNEL_ACCESS_BUDGET_MS = 100;
    static constexpr uint32_t LOW_POWER_CHANNEL_ACCESS_BUDGET_MS = 600;
    static constexpr std::array<uint8_t, 5> BACKOFF_CEILINGS_MS = {4, 8, 16, 32, 40};
    static constexpr uint8_t CHANNEL_CHECK_READINGS = 5;
    static constexpr uint32_t CHANNEL_CHECK_READING_GAP_MS = 1;
    static constexpr uint8_t LBT_MAX_RETRIES = 5;
    static constexpr uint32_t LBT_RETRY_DELAY_MS = 5;
    static constexpr std::array<uint32_t, 3> REQUEST_CHANNELS = {CHANNEL2, CHANNEL1, CHANNEL3};

    // Preambles, set as durations in laberning's component and converted to
    // programmed SX1276 bytes at the IoHomeOn coder's 10 bits per byte and
    // 38400 bit/s (preamble_bytes_for_ms(), rounded): START_PREAMBLE_MS 10 ms,
    // RESPONSE_PREAMBLE_MS 3.5 ms, LOW_POWER_WAKE_MS 515 ms.
    static constexpr uint16_t START_PREAMBLE_BYTES = 38;
    static constexpr uint16_t RESPONSE_PREAMBLE_BYTES = 13;
    static constexpr uint16_t LOW_POWER_WAKE_PREAMBLE_BYTES = 1978;
    static constexpr uint32_t LINE_RATE_BPS = 38400;
    static constexpr uint32_t BITS_PER_PREAMBLE_BYTE = 10;

    static constexpr uint8_t CMD_EXECUTE = 0x00;
    static constexpr uint8_t CMD_PRIVATE = 0x03;
    static constexpr uint8_t CMD_PRIVATE_RESP = 0x04;
    static constexpr uint8_t CMD_DISCOVER_REQ = 0x28;
    static constexpr uint8_t CMD_DISCOVER_RESP = 0x29;
    static constexpr uint8_t CMD_DISCOVER_SPE_REQ = 0x2A;
    static constexpr uint8_t CMD_DISCOVER_SPE_RESP = 0x2B;
    static constexpr uint8_t CMD_DISCOVER_CONFIRM = 0x2C;
    static constexpr uint8_t CMD_DISCOVER_CONFIRM_ACK = 0x2D;
    static constexpr uint8_t CMD_KEY_INIT = 0x31;
    static constexpr uint8_t CMD_KEY_TRANSFER = 0x32;
    static constexpr uint8_t CMD_KEY_CONFIRM = 0x33;
    static constexpr uint8_t CMD_CHALLENGE_REQ = 0x3C;
    static constexpr uint8_t CMD_CHALLENGE_RESP = 0x3D;
    static constexpr uint8_t CMD_GET_NAME = 0x50;
    static constexpr uint8_t CMD_GET_NAME_RESP = 0x51;
    static constexpr uint8_t CMD_SET_CONFIG1 = 0x6F;
    static constexpr uint8_t CMD_ERROR_RESP = 0xFE;

    static constexpr uint8_t POS_STOP = 0xD2;
    static constexpr uint8_t POS_FAVORITE = 0xD8;
    static constexpr uint16_t STATUS_POS_MAX = 0xC800;
    // EXECUTE: originator "user remote" and ACEI (level user default << 5 |
    // extended << 1 | valid) = 0x63 (proto_commands.cpp EXECUTE_ACEI).
    static constexpr uint8_t EXECUTE_ORIGINATOR = 0x01;
    static constexpr uint8_t EXECUTE_ACEI = 0x63;
    static constexpr uint8_t PRIVATE_GET_POSITION_STATUS = 0x03;

    static constexpr IOHC::address BROADCAST_DISCOVER = {0x00, 0x00, 0x3B};
    static constexpr uint8_t SET_CONFIG1_PAYLOAD[5] = {0xE0, 0x10, 0x0A, 0x08, 0x00};
    static constexpr uint8_t DISCOVERY_RESP_FLAGS_OFFSET = 6;
    static constexpr uint8_t DISCOVERY_FLAGS_POWER_SAVE_MASK = 0x03;
    static constexpr uint8_t POWER_SAVE_LOW_POWER = 1;
    // 0x04 answer to a status read: state byte, then target and current
    // position (hub_status.cpp PRIVATE_RESPONSE_*), remaining seconds.
    static constexpr uint8_t PRIVATE_RESPONSE_TARGET_OFFSET = 2;
    static constexpr uint8_t PRIVATE_RESPONSE_CURRENT_OFFSET = 4;
    static constexpr uint8_t PRIVATE_RESPONSE_REMAINING_TIME_OFFSET = 6;
    static constexpr uint8_t PRIVATE_RESPONSE_REMAINING_TIME_MIN_DATA_LEN = 8;
    static constexpr uint16_t PRIVATE_RESPONSE_REMAINING_TIME_NONE = 0xFFFF;
    static constexpr uint8_t STATUS_STOPPED = 0x01;
    // 0x71 status update (hub_status.cpp, proto_codecs.h): event id at 2, a
    // position update carries EVENT_STATUS_CHANGE with target and current.
    static constexpr uint8_t CMD_STATUS_UPDATE = 0x71;
    static constexpr uint8_t CMD_STATUS_UPDATE_RESP = 0x72;
    static constexpr uint8_t STATUS_UPDATE_ACK_PAYLOAD[2] = {0x05, 0x00};
    static constexpr uint8_t STATUS_UPDATE_MIN_DATA_LEN = 11;
    static constexpr uint8_t STATUS_UPDATE_EVENT_ID_OFFSET = 2;
    static constexpr uint16_t EVENT_STATUS_CHANGE = 0x100A;
    static constexpr uint8_t STATUS_UPDATE_TARGET_OFFSET = 5;
    static constexpr uint8_t STATUS_UPDATE_CURRENT_OFFSET = 7;

    static constexpr uint32_t SCAN_CHANNELS[3] = {CHANNEL2, CHANNEL1, CHANNEL3};
    static constexpr uint8_t ROLL_CALL_ATTEMPTS = 6;  // low-power pass, then always-alive, 3 channels each

    static const char *addr_str(const IOHC::address &a, char *buf) {
        snprintf(buf, 7, "%02X%02X%02X", a[0], a[1], a[2]);
        return buf;
    }

    static bool time_reached(uint32_t now, uint32_t due) { return static_cast<int32_t>(now - due) >= 0; }

    // Execution state, bits [2:0] of a status byte (proto_device_model.h):
    // 2/3 waiting, 4 executing are moving; 0, 1, 5 are at rest; 6/7 undefined.
    static bool reads_stopped_by_status_byte(uint8_t status_byte) {
        const uint8_t state = status_byte & 0x07;
        if (state > 5)
            return (status_byte & STATUS_STOPPED) != 0;
        return !(state == 2 || state == 3 || state == 4);
    }

    // laberning's status_poll_interval at its default (3 s), shortened by the
    // motor's own remaining-time count and capped after a stop.
    static uint32_t settle_delay_ms(uint32_t remaining_delay_ms, bool cap_for_stop) {
        uint32_t delay = DEFAULT_SETTLE_POLL_DELAY_MS;
        if (remaining_delay_ms != 0)
            delay = std::min(delay, remaining_delay_ms);
        if (cap_for_stop)
            delay = std::min(delay, STOP_SETTLE_POLL_CAP_MS);
        return delay;
    }

    static uint32_t periodic_poll_first_delay_ms(uint32_t random) {
        return PERIODIC_POLL_INITIAL_DELAY_MS + (random % (PERIODIC_POLL_INITIAL_SPREAD_MS + 1));
    }

    static uint32_t periodic_poll_jittered_interval_ms(uint32_t interval_ms, uint32_t random) {
        const uint64_t jitter = static_cast<uint64_t>(interval_ms) * PERIODIC_POLL_JITTER_PERCENT / 100;
        const uint64_t offset = (static_cast<uint64_t>(random) * 2 * jitter) / UINT32_MAX;  // 0 .. 2 * jitter
        return static_cast<uint32_t>(interval_ms - jitter + offset);
    }

    template<size_t N> static uint32_t ladder_step(const std::array<uint32_t, N> &ladder, uint8_t failures) {
        const size_t index = failures <= 1 ? 0 : failures - 1;
        return ladder[std::min(index, N - 1)];
    }

    // --- ChannelQuietLevel ---

    void ChannelQuietLevel::Channel::halve() {
        count = 0;
        for (uint16_t &bin : bins) {
            bin = static_cast<uint16_t>(bin / 2);
            count = static_cast<uint16_t>(count + bin);
        }
    }

    std::optional<size_t> ChannelQuietLevel::channel_index(uint32_t freq_hz) {
        for (size_t i = 0; i < REQUEST_CHANNELS.size(); i++) {
            if (REQUEST_CHANNELS[i] == freq_hz) return i;
        }
        return std::nullopt;
    }

    void ChannelQuietLevel::add_reading(uint32_t freq_hz, int16_t level_dbm) {
        const auto index = channel_index(freq_hz);
        if (!index.has_value()) return;
        Channel &channel = channels_[*index];
        const int16_t clamped = std::clamp(level_dbm, MIN_DBM, MAX_DBM);
        channel.bins[static_cast<size_t>(clamped - MIN_DBM)]++;
        channel.count++;
        if (channel.count >= QUIET_LEVEL_HALVING_READINGS) channel.halve();
    }

    std::optional<int16_t> ChannelQuietLevel::quiet_level_dbm(uint32_t freq_hz) const {
        const auto index = channel_index(freq_hz);
        if (!index.has_value()) return std::nullopt;
        const Channel &channel = channels_[*index];
        if (channel.count < QUIET_LEVEL_MIN_READINGS) return std::nullopt;
        const uint32_t middle = (channel.count + 1) / 2;
        uint32_t seen = 0;
        for (size_t bin = 0; bin < channel.bins.size(); bin++) {
            seen += channel.bins[bin];
            if (seen >= middle) return static_cast<int16_t>(MIN_DBM + static_cast<int16_t>(bin));
        }
        return std::nullopt;
    }

    std::optional<int16_t> ChannelQuietLevel::board_quiet_level_dbm() const {
        std::optional<int16_t> lowest;
        for (const uint32_t freq_hz : REQUEST_CHANNELS) {
            const auto level = quiet_level_dbm(freq_hz);
            if (!level.has_value()) return std::nullopt;
            if (!lowest.has_value() || *level < *lowest) lowest = level;
        }
        return lowest;
    }

    // --- Identity ---

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
        identity_prefs_.begin("iohc2wgw", false);
        if (!fixed_controller_hex.empty()) {
            hexStringToBytes(fixed_controller_hex, controller_address_);
            ESP_LOGCONFIG(TAG, "Using fixed 2W controller identity %02X%02X%02X from YAML", controller_address_[0],
                          controller_address_[1], controller_address_[2]);
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

    void IOHCController2W::reset_sniff_state() { sniff_.state = SniffState::IDLE; }

    // --- Channel access and transmission ---

    void IOHCController2W::note_idle_reading(uint32_t freq_hz, int16_t level_dbm) {
        quiet_level_.add_reading(freq_hz, level_dbm);
        const auto quiet = quiet_level_.board_quiet_level_dbm();
        if (!quiet.has_value()) return;
        const int16_t limit = busy_limit_dbm_();
        if (logged_busy_limit_dbm_ == 0 || std::abs(limit - logged_busy_limit_dbm_) >= BUSY_LIMIT_LOG_STEP_DB) {
            logged_busy_limit_dbm_ = limit;
            ESP_LOGI(TAG, "Busy channel limit %d dBm, learned from this board's quiet level (%d dBm)", limit, *quiet);
        }
    }

    int16_t IOHCController2W::busy_limit_dbm_() const {
        const auto quiet = quiet_level_.board_quiet_level_dbm();
        if (!quiet.has_value()) return CHANNEL_LIMIT_UNTIL_QUIET_LEVEL_DBM;
        return std::clamp(static_cast<int16_t>(*quiet + QUIET_LEVEL_MARGIN_DB), BUSY_LIMIT_FLOOR_DBM,
                          BUSY_LIMIT_CEILING_DBM);
    }

    int16_t IOHCController2W::measure_channel_level_(uint32_t freq) {
        radio_->retune(freq);
        esphome::delayMicroseconds(500);  // RSSI settles after the PLL retune
        return static_cast<int16_t>(Radio::readRssi());
    }

    uint32_t IOHCController2W::draw_request_channel_(bool wake_up_channels, uint32_t avoid_freq) {
        if (wake_up_channels) return CHANNEL2;
        std::array<uint32_t, 3> candidates{};
        size_t count = 0;
        for (const uint32_t freq : REQUEST_CHANNELS) {
            if (freq != avoid_freq) candidates[count++] = freq;
        }
        return count == 0 ? CHANNEL2 : candidates[esp_random() % count];
    }

    uint32_t IOHCController2W::tx_time_ms_(uint16_t preamble, uint8_t frame_len) {
        // Preamble, 3 sync bytes, the frame and its 2 CRC bytes, 10 bits per byte.
        const uint32_t bits = (static_cast<uint32_t>(preamble) + 3 + frame_len + 2) * BITS_PER_PREAMBLE_BYTE;
        return (bits * 1000 + LINE_RATE_BPS - 1) / LINE_RATE_BPS;
    }

    void IOHCController2W::listen_on_(uint32_t freq) {
        if (owner_ == nullptr) return;
        if (freq == CHANNEL2)
            owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::HOLD_REQUEST);
        else
            owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::HOLD_FREQ, freq);
    }

    IOHC::iohcPacket *IOHCController2W::build_request_(const IOHC::address &dst, uint8_t cmd, const uint8_t *data,
                                                       uint8_t data_len, bool start, bool end, bool low_power) const {
        auto *packet = new iohcPacket;
        forge_2w_frame(packet, controller_address_, dst, start, end, low_power);
        packet->payload.packet.header.cmd = cmd;
        if (data_len > 0) {
            packet->payload.packet.header.CtrlByte1.asStruct.MsgLen += data_len;
            memcpy(packet->payload.buffer + sizeof(_header), data, data_len);
        }
        packet->buffer_length = packet->payload.packet.header.CtrlByte1.asStruct.MsgLen + 1;
        // Every 2W frame of this class is a single frame on its own timing.
        packet->skip_send_gap = true;
        return packet;
    }

    void IOHCController2W::send_at_once_(iohcPacket *packet, uint32_t freq, uint16_t preamble) {
        packet->frequency = freq;
        packet->start_preamble = preamble;
        radio_->retune(freq);
        radio_->send(packet);
    }

    void IOHCController2W::send_fixed_channel_(iohcPacket *packet, uint32_t freq, uint16_t preamble) {
        packet->frequency = freq;
        packet->start_preamble = preamble;
        const int16_t limit = busy_limit_dbm_();
        for (uint8_t lbt = 0; lbt < LBT_MAX_RETRIES; lbt++) {
            const int16_t level = measure_channel_level_(freq);
            if (level < limit) break;
            ESP_LOGD(TAG, "LBT: channel busy (RSSI %d dBm on %lu Hz), retry %u/%u", level,
                     static_cast<unsigned long>(freq), lbt + 1, LBT_MAX_RETRIES);
            esphome::delay(LBT_RETRY_DELAY_MS);
        }
        radio_->retune(freq);
        radio_->send(packet);
    }

    IOHCController2W::ChannelAccessStatus IOHCController2W::send_with_channel_access_(iohcPacket *packet,
                                                                                      uint16_t preamble,
                                                                                      bool wake_up_channels,
                                                                                      uint32_t previous_freq,
                                                                                      uint32_t &out_freq) {
        uint32_t freq = draw_request_channel_(wake_up_channels, previous_freq);
        const int16_t limit = busy_limit_dbm_();
        const uint32_t budget_ms = wake_up_channels ? LOW_POWER_CHANNEL_ACCESS_BUDGET_MS : CHANNEL_ACCESS_BUDGET_MS;
        const uint32_t begin_ms = esphome::millis();
        for (uint8_t busy_checks = 0;; busy_checks++) {
            bool busy = false;
            int16_t level = 0;
            for (uint8_t reading = 0; reading < CHANNEL_CHECK_READINGS; reading++) {
                level = measure_channel_level_(freq);
                if (level >= limit) {
                    busy = true;
                    break;
                }
                if (reading + 1 < CHANNEL_CHECK_READINGS) esphome::delay(CHANNEL_CHECK_READING_GAP_MS);
            }
            if (!busy) {
                packet->frequency = freq;
                packet->start_preamble = preamble;
                radio_->send(packet);
                out_freq = freq;
                return ChannelAccessStatus::SENT;
            }
            ESP_LOGD(TAG, "Channel busy (RSSI %d dBm on %lu Hz), check %u", level, static_cast<unsigned long>(freq),
                     busy_checks + 1);
            const uint32_t elapsed_ms = esphome::millis() - begin_ms;
            if (elapsed_ms >= budget_ms) break;
            const uint8_t ceiling =
                BACKOFF_CEILINGS_MS[std::min<size_t>(busy_checks, BACKOFF_CEILINGS_MS.size() - 1)];
            const uint32_t drawn_ms = 1 + (esp_random() % ceiling);
            // The back-off is spent listening on the busy channel.
            esphome::delay(std::min(drawn_ms, budget_ms - elapsed_ms));
            freq = draw_request_channel_(wake_up_channels, freq);
        }
        delete packet;
        out_freq = freq;
        return ChannelAccessStatus::CHANNEL_BUSY;
    }

    bool IOHCController2W::from_device_to_us_(const IOHC::iohcPacket *packet, const IOHC::address &device) const {
        return !packet->payload.packet.header.CtrlByte1.asStruct.Protocol &&
               memcmp(packet->payload.packet.header.source, device, sizeof(IOHC::address)) == 0 &&
               memcmp(packet->payload.packet.header.target, controller_address_, sizeof(IOHC::address)) == 0;
    }

    // --- Attempt plans (decisions_attempts.h) ---

    uint16_t IOHCController2W::AttemptPlan::window_for_attempt(uint8_t attempt, uint8_t attempts) const {
        if (short_lead && attempt == 1) return short_lead_window_ms;
        return (attempt < attempts && early_window_ms != 0) ? early_window_ms : window_ms;
    }

    static uint8_t attempts_that_fit(uint32_t budget_ms, uint32_t attempt_tx_ms, uint32_t early_window_ms,
                                     uint32_t last_window_ms, uint8_t cap) {
        const uint32_t early_ms = attempt_tx_ms + EXCHANGE_ATTEMPT_OVERHEAD_MS + early_window_ms;
        const uint32_t last_ms = attempt_tx_ms + EXCHANGE_ATTEMPT_OVERHEAD_MS + last_window_ms;
        if (budget_ms == 0 || early_ms == 0) return cap;
        if (last_ms >= budget_ms) return 1;
        const uint32_t fitting = 1 + ((budget_ms - 1 - last_ms) / early_ms);
        return static_cast<uint8_t>(std::max<uint32_t>(1, std::min<uint32_t>(fitting, cap)));
    }

    IOHCController2W::AttemptPlan IOHCController2W::count_plan_(uint8_t tries, bool start_frame) const {
        AttemptPlan plan;
        plan.max_attempts = std::max<uint8_t>(1, std::min<uint8_t>(tries, EXCHANGE_RETRY_COUNT));
        plan.gap_ms = EXCHANGE_RETRY_DELAY_MS;
        plan.window_ms = start_frame ? RESPONSE_START_WAIT_MS : RESPONSE_WAIT_MS;
        return plan;
    }

    IOHCController2W::AttemptPlan IOHCController2W::command_plan_(bool low_power, bool repeat_safe,
                                                                  uint32_t tx_ms) const {
        // A stored-position command (favorite) keeps the plain three attempts.
        if (!repeat_safe) {
            AttemptPlan plan = count_plan_(EXCHANGE_RETRY_COUNT, true);
            plan.bounded_by_attempts = low_power;
            return plan;
        }
        AttemptPlan plan = count_plan_(EXCHANGE_RETRY_COUNT, true);
        if (low_power) {
            plan.bounded_by_attempts = true;
            return plan;
        }
        plan.gap_ms = 0;
        plan.early_window_ms = std::min<uint16_t>(EXCHANGE_EARLY_RESPONSE_WAIT_MS, plan.window_ms);
        plan.max_attempts =
            attempts_that_fit(EXCHANGE_TOTAL_BUDGET_MS, tx_ms, plan.early_window_ms, plan.window_ms, EXCHANGE_MAX_ATTEMPTS);
        return plan;
    }

    IOHCController2W::AttemptPlan IOHCController2W::settling_plan_(bool low_power, bool short_lead,
                                                                   uint32_t tx_ms) const {
        AttemptPlan plan = count_plan_(EXCHANGE_RETRY_COUNT, true);
        plan.gap_ms = 0;
        if (low_power) {
            plan.window_ms = LOW_POWER_RESPONSE_WAIT_MS;
            plan.max_attempts = LOW_POWER_SETTLING_ATTEMPTS;
            plan.bounded_by_attempts = true;
            if (short_lead) {
                plan.short_lead = true;
                plan.short_lead_window_ms = std::min<uint16_t>(LOW_POWER_SHORT_LEAD_WINDOW_MS, plan.window_ms);
                plan.max_attempts++;
            }
            return plan;
        }
        plan.early_window_ms = std::min<uint16_t>(EXCHANGE_EARLY_RESPONSE_WAIT_MS, plan.window_ms);
        plan.max_attempts =
            attempts_that_fit(EXCHANGE_TOTAL_BUDGET_MS, tx_ms, plan.early_window_ms, plan.window_ms, EXCHANGE_MAX_ATTEMPTS);
        return plan;
    }

    IOHCController2W::AttemptPlan IOHCController2W::progress_plan_(bool start_frame) const {
        return count_plan_(SCHEDULED_POLL_MAX_TRIES, start_frame);
    }

    // --- Wake belief (decisions_attempts.h) ---

    IOHCController2W::WakeBelief IOHCController2W::wake_belief_for_(const IOHC::address &dst, bool low_power,
                                                                     bool start_frame, uint8_t cmd, bool is_stop,
                                                                     bool state_unknown) const {
        if (!low_power || !start_frame) return WakeBelief::NONE;
        auto it = sessions_.find(pack_address(dst));
        // A destination with no record has no evidence: asleep, the safe default.
        if (it == sessions_.end()) return WakeBelief::ASLEEP;
        const Session2W &s = it->second;
        const uint32_t now = esphome::millis();
        if (is_stop || (s.last_moving_evidence_ms != 0 && (now - s.last_moving_evidence_ms) < LOW_POWER_MAX_TRAVEL_MS))
            return WakeBelief::AWAKE;
        if (cmd == CMD_PRIVATE &&
            (state_unknown || s.last_status_ms == 0 || (now - s.last_status_ms) >= LOW_POWER_MAX_TRAVEL_MS))
            return WakeBelief::UNKNOWN;
        return WakeBelief::ASLEEP;
    }

    uint16_t IOHCController2W::preamble_for_try_(uint8_t try_index) const {
        if (exchange_.preamble_override != 0) return exchange_.preamble_override;
        if (!exchange_.start_frame) return RESPONSE_PREAMBLE_BYTES;
        if (!exchange_.low_power) return START_PREAMBLE_BYTES;
        if (exchange_.plan.short_lead && try_index == 1) return START_PREAMBLE_BYTES;
        if (exchange_.belief == WakeBelief::AWAKE && try_index == 1) return START_PREAMBLE_BYTES;
        return LOW_POWER_WAKE_PREAMBLE_BYTES;
    }

    // --- Send-and-receive ---

    bool IOHCController2W::start_exchange_(ExchangeFor what, const IOHC::address &dst, uint8_t cmd,
                                           const uint8_t *data, uint8_t data_len, bool start_frame, bool low_power,
                                           const AttemptPlan &plan, bool wake_up_channels, uint16_t preamble_override,
                                           uint32_t session_freq) {
        if (exchange_.active) return false;
        exchange_ = Exchange{};
        exchange_.active = true;
        exchange_.what = what;
        memcpy(exchange_.dst, dst, sizeof(IOHC::address));
        exchange_.cmd = cmd;
        exchange_.data_len = data_len;
        if (data_len > 0) memcpy(exchange_.data, data, data_len);
        exchange_.start_frame = start_frame;
        exchange_.low_power = low_power;
        exchange_.wake_up_channels = wake_up_channels;
        exchange_.preamble_override = preamble_override;
        exchange_.session_freq = session_freq;
        exchange_.plan = plan;
        exchange_.is_stop = cmd == CMD_EXECUTE && data_len > 2 && data[2] == POS_STOP;
        exchange_.belief = preamble_override != 0
                               ? WakeBelief::NONE
                               : wake_belief_for_(dst, low_power, start_frame, cmd, exchange_.is_stop, false);
        exchange_.tries_allowed = plan.max_attempts;
        exchange_.begin_ms = esphome::millis();
        next_try_();
        run_exchange_();
        return true;
    }

    void IOHCController2W::run_exchange_() {
        // send_and_receive(): the exchange listens for its replies itself
        // until it ends. Its completion handler can start the next exchange,
        // which runs to its end inside this one.
        // A frame the exchange does not take is only overheard (RSSI, passive
        // status), like that component's report_overheard_(): the 2W
        // controller does not act on it afterwards.
        while (exchange_.active) {
            radio_->startQueuedSend();
            if (owner_ != nullptr) owner_->service_hop();
            const uint32_t now = esphome::millis();
            if (time_reached(now, exchange_.due_ms)) {
                on_exchange_timeout_();
                continue;
            }
            const uint32_t wait_ms = std::min<uint32_t>(exchange_.due_ms - now, EXCHANGE_LISTEN_SLICE_MS);
            iohcPacket frame;
            if (owner_ == nullptr || !owner_->wait_for_frame(frame, wait_ms)) continue;
            exchange_frame_(&frame);
            if (owner_ != nullptr) owner_->defer_frame(frame, true);
        }
    }

    void IOHCController2W::on_exchange_timeout_() {
        switch (exchange_.phase) {
            case ExchangePhase::GAP:
                transmit_try_();
                break;
            case ExchangePhase::WAIT_FIRST:
                ESP_LOGI(TAG, "Try %u ended: no first response for cmd 0x%02X", exchange_.tries, exchange_.cmd);
                next_try_();
                break;
            case ExchangePhase::WAIT_FINAL:
                try_ended_unconfirmed_();
                break;
        }
    }

    bool IOHCController2W::exchange_frame_(IOHC::iohcPacket *packet) {
        if (!from_device_to_us_(packet, exchange_.dst)) return false;
        if (exchange_.phase == ExchangePhase::GAP) return false;
        const uint8_t cmd = packet->payload.packet.header.cmd;
        const uint8_t *data = packet->payload.buffer + sizeof(_header);
        const uint8_t data_len = packet->buffer_length > sizeof(_header)
                                     ? static_cast<uint8_t>(packet->buffer_length - sizeof(_header))
                                     : 0;
        if (exchange_.phase == ExchangePhase::WAIT_FIRST && cmd == CMD_CHALLENGE_REQ) {
            if (data_len != 6) {
                exchange_.due_ms = esphome::millis();  // cannot be answered: the try ends
                return true;
            }
            answer_challenge_(data);
            return true;
        }
        exchange_.reply_cmd = cmd;
        exchange_.reply_len = std::min<uint8_t>(data_len, sizeof(exchange_.reply_data));
        memcpy(exchange_.reply_data, data, exchange_.reply_len);
        end_exchange_(ExchangeOutcome::SUCCESS_WITH_RESPONSE);
        return true;
    }

    void IOHCController2W::next_try_() {
        Exchange &x = exchange_;
        if (x.tries >= x.tries_allowed) {
            end_exchange_(x.unconfirmed_tries > 0 ? ExchangeOutcome::SUCCESS_UNCONFIRMED : ExchangeOutcome::FAILED);
            return;
        }
        const uint32_t gap_ms = x.tries == 0 ? 0 : std::max<uint32_t>(x.plan.gap_ms, x.follow_up_gap_ms);
        if (x.tries > 0 && !x.plan.bounded_by_attempts) {
            const uint32_t window = x.plan.window_for_attempt(x.tries + 1, x.tries_allowed);
            const uint32_t tx = tx_time_ms_(preamble_for_try_(x.tries + 1), sizeof(_header) + x.data_len);
            const uint32_t elapsed = esphome::millis() - x.begin_ms;
            if (!(elapsed + gap_ms + tx + EXCHANGE_ATTEMPT_OVERHEAD_MS + window + x.auth_tail_ms <
                  EXCHANGE_TOTAL_BUDGET_MS)) {
                ESP_LOGI(TAG, "Exchange budget exhausted after %u tries for cmd 0x%02X (%lu of %lu ms)", x.tries,
                         x.cmd, static_cast<unsigned long>(elapsed),
                         static_cast<unsigned long>(EXCHANGE_TOTAL_BUDGET_MS));
                end_exchange_(x.unconfirmed_tries > 0 ? ExchangeOutcome::SUCCESS_UNCONFIRMED
                                                      : ExchangeOutcome::FAILED);
                return;
            }
        }
        x.phase = ExchangePhase::GAP;
        x.due_ms = esphome::millis() + gap_ms;
        if (gap_ms == 0) transmit_try_();
    }

    void IOHCController2W::transmit_try_() {
        Exchange &x = exchange_;
        x.tries++;
        x.try_preamble = preamble_for_try_(x.tries);
        // End frames are only ever the device's: every request here opens or
        // continues an exchange.
        iohcPacket *packet = build_request_(x.dst, x.cmd, x.data, x.data_len, x.start_frame, false, x.low_power);
        const uint8_t frame_len = packet->buffer_length;
        const uint32_t window = x.plan.window_for_attempt(x.tries, x.tries_allowed);
        // The wait state is set before the frame goes out.
        x.phase = ExchangePhase::WAIT_FIRST;
        x.due_ms = esphome::millis() + 0x7FFFFFFF;
        if (x.start_frame) {
            uint32_t freq = 0;
            const bool wake_up = x.wake_up_channels || x.low_power;
            const ChannelAccessStatus access =
                send_with_channel_access_(packet, x.try_preamble, wake_up, x.previous_freq, freq);
            x.previous_freq = freq;
            if (access == ChannelAccessStatus::CHANNEL_BUSY) {
                ESP_LOGI(TAG, "Try %u ended: channel busy for cmd 0x%02X, nothing sent, last checked %lu Hz",
                         x.tries, x.cmd, static_cast<unsigned long>(freq));
                next_try_();
                return;
            }
            x.request_freq = freq;
        } else {
            x.request_freq = x.session_freq != 0 ? x.session_freq : CHANNEL2;
            send_at_once_(packet, x.request_freq, x.try_preamble);
        }
        listen_on_(x.request_freq);
        x.due_ms = esphome::millis() + tx_time_ms_(x.try_preamble, frame_len) + window;
    }

    void IOHCController2W::answer_challenge_(const uint8_t *challenge6) {
        Exchange &x = exchange_;
        // The 0x3D authenticates the challenged frame (its command byte and
        // payload) under this bridge's system key. Continuation frame,
        // low-power set, response preamble, at once on the request channel.
        std::vector<uint8_t> request_data(x.data, x.data + x.data_len);
        uint8_t answer[6];
        iohcCrypto::build_challenge_answer(x.cmd, request_data, challenge6, system_key_, answer);
        iohcPacket *reply = build_request_(x.dst, CMD_CHALLENGE_RESP, answer, sizeof(answer), false, false, true);
        const uint8_t frame_len = reply->buffer_length;
        x.saw_challenge = true;
        x.phase = ExchangePhase::WAIT_FINAL;
        x.due_ms = esphome::millis() + 0x7FFFFFFF;
        send_at_once_(reply, x.request_freq, RESPONSE_PREAMBLE_BYTES);
        x.due_ms = esphome::millis() + tx_time_ms_(RESPONSE_PREAMBLE_BYTES, frame_len) + RESPONSE_WAIT_MS;
    }

    void IOHCController2W::try_ended_unconfirmed_() {
        Exchange &x = exchange_;
        // The device challenged, so it received the request; whether our
        // answer arrived is not observable. How many more tries to spend is
        // after_unconfirmed_().
        x.unconfirmed_tries++;
        const uint8_t tries_left = static_cast<uint8_t>(x.tries_allowed - x.tries);
        uint8_t further = 0;
        uint32_t gap_ms = 0;
        if (tries_left > 0) {
            if (x.cmd == CMD_EXECUTE) {
                auto it = sessions_.find(pack_address(x.dst));
                const bool confirms = it != sessions_.end() && it->second.confirms_execute;
                const bool repeatable = x.data_len > 2 && x.data[2] != POS_FAVORITE;
                if (confirms && repeatable && x.unconfirmed_tries <= UNCONFIRMED_EXECUTE_MAX_RESENDS) {
                    further = UNCONFIRMED_EXECUTE_MAX_RESENDS;
                    gap_ms = UNCONFIRMED_EXECUTE_RESEND_DELAY_MS;
                }
            } else {
                further = static_cast<uint8_t>(EXCHANGE_RETRY_COUNT - std::min<uint8_t>(x.tries, EXCHANGE_RETRY_COUNT));
            }
        }
        if (further == 0) {
            end_exchange_(ExchangeOutcome::SUCCESS_UNCONFIRMED);
            return;
        }
        x.follow_up_gap_ms = gap_ms;
        x.auth_tail_ms = x.cmd == CMD_EXECUTE ? 0 : RESPONSE_WAIT_MS;
        x.tries_allowed = std::min<uint8_t>(x.tries_allowed, static_cast<uint8_t>(x.tries + 1 + further));
        if (x.cmd == CMD_EXECUTE)
            ESP_LOGI(TAG, "Try %u accepted without a closing reply for cmd 0x%02X: re-sending", x.tries, x.cmd);
        next_try_();
    }

    void IOHCController2W::end_exchange_(ExchangeOutcome outcome) {
        exchange_.active = false;
        if (job_ == PairJob::NONE && owner_ != nullptr) owner_->release_hop_plan();
        exchange_done_(outcome);
    }

    void IOHCController2W::exchange_done_(ExchangeOutcome outcome) {
        switch (exchange_.what) {
            case ExchangeFor::COMMAND:
                command_done_(outcome);
                return;
            case ExchangeFor::STATUS_READ:
                status_read_done_(outcome);
                return;
            default:
                break;
        }

        char buf[7];
        addr_str(device_, buf);
        const bool answered = outcome == ExchangeOutcome::SUCCESS_WITH_RESPONSE;
        const uint8_t reply_cmd = exchange_.reply_cmd;
        switch (exchange_.what) {
            case ExchangeFor::KEY_TRANSFER:
                if (answered && reply_cmd == CMD_KEY_CONFIRM) {
                    ESP_LOGI(TAG, "Key exchange: %s confirmed the key (0x33)%s", buf,
                             exchange_.saw_challenge ? " after challenging the key transfer" : "");
                    key_held_();
                    return;
                }
                if (answered && reply_cmd == CMD_ERROR_RESP) {
                    key_refused_ = true;
                    ESP_LOGW(TAG, "Key transfer: device %s refused with 0x%02X", buf,
                             exchange_.reply_len > 0 ? exchange_.reply_data[0] : 0);
                } else if (answered) {
                    ESP_LOGW(TAG, "Key transfer: device responded with cmd 0x%02X (expected KEY_CONFIRM 0x33)",
                             reply_cmd);
                }
                key_round_failed_();
                return;

            case ExchangeFor::VERIFY_KEY:
                if (answered && exchange_.saw_challenge && reply_cmd == CMD_GET_NAME_RESP) {
                    ESP_LOGW(TAG, "Key exchange: %s never confirmed the key, but it answered an authenticated request "
                                  "with it. Treating it as paired.",
                             buf);
                    key_held_();
                    return;
                }
                if (answered && reply_cmd == CMD_ERROR_RESP && exchange_.reply_len > 0) {
                    ESP_LOGW(TAG, "Key exchange failed: %s refused the name read with 0x%02X, so it does not act as a "
                                  "paired device",
                             buf, exchange_.reply_data[0]);
                } else {
                    ESP_LOGW(TAG, "Key exchange failed: %s does not answer authenticated requests with our key", buf);
                }
                finish_job_(key_refused_ ? "key_transfer_refused" : "key_exchange_failed");
                return;

            case ExchangeFor::SET_CONFIG1:
                if (!answered)
                    ESP_LOGI(TAG, "Pairing: no reply from %s to the optional SetConfig1 (0x6F); the pairing is "
                                  "complete",
                             buf);
                complete_paired_();
                return;

            default:
                return;
        }
    }

    // --- Pairing engine ---

    bool IOHCController2W::start_job_(PairJob job, const char *opening) {
        if (job_ != PairJob::NONE || exchange_.active) {
            ESP_LOGW(TAG, "A pairing attempt, scan or command is running - try again when it has finished");
            return false;
        }
        job_ = job;
        job_started_ms_ = esphome::millis();
        device_known_ = false;
        discovery_low_power_ = false;
        saw_key_challenge_ = false;
        key_refused_ = false;
        discovery_attempt_ = 0;
        confirm_try_ = 0;
        key_round_ = 0;
        session_freq_ = CHANNEL2;
        roll_call_attempt_ = 0;
        roll_call_responders_.clear();
        if (owner_ != nullptr) owner_->set_bonding_hop_wanted(true);
        ESP_LOGI(TAG, "%s", opening);
        return true;
    }

    void IOHCController2W::finish_job_(const char *outcome) {
        ESP_LOGI(TAG, "Pairing attempt ended: %s (%lu ms)", outcome,
                 static_cast<unsigned long>(esphome::millis() - job_started_ms_));
        step_ = PairStep::IDLE;
        job_ = PairJob::NONE;
        if (owner_ != nullptr) {
            owner_->release_hop_plan();
            owner_->set_bonding_hop_wanted(false);
        }
    }

    void IOHCController2W::discover_and_pair() {
        if (!start_job_(PairJob::DISCOVER_AND_PAIR, "Starting device discovery...")) return;
        listen_on_(CHANNEL2);  // dwell on CH2 before discovery hopping begins
        step_due_ms_ = esphome::millis() + PAIRING_DISCOVERY_INITIAL_DWELL_MS;
        step_ = PairStep::DISCOVERY_DWELL;
        run_job_();
    }

    void IOHCController2W::pair_device(const std::string &device_id_hex) {
        IOHC::address id{};
        if (device_id_hex.size() != 6 || hexStringToBytes(device_id_hex, id) != 3) {
            ESP_LOGW(TAG, "Pair %s: the device ID must be exactly 6 hex characters", device_id_hex.c_str());
            return;
        }
        esphome::iohc::IOHCCover *declared = nullptr;
        if (owner_ != nullptr) {
            for (auto *cover : owner_->covers()) {
                if (cover->motor_matches(id)) {
                    declared = cover;
                    break;
                }
            }
        }
        if (declared == nullptr) {
            ESP_LOGW(TAG, "Pair %s: no cover of the YAML declares it (motor_address), so there is no power class to "
                          "address it with",
                     device_id_hex.c_str());
            return;
        }
        if (!start_job_(PairJob::PAIR_BY_ADDRESS, "Pairing a declared device by address, without discovery..."))
            return;
        memcpy(device_, id, sizeof(IOHC::address));
        device_known_ = true;
        discovery_low_power_ = declared->low_power_2w();
        char buf[7];
        ESP_LOGI(TAG, "Pairing %s (%s) directly; it must have its pairing window open", addr_str(device_, buf),
                 declared->get_name().c_str());
        start_key_round_(1);
        run_job_();
    }

    void IOHCController2W::send_discovery_attempt_(uint8_t attempt) {
        discovery_attempt_ = attempt;
        iohcPacket *packet = build_request_(BROADCAST_DISCOVER, CMD_DISCOVER_REQ, nullptr, 0, true, true, false);
        // A discovery reply essentially never lands back on the request
        // channel: the wait listens on the two others.
        if (owner_ != nullptr) owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::SKIP_FREQ, CHANNEL2);
        step_ = PairStep::DISCOVERY_WAIT;
        step_due_ms_ = esphome::millis() + 0x7FFFFFFF;
        const uint8_t frame_len = packet->buffer_length;
        send_fixed_channel_(packet, CHANNEL2, START_PREAMBLE_BYTES);
        step_due_ms_ = esphome::millis() + tx_time_ms_(START_PREAMBLE_BYTES, frame_len) + PAIRING_DISCOVERY_WAIT_MS;
    }

    void IOHCController2W::on_discovery_response_(const IOHC::iohcPacket *packet, const uint8_t *data,
                                                   uint8_t data_len) {
        memcpy(device_, packet->payload.packet.header.source, sizeof(IOHC::address));
        device_known_ = true;
        uint8_t flags = 0;
        if (data_len > DISCOVERY_RESP_FLAGS_OFFSET) {
            flags = data[DISCOVERY_RESP_FLAGS_OFFSET];
            discovery_low_power_ = (flags & DISCOVERY_FLAGS_POWER_SAVE_MASK) == POWER_SAVE_LOW_POWER;
        }
        // The discover-confirm goes out from loop(), not from this receive
        // task, so only one task drives the radio's channel.
        confirm_try_ = 0;
        step_ = PairStep::CONFIRM_WAIT;
        step_due_ms_ = esphome::millis();
        char buf[7];
        ESP_LOGI(TAG, "Discovery: device %s answered on %lu Hz (%.1f dBm), flags=0x%02X power_save=%s",
                 addr_str(device_, buf), static_cast<unsigned long>(packet->frequency), packet->rssi, flags,
                 discovery_low_power_ ? "low_power" : "always_alive");
    }

    void IOHCController2W::send_discover_confirm_(uint8_t try_index) {
        confirm_try_ = try_index;
        // Start frame; low-power bit follows the device's power class, no ACK
        // (pairing_discover_confirm: send). It opens a pairing session through
        // channel access on the wake-up channels (CH2).
        iohcPacket *packet = build_request_(device_, CMD_DISCOVER_CONFIRM, nullptr, 0, true, false, discovery_low_power_);
        const uint8_t frame_len = packet->buffer_length;
        step_ = PairStep::CONFIRM_WAIT;
        step_due_ms_ = esphome::millis() + 0x7FFFFFFF;
        uint32_t freq = 0;
        if (send_with_channel_access_(packet, START_PREAMBLE_BYTES, true, session_freq_, freq) ==
            ChannelAccessStatus::CHANNEL_BUSY) {
            ESP_LOGI(TAG, "Discover confirm: channel busy, nothing sent (try %u/%u)", try_index,
                     PAIRING_DISCOVER_CONFIRM_TRIES);
            step_due_ms_ = esphome::millis();
            return;
        }
        session_freq_ = freq;
        // Try 2 rotates through all channels, tries 1 and 3 hold the request
        // channel (discover_confirm_try_rotates()).
        if (try_index == 2 && owner_ != nullptr)
            owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::ALL);
        else
            listen_on_(session_freq_);
        step_due_ms_ =
            esphome::millis() + tx_time_ms_(START_PREAMBLE_BYTES, frame_len) + PAIRING_DISCOVER_CONFIRM_ACK_TIMEOUT_MS;
    }

    void IOHCController2W::start_key_round_(uint8_t round) {
        key_round_ = round;
        if (round > 1)
            ESP_LOGI(TAG, "Retrying key exchange (attempt %u/%u)...", round, PAIRING_KEY_EXCHANGE_RETRIES);
        // Key-init: start frame, low-power bit set (the fixed `48 20` shape),
        // opening a pairing session on the wake-up channels.
        iohcPacket *packet = build_request_(device_, CMD_KEY_INIT, nullptr, 0, true, false, true);
        const uint8_t frame_len = packet->buffer_length;
        step_ = PairStep::KEY_CHALLENGE_WAIT;
        step_due_ms_ = esphome::millis() + 0x7FFFFFFF;
        uint32_t freq = 0;
        if (send_with_channel_access_(packet, START_PREAMBLE_BYTES, true, session_freq_, freq) ==
            ChannelAccessStatus::CHANNEL_BUSY) {
            ESP_LOGI(TAG, "Key exchange: channel busy, key-init not sent");
            step_due_ms_ = esphome::millis();
            return;
        }
        session_freq_ = freq;
        listen_on_(session_freq_);
        step_due_ms_ = esphome::millis() + tx_time_ms_(START_PREAMBLE_BYTES, frame_len) + PAIRING_KEY_CHALLENGE_TIMEOUT_MS;
    }

    void IOHCController2W::key_round_failed_() {
        // An explicit refusal ends the retries; otherwise the next round
        // follows after the retry gap.
        if (!key_refused_ && key_round_ < PAIRING_KEY_EXCHANGE_RETRIES) {
            step_due_ms_ = esphome::millis() + EXCHANGE_RETRY_DELAY_MS;
            step_ = PairStep::KEY_ROUND_GAP;
            return;
        }
        // The device challenged our key-init, so it was talking to us; it may
        // still have taken a key whose confirm we lost.
        if (saw_key_challenge_) {
            verify_device_holds_key_();
            return;
        }
        ESP_LOGW(TAG, "Key exchange failed");
        finish_job_(key_refused_ ? "key_transfer_refused" : "key_exchange_failed");
    }

    void IOHCController2W::verify_device_holds_key_() {
        char buf[7];
        ESP_LOGI(TAG, "Key exchange: not confirmed; checking whether %s took the key anyway (authenticated name read)",
                 addr_str(device_, buf));
        step_ = PairStep::EXCHANGE;
        start_exchange_(ExchangeFor::VERIFY_KEY, device_, CMD_GET_NAME, nullptr, 0, true, discovery_low_power_,
                        count_plan_(PAIRING_KEY_VERIFY_MAX_TRIES, true), true, START_PREAMBLE_BYTES, 0);
    }

    void IOHCController2W::key_held_() {
        // Phase 3: optional SetConfig1, fixed `4D 20` shape. The key is held already.
        step_ = PairStep::EXCHANGE;
        start_exchange_(ExchangeFor::SET_CONFIG1, device_, CMD_SET_CONFIG1, SET_CONFIG1_PAYLOAD,
                        sizeof(SET_CONFIG1_PAYLOAD), true, true, count_plan_(PAIRING_SET_CONFIG1_MAX_TRIES, true), true,
                        START_PREAMBLE_BYTES, 0);
    }

    void IOHCController2W::complete_paired_() {
        char buf[7];
        addr_str(device_, buf);
        const esphome::iohc::IOHCCover *declared = nullptr;
        if (owner_ != nullptr) {
            for (auto *cover : owner_->covers()) {
                if (cover->motor_matches(device_)) {
                    declared = cover;
                    break;
                }
            }
        }
        if (declared != nullptr) {
            ESP_LOGI(TAG, "Device %s took the key and is paired to this bridge (%s). Set that cover's Mode to Two-Way "
                          "to control it over 2W.",
                     buf, declared->get_name().c_str());
        } else {
            ESP_LOGW(TAG, "Device %s took the key and is paired to this bridge. No cover declares it yet: set the "
                          "cover's motor_address to \"%c%c%c%c%c%c\", reflash, then set its Mode to Two-Way.",
                     buf, tolower(buf[0]), tolower(buf[1]), tolower(buf[2]), tolower(buf[3]), tolower(buf[4]),
                     tolower(buf[5]));
        }
        finish_job_("paired");
    }

    // --- Roll-call ---

    void IOHCController2W::scan_paired_devices() {
        if (!start_job_(PairJob::SCAN, "Scan Paired Devices: roll-call under this bridge's key...")) return;
        send_roll_call_attempt_();
        run_job_();
    }

    void IOHCController2W::send_roll_call_attempt_() {
        const bool low_power_pass = roll_call_attempt_ < 3;
        roll_call_tx_freq_ = SCAN_CHANNELS[roll_call_attempt_ % 3];

        // 0x2A: a fresh 6-byte nonce, then the HMAC of the command byte alone
        // with the nonce as the challenge, under the system key.
        uint8_t payload[12];
        for (uint8_t i = 0; i < 6; i++) payload[i] = esp_random() & 0xff;
        std::vector<uint8_t> no_data;
        iohcCrypto::build_challenge_answer(CMD_DISCOVER_SPE_REQ, no_data, payload, system_key_, payload + 6);

        iohcPacket *packet =
            build_request_(BROADCAST_DISCOVER, CMD_DISCOVER_SPE_REQ, payload, sizeof(payload), true, true, low_power_pass);
        packet->payload.packet.header.CtrlByte2.asStruct.Unk3 = low_power_pass ? 1 : 0;  // CTRL1_ACK
        const uint8_t frame_len = packet->buffer_length;
        if (owner_ != nullptr) {
            if (low_power_pass)
                owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::ALL);
            else
                owner_->set_hop_plan(esphome::iohc::IOHCComponent::HopPlan::SKIP_FREQ, roll_call_tx_freq_);
        }
        const uint16_t preamble = low_power_pass ? LOW_POWER_WAKE_PREAMBLE_BYTES : START_PREAMBLE_BYTES;
        step_ = PairStep::ROLL_CALL_WAIT;
        step_due_ms_ = esphome::millis() + 0x7FFFFFFF;
        send_fixed_channel_(packet, roll_call_tx_freq_, preamble);
        step_due_ms_ = esphome::millis() + tx_time_ms_(preamble, frame_len) + PAIRING_DISCOVERY_WAIT_MS;
        ESP_LOGD(TAG, "Roll-call attempt %u/%u (pass=%s, tx %lu Hz)", roll_call_attempt_ + 1, ROLL_CALL_ATTEMPTS,
                 low_power_pass ? "low_power" : "always_alive", static_cast<unsigned long>(roll_call_tx_freq_));
    }

    void IOHCController2W::report_roll_call_() {
        size_t known = 0;
        std::string lines;
        for (const auto &r : roll_call_responders_) {
            char buf[7];
            addr_str(r.src, buf);
            const esphome::iohc::IOHCCover *declared = nullptr;
            if (owner_ != nullptr) {
                for (auto *cover : owner_->covers()) {
                    if (cover->motor_matches(r.src)) {
                        declared = cover;
                        break;
                    }
                }
            }
            const char *power_save = !r.has_flags ? "unknown"
                                     : (r.flags & DISCOVERY_FLAGS_POWER_SAVE_MASK) == POWER_SAVE_LOW_POWER
                                         ? "low_power"
                                         : "always_alive";
            char line[160];
            if (declared != nullptr) {
                known++;
                snprintf(line, sizeof(line), "\n  %s: %s, rssi=%.1f dBm, power_save=%s", buf,
                         declared->get_name().c_str(), r.rssi, power_save);
            } else {
                snprintf(line, sizeof(line), "\n  %s: no cover declares it (motor_address), rssi=%.1f dBm, "
                                             "power_save=%s",
                         buf, r.rssi, power_save);
            }
            lines += line;
        }
        ESP_LOGI(TAG, "Roll-call: %u device%s detected (%u known, %u unknown)%s",
                 static_cast<unsigned>(roll_call_responders_.size()), roll_call_responders_.size() == 1 ? "" : "s",
                 static_cast<unsigned>(known), static_cast<unsigned>(roll_call_responders_.size() - known),
                 lines.c_str());
        finish_job_("scan complete");
    }

    // --- Cover commands and status reads ---

    void IOHCController2W::send_command(const IOHC::address &to, bool low_power, uint8_t main0, uint8_t main1,
                                         esphome::iohc::IOHCCover *cover) {
        Session2W &s = sessions_[pack_address(to)];
        s.cover = cover;
        s.low_power = low_power;
        // A newer command replaces one still waiting for the radio.
        s.command_pending = true;
        s.pending_main0 = main0;
        s.pending_main1 = main1;
        run_pending_work_();
    }

    void IOHCController2W::register_cover(const IOHC::address &motor, esphome::iohc::IOHCCover *cover, bool low_power,
                                          uint32_t refresh_interval_ms) {
        Session2W &s = sessions_[pack_address(motor)];
        s.cover = cover;
        s.low_power = low_power;
        const uint32_t now = esphome::millis();
        s.boot_read_ms = now + INITIAL_STATUS_REQUEST_DELAY_MS;
        if (s.boot_read_ms == 0) s.boot_read_ms = 1;
        s.refresh_interval_ms = refresh_interval_ms;
        s.next_refresh_ms = refresh_interval_ms != 0 ? now + periodic_poll_first_delay_ms(esp_random()) : 0;
        if (refresh_interval_ms != 0 && s.next_refresh_ms == 0) s.next_refresh_ms = 1;
    }

    void IOHCController2W::note_status_reported_(Session2W &s) {
        if (s.refresh_interval_ms == 0) return;
        s.next_refresh_ms = esphome::millis() + periodic_poll_jittered_interval_ms(s.refresh_interval_ms, esp_random());
        if (s.next_refresh_ms == 0) s.next_refresh_ms = 1;
    }

    void IOHCController2W::run_pending_work_() {
        if (exchange_.active || job_ != PairJob::NONE) return;
        for (auto &kv : sessions_) {
            if (!kv.second.command_pending) continue;
            IOHC::address to{static_cast<uint8_t>(kv.first >> 16), static_cast<uint8_t>(kv.first >> 8),
                             static_cast<uint8_t>(kv.first)};
            run_command_(to, kv.second);
            return;
        }
        const uint32_t now = esphome::millis();
        for (auto &kv : sessions_) {
            Session2W &s = kv.second;
            if (s.next_poll_ms == 0 || !time_reached(now, s.next_poll_ms)) continue;
            IOHC::address to{static_cast<uint8_t>(kv.first >> 16), static_cast<uint8_t>(kv.first >> 8),
                             static_cast<uint8_t>(kv.first)};
            start_status_read_(to, s);
            return;
        }
        // Background reads last: the read after boot, then the refresh at
        // rest, only for a cover whose Mode is Two-Way.
        for (auto &kv : sessions_) {
            Session2W &s = kv.second;
            const bool two_way =
                s.cover != nullptr && s.cover->get_mode() == esphome::iohc::IOHCCover::Mode::TWO_WAY;
            IOHC::address to{static_cast<uint8_t>(kv.first >> 16), static_cast<uint8_t>(kv.first >> 8),
                             static_cast<uint8_t>(kv.first)};
            if (s.boot_read_ms != 0 && time_reached(now, s.boot_read_ms)) {
                s.boot_read_ms = 0;
                if (!two_way) continue;
                start_status_read_(to, s);
                return;
            }
            if (s.next_refresh_ms != 0 && time_reached(now, s.next_refresh_ms)) {
                s.next_refresh_ms = now + periodic_poll_jittered_interval_ms(s.refresh_interval_ms, esp_random());
                if (s.next_refresh_ms == 0) s.next_refresh_ms = 1;
                if (!two_way) continue;
                char buf[7];
                ESP_LOGD(TAG, "Device %s: no status reported for a refresh interval (%lu s): reading it",
                         addr_str(to, buf), static_cast<unsigned long>(s.refresh_interval_ms / 1000));
                start_status_read_(to, s);
                return;
            }
        }
    }

    void IOHCController2W::run_command_(const IOHC::address &to, Session2W &s) {
        s.command_pending = false;
        const uint8_t main0 = s.pending_main0;
        const uint8_t main1 = s.pending_main1;
        // Main-parameter EXECUTE (create_execute_main_parameter() /
        // create_execute_command()): originator, ACEI, main parameter, two zero bytes.
        const uint8_t payload[6] = {EXECUTE_ORIGINATOR, EXECUTE_ACEI, main0, main1, 0x00, 0x00};
        const bool repeat_safe = main0 != POS_FAVORITE;
        const uint16_t fixed_preamble = s.low_power ? LOW_POWER_WAKE_PREAMBLE_BYTES : START_PREAMBLE_BYTES;
        const uint32_t tx_ms = tx_time_ms_(fixed_preamble, sizeof(_header) + sizeof(payload));
        // A new command starts its own follow-up window.
        s.next_poll_ms = 0;
        s.poll_deadline_ms = 0;
        char buf[7];
        ESP_LOGI(TAG, "Sending 2W command (main=%02X%02X) to %s", main0, main1, addr_str(to, buf));
        start_exchange_(ExchangeFor::COMMAND, to, CMD_EXECUTE, payload, sizeof(payload), true, s.low_power,
                        command_plan_(s.low_power, repeat_safe, tx_ms), false, 0, 0);
    }

    void IOHCController2W::command_done_(ExchangeOutcome outcome) {
        Session2W &s = sessions_[pack_address(exchange_.dst)];
        char buf[7];
        addr_str(exchange_.dst, buf);
        const bool answered = outcome == ExchangeOutcome::SUCCESS_WITH_RESPONSE;
        if (answered && exchange_.reply_cmd == CMD_ERROR_RESP) {
            ESP_LOGW(TAG, "2W command to %s refused with 0x%02X", buf,
                     exchange_.reply_len > 0 ? exchange_.reply_data[0] : 0);
            schedule_poll_failure_(s, exchange_.saw_challenge);
            return;
        }
        if (outcome == ExchangeOutcome::FAILED) {
            ESP_LOGW(TAG, "Command 0x00 failed for device %s: no valid response (tries=%u)", buf, exchange_.tries);
            schedule_poll_failure_(s, exchange_.saw_challenge);
            return;
        }
        if (answered) {
            s.confirms_execute = true;
            if (exchange_.reply_cmd == CMD_PRIVATE_RESP && exchange_.reply_len >= 1)
                apply_private_response_(s, /*trust_position=*/false);
            ESP_LOGI(TAG, "2W command to %s confirmed", buf);
        } else {
            ESP_LOGI(TAG, "2W command to %s accepted without a closing reply", buf);
        }
        // An accepted movement command means the receiver is travelling now.
        if (!exchange_.is_stop) s.last_moving_evidence_ms = esphome::millis();
        s.status_poll_failures = 0;
        s.auth_poll_failures = 0;
        s.stop_settle = exchange_.is_stop;
        s.settling_read = true;
        begin_status_poll_tracking_(s, settle_delay_ms(0, exchange_.is_stop));
    }

    void IOHCController2W::begin_status_poll_tracking_(Session2W &s, uint32_t delay_ms) {
        const uint32_t now = esphome::millis();
        s.next_poll_ms = now + delay_ms;
        if (s.next_poll_ms == 0) s.next_poll_ms = 1;
        if (s.poll_deadline_ms == 0) s.poll_deadline_ms = now + MAX_TRACKED_STATUS_POLL_WINDOW_MS;
    }

    void IOHCController2W::start_status_read_(const IOHC::address &to, Session2W &s) {
        s.next_poll_ms = 0;
        const uint8_t payload[3] = {PRIVATE_GET_POSITION_STATUS, 0x00, 0x00};
        const bool in_retry_band = s.auth_poll_failures == 0 &&
                                   s.status_poll_failures >= SCHEDULED_POLL_RETRY_GRACE_FIRST_FAILURE &&
                                   s.status_poll_failures <= SCHEDULED_POLL_RETRY_GRACE_LAST_FAILURE;
        const bool settling = s.settling_read || s.stop_settle || in_retry_band;
        const WakeBelief belief = wake_belief_for_(to, s.low_power, true, CMD_PRIVATE, false, false);
        const uint16_t fixed_preamble = s.low_power ? LOW_POWER_WAKE_PREAMBLE_BYTES : START_PREAMBLE_BYTES;
        const uint32_t tx_ms = tx_time_ms_(fixed_preamble, sizeof(_header) + sizeof(payload));
        const AttemptPlan plan = settling ? settling_plan_(s.low_power, s.low_power && belief == WakeBelief::UNKNOWN, tx_ms)
                                          : progress_plan_(true);
        start_exchange_(ExchangeFor::STATUS_READ, to, CMD_PRIVATE, payload, sizeof(payload), true, s.low_power, plan,
                        false, 0, 0);
    }

    void IOHCController2W::status_read_done_(ExchangeOutcome outcome) {
        Session2W &s = sessions_[pack_address(exchange_.dst)];
        if (outcome == ExchangeOutcome::SUCCESS_WITH_RESPONSE && exchange_.reply_cmd == CMD_PRIVATE_RESP &&
            exchange_.reply_len >= 6) {
            s.status_poll_failures = 0;
            s.auth_poll_failures = 0;
            s.settling_read = false;
            s.stop_settle = false;
            apply_private_response_(s, /*trust_position=*/true);
            return;
        }
        // A read exists to obtain a payload: getting none is a failure.
        schedule_poll_failure_(s, exchange_.saw_challenge);
    }

    void IOHCController2W::schedule_poll_failure_(Session2W &s, bool saw_challenge) {
        uint32_t backoff_ms;
        if (saw_challenge) {
            s.status_poll_failures = 0;
            if (s.auth_poll_failures < UINT8_MAX) s.auth_poll_failures++;
            backoff_ms = ladder_step(AUTH_RETRY_LADDER_MS, s.auth_poll_failures);
        } else {
            s.auth_poll_failures = 0;
            if (s.status_poll_failures < UINT8_MAX) s.status_poll_failures++;
            backoff_ms = ladder_step(SILENT_RETRY_LADDER_MS, s.status_poll_failures);
        }
        const uint32_t now = esphome::millis();
        if (s.poll_deadline_ms != 0 && !time_reached(now + backoff_ms, s.poll_deadline_ms + 1)) {
            s.next_poll_ms = now + backoff_ms;
            return;
        }
        s.next_poll_ms = 0;
        s.poll_deadline_ms = 0;
    }

    void IOHCController2W::apply_private_response_(Session2W &s, bool trust_position) {
        const uint8_t *d = exchange_.reply_data;
        const uint8_t len = exchange_.reply_len;
        std::optional<uint16_t> target, current, remaining_s;
        if (trust_position && len >= PRIVATE_RESPONSE_CURRENT_OFFSET + 2) {
            target = (static_cast<uint16_t>(d[PRIVATE_RESPONSE_TARGET_OFFSET]) << 8) |
                     d[PRIVATE_RESPONSE_TARGET_OFFSET + 1];
            current = (static_cast<uint16_t>(d[PRIVATE_RESPONSE_CURRENT_OFFSET]) << 8) |
                      d[PRIVATE_RESPONSE_CURRENT_OFFSET + 1];
        }
        // The remaining time is read only from a status-poll reply: an
        // EXECUTE's 0x04 holds something else in those bytes.
        if (trust_position && len >= PRIVATE_RESPONSE_REMAINING_TIME_MIN_DATA_LEN) {
            const uint16_t remaining = (static_cast<uint16_t>(d[PRIVATE_RESPONSE_REMAINING_TIME_OFFSET]) << 8) |
                                       d[PRIVATE_RESPONSE_REMAINING_TIME_OFFSET + 1];
            if (remaining != PRIVATE_RESPONSE_REMAINING_TIME_NONE) remaining_s = remaining;
        }
        apply_status_report_(s, exchange_.dst, d[0], target, current, remaining_s, trust_position);
    }

    void IOHCController2W::apply_status_report_(Session2W &s, const IOHC::address &motor, uint8_t status_byte,
                                                std::optional<uint16_t> target, std::optional<uint16_t> current,
                                                std::optional<uint16_t> remaining_s, bool trusted_poll) {
        const uint32_t now = esphome::millis();
        const bool reported_stopped = reads_stopped_by_status_byte(status_byte);
        s.last_status_ms = now;
        note_status_reported_(s);
        char buf[7];
        addr_str(motor, buf);

        if (target.has_value() && current.has_value()) {
            ESP_LOGI(TAG, "Device %s: %s, current %.1f%% closed, target %.1f%% closed", buf,
                     reported_stopped ? "stopped" : "moving", *current / 512.0f, *target / 512.0f);
            if (*current <= STATUS_POS_MAX && s.cover != nullptr)
                s.cover->update_real_position_authoritative(*current / 512.0f);
        }

        // The wake belief follows the device's own flag.
        s.last_moving_evidence_ms = reported_stopped ? 0 : now;

        if (remaining_s.has_value() && *remaining_s != 0) s.reports_remaining_time = true;
        if (remaining_s.has_value() && *remaining_s == 0 && !s.reports_remaining_time) remaining_s.reset();
        if (reported_stopped) remaining_s.reset();
        if (trusted_poll && remaining_s.has_value())
            s.moving_until_ms = now + (*remaining_s * 1000u) + PREDICTED_END_MARGIN_MS;

        if (reported_stopped) {
            // An EXECUTE's acknowledgement carries no trusted position: its
            // confirmation read still runs.
            if (trusted_poll || target.has_value()) {
                s.next_poll_ms = 0;
                s.poll_deadline_ms = 0;
            }
            return;
        }
        // A moving report opens the follow-up window when none is open; one
        // that finds the window expired closes it.
        if (s.poll_deadline_ms == 0) {
            s.poll_deadline_ms = now + MAX_TRACKED_STATUS_POLL_WINDOW_MS;
        } else if (time_reached(now, s.poll_deadline_ms)) {
            s.next_poll_ms = 0;
            s.poll_deadline_ms = 0;
            return;
        }
        // A moving device that counts down is read once more, with the
        // settling plan, at the end of its count.
        if (trusted_poll) s.settling_read = remaining_s.has_value();
        const uint32_t remaining_delay_ms =
            remaining_s.has_value() ? (*remaining_s * 1000u) + PRIVATE_RESPONSE_REMAINING_TIME_BIAS_MS : 0;
        const uint32_t delay_ms = settle_delay_ms(remaining_delay_ms, false);
        const uint32_t next = now + delay_ms;
        // A status update only moves a planned poll earlier, never later.
        if (trusted_poll || s.next_poll_ms == 0 || time_reached(s.next_poll_ms, next)) s.next_poll_ms = next;
        ESP_LOGD(TAG, "Device %s: next status poll in %lu ms", buf, static_cast<unsigned long>(delay_ms));
    }

    // --- Device-originated status updates ---

    bool IOHCController2W::bonding_active() const {
        if (job_ != PairJob::NONE || exchange_.active || inbound_.active) return true;
        // A Two-Way cover's motor may send its own status update at any time.
        if (owner_ != nullptr) {
            for (auto *cover : owner_->covers()) {
                if (cover->get_mode() == esphome::iohc::IOHCCover::Mode::TWO_WAY) return true;
            }
        }
        return false;
    }

    void IOHCController2W::on_status_update_(const IOHC::iohcPacket *packet, const uint8_t *data, uint8_t data_len) {
        const auto &src = packet->payload.packet.header.source;
        // Only a device this bridge controls: a Two-Way cover's motor.
        esphome::iohc::IOHCCover *declared = nullptr;
        if (owner_ != nullptr) {
            for (auto *cover : owner_->covers()) {
                if (cover->motor_matches(src) && cover->get_mode() == esphome::iohc::IOHCCover::Mode::TWO_WAY) {
                    declared = cover;
                    break;
                }
            }
        }
        if (declared == nullptr || inbound_.active || data_len > sizeof(inbound_.update)) return;
        inbound_ = InboundAuth{};
        inbound_.active = true;
        memcpy(inbound_.src, src, sizeof(IOHC::address));
        memcpy(inbound_.update, data, data_len);
        inbound_.update_len = data_len;
        inbound_.freq = packet->frequency;
        for (uint8_t &b : inbound_.challenge) b = esp_random() & 0xff;
        Session2W &s = sessions_[pack_address(src)];
        s.cover = declared;
        s.low_power = declared->low_power_2w();
        // Our challenge: continuation frame, low-power clear, response
        // preamble, at once on the channel the update arrived on.
        iohcPacket *challenge =
            build_request_(inbound_.src, CMD_CHALLENGE_REQ, inbound_.challenge, sizeof(inbound_.challenge), false,
                           false, false);
        const uint8_t frame_len = challenge->buffer_length;
        send_at_once_(challenge, inbound_.freq, RESPONSE_PREAMBLE_BYTES);
        inbound_.due_ms = esphome::millis() + tx_time_ms_(RESPONSE_PREAMBLE_BYTES, frame_len) + RESPONSE_WAIT_MS;

        // challenge_sender(): wait for the device's answer, then acknowledge.
        char buf[7];
        addr_str(inbound_.src, buf);
        bool verified = false;
        while (inbound_.active && !verified) {
            radio_->startQueuedSend();
            if (owner_ != nullptr) owner_->service_hop();
            const uint32_t now = esphome::millis();
            if (time_reached(now, inbound_.due_ms)) {
                ESP_LOGW(TAG, "Status update from %s: no challenge answer", buf);
                inbound_.active = false;
                break;
            }
            const uint32_t wait_ms = std::min<uint32_t>(inbound_.due_ms - now, EXCHANGE_LISTEN_SLICE_MS);
            iohcPacket frame;
            if (owner_ == nullptr || !owner_->wait_for_frame(frame, wait_ms)) continue;
            const bool answer = from_device_to_us_(&frame, inbound_.src) &&
                                frame.payload.packet.header.cmd == CMD_CHALLENGE_RESP;
            if (answer) {
                const uint8_t len = frame.buffer_length > sizeof(_header)
                                        ? static_cast<uint8_t>(frame.buffer_length - sizeof(_header))
                                        : 0;
                verified = on_status_update_answer_(frame.payload.buffer + sizeof(_header), len);
                if (!verified) inbound_.active = false;
            }
            owner_->defer_frame(frame, true);
        }
        if (verified) {
            // 0x72: final frame, low-power set, response preamble, on all three
            // channels one after the other.
            for (const uint32_t freq : inbound_.ack_order) {
                wait_tx_idle_(STATUS_UPDATE_ACK_TX_WAIT_MS);
                iohcPacket *ack = build_request_(inbound_.src, CMD_STATUS_UPDATE_RESP, STATUS_UPDATE_ACK_PAYLOAD,
                                                 sizeof(STATUS_UPDATE_ACK_PAYLOAD), false, true, true);
                send_at_once_(ack, freq, RESPONSE_PREAMBLE_BYTES);
            }
            wait_tx_idle_(STATUS_UPDATE_ACK_TX_WAIT_MS);
        }
        inbound_.active = false;
        if (!exchange_.active && job_ == PairJob::NONE && owner_ != nullptr) owner_->release_hop_plan();
        if (verified) apply_inbound_update_();
    }

    void IOHCController2W::wait_tx_idle_(uint32_t timeout_ms) {
        const uint32_t begin = esphome::millis();
        while (!radio_->tx_idle() && esphome::millis() - begin < timeout_ms) {
            radio_->startQueuedSend();
            esphome::delay(1);
        }
    }

    bool IOHCController2W::on_status_update_answer_(const uint8_t *data, uint8_t data_len) {
        char buf[7];
        addr_str(inbound_.src, buf);
        std::vector<uint8_t> update(inbound_.update, inbound_.update + inbound_.update_len);
        uint8_t expected[6];
        iohcCrypto::build_challenge_answer(CMD_STATUS_UPDATE, update, inbound_.challenge, system_key_, expected);
        if (data_len < 6 || memcmp(expected, data, 6) != 0) {
            ESP_LOGW(TAG, "Status update from %s: challenge answer does not verify under this bridge's key", buf);
            return false;
        }
        // Acknowledge on all three channels, the receive channel first.
        const std::array<uint32_t, 3> order{CHANNEL1, CHANNEL2, CHANNEL3};
        inbound_.ack_order = {inbound_.freq, 0, 0};
        size_t next = 1;
        for (const uint32_t freq : order) {
            if (freq != inbound_.freq && next < 3) inbound_.ack_order[next++] = freq;
        }
        if (next < 3) inbound_.ack_order = order;  // receive channel not one of the three
        return true;
    }

    void IOHCController2W::apply_inbound_update_() {
        char buf[7];
        addr_str(inbound_.src, buf);
        // Apply the update: an EVENT_STATUS_CHANGE carries the position.
        const uint8_t *d = inbound_.update;
        const uint8_t len = inbound_.update_len;
        if (len < STATUS_UPDATE_MIN_DATA_LEN) return;
        const uint16_t event_id = (static_cast<uint16_t>(d[STATUS_UPDATE_EVENT_ID_OFFSET]) << 8) |
                                  d[STATUS_UPDATE_EVENT_ID_OFFSET + 1];
        if (event_id != EVENT_STATUS_CHANGE) return;
        const uint16_t target = (static_cast<uint16_t>(d[STATUS_UPDATE_TARGET_OFFSET]) << 8) |
                                d[STATUS_UPDATE_TARGET_OFFSET + 1];
        const uint16_t current = (static_cast<uint16_t>(d[STATUS_UPDATE_CURRENT_OFFSET]) << 8) |
                                 d[STATUS_UPDATE_CURRENT_OFFSET + 1];
        Session2W &s = sessions_[pack_address(inbound_.src)];
        ESP_LOGI(TAG, "Status update from %s (authenticated)", buf);
        apply_status_report_(s, inbound_.src, d[0], target, current, std::nullopt, false);
    }

    // --- loop / handle_frame ---

    void IOHCController2W::loop() {
        // Exchanges, status-update authentications and pairing jobs run to
        // their end where they start; nothing of theirs is left for loop().
        if (inbound_.active || exchange_.active || job_ != PairJob::NONE) return;
        run_pending_work_();
    }

    void IOHCController2W::run_job_() {
        // The pairing engine's listens: each step waits for its frames itself,
        // the receiver hopping as the step's plan says, until the job ends.
        while (job_ != PairJob::NONE) {
            radio_->startQueuedSend();
            if (owner_ != nullptr) owner_->service_hop();
            const uint32_t now = esphome::millis();
            if (step_ != PairStep::EXCHANGE && time_reached(now, step_due_ms_)) {
                on_step_due_();
                continue;
            }
            iohcPacket frame;
            if (owner_ == nullptr || !owner_->wait_for_frame(frame, EXCHANGE_LISTEN_SLICE_MS)) continue;
            handle_frame(&frame);
            owner_->defer_frame(frame, true);
        }
    }

    void IOHCController2W::on_step_due_() {
        const uint32_t now = esphome::millis();
        switch (step_) {
            case PairStep::DISCOVERY_DWELL:
                send_discovery_attempt_(1);
                break;

            case PairStep::DISCOVERY_WAIT:
                if (discovery_attempt_ < PAIRING_DISCOVERY_MAX_ATTEMPTS) {
                    ESP_LOGI(TAG, "Discovery attempt %u/%u for cmd=0x28: no response, retrying...", discovery_attempt_,
                             PAIRING_DISCOVERY_MAX_ATTEMPTS);
                    send_discovery_attempt_(discovery_attempt_ + 1);
                } else {
                    ESP_LOGW(TAG, "No device responded to discovery");
                    finish_job_("no_response");
                }
                break;

            case PairStep::CONFIRM_WAIT:
                if (confirm_try_ < PAIRING_DISCOVER_CONFIRM_TRIES) {
                    send_discover_confirm_(confirm_try_ + 1);
                } else {
                    char buf[7];
                    ESP_LOGI(TAG, "Discover confirm: no 0x2D from %s after %u tries, continuing with key exchange",
                             addr_str(device_, buf), PAIRING_DISCOVER_CONFIRM_TRIES);
                    step_due_ms_ = now + PAIRING_KEY_INIT_DELAY_MS;
                    step_ = PairStep::KEY_INIT_DELAY;
                }
                break;

            case PairStep::KEY_INIT_DELAY:
                start_key_round_(1);
                break;

            case PairStep::KEY_ROUND_GAP:
                start_key_round_(key_round_ + 1);
                break;

            case PairStep::KEY_CHALLENGE_WAIT:
                ESP_LOGW(TAG, "Key exchange: no challenge received");
                key_round_failed_();
                break;

            case PairStep::ROLL_CALL_WAIT:
                roll_call_attempt_++;
                if (roll_call_attempt_ < ROLL_CALL_ATTEMPTS)
                    send_roll_call_attempt_();
                else
                    report_roll_call_();
                break;

            default:
                // No step has a wait pending: end the job rather than block.
                finish_job_("stopped: no pairing step pending");
                break;
        }
    }

    bool IOHCController2W::handle_frame(IOHC::iohcPacket *packet) {
        const bool two_way = !packet->payload.packet.header.CtrlByte1.asStruct.Protocol;

        // Passive key-sniff (Findings 27/28): observes only, never consumes.
        if (two_way) {
            const auto &src = packet->payload.packet.header.source;
            const auto &dst = packet->payload.packet.header.target;
            const uint8_t cmd = packet->payload.packet.header.cmd;
            if (sniff_.state == SniffState::IDLE && cmd == CMD_KEY_INIT) {
                memcpy(sniff_.box, src, sizeof(IOHC::address));
                memcpy(sniff_.device, dst, sizeof(IOHC::address));
                sniff_.state = SniffState::SAW_KEY_INIT;
            } else if (sniff_.state == SniffState::SAW_KEY_INIT && cmd == CMD_CHALLENGE_REQ &&
                       memcmp(src, sniff_.device, sizeof(IOHC::address)) == 0 &&
                       memcmp(dst, sniff_.box, sizeof(IOHC::address)) == 0) {
                memcpy(sniff_.challenge, packet->payload.packet.msg.p0x3c.challenge, sizeof(sniff_.challenge));
                sniff_.state = SniffState::SAW_CHALLENGE;
            } else if (sniff_.state == SniffState::SAW_CHALLENGE && cmd == CMD_KEY_TRANSFER &&
                       memcmp(src, sniff_.box, sizeof(IOHC::address)) == 0 &&
                       memcmp(dst, sniff_.device, sizeof(IOHC::address)) == 0 &&
                       packet->buffer_length >= sizeof(_header) + sizeof(_p0x32)) {
                if (memcmp(src, controller_address_, sizeof(IOHC::address)) != 0) {
                    uint8_t derived_key[16];
                    iohcCrypto::build_key_transfert_response(sniff_.challenge,
                                                              packet->payload.packet.msg.p0x32.encrypted_key,
                                                              derived_key);
                    std::string hex = bytesToHexString(derived_key, 16);
                    ESP_LOGW(TAG, "2W key sniff SUCCESS: derived real system_key from %02X%02X%02X's pairing with "
                                  "%02X%02X%02X: %s",
                             sniff_.box[0], sniff_.box[1], sniff_.box[2], sniff_.device[0], sniff_.device[1],
                             sniff_.device[2], hex.c_str());
                }
                reset_sniff_state();
            }
        }
        if (!two_way) return false;

        const uint8_t cmd = packet->payload.packet.header.cmd;
        const uint8_t *data = packet->payload.buffer + sizeof(_header);
        const uint8_t data_len = packet->buffer_length > sizeof(_header)
                                     ? static_cast<uint8_t>(packet->buffer_length - sizeof(_header))
                                     : 0;

        // A device's status update to us (authenticated in on_status_update_()).
        if (cmd == CMD_STATUS_UPDATE && !exchange_.active && job_ == PairJob::NONE &&
            memcmp(packet->payload.packet.header.target, controller_address_, sizeof(IOHC::address)) == 0) {
            on_status_update_(packet, data, data_len);
            return true;
        }

        // The running exchange takes its frames itself (run_exchange_()).
        if (exchange_.active) return false;

        if (job_ == PairJob::NONE) return false;
        const bool to_us =
            memcmp(packet->payload.packet.header.target, controller_address_, sizeof(IOHC::address)) == 0;

        switch (step_) {
            case PairStep::DISCOVERY_WAIT:
                if (cmd != CMD_DISCOVER_RESP || !to_us) return false;
                on_discovery_response_(packet, data, data_len);
                return true;

            case PairStep::CONFIRM_WAIT:
                if (!device_known_ || !from_device_to_us_(packet, device_)) return false;
                if (cmd == CMD_DISCOVER_CONFIRM_ACK) {
                    step_due_ms_ = esphome::millis() + PAIRING_KEY_INIT_DELAY_MS;
                    step_ = PairStep::KEY_INIT_DELAY;
                    ESP_LOGI(TAG, "Discover confirm: 0x2D on %lu Hz (try %u/%u)",
                             static_cast<unsigned long>(packet->frequency), confirm_try_, PAIRING_DISCOVER_CONFIRM_TRIES);
                    return true;
                }
                if (cmd == CMD_ERROR_RESP) {
                    step_due_ms_ = esphome::millis() + PAIRING_KEY_INIT_DELAY_MS;
                    step_ = PairStep::KEY_INIT_DELAY;
                    ESP_LOGW(TAG, "Discover confirm: device answered 0x2C with error 0x%02X, continuing with key "
                                  "exchange",
                             data_len > 0 ? data[0] : 0);
                    return true;
                }
                return false;

            case PairStep::KEY_CHALLENGE_WAIT:
                if (!device_known_ || !from_device_to_us_(packet, device_)) return false;
                // Some devices send 0x33 directly after 0x31 without requiring 0x32.
                if (cmd == CMD_KEY_CONFIRM) {
                    ESP_LOGI(TAG, "Device accepted key immediately (0x33 without 0x32 exchange)");
                    key_held_();
                    return true;
                }
                if (cmd != CMD_CHALLENGE_REQ || data_len != 6) return false;
                saw_key_challenge_ = true;
                {
                    // The key goes out at once against this challenge, on the
                    // key-init's session channel; the log line follows it.
                    uint8_t enc_key[16];
                    iohcCrypto::build_key_transfert_response(data, system_key_, enc_key);
                    step_ = PairStep::EXCHANGE;
                    start_exchange_(ExchangeFor::KEY_TRANSFER, device_, CMD_KEY_TRANSFER, enc_key, sizeof(enc_key),
                                    false, true, count_plan_(EXCHANGE_RETRY_COUNT, false), true, 0, session_freq_);
                }
                ESP_LOGI(TAG, "Challenge (0x3C) received on %lu Hz (%.1f dBm), key transfer (0x32) sent",
                         static_cast<unsigned long>(packet->frequency), packet->rssi);
                return true;

            case PairStep::ROLL_CALL_WAIT:
                if (cmd != CMD_DISCOVER_SPE_RESP || !to_us) return false;
                {
                    const auto &src = packet->payload.packet.header.source;
                    for (auto &r : roll_call_responders_) {
                        if (memcmp(r.src, src, sizeof(IOHC::address)) == 0) return true;
                    }
                    RollCallResponder r;
                    memcpy(r.src, src, sizeof(IOHC::address));
                    r.rssi = packet->rssi;
                    r.freq = packet->frequency;
                    if (data_len > DISCOVERY_RESP_FLAGS_OFFSET) {
                        r.flags = data[DISCOVERY_RESP_FLAGS_OFFSET];
                        r.has_flags = true;
                    }
                    roll_call_responders_.push_back(r);
                    ESP_LOGD(TAG, "Roll-call reply src=%02X%02X%02X rx=%lu Hz rssi=%.1f", src[0], src[1], src[2],
                             static_cast<unsigned long>(packet->frequency), packet->rssi);
                }
                return true;

            default:
                return false;
        }
    }
}
