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

// This bridge acting as a 2W controller (the role TaHoma plays), ported from
// laberning/home_io_control:
//   - PairingEngine (pairing_engine.cpp): discover_and_pair(),
//     pair_declared_device() and the scan_paired_devices roll-call (roll_call.cpp);
//   - ExchangeEngine::send_and_receive() (exchange_engine.cpp): the attempt
//     plans (decisions_attempts.h), channel access with a busy limit learned
//     from the board's quiet level (decisions_channel.h, channel_quiet_level.h),
//     the low-power wake belief, the challenge answer and the repeat rules of
//     an unconfirmed request;
//   - the cover command path (hub_operations.cpp): the EXECUTE payload, the
//     confirmation status read and the follow-up reads of a moving device
//     (hub_status.cpp, status_poll_policy.cpp).
// As in that component, an exchange blocks while it waits: it sends a try and
// then listens for the reply itself (run_exchange_(), its send_and_receive()
// with wait_for_packet()), answering a challenge at once, until the exchange
// ends. So do a pairing job's waits (run_job_(), the pairing engine's
// listens) and the authentication of a device's status update. A frame that
// arrives meanwhile and is not theirs is only overheard. Only one exchange
// runs at a time.
//
// As in that component, a device is commanded under this bridge's one system
// key without any per-device "bonded" record: a cover whose Mode is Two-Way is
// treated as paired, the way a 2W device declared in that component's YAML is.

#ifndef IOHC_CONTROLLER2W_H
#define IOHC_CONTROLLER2W_H

#include <Preferences.h>
#include <array>
#include <optional>
#include <string>
#include <vector>
#include <unordered_map>
#include "iohcRadio.h"
#include "iohcPacket.h"

namespace esphome {
namespace iohc {
class IOHCCover;
class IOHCComponent;
}  // namespace iohc
}  // namespace esphome

namespace IOHC {

    // What the pairing engine is doing (laberning's PairingState, one per wait).
    enum class PairStep : uint8_t {
        IDLE,
        DISCOVERY_DWELL,     // initial dwell on CH2 before the first 0x28
        DISCOVERY_WAIT,      // 0x28 sent, waiting for a 0x29
        CONFIRM_WAIT,        // 0x2C sent, waiting for a 0x2D (silence is not an error)
        KEY_INIT_DELAY,      // pause after the discover-confirm step, before the first 0x31
        KEY_ROUND_GAP,       // retry gap before the next key-exchange round
        KEY_CHALLENGE_WAIT,  // 0x31 sent, waiting for the 0x3C (or a direct 0x33)
        EXCHANGE,            // a send-and-receive of the pairing flow (0x32, 0x50 or 0x6F)
        ROLL_CALL_WAIT,      // 0x2A sent, collecting 0x2B replies
    };

    enum class PairJob : uint8_t { NONE, DISCOVER_AND_PAIR, PAIR_BY_ADDRESS, SCAN };

    // Passive key-sniffing (Findings 27/28): watches 2W traffic for a
    // KEY_INIT(0x31) -> CHALLENGE_REQUEST(0x3C) -> KEY_TRANSFER(0x32) sequence
    // between two OTHER devices (e.g. TaHoma pairing a motor).
    enum class SniffState : uint8_t {
        IDLE,
        SAW_KEY_INIT,
        SAW_CHALLENGE,
    };

    // The board's quiet level per channel (laberning's ChannelQuietLevel): a
    // histogram of idle RSSI readings, median per channel, halved after
    // QUIET_LEVEL_HALVING_READINGS readings.
    class ChannelQuietLevel {
    public:
        void add_reading(uint32_t freq_hz, int16_t level_dbm);
        std::optional<int16_t> quiet_level_dbm(uint32_t freq_hz) const;
        // The lowest channel quiet level, once every channel has enough readings.
        std::optional<int16_t> board_quiet_level_dbm() const;

    private:
        static constexpr int16_t MIN_DBM = -130;
        static constexpr int16_t MAX_DBM = -60;
        struct Channel {
            std::array<uint16_t, MAX_DBM - MIN_DBM + 1> bins{};
            uint16_t count{0};
            void halve();
        };
        static std::optional<size_t> channel_index(uint32_t freq_hz);
        std::array<Channel, 3> channels_{};
    };

    // Per-motor state of a Two-Way cover (laberning's IoDevice fields that the
    // command path reads and writes), keyed by the motor's 2W address.
    struct Session2W {
        esphome::iohc::IOHCCover *cover{nullptr};
        bool low_power{false};
        // Wake belief evidence (decisions::TargetEvidence).
        uint32_t last_moving_evidence_ms{0};
        uint32_t moving_until_ms{0};
        uint32_t last_status_ms{0};
        bool confirms_execute{false};
        bool reports_remaining_time{false};
        // A command waiting for the radio (one exchange at a time).
        bool command_pending{false};
        uint8_t pending_main0{0};
        uint8_t pending_main1{0};
        // Status polling (StatusPollPolicy's PollTracking).
        uint32_t next_poll_ms{0};      // 0 = no poll planned
        uint32_t poll_deadline_ms{0};  // end of the tracked follow-up window
        bool settling_read{false};     // the next poll decides where the motor ended up
        bool stop_settle{false};
        uint8_t status_poll_failures{0};
        uint8_t auth_poll_failures{0};
        // The read after boot (the entity's initial status request) and the
        // periodic refresh at rest (status_refresh_interval, 0 = off).
        uint32_t boot_read_ms{0};       // 0 = done
        uint32_t refresh_interval_ms{0};
        uint32_t next_refresh_ms{0};    // 0 = none planned
    };

    class IOHCController2W {
    public:
        // fixed_controller_hex/fixed_system_key_hex: the YAML identity
        // (controller_address/system_key); empty = generate and persist once.
        void begin(iohcRadio *radio, esphome::iohc::IOHCComponent *owner, const std::string &fixed_controller_hex = "",
                   const std::string &fixed_system_key_hex = "");

        void loop();

        // Called from IOHCComponent::on_receive() for every frame. Returns true
        // if the running exchange or a pairing step consumed it.
        bool handle_frame(IOHC::iohcPacket *packet);

        // laberning's Discover & Pair: broadcast 0x28, pair whichever device
        // answers. Put exactly one device in pairing mode first.
        void discover_and_pair();

        // laberning's pair_device action: the key exchange straight to a
        // device a cover declares as its motor_address, without discovery.
        void pair_device(const std::string &device_id_hex);

        // laberning's scan_paired_devices: a 0x2A roll-call under this
        // bridge's key; every device that holds it answers with a 0x2B.
        void scan_paired_devices();

        // A 2W command (EXECUTE main bytes: Open/Close/Position/Stop) to a
        // Two-Way cover's motor, under this bridge's system key.
        void send_command(const IOHC::address &to, bool low_power, uint8_t main0, uint8_t main1,
                          esphome::iohc::IOHCCover *cover);

        // A cover with a motor_address, from its setup(): while its Mode is
        // Two-Way, its status is read after boot and, with a refresh interval,
        // whenever it has reported none for that long.
        void register_cover(const IOHC::address &motor, esphome::iohc::IOHCCover *cover, bool low_power,
                            uint32_t refresh_interval_ms);

        // An idle RSSI reading of the channel the receiver sat on (called on
        // every QUIET_LEVEL_READING_EVERY_N_HOPS-th hop, as in laberning's hub).
        void note_idle_reading(uint32_t freq_hz, int16_t level_dbm);

        // True while a pairing attempt, a scan or an exchange runs. The hub
        // decodes received frames during that time regardless of the debug switches.
        bool bonding_active() const;

    private:
        static uint32_t pack_address(const IOHC::address &addr);

        void load_or_generate_identity(const std::string &fixed_controller_hex);
        void load_or_generate_system_key(const std::string &fixed_system_key_hex);
        void reset_sniff_state();

        // --- Pairing engine (pairing_engine.cpp) ---
        bool start_job_(PairJob job, const char *opening);
        void finish_job_(const char *outcome);
        void send_discovery_attempt_(uint8_t attempt);
        void on_discovery_response_(const IOHC::iohcPacket *packet, const uint8_t *data, uint8_t data_len);
        void send_discover_confirm_(uint8_t try_index);
        void start_key_round_(uint8_t round);
        // Runs the started pairing job to its end (the pairing engine's
        // blocking listens); on_step_due_() advances a step whose wait ran out.
        void run_job_();
        void on_step_due_();
        void key_round_failed_();
        void key_held_();
        void complete_paired_();
        void verify_device_holds_key_();

        // --- Send-and-receive (exchange_engine.cpp) ---
        enum class ExchangeFor : uint8_t { KEY_TRANSFER, VERIFY_KEY, SET_CONFIG1, COMMAND, STATUS_READ };
        enum class ExchangePhase : uint8_t { GAP, WAIT_FIRST, WAIT_FINAL };
        enum class ExchangeOutcome : uint8_t { SUCCESS_WITH_RESPONSE, SUCCESS_UNCONFIRMED, FAILED };
        enum class WakeBelief : uint8_t { NONE, ASLEEP, AWAKE, UNKNOWN };
        struct AttemptPlan {
            uint8_t max_attempts{3};
            uint16_t gap_ms{0};
            uint16_t window_ms{0};
            uint16_t early_window_ms{0};
            bool short_lead{false};
            uint16_t short_lead_window_ms{0};
            bool bounded_by_attempts{false};
            uint16_t window_for_attempt(uint8_t attempt, uint8_t attempts) const;
        };
        struct Exchange {
            bool active{false};
            ExchangeFor what{ExchangeFor::KEY_TRANSFER};
            ExchangePhase phase{ExchangePhase::GAP};
            IOHC::address dst{};
            uint8_t cmd{0};
            uint8_t data[16]{};
            uint8_t data_len{0};
            bool start_frame{false};
            bool low_power{false};
            bool wake_up_channels{false};  // pairing frames: CH2 only
            uint16_t preamble_override{0}; // pairing frames: fixed preamble
            uint32_t session_freq{0};      // a continuation goes out on this channel
            AttemptPlan plan;
            WakeBelief belief{WakeBelief::NONE};
            uint8_t tries_allowed{0};
            uint8_t tries{0};
            uint8_t unconfirmed_tries{0};
            uint32_t begin_ms{0};
            uint32_t due_ms{0};
            uint32_t follow_up_gap_ms{0};
            uint32_t auth_tail_ms{0};
            uint32_t request_freq{0};
            uint32_t previous_freq{0};
            uint16_t try_preamble{0};
            bool saw_challenge{false};
            // Reply of a successful exchange.
            uint8_t reply_cmd{0};
            uint8_t reply_data[32]{};
            uint8_t reply_len{0};
            // COMMAND / STATUS_READ bookkeeping.
            bool is_stop{false};
            bool log_held{false};  // this try holds the frame log
        } exchange_;
        bool start_exchange_(ExchangeFor what, const IOHC::address &dst, uint8_t cmd, const uint8_t *data,
                             uint8_t data_len, bool start_frame, bool low_power, const AttemptPlan &plan,
                             bool wake_up_channels, uint16_t preamble_override, uint32_t session_freq);
        // Runs the started exchange to its end (send_and_receive()).
        void run_exchange_();
        void on_exchange_timeout_();
        // A frame for the running exchange: true if it took it.
        bool exchange_frame_(IOHC::iohcPacket *packet);
        void next_try_();
        // The frame log hold of one try (frame_log_hold()).
        void hold_try_log_();
        void release_try_log_();
        void transmit_try_();
        void answer_challenge_(const uint8_t *challenge6);
        void try_ended_unconfirmed_();
        void end_exchange_(ExchangeOutcome outcome);
        void exchange_done_(ExchangeOutcome outcome);
        WakeBelief wake_belief_for_(const IOHC::address &dst, bool low_power, bool start_frame, uint8_t cmd,
                                    bool is_stop, bool state_unknown) const;
        uint16_t preamble_for_try_(uint8_t try_index) const;
        AttemptPlan count_plan_(uint8_t tries, bool start_frame) const;
        AttemptPlan command_plan_(bool low_power, bool repeat_safe, uint32_t tx_ms) const;
        AttemptPlan settling_plan_(bool low_power, bool short_lead, uint32_t tx_ms) const;
        AttemptPlan progress_plan_(bool start_frame) const;

        // --- Cover commands and status reads (hub_operations.cpp, hub_status.cpp) ---
        void run_pending_work_();
        void run_command_(const IOHC::address &to, Session2W &s);
        void start_status_read_(const IOHC::address &to, Session2W &s);
        void command_done_(ExchangeOutcome outcome);
        void status_read_done_(ExchangeOutcome outcome);
        void apply_private_response_(Session2W &s, bool trust_position);
        // Applies a decoded report (status byte, target and current position,
        // optional remaining time) to the cover and plans the follow-up read.
        void apply_status_report_(Session2W &s, const IOHC::address &motor, uint8_t status_byte,
                                  std::optional<uint16_t> target, std::optional<uint16_t> current,
                                  std::optional<uint16_t> remaining_s, bool trusted_poll);

        // --- Device-originated status updates (hub_status.cpp, challenge_sender()) ---
        // A 0x71 to this bridge is authenticated first: our 0x3C, the device's
        // 0x3D checked under the system key, then the 0x72 acknowledgement on
        // all three channels (the receive channel first), then the update applied.
        void on_status_update_(const IOHC::iohcPacket *packet, const uint8_t *data, uint8_t data_len);
        // True when the device's 0x3D verifies; fills ack_order.
        bool on_status_update_answer_(const uint8_t *data, uint8_t data_len);
        void apply_inbound_update_();
        // Waits until the radio has sent what it was sending.
        void wait_tx_idle_(uint32_t timeout_ms);
        struct InboundAuth {
            bool active{false};
            IOHC::address src{};
            uint8_t update[32]{};
            uint8_t update_len{0};
            uint8_t challenge[6]{};
            uint32_t freq{0};
            uint32_t due_ms{0};
            std::array<uint32_t, 3> ack_order{};
        } inbound_;
        void schedule_poll_failure_(Session2W &s, bool saw_challenge);
        // Every status a device reports moves its next refresh a whole interval out.
        void note_status_reported_(Session2W &s);
        void begin_status_poll_tracking_(Session2W &s, uint32_t delay_ms);

        // --- Roll-call (roll_call.cpp) ---
        void send_roll_call_attempt_();
        void report_roll_call_();

        // --- Channel access and transmission ---
        enum class ChannelAccessStatus : uint8_t { SENT, CHANNEL_BUSY };
        int16_t busy_limit_dbm_() const;
        int16_t measure_channel_level_(uint32_t freq);
        uint32_t draw_request_channel_(bool wake_up_channels, uint32_t avoid_freq);
        // A request that opens a session takes a free channel (ADR 0096):
        // CH2 for a low-power or pairing frame, any of the three otherwise,
        // backing off while busy; nothing is sent when the access time runs
        // out busy.
        ChannelAccessStatus send_with_channel_access_(iohcPacket *packet, uint16_t preamble, bool wake_up_channels,
                                                      uint32_t previous_freq, uint32_t &out_freq);
        // A sender that keeps a fixed channel (discovery, roll-call): up to
        // LBT_MAX_RETRIES checks against the busy limit, then sent anyway.
        void send_fixed_channel_(iohcPacket *packet, uint32_t freq, uint16_t preamble);
        // A frame that answers within a session goes out at once on its channel.
        void send_at_once_(iohcPacket *packet, uint32_t freq, uint16_t preamble);
        static uint32_t tx_time_ms_(uint16_t preamble, uint8_t frame_len);
        void listen_on_(uint32_t freq);
        bool from_device_to_us_(const IOHC::iohcPacket *packet, const IOHC::address &device) const;
        IOHC::iohcPacket *build_request_(const IOHC::address &dst, uint8_t cmd, const uint8_t *data, uint8_t data_len,
                                         bool start, bool end, bool low_power) const;

        iohcRadio *radio_{};
        esphome::iohc::IOHCComponent *owner_{nullptr};
        ::Preferences identity_prefs_;
        IOHC::address controller_address_{};
        uint8_t system_key_[16]{};
        ChannelQuietLevel quiet_level_;
        int16_t logged_busy_limit_dbm_{0};

        // Pairing context (laberning's pairing::PairingContext).
        PairJob job_{PairJob::NONE};
        PairStep step_{PairStep::IDLE};
        uint32_t step_due_ms_{0};
        uint32_t job_started_ms_{0};
        IOHC::address device_{};
        bool device_known_{false};
        bool discovery_low_power_{false};
        bool saw_key_challenge_{false};
        bool key_refused_{false};
        uint8_t discovery_attempt_{0};
        uint8_t confirm_try_{0};
        uint8_t key_round_{0};
        uint32_t session_freq_{0};

        struct RollCallResponder {
            IOHC::address src{};
            float rssi{0};
            uint32_t freq{0};
            uint8_t flags{0};
            bool has_flags{false};
        };
        uint8_t roll_call_attempt_{0};
        uint32_t roll_call_tx_freq_{0};
        std::vector<RollCallResponder> roll_call_responders_;

        std::unordered_map<uint32_t, Session2W> sessions_;  // keyed by pack_address(motor)

        struct KeySniff {
            SniffState state{SniffState::IDLE};
            IOHC::address box{};
            IOHC::address device{};
            uint8_t challenge[6]{};
        } sniff_;
    };
}
#endif
