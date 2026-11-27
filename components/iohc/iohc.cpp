#include "iohc.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "cover/iohc_cover.h"
#include "esp_timer.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc";

// MAX_FREQS stays 1 here (iohcRadio's own ISR-driven auto-hop stays
// permanently disabled, see iohc.h's maybe_hop_() comment) - CH2 is still
// the single frequency iohcRadio itself is "started" on; the 3-channel hop
// is layered on top cooperatively via maybe_hop_() below.
static uint32_t scan_freqs[1] = {CHANNEL2};

// Cooperative hop, always running (laberning/home_io_control's idle
// listening), with its timing: HOP_TIME_US per channel for the idle
// listen (proto_timing.h), the SX1276 hop slice for the rotating listens of
// pairing and the roll-call (sx1276_discovery_hop_slice_ms), and the hold
// rules of a reception in progress (radio_sx1276.h, radio_preamble_hold.h,
// radio_interface.h, proto_timing.h).
static constexpr uint32_t IDLE_HOP_TIME_US = 2700;
static constexpr uint32_t ROTATING_HOP_SLICE_US = 5000;
static constexpr uint32_t PREAMBLE_LINGER_DWELL_MS = 15;
static constexpr uint32_t IDLE_PREAMBLE_HOLD_MS = 30;
static constexpr uint32_t IDLE_RENEW_REDETECT_MS = 4;
static constexpr uint32_t IDLE_PREAMBLE_HOLD_CHAIN_MS = 300;
static constexpr uint32_t RX_HOP_HOLDOFF_US = 12000;
// A held plan (no hop) clears a PREAMBLE flag that no frame followed after this.
static constexpr uint32_t STALE_PREAMBLE_MS = 30;
static constexpr uint8_t QUIET_LEVEL_READING_EVERY_N_HOPS = 10;

void IOHCComponent::setup() {
  ESP_LOGCONFIG(TAG, "Starting IOHC radio (vendored rspaargaren/iohomecontrol stack)...");
  IOHC::IohcPacketDelegate rx_cb(&IOHCComponent::on_receive, this);
  IOHC::iohcRadio::getInstance()->start(1, scan_freqs, 0, rx_cb, nullptr);
  // The idle listening hops every 2.7 ms, faster than ESPHome's normal loop.
  high_freq_loop_.start();
  controller2w_.begin(IOHC::iohcRadio::getInstance(), this, fixed_controller_hex_, fixed_system_key_hex_);
  key_extractor_.begin(IOHC::iohcRadio::getInstance(), this);
}

void IOHCComponent::loop() {
  // Bonding-attempt timeouts and per-command timeouts - see
  // iohc_controller2w.cpp's loop().
  controller2w_.loop();
  // "Recover System Key" responder's own timers (10 minute arm window, 1
  // minute post-extraction grace, the deferred slotted discovery answer) -
  // see iohc_key_extraction.cpp's loop().
  key_extractor_.loop();
  maybe_hop_();
  // Drains one queued send batch if the radio is idle and the minimum
  // inter-send cooldown has elapsed - see iohcRadio::startQueuedSend()'s own
  // comment. Also called directly from iohcRadio::send(), so this is what
  // catches a batch that arrived while still cooling down and retries it
  // once the cooldown passes, rather than leaving it stuck until some
  // unrelated new send() happens to come in.
  IOHC::iohcRadio::getInstance()->startQueuedSend();
}

void IOHCComponent::maybe_hop_() {
  const uint32_t now = esphome::micros();
  const uint32_t now_ms = esphome::millis();
  auto state = IOHC::iohcRadio::radioState;

  // While a pairing attempt or scan waits, say how the listening goes: hops,
  // preambles seen, stuck preambles cleared. One line per 2 s.
  if (bonding_hop_wanted_) {
    if (now_ms - hop_stats_ms_ >= 2000) {
      ESP_LOGI(TAG, "Bonding RX: %lu hops, %lu preambles, %lu stuck preambles cleared in the last %lu ms",
               (unsigned long) hop_count_, (unsigned long) preamble_count_, (unsigned long) stale_recoveries_,
               (unsigned long) (now_ms - hop_stats_ms_));
      hop_stats_ms_ = now_ms;
      hop_count_ = preamble_count_ = stale_recoveries_ = 0;
    }
  }

  // A plan set by a pairing step, scan or exchange wins over the idle
  // listening while it runs.
  const HopPlan plan = plan_override_ ? hop_plan_ : HopPlan::ALL;
  const bool rotating = plan == HopPlan::ALL || plan == HopPlan::SKIP_FREQ;
  const bool idle_listen = rotating && !plan_override_;

  if (state == IOHC::iohcRadio::RadioState::TX) {
    sync_since_us_ = 0;
    return;
  }

  // A frame whose sync word arrived holds the channel, bounded by
  // RX_HOP_HOLDOFF_US so a sync with no frame behind it cannot wedge the hop.
  if (state == IOHC::iohcRadio::RadioState::PAYLOAD) {
    preamble_hold_active_ = false;
    awaiting_redetect_ = false;
    if (sync_since_us_ == 0) {
      sync_since_us_ = now;
    } else if (now - sync_since_us_ >= RX_HOP_HOLDOFF_US) {
      IOHC::iohcRadio::getInstance()->recover_rx();
      stale_recoveries_++;
      sync_since_us_ = 0;
    }
    return;
  }
  sync_since_us_ = 0;

  // A preamble on this channel holds it. Rotating listens of pairing and the
  // roll-call linger PREAMBLE_LINGER_DWELL_MS once; the idle listen holds
  // IDLE_PREAMBLE_HOLD_MS slices, renewed on the same channel (receiver
  // restarted, IDLE_RENEW_REDETECT_MS to see the preamble again) up to
  // IDLE_PREAMBLE_HOLD_CHAIN_MS (laberning's PreambleHold::hold_renewing()).
  // A held plan only clears a stale flag so it cannot freeze reception.
  const bool preamble_seen = state == IOHC::iohcRadio::RadioState::PREAMBLE;
  if (preamble_seen && !preamble_counted_) {
    preamble_counted_ = true;
    preamble_count_++;
  }
  if (!preamble_seen)
    preamble_counted_ = false;
  {
    const uint32_t slice_ms = idle_listen ? IDLE_PREAMBLE_HOLD_MS : (rotating ? PREAMBLE_LINGER_DWELL_MS : STALE_PREAMBLE_MS);
    const uint32_t chain_ms = idle_listen ? IDLE_PREAMBLE_HOLD_CHAIN_MS : slice_ms;
    if (preamble_hold_active_) {
      if (now_ms - preamble_slice_start_ms_ < slice_ms)
        return;
      preamble_hold_active_ = false;
      // The receiver is restarted on the held channel either way: a retune
      // leaves the PreambleDetect flag latched.
      IOHC::iohcRadio::getInstance()->recover_rx();
      stale_recoveries_++;
      if (now_ms - preamble_chain_start_ms_ < chain_ms) {
        awaiting_redetect_ = true;
        redetect_since_ms_ = now_ms;
        return;
      }
      awaiting_redetect_ = false;
    } else if (preamble_seen) {
      preamble_hold_active_ = true;
      preamble_slice_start_ms_ = now_ms;
      if (!awaiting_redetect_)
        preamble_chain_start_ms_ = now_ms;
      awaiting_redetect_ = false;
      return;
    } else if (awaiting_redetect_) {
      if (now_ms - redetect_since_ms_ < IDLE_RENEW_REDETECT_MS)
        return;
      awaiting_redetect_ = false;
    }
  }

  if (!rotating)
    return;
  // Dwell per channel: laberning's HOP_TIME_US for the idle listen, the
  // SX1276 hop slice for a rotating listen of pairing and the roll-call.
  const uint32_t dwell_us = idle_listen ? IDLE_HOP_TIME_US : ROTATING_HOP_SLICE_US;
  if ((now - last_hop_us_) <= dwell_us)
    return;
  last_hop_us_ = now;
  // laberning's rotation: CH2 -> CH3 -> CH1 -> CH2, skipping `skip_freq_`.
  uint32_t freq = hop_freq_;
  do {
    freq = freq == CHANNEL1 ? CHANNEL2 : (freq == CHANNEL3 ? CHANNEL1 : CHANNEL3);
  } while (plan == HopPlan::SKIP_FREQ && freq == skip_freq_);
  // The receiver has sat on the channel it leaves for the whole dwell: one
  // idle reading of it every QUIET_LEVEL_READING_EVERY_N_HOPS hops teaches
  // the 2W channel access this board's quiet level.
  if (++hops_since_reading_ >= QUIET_LEVEL_READING_EVERY_N_HOPS) {
    hops_since_reading_ = 0;
    controller2w_.note_idle_reading(hop_freq_, static_cast<int16_t>(Radio::readRssi()));
  }
  hop_count_++;
  hop_freq_ = freq;
  IOHC::iohcRadio::getInstance()->retune(freq);
}

void IOHCComponent::set_hop_plan(HopPlan plan, uint32_t freq) {
  hop_plan_ = plan;
  plan_override_ = true;
  if (plan == HopPlan::SKIP_FREQ)
    skip_freq_ = freq != 0 ? freq : CHANNEL2;
  if (plan == HopPlan::HOLD_REQUEST) {
    IOHC::iohcRadio::getInstance()->retune(CHANNEL2);
    hop_freq_ = CHANNEL2;
  } else if (plan == HopPlan::HOLD_FREQ) {
    hold_freq_ = freq != 0 ? freq : CHANNEL2;
    IOHC::iohcRadio::getInstance()->retune(hold_freq_);
    hop_freq_ = hold_freq_;
  }
}

void IOHCComponent::release_hop_plan() {
  // Back to the idle listening.
  plan_override_ = false;
  hop_plan_ = HopPlan::ALL;
}

void IOHCComponent::set_bonding_hop_wanted(bool wanted) { bonding_hop_wanted_ = wanted; }

void IOHCComponent::set_responder_hop_wanted(bool wanted) { responder_hop_wanted_ = wanted; }

bool IOHCComponent::on_receive(IOHC::iohcPacket *packet) {
  this->packets_received_++;

  // While both switches are off, this bridge only transmits and never decodes
  // received traffic (see set_passive_decode_wanted() in iohc.h).
  if (!passive_decode_wanted_ && !confirm_resend_wanted_ && !controller2w_.bonding_active() &&
      !key_extractor_.armed()) {
    return false;
  }

  if (passive_decode_wanted_) {
    ESP_LOGI(TAG, "Frame #%lu received, length=%u, rssi=%.1f dBm", this->packets_received_, packet->buffer_length,
             packet->rssi);
  }

  // This bridge's own 2W bonding (Phase 3) - only ever consumes a frame
  // while a bonding attempt is armed AND the frame's source matches that
  // attempt's specific target motor (see IOHCController2W::handle_frame()),
  // so this is a cheap no-op in the common case.
  controller2w_.handle_frame(packet);

  // "Recover System Key" responder - only ever consumes a frame while
  // armed AND the frame is one of its own (0x28, or 0x2C/0x31/0x32/0x36/0x3C
  // addressed to its throwaway ID), so this is a cheap no-op otherwise. See
  // iohc_key_extraction.h.
  key_extractor_.handle_frame(packet);

  const auto &header = packet->payload.packet.header;
  const uint8_t *source = header.source;

  // RSSI of every frame a cover's own motor sent (1W or 2W, decoded or not).
  for (auto *cover : this->status_covers_) {
    if (cover->motor_matches(source)) {
      cover->update_last_rssi(packet->rssi);
    }
  }

  // Passive 2W status: a 2W (Protocol bit clear) cmd 0x04 "Private Command
  // Answer", sent by a motor to whichever controller polled it. Layout in
  // iohcPacket.h (_p0x04_14): status, one unknown byte, Main (position echo,
  // raw/512 = closure %, 0 open to 100 closed), four unmapped bytes, then the
  // 3-byte address of the controller that last commanded the motor. After a
  // 1W command of ours that address is our own cover's remote address, so the
  // answer can be matched without knowing the motor's address. Needs the
  // whole struct in the frame (buffer_length >= 20).
  //
  // Nothing from this path writes into a cover's own position or state: the
  // RX pipeline has no integrity check, so a garbled frame could otherwise
  // corrupt the entity used for control. It only reaches the standalone
  // sensors and the log (see IOHCCover::on_status_heard()).
  if (!header.CtrlByte1.asStruct.Protocol && header.cmd == 0x04 && packet->buffer_length >= 20 &&
      packet->buffer_length <= MAX_FRAME_LEN) {
    const auto &answer = packet->payload.packet.msg.p0x04_14;
    this->last_status_ms_ = static_cast<uint32_t>(esp_timer_get_time() / 1000);
    IOHCCover *match = nullptr;
    bool commander_is_ours = false;
    for (auto *cover : this->status_covers_) {
      if (cover->commander_matches(answer.commanding_controller)) {
        match = cover;
        commander_is_ours = true;
        break;
      }
    }
    if (match == nullptr) {
      for (auto *cover : this->status_covers_) {
        if (cover->motor_matches(source)) {
          match = cover;
          break;
        }
      }
    }
    if (match == nullptr) {
      ESP_LOGD(TAG, "Status answer from %02X%02X%02X (commander %02X%02X%02X) matches no cover", source[0], source[1],
               source[2], answer.commanding_controller[0], answer.commanding_controller[1],
               answer.commanding_controller[2]);
    } else {
      uint16_t raw = (static_cast<uint16_t>(answer.main[0]) << 8) | answer.main[1];
      // Only 0x0000-0xC800 is a closure percentage. Main echoes the last
      // command's parameter: after Stop (0xD200), Vent/Favorite (0xD800),
      // Secured target (0xD100) or Default (0xD300) it is not a position, and
      // dividing it would show a plausible but wrong 100% (seen 2026-07-12).
      bool position_valid = raw <= 0xC800;
      match->on_status_heard(position_valid ? raw / 512.0f : 0.0f, position_valid, answer.status, commander_is_ours,
                             packet->rssi, static_cast<float>(packet->afc));
    }
  }

  return true;
}

bool IOHCComponent::status_feedback_available() const {
  if (this->last_status_ms_ == 0) {
    return false;
  }
  uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  return (now - this->last_status_ms_) < 6UL * 60UL * 60UL * 1000UL;
}

void IOHCComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "IOHC (Somfy IO / io-homecontrol) radio:");
  ESP_LOGCONFIG(TAG, "  Phase 0/1: vendored radio stack, single fixed channel (868.95MHz), RX only");
}

}  // namespace iohc
}  // namespace esphome
