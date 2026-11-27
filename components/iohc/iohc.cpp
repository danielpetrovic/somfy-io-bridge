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

// Fast cooperative hop cadence (Finding 31/32) - matches
// home_io_control/proto_timing.h's own documented ~2.7ms/3-channel idle
// hop (roughly 900us/channel), only while something has explicitly
// requested it (see iohc.h's set_manual_hop_wanted()/set_bonding_hop_wanted()).
// Real achieved rate depends on how fast ESPHome's own loop() cycles on
// this hardware - this is the target, not a guarantee, since there's no
// hardware timer backing it (deliberately, see iohc.h).
static constexpr uint32_t HOP_INTERVAL_US = 900;
static constexpr uint32_t HOP_CHANNELS[3] = {CHANNEL2, CHANNEL1, CHANNEL3};
// Discovery replies come back on CH1 or CH3, almost never on the CH2 request
// channel, so the discovery wait only visits those two.
static constexpr uint32_t HOP_CHANNELS_DISCOVERY[2] = {CHANNEL1, CHANNEL3};
// A PREAMBLE state longer than this with no frame behind it is a stale flag.
static constexpr uint32_t STALE_PREAMBLE_US = 30000;

void IOHCComponent::setup() {
  ESP_LOGCONFIG(TAG, "Starting IOHC radio (vendored rspaargaren/iohomecontrol stack)...");
  IOHC::IohcPacketDelegate rx_cb(&IOHCComponent::on_receive, this);
  IOHC::iohcRadio::getInstance()->start(1, scan_freqs, 0, rx_cb, nullptr);
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
  // Disabled (CH2 only) unless something explicitly wants it - see iohc.h's
  // comment on why this isn't the default (Finding 32). The key-extraction
  // responder never actually hops (its HopPlan::HOLD_FREQ branch below is a
  // no-op, same as HOLD_REQUEST) - it only wants this function running for
  // the stale-PREAMBLE recovery check.
  if (!manual_hop_wanted_ && !bonding_hop_wanted_ && !responder_hop_wanted_)
    return;

  uint32_t now = esphome::micros();
  auto state = IOHC::iohcRadio::radioState;

  // While a bonding attempt waits for the 0x29 reply, say how the listening
  // goes: hops, preambles seen, stuck preambles cleared. One line per 2 s.
  if (bonding_hop_wanted_) {
    uint32_t now_ms = esphome::millis();
    if (now_ms - hop_stats_ms_ >= 2000) {
      ESP_LOGI(TAG, "Bonding RX: %lu hops, %lu preambles, %lu stuck preambles cleared in the last %lu ms",
               (unsigned long) hop_count_, (unsigned long) preamble_count_, (unsigned long) stale_recoveries_,
               (unsigned long) (now_ms - hop_stats_ms_));
      hop_stats_ms_ = now_ms;
      hop_count_ = preamble_count_ = stale_recoveries_ = 0;
    }
  }

  // A retune while receiving leaves the chip's PreambleDetect flag latched
  // and the interrupt is rising-edge only, so a PREAMBLE state that no frame
  // follows would last forever and freeze the hop on one channel. A real
  // discovery reply has a preamble of about 14 ms, so 30 ms in PREAMBLE
  // without a frame behind it is stale: cycle the receiver once.
  if (state == IOHC::iohcRadio::RadioState::PREAMBLE) {
    if (!preamble_counted_) {
      preamble_counted_ = true;
      preamble_count_++;
    }
    if (preamble_since_us_ == 0) {
      preamble_since_us_ = now;
    } else if ((now - preamble_since_us_) > STALE_PREAMBLE_US) {
      IOHC::iohcRadio::getInstance()->recover_rx();
      stale_recoveries_++;
      preamble_since_us_ = 0;
      preamble_counted_ = false;
    }
    return;
  }
  preamble_since_us_ = 0;
  preamble_counted_ = false;

  // Never hop while a frame is arriving or a TX is in flight.
  if (state == IOHC::iohcRadio::RadioState::PAYLOAD || state == IOHC::iohcRadio::RadioState::TX) {
    return;
  }

  // Which channels to visit. The manual switch always sweeps all three; an
  // armed bonding attempt or key-extraction session follows its plan (see
  // HopPlan).
  HopPlan plan = manual_hop_wanted_ ? HopPlan::ALL : hop_plan_;
  if (plan == HopPlan::HOLD_REQUEST || plan == HopPlan::HOLD_FREQ)
    return;
  if ((now - last_hop_us_) < HOP_INTERVAL_US)
    return;
  last_hop_us_ = now;
  uint32_t freq;
  if (plan == HopPlan::DISCOVERY) {
    hop_channel_idx_ = (hop_channel_idx_ + 1) % 2;
    freq = HOP_CHANNELS_DISCOVERY[hop_channel_idx_];
  } else {
    hop_channel_idx_ = (hop_channel_idx_ + 1) % 3;
    freq = HOP_CHANNELS[hop_channel_idx_];
  }
  hop_count_++;
  IOHC::iohcRadio::getInstance()->retune(freq);
}

void IOHCComponent::hop_wanted_changed_() {
  // Only an active bonding attempt needs the fast loop (it actually hops,
  // many times a second, to catch a discovery reply): the key-extraction
  // responder sits on one channel throughout and only needs maybe_hop_()
  // running at the ordinary loop() rate for its stale-PREAMBLE check.
  if (bonding_hop_wanted_) {
    high_freq_loop_.start();
  } else {
    high_freq_loop_.stop();
  }
  if (!manual_hop_wanted_ && !bonding_hop_wanted_ && !responder_hop_wanted_) {
    // Nobody wants the hop anymore - go back to CH2 immediately rather
    // than waiting for the next (now-disabled) hop tick to happen to land
    // there.
    IOHC::iohcRadio::getInstance()->retune(CHANNEL2);
    hop_plan_ = HopPlan::ALL;
    preamble_since_us_ = 0;
    preamble_counted_ = false;
  }
}

void IOHCComponent::set_hop_plan(HopPlan plan, uint32_t hold_freq) {
  hop_plan_ = plan;
  if (manual_hop_wanted_)
    return;  // the manual switch always sweeps all three regardless.
  if (plan == HopPlan::HOLD_REQUEST) {
    IOHC::iohcRadio::getInstance()->retune(CHANNEL2);
  } else if (plan == HopPlan::HOLD_FREQ) {
    hold_freq_ = hold_freq != 0 ? hold_freq : CHANNEL2;
    IOHC::iohcRadio::getInstance()->retune(hold_freq_);
  }
}

void IOHCComponent::set_manual_hop_wanted(bool wanted) {
  manual_hop_wanted_ = wanted;
  hop_wanted_changed_();
}

void IOHCComponent::set_bonding_hop_wanted(bool wanted) {
  bonding_hop_wanted_ = wanted;
  hop_wanted_changed_();
}

void IOHCComponent::set_responder_hop_wanted(bool wanted) {
  responder_hop_wanted_ = wanted;
  hop_wanted_changed_();
}

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
