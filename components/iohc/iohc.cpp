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

void IOHCComponent::setup() {
  ESP_LOGCONFIG(TAG, "Starting IOHC radio (vendored rspaargaren/iohomecontrol stack)...");
  IOHC::IohcPacketDelegate rx_cb(&IOHCComponent::on_receive, this);
  IOHC::iohcRadio::getInstance()->start(1, scan_freqs, 0, rx_cb, nullptr);
  controller2w_.begin(IOHC::iohcRadio::getInstance(), this, fixed_controller_hex_, fixed_system_key_hex_);
}

void IOHCComponent::loop() {
  // Bonding-attempt timeouts and per-command timeouts - see
  // iohc_controller2w.cpp's loop().
  controller2w_.loop();
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
  // comment on why this isn't the default (Finding 32).
  if (!manual_hop_wanted_ && !bonding_hop_wanted_)
    return;
  // Never yank the radio off-frequency mid-reception, and never hop while
  // a TX is in flight (belt-and-suspenders - see iohc.h's comment on why
  // there's no actual race here, this is just cheap extra safety).
  auto state = IOHC::iohcRadio::radioState;
  if (state == IOHC::iohcRadio::RadioState::PREAMBLE || state == IOHC::iohcRadio::RadioState::PAYLOAD ||
      state == IOHC::iohcRadio::RadioState::TX) {
    return;
  }
  uint32_t now = esphome::micros();
  if ((now - last_hop_us_) < HOP_INTERVAL_US)
    return;
  last_hop_us_ = now;
  hop_channel_idx_ = (hop_channel_idx_ + 1) % 3;
  IOHC::iohcRadio::getInstance()->retune(HOP_CHANNELS[hop_channel_idx_]);
}

void IOHCComponent::hop_wanted_changed_() {
  if (!manual_hop_wanted_ && !bonding_hop_wanted_) {
    // Nobody wants the hop anymore - go back to CH2 immediately rather
    // than waiting for the next (now-disabled) hop tick to happen to land
    // there.
    IOHC::iohcRadio::getInstance()->retune(CHANNEL2);
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

bool IOHCComponent::on_receive(IOHC::iohcPacket *packet) {
  this->packets_received_++;

  // While both switches are off, this bridge only transmits and never decodes
  // received traffic (see set_passive_decode_wanted() in iohc.h).
  if (!passive_decode_wanted_ && !confirm_resend_wanted_) {
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
