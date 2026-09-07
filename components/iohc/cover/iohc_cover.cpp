#include "iohc_cover.h"
#include "esphome/core/log.h"
#include "../iohcCryptoHelpers.h"
#include <algorithm>
#include <cmath>
#include <functional>

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.cover";

// ESP32 NVS namespace names are capped at 15 chars - hash the nvs_key down
// to a short, fixed, deterministic one, same approach as iohc_remote1w.cpp.
static std::string cover_nvs_namespace_for(const std::string &nvs_key) {
  size_t h = std::hash<std::string>{}(nvs_key);
  char buf[11];
  snprintf(buf, sizeof(buf), "ic%08x", static_cast<unsigned>(h));
  return std::string(buf);
}

void IOHCCover::setup() {
  remote_.set_type(type_);
  remote_.set_manufacturer(manufacturer_);
  // Bonded identity/sequence persist per-cover (see iohc_remote1w.cpp) -
  // nvs_key_ is only used to derive a stable NVS namespace, not as a
  // protocol address.
  remote_.begin(IOHC::iohcRadio::getInstance(), this->nvs_key_, fixed_node_hex_, fixed_key_hex_);

  cover_prefs_namespace_ = cover_nvs_namespace_for(this->nvs_key_);
  cover_prefs_.begin(cover_prefs_namespace_.c_str(), false);
  mode_ = static_cast<Mode>(cover_prefs_.getUChar("mode", static_cast<uint8_t>(Mode::POSITION)));
  // Live value the remote actually uses - falls back to the compile-time
  // YAML-resolved seed only on first-ever boot (no persisted value yet).
  // Set live via the My Pattern switch from here on - see
  // set_my_pattern_extended() below. Key is "my_pattern_ext", NOT the
  // more obvious "my_pattern_extended" - ESP32 NVS keys are capped at 15
  // characters (confirmed the hard way, 2026-08-14: the switch appeared
  // to toggle in HA but never actually persisted across a reboot, since
  // both putBool()/getBool() on a 20-character key silently do nothing).
  my_pattern_extended_ = cover_prefs_.getBool("my_pattern_ext", my_pattern_default_extended_);
  remote_.set_my_pattern_extended(my_pattern_extended_);
  // Same NVS pattern as my_pattern_extended_ above - "invert" is well
  // under the 15-char key limit, no truncation risk.
  invert_ = cover_prefs_.getBool("invert", invert_default_);

  remote_.set_travel_time_open(TRAVEL_TIME_OPEN);
  remote_.set_travel_time_close(TRAVEL_TIME_CLOSE);

  // Restore last known position (defaults to open if never set). Persisted
  // "position" is always HA-space (see control()/loop()'s own comments) -
  // the tracker itself is raw/motor-space, so this needs converting back
  // on an inverted cover, or the local travel-time estimate starts out
  // backwards until the next real move corrects it.
  this->position = cover_prefs_.getFloat("position", 1.0f);
  if (mode_ == Mode::POSITION) {
    float raw_position = invert_ ? (1.0f - this->position) : this->position;
    remote_.position_tracker().setPosition(raw_position * 100.0f);
  }

}

void IOHCCover::set_motor_address(const std::string &motor_address_hex) {
  if (motor_address_hex.empty())
    return;
  hexStringToBytes(motor_address_hex, motor_address_);
  has_motor_address_ = true;
}

void IOHCCover::update_real_position_authoritative(float closure_percent) {
  closure_percent = std::clamp(closure_percent, 0.0f, 100.0f);
  // closure (0=open/100=closed) -> HA cover convention (0=closed/1=open) -
  // this comes from the motor's own reported physical state, independent
  // of which RemoteButton we chose to send, so an inverted cover needs its
  // own flip here too (Invert Direction), separate from the control()-side
  // inversion below.
  float open_percent = invert_ ? (closure_percent / 100.0f) : (1.0f - closure_percent / 100.0f);
  ESP_LOGI(TAG, "2W command confirmed - real position now %.0f%% open", open_percent * 100.0f);
  this->position = open_percent;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  target_position_ = -1.0f;
  cover_prefs_.putFloat("position", open_percent);
  this->publish_state();
}

void IOHCCover::set_mode(Mode mode) {
  mode_ = mode;
  cover_prefs_.putUChar("mode", static_cast<uint8_t>(mode));
}

void IOHCCover::set_my_pattern_extended(bool extended) {
  my_pattern_extended_ = extended;
  cover_prefs_.putBool("my_pattern_ext", extended); // 15-char NVS key limit - see setup()'s own comment
  remote_.set_my_pattern_extended(extended);
}

void IOHCCover::set_invert(bool invert) {
  invert_ = invert;
  cover_prefs_.putBool("invert", invert);
}

void IOHCCover::loop() {
  if (mode_ != Mode::POSITION)
    return; // MY/TWO_WAY don't tick a time-based position estimate

  auto &tracker = remote_.position_tracker();
  tracker.update();

  // BlindPosition only knows how to run to a full extreme (0 or 100) - it
  // has no concept of an intermediate target. Without this check the local
  // estimate would keep running to 0%/100% regardless of what was actually
  // requested, even though the real motor stops correctly on its own.
  if (tracker.isMoving() && target_position_ >= 0.0f) {
    // target_raw_opening_ reflects the tracker's own real direction, not
    // current_operation (HA space) - see its own comment in iohc_cover.h
    // for why the two can disagree when Invert Direction is on.
    bool reached_opening = target_raw_opening_ && tracker.getPosition() >= target_position_;
    bool reached_closing = !target_raw_opening_ && tracker.getPosition() <= target_position_;
    if (reached_opening || reached_closing) {
      tracker.setPosition(target_position_);
      tracker.stop();
    }
  }

  // Round to the nearest whole percent - the real motor's own resolution
  // (confirmed via passive 2W decode) is always a whole percent, so a
  // finer-grained local estimate is just noise (extra publish_state()/log
  // spam) with no real precision behind it.
  float pos_raw = std::round(tracker.getPosition()) / 100.0f;
  // The tracker follows whichever RemoteButton control() actually sent -
  // once that's flipped by Invert Direction, the tracker's own running
  // estimate is motor-side, not HA-side, and needs converting back before
  // publish. target_position_ above stays in the tracker's own raw space
  // deliberately, so the reached-target check just above needs no change.
  float pos = invert_ ? (1.0f - pos_raw) : pos_raw;
  bool position_changed = std::fabs(pos - this->position) > 0.001f;
  bool now_idle = !tracker.isMoving() && this->current_operation != cover::COVER_OPERATION_IDLE;

  if (!position_changed && !now_idle)
    return;

  this->position = pos;
  if (now_idle)
    this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void IOHCCover::dump_config() {
  LOG_COVER("", "Somfy IOHC Cover", this);
  ESP_LOGCONFIG(TAG, "  Paired: %s", remote_.is_paired() ? "yes" : "no");
  const char *mode_name =
      mode_ == Mode::POSITION ? "Position" : mode_ == Mode::MY ? "Open / My / Close" : "Two-Way (Experimental)";
  ESP_LOGCONFIG(TAG, "  Mode: %s", mode_name);
  ESP_LOGCONFIG(TAG, "  Invert Direction: %s", invert_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Travel time open/close: %lus / %lus (fixed)", TRAVEL_TIME_OPEN, TRAVEL_TIME_CLOSE);
}

cover::CoverTraits IOHCCover::get_traits() {
  auto traits = cover::CoverTraits();
  // Traits stay constant across all 3 modes (position support always on) so
  // switching modes at runtime doesn't require HA to re-discover the entity.
  traits.set_is_assumed_state(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(false);
  traits.set_supports_stop(true);
  return traits;
}

void IOHCCover::press_my() {
  remote_.cmd(IOHC::RemoteButton::Vent);
  if (mode_ == Mode::MY) {
    this->position = 0.5f;
    cover_prefs_.putFloat("position", 0.5f);
  } else {
    target_position_ = -1.0f;
  }
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void IOHCCover::press_prog2w() {
  if (!has_motor_address_) {
    ESP_LOGW(TAG, "Program (2W) pressed but this cover has no motor_address configured - see README's Real "
                  "position feedback section for how to look it up. Refusing to arm (will not listen for just "
                  "any DISCOVER broadcast).");
    return;
  }
  parent_->controller2w().arm_bonding(motor_address_, this);
}

void IOHCCover::control(const cover::CoverCall &call) {
  if (mode_ == Mode::TWO_WAY) {
    // Phase 3d - real 2W command, only possible once this motor has
    // actually bonded via arm_bonding(). The corrected, controller-initiated
    // bonding sequence and crypto (Finding 20 onward) have been validated
    // against real captured frames, but this bridge's own Program (2W)
    // attempt has never yet completed a full bonding cycle on real hardware
    // (Findings 21/23/25/26 - repeated real tests, always zero 0x29 reply) -
    // see IOHCController2W::send_command()'s own bonded check for the
    // actual gate, which is why every command here still refuses in
    // practice. Same main-byte formula as 1W (Finding 11), reused rather
    // than re-derived.
    if (!has_motor_address_) {
      ESP_LOGW(TAG, "Two-Way mode selected but this cover has no motor_address configured - refusing");
      return;
    }
    uint8_t main0, main1 = 0x00;
    if (call.get_stop()) {
      main0 = 0xD2;
    } else if (call.get_position().has_value()) {
      int percent = static_cast<int>(lroundf(*call.get_position() * 100.0f));
      // Same Invert Direction handling as the 1W path below - flip to
      // raw/motor-space before deciding what actually gets sent.
      int raw_percent = invert_ ? (100 - percent) : percent;
      if (raw_percent >= 100) {
        main0 = 0x00;
      } else if (raw_percent <= 0) {
        main0 = 0xC8;
      } else {
        main0 = static_cast<uint8_t>((100 - raw_percent) * 2);
      }
    } else {
      return;
    }
    parent_->controller2w().send_command(motor_address_, main0, main1, this);
    return;
  }

  if (call.get_stop()) {
    remote_.cmd(IOHC::RemoteButton::Stop);
    if (mode_ == Mode::MY) {
      this->position = 0.5f;
      cover_prefs_.putFloat("position", 0.5f);
    } else {
      target_position_ = -1.0f;
      // tracker.getPosition() is raw/motor-space (see loop()'s own
      // comment) - needs the same HA-space conversion before persisting,
      // or an inverted cover's stored position silently ends up wrong.
      float raw = remote_.position_tracker().getPosition() / 100.0f;
      cover_prefs_.putFloat("position", invert_ ? (1.0f - raw) : raw);
    }
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->publish_state();
    return;
  }

  if (!call.get_position().has_value())
    return;

  float target = *call.get_position(); // 0 (closed) .. 1 (open)
  int percent = static_cast<int>(lroundf(target * 100.0f));

  if (mode_ == Mode::MY) {
    // 3 discrete states, any intermediate request maps to the "My"/favorite
    // position. Unlike RTS (where My and Stop are genuinely the same
    // physical button), IO's real "My" is a distinct command (Vent,
    // main=0xd8) from Stop (main=0xd2) - confirmed via a live capture of a
    // real TaHoma "My" press. Stop only does something while the motor is
    // actively moving, which is why this previously did nothing when the
    // cover was already idle. The dedicated Stop button above (call.get_stop())
    // still sends real Stop, for interrupting an in-progress move.
    // this->position always echoes back exactly what was requested,
    // regardless of Invert Direction - it's HA-facing, and what the user
    // asked for is correct either way. Only which RemoteButton actually
    // gets sent to the motor flips.
    if (percent >= 100) {
      remote_.cmd(invert_ ? IOHC::RemoteButton::Close : IOHC::RemoteButton::Open);
      this->position = 1.0f;
    } else if (percent <= 0) {
      remote_.cmd(invert_ ? IOHC::RemoteButton::Open : IOHC::RemoteButton::Close);
      this->position = 0.0f;
    } else {
      remote_.cmd(IOHC::RemoteButton::Vent); // direction-agnostic, no invert needed
      this->position = 0.5f;
    }
    cover_prefs_.putFloat("position", this->position);
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->publish_state();
    return;
  }

  // Mode::POSITION. target_position_ and target_raw_opening_ are
  // deliberately kept in raw/motor space throughout (matching
  // tracker.getPosition(), which follows whichever RemoteButton actually
  // gets sent) - loop()'s reached-target check compares against those
  // directly. current_operation and the persisted "position" stay in HA
  // space (percent, unconverted) since both are user-facing.
  if (percent >= 100) {
    remote_.cmd(invert_ ? IOHC::RemoteButton::Close : IOHC::RemoteButton::Open);
    target_position_ = invert_ ? 0.0f : 100.0f;
    target_raw_opening_ = !invert_;
    this->current_operation = cover::COVER_OPERATION_OPENING;
  } else if (percent <= 0) {
    remote_.cmd(invert_ ? IOHC::RemoteButton::Open : IOHC::RemoteButton::Close);
    target_position_ = invert_ ? 100.0f : 0.0f;
    target_raw_opening_ = invert_;
    this->current_operation = cover::COVER_OPERATION_CLOSING;
  } else {
    float current_raw = remote_.position_tracker().getPosition();
    float current_ha = invert_ ? (100.0f - current_raw) : current_raw;
    int raw_percent = invert_ ? (100 - percent) : percent;
    remote_.cmd(IOHC::RemoteButton::Position, raw_percent);
    target_position_ = static_cast<float>(raw_percent);
    // Mirrors the same comparison iohc_remote1w.cpp's own Position case
    // uses internally to decide startOpening()/startClosing() on the
    // tracker (raw_percent vs the tracker's own pre-command raw
    // position) - has to agree with that, not with current_operation's
    // HA-space comparison below.
    target_raw_opening_ = raw_percent > current_raw;
    this->current_operation =
        (percent > current_ha) ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
  }
  cover_prefs_.putFloat("position", percent / 100.0f);
  this->publish_state();
}

}  // namespace iohc
}  // namespace esphome
