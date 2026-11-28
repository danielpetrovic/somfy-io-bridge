#include "iohc_cover.h"
#include "esphome/core/log.h"
#include "../iohcCryptoHelpers.h"
#include "esp_timer.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.cover";

// Confirm-and-resend tuning. A status answer is heard about 35 s after a
// command (TaHoma polls the motor), so the window is 60 s plus the travel time.
static constexpr float CONFIRM_TOLERANCE = 3.0f;          // percent
static constexpr uint32_t CONFIRM_WINDOW_MS = 60000;      // plus the travel time
static constexpr uint8_t MAX_RESENDS = 2;
static constexpr float ADOPT_TOLERANCE = 3.0f;            // percent
static constexpr uint32_t ADOPT_QUIET_MS = 30000;         // after the travel time since our last command

static uint32_t now_ms() { return static_cast<uint32_t>(esp_timer_get_time() / 1000); }

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
  if (mode_ == Mode::MY) {
    // "Open / My / Close" was removed from the select's options 2026-11-27
    // (unused). A cover whose NVS still holds the old value 1 migrates to
    // Position, the only remaining mode that still covers the same ground
    // (percentage targets) - never left reinterpreted as whatever a future
    // mode might someday reuse that numeric value for.
    ESP_LOGW(TAG, "This cover's saved mode was 'Open / My / Close', which has been removed - migrated to Position");
    mode_ = Mode::POSITION;
    cover_prefs_.putUChar("mode", static_cast<uint8_t>(mode_));
  }
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

  // "travel_open"/"travel_close" - well under the 15-char NVS key limit.
  travel_time_open_ = cover_prefs_.getUInt("travel_open", travel_time_open_default_);
  travel_time_close_ = cover_prefs_.getUInt("travel_close", travel_time_close_default_);
  apply_travel_times_();

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

  parent_->register_status_cover(this);
  // Two-Way: the motor's status is read after boot (and refreshed at rest
  // when status_refresh_interval is set); until then the saved position shows.
  if (has_motor_address_)
    parent_->controller2w().register_cover(motor_address_, this, low_power_2w_, status_refresh_interval_ms_);

  // Drop the value an earlier version of this component persisted under this
  // key (July 2026): it is stale and nothing reads it any more.
  cover_prefs_.remove("target_closure");
}

void IOHCCover::set_motor_address(const std::string &motor_address_hex) {
  if (motor_address_hex.empty())
    return;
  hexStringToBytes(motor_address_hex, motor_address_);
  has_motor_address_ = true;
}

bool IOHCCover::commander_matches(const uint8_t *commander) const {
  return memcmp(remote_.node(), commander, sizeof(IOHC::address)) == 0;
}

bool IOHCCover::motor_matches(const uint8_t *source) const {
  return has_motor_address_ && memcmp(motor_address_, source, sizeof(IOHC::address)) == 0;
}

void IOHCCover::on_status_heard(float target_closure, bool target_valid, float current_closure, bool current_valid,
                                bool stopped, uint8_t status, bool commander_is_ours, float rssi_dbm, float afc_hz) {
  uint32_t last_cmd = remote_.last_cmd_ms();
  uint32_t now = static_cast<uint32_t>(esp_timer_get_time() / 1000);
  char age[32];
  if (last_cmd == 0) {
    snprintf(age, sizeof(age), "no command since boot");
  } else {
    snprintf(age, sizeof(age), "%.1f s after our last command", (now - last_cmd) / 1000.0f);
  }
  char target_text[16];
  if (target_valid) {
    snprintf(target_text, sizeof(target_text), "%.0f%%", target_closure);
  } else {
    snprintf(target_text, sizeof(target_text), "none");
  }
  ESP_LOGI(TAG, "%s: status heard, %s, current %.0f%% closed, target %s, status=0x%02X commander=%s, %.1f dBm, "
                "afc %+.0f Hz, %s",
           LOG_STR_ARG(this->get_log_name()), stopped ? "stopped" : "moving", current_closure, target_text, status,
           commander_is_ours ? "us" : "other", rssi_dbm, afc_hz, age);
  update_last_rssi(rssi_dbm);

  if (!parent_->confirm_resend_wanted() || mode_ != Mode::POSITION) {
    return;
  }

  // Confirmation: the motor took the last command, its target is where that
  // command sent it (while it still travels, too). The cover already shows
  // that target, so nothing else to do.
  if (target_valid) {
    target_closure = std::clamp(target_closure, 0.0f, 100.0f);
    if (confirm_pending_ && std::fabs(target_closure - pending_expected_closure_) <= CONFIRM_TOLERANCE) {
      ESP_LOGI(TAG, "%s: command confirmed, motor target %.0f%% closed, %.1f s after the first send, %u resend(s)",
               LOG_STR_ARG(this->get_log_name()), target_closure, (now_ms() - pending_first_ms_) / 1000.0f,
               static_cast<unsigned>(pending_resends_));
      confirm_pending_ = false;
      return;
    }
  }

  // Adopting the real position: the motor's current position, once it
  // reports itself at rest. Only here is the heard position allowed to change
  // this cover: the hub matched the frame on a 24 bit address (the commander
  // or the motor address), which a garbled frame will not hit by chance. Not
  // while a command is waiting for its confirmation (a resend is pending),
  // not while the cover is moving, and not until the motor has surely stopped
  // after our last command.
  if (!current_valid || !stopped) {
    return;
  }
  const float closure_percent = std::clamp(current_closure, 0.0f, 100.0f);
  if (confirm_pending_ || this->current_operation != cover::COVER_OPERATION_IDLE) {
    return;
  }
  uint32_t travel_ms = std::max(travel_time_open_, travel_time_close_) * 1000UL;
  if (last_cmd != 0 && (now - last_cmd) < travel_ms + ADOPT_QUIET_MS) {
    return;
  }
  float real_open_raw = 100.0f - closure_percent;  // motor space, same as the tracker
  if (std::fabs(real_open_raw - remote_.position_tracker().getPosition()) > ADOPT_TOLERANCE) {
    adopt_real_position_(closure_percent);
  }
}

void IOHCCover::adopt_real_position_(float closure_percent) {
  float real_open_raw = 100.0f - closure_percent;
  ESP_LOGI(TAG, "%s: taking over the real position %.0f%% open (estimate was %.0f%%)",
           LOG_STR_ARG(this->get_log_name()), real_open_raw, remote_.position_tracker().getPosition());
  remote_.position_tracker().setPosition(real_open_raw);
  remote_.position_tracker().stop();
  target_position_ = -1.0f;
  this->position = invert_ ? (1.0f - real_open_raw / 100.0f) : (real_open_raw / 100.0f);
  this->current_operation = cover::COVER_OPERATION_IDLE;
  cover_prefs_.putFloat("position", this->position);
  this->publish_state();
}

void IOHCCover::arm_confirmation_(IOHC::RemoteButton button, int percent, float expected_closure) {
  if (!parent_->confirm_resend_wanted()) {
    confirm_pending_ = false;
    return;
  }
  confirm_pending_ = true;
  pending_button_ = button;
  pending_percent_ = percent;
  pending_expected_closure_ = std::clamp(expected_closure, 0.0f, 100.0f);
  pending_first_ms_ = pending_sent_ms_ = now_ms();
  pending_resends_ = 0;
}

void IOHCCover::check_confirmation_() {
  if (!confirm_pending_) {
    return;
  }
  if (!parent_->confirm_resend_wanted()) {
    confirm_pending_ = false;
    return;
  }
  uint32_t now = now_ms();
  uint32_t travel_ms = std::max(travel_time_open_, travel_time_close_) * 1000UL;
  if ((now - pending_sent_ms_) < CONFIRM_WINDOW_MS + travel_ms) {
    return;
  }
  if (!parent_->status_feedback_available()) {
    ESP_LOGI(TAG, "%s: no motor status heard for 6 hours, so nothing confirms commands: not resending",
             LOG_STR_ARG(this->get_log_name()));
    confirm_pending_ = false;
    return;
  }
  if (pending_resends_ >= MAX_RESENDS) {
    ESP_LOGW(TAG, "%s: command not confirmed after %u resend(s), expected %.0f%% closed. Giving up.",
             LOG_STR_ARG(this->get_log_name()), static_cast<unsigned>(pending_resends_), pending_expected_closure_);
    confirm_pending_ = false;
    return;
  }
  pending_resends_++;
  ESP_LOGI(TAG, "%s: no matching status within %.0f s, sending the command again (resend %u of %u)",
           LOG_STR_ARG(this->get_log_name()), (now - pending_sent_ms_) / 1000.0f,
           static_cast<unsigned>(pending_resends_), static_cast<unsigned>(MAX_RESENDS));
  remote_.cmd_resend(pending_button_, pending_percent_);
  pending_sent_ms_ = now;
}

void IOHCCover::update_last_rssi(float rssi_dbm) {
  if (last_rssi_sensor_ != nullptr) {
    last_rssi_sensor_->publish_state(rssi_dbm);
  }
}

cover::CoverOperation IOHCCover::operation_toward_(float from_closure, float to_closure) const {
  if (from_closure < 0.0f || to_closure < 0.0f || from_closure == to_closure)
    return cover::COVER_OPERATION_IDLE;
  // Closure falls toward open; Invert Direction shows the other way round.
  const bool opening = invert_ ? (to_closure > from_closure) : (to_closure < from_closure);
  return opening ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
}

void IOHCCover::on_2w_report(std::optional<float> current_closure, std::optional<float> target_closure, bool moving) {
  const float previous = last_2w_closure_;
  if (current_closure.has_value()) {
    const float closure = std::clamp(*current_closure, 0.0f, 100.0f);
    // closure (0=open/100=closed) -> HA (0=closed/1=open), mirrored by
    // Invert Direction like control().
    this->position = invert_ ? (closure / 100.0f) : (1.0f - closure / 100.0f);
    cover_prefs_.putFloat("position", this->position);
    // A decoded position is the observation the prediction stood in for.
    predicted_2w_target_ = -1.0f;
  }
  const float target = target_closure.has_value() ? std::clamp(*target_closure, 0.0f, 100.0f) : predicted_2w_target_;
  const float current = current_closure.has_value() ? std::clamp(*current_closure, 0.0f, 100.0f) : -1.0f;

  if (!moving) {
    this->current_operation = cover::COVER_OPERATION_IDLE;
  } else if (target >= 0.0f && current >= 0.0f && target != current) {
    // Travelling toward a target it reports, from a position it reports.
    this->current_operation = operation_toward_(current, target);
  } else if (target >= 0.0f && current < 0.0f) {
    // Moving, live position withheld: from the last position seen.
    if (previous != target)
      this->current_operation = operation_toward_(previous, target);
  } else if (current >= 0.0f) {
    // No target, or still its pre-command target: the direction follows a
    // real change of position, IDLE until there is one.
    this->current_operation = operation_toward_(previous, current);
  }
  if (current >= 0.0f)
    last_2w_closure_ = current;
  stop_pending_ = false;
  target_position_ = -1.0f;
  ESP_LOGD(TAG, "%s: Two-Way report, %.0f%% open, %s", LOG_STR_ARG(this->get_log_name()), this->position * 100.0f,
           this->current_operation == cover::COVER_OPERATION_OPENING   ? "opening"
           : this->current_operation == cover::COVER_OPERATION_CLOSING ? "closing"
                                                                        : "idle");
  this->publish_state();
}

void IOHCCover::on_2w_command_failed(bool was_stop) {
  if (was_stop && stop_pending_) {
    // The stop did not get through: the movement it replaced goes on.
    this->current_operation = stop_replaced_operation_;
  } else {
    this->current_operation = cover::COVER_OPERATION_IDLE;
  }
  stop_pending_ = false;
  predicted_2w_target_ = -1.0f;
  this->publish_state();
}

void IOHCCover::set_mode(Mode mode) {
  // Position (1W) estimates from its tracker: start it from the shown
  // position, or loop() would publish whatever the tracker held before.
  if (mode == Mode::POSITION && mode_ != Mode::POSITION) {
    float raw_position = invert_ ? (1.0f - this->position) : this->position;
    remote_.position_tracker().setPosition(raw_position * 100.0f);
    remote_.position_tracker().stop();
    target_position_ = -1.0f;
  }
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
  apply_travel_times_();
}

void IOHCCover::set_travel_time_open(uint32_t seconds) {
  travel_time_open_ = seconds;
  cover_prefs_.putUInt("travel_open", seconds);
  apply_travel_times_();
}

void IOHCCover::set_travel_time_close(uint32_t seconds) {
  travel_time_close_ = seconds;
  cover_prefs_.putUInt("travel_close", seconds);
  apply_travel_times_();
}

void IOHCCover::apply_travel_times_() {
  // travel_time_open_/travel_time_close_ are always HA-space (see their own
  // comment in iohc_cover.h); BlindPosition's startOpening()/startClosing()
  // operate in raw/motor space, same open/close distinction as
  // target_raw_opening_ elsewhere in this file - swap which value lands in
  // which slot when Invert Direction is on.
  if (invert_) {
    remote_.set_travel_time_open(travel_time_close_);
    remote_.set_travel_time_close(travel_time_open_);
  } else {
    remote_.set_travel_time_open(travel_time_open_);
    remote_.set_travel_time_close(travel_time_close_);
  }
}

void IOHCCover::loop() {
  this->check_confirmation_();
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
  if (now_idle) {
    this->current_operation = cover::COVER_OPERATION_IDLE;
    // The estimate settled: keep it as the last known position.
    cover_prefs_.putFloat("position", this->position);
  }
  this->publish_state();
}

void IOHCCover::dump_config() {
  LOG_COVER("", "Somfy IOHC Cover", this);
  ESP_LOGCONFIG(TAG, "  Paired: %s", remote_.is_paired() ? "yes" : "no");
  const char *mode_name =
      mode_ == Mode::POSITION ? "Position (1W)" : "Two-Way (2W)";
  ESP_LOGCONFIG(TAG, "  Mode: %s", mode_name);
  ESP_LOGCONFIG(TAG, "  Invert Direction: %s", invert_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  Travel time open/close: %lus / %lus", travel_time_open_, travel_time_close_);
}

cover::CoverTraits IOHCCover::get_traits() {
  auto traits = cover::CoverTraits();
  // Traits stay constant across both modes (position support always on) so
  // switching modes at runtime doesn't require HA to re-discover the entity.
  traits.set_is_assumed_state(true);
  traits.set_supports_position(true);
  traits.set_supports_toggle(false);
  traits.set_supports_stop(true);
  return traits;
}

void IOHCCover::press_my() {
  remote_.cmd(IOHC::RemoteButton::Vent);
  // No known target position (My goes to a stored favorite, not a percentage
  // we commanded), so there is nothing to clamp the travel-time estimate to.
  target_position_ = -1.0f;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->publish_state();
}

void IOHCCover::press_prog2w() { parent_->controller2w().discover_and_pair(); }

void IOHCCover::control(const cover::CoverCall &call) {
  if (mode_ == Mode::TWO_WAY) {
    // 2W command under this bridge's system key. The motor must hold that
    // key (paired with Program (2W) or the pair_device action); same
    // main-byte formula as 1W (Finding 11).
    if (!has_motor_address_) {
      ESP_LOGW(TAG, "Two-Way mode selected but this cover has no motor_address configured - refusing");
      return;
    }
    uint8_t main0, main1 = 0x00;
    // The shown closure (motor space) the predicted direction starts from.
    const float shown_closure =
        last_2w_closure_ >= 0.0f ? last_2w_closure_
                                 : 100.0f - (invert_ ? (1.0f - this->position) : this->position) * 100.0f;
    if (call.get_stop()) {
      main0 = 0xD2;
      // apply_optimistic_stop(): shown at rest at once; a failed stop gives
      // the movement back.
      stop_replaced_operation_ = this->current_operation;
      stop_pending_ = true;
      predicted_2w_target_ = -1.0f;
      this->current_operation = cover::COVER_OPERATION_IDLE;
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
      // apply_optimistic_target(): the direction toward the commanded
      // target shows at once, until a report supersedes it.
      predicted_2w_target_ = static_cast<float>(100 - std::clamp(raw_percent, 0, 100));
      stop_pending_ = false;
      this->current_operation = operation_toward_(shown_closure, predicted_2w_target_);
    } else {
      return;
    }
    this->publish_state();
    parent_->controller2w().send_command(motor_address_, low_power_2w_, main0, main1, this);
    return;
  }

  if (call.get_stop()) {
    confirm_pending_ = false;
    remote_.cmd(IOHC::RemoteButton::Stop);
    target_position_ = -1.0f;
    // tracker.getPosition() is raw/motor-space (see loop()'s own
    // comment) - needs the same HA-space conversion before persisting,
    // or an inverted cover's stored position silently ends up wrong.
    float raw = remote_.position_tracker().getPosition() / 100.0f;
    cover_prefs_.putFloat("position", invert_ ? (1.0f - raw) : raw);
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->publish_state();
    return;
  }

  if (!call.get_position().has_value())
    return;

  float target = *call.get_position(); // 0 (closed) .. 1 (open)
  int percent = static_cast<int>(lroundf(target * 100.0f));

  // Mode::POSITION. target_position_ and target_raw_opening_ are
  // deliberately kept in raw/motor space throughout (matching
  // tracker.getPosition(), which follows whichever RemoteButton actually
  // gets sent) - loop()'s reached-target check compares against those
  // directly. current_operation and the persisted "position" stay in HA
  // space (percent, unconverted) since both are user-facing.
  if (percent >= 100) {
    IOHC::RemoteButton sent = invert_ ? IOHC::RemoteButton::Close : IOHC::RemoteButton::Open;
    remote_.cmd(sent);
    target_position_ = invert_ ? 0.0f : 100.0f;
    target_raw_opening_ = !invert_;
    this->current_operation = cover::COVER_OPERATION_OPENING;
    arm_confirmation_(sent, -1, 100.0f - target_position_);
  } else if (percent <= 0) {
    IOHC::RemoteButton sent = invert_ ? IOHC::RemoteButton::Open : IOHC::RemoteButton::Close;
    remote_.cmd(sent);
    target_position_ = invert_ ? 100.0f : 0.0f;
    target_raw_opening_ = invert_;
    this->current_operation = cover::COVER_OPERATION_CLOSING;
    arm_confirmation_(sent, -1, 100.0f - target_position_);
  } else {
    float current_raw = remote_.position_tracker().getPosition();
    float current_ha = invert_ ? (100.0f - current_raw) : current_raw;
    int raw_percent = invert_ ? (100 - percent) : percent;
    remote_.cmd(IOHC::RemoteButton::Position, raw_percent);
    target_position_ = static_cast<float>(raw_percent);
    arm_confirmation_(IOHC::RemoteButton::Position, raw_percent, 100.0f - target_position_);
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
