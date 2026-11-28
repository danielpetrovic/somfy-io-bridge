#pragma once

#include <optional>

#include "esphome/core/component.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "../iohc.h"
#include "../iohc_remote1w.h"
#include <Preferences.h>

namespace esphome {
namespace iohc {

class IOHCCover : public cover::Cover, public Component {
 public:
  // POSITION (default): sends real percentage targets to the motor over 1W,
  //   which reliably lands exactly where commanded. The displayed position
  //   between send and arrival is a local BlindPosition travel-time
  //   estimate - purely cosmetic (animates "still moving" in HA), clamped to
  //   land exactly on the commanded target regardless of timing drift.
  // TWO_WAY: real motor-reported position via the 2W challenge/response
  //   layer - not implemented yet, selecting it just logs a warning and
  //   ignores commands.
  //
  // MY (retired 2026-11-27, kept only as a migration target below): the RTS
  // bridge's own default model - 3 discrete states (0.0 closed / 0.5
  // MY-or-stopped / 1.0 open), no time tracking. Removed from the select's
  // options (select/__init__.py) as unused on this bridge's own install, and
  // no other known install was ever confirmed using it either. Numeric
  // value kept so a cover whose persisted "mode" byte is still 1 can be
  // recognized and migrated to Position rather than silently reinterpreted
  // as something else - see setup()'s load of this value. Never construct
  // this value outside that one migration check.
  enum class Mode : uint8_t { POSITION = 0, MY = 1, TWO_WAY = 2 };

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  cover::CoverTraits get_traits() override;

  void set_parent(IOHCComponent *parent) { parent_ = parent; }
  void set_type(uint8_t type) { type_ = type; }
  void set_manufacturer(uint8_t manufacturer) { manufacturer_ = manufacturer; }
  // Compile-time seed only - resolved (auto/simple/extended -> plain bool)
  // by cover/__init__.py's my_pattern option, called by codegen before
  // setup()/cover_prefs_ exist. Only used as the NVS fallback on
  // first-ever boot - see setup() and set_my_pattern_extended() below for
  // the actual live value the remote uses.
  void set_my_pattern_default_extended(bool extended) { my_pattern_default_extended_ = extended; }

  bool get_my_pattern_extended() const { return my_pattern_extended_; }
  // My Pattern switch's write_state() entry point - persists to NVS and
  // immediately forwards the new value to remote_, mirrors set_mode()'s
  // own pattern exactly. See IOHC::RemoteButton::Vent/SetMy in
  // iohc_remote1w.h for what this actually controls.
  void set_my_pattern_extended(bool extended);

  // Compile-time seed only - resolved from cover/__init__.py's invert
  // option, called by codegen before setup()/cover_prefs_ exist. Only
  // used as the NVS fallback on first-ever boot - see setup() and
  // set_invert() below for the actual live value control()/loop() use.
  void set_invert_default(bool invert) { invert_default_ = invert; }

  bool get_invert() const { return invert_; }
  // Invert Direction switch's write_state() entry point - persists to NVS
  // and updates the live value, mirrors set_my_pattern_extended()'s own
  // pattern exactly. See control()'s own comments for what this actually
  // flips (which RemoteButton gets sent, and the HA-space/motor-space
  // conversion on both the command and position-readback sides).
  void set_invert(bool invert);

  // Compile-time seeds only - resolved from cover/__init__.py's
  // travel_time_open/travel_time_close options, called by codegen before
  // setup()/cover_prefs_ exist. Only used as the NVS fallback on
  // first-ever boot - see setup() and set_travel_time_open()/
  // set_travel_time_close() below for the actual live values.
  void set_travel_time_open_default(uint32_t seconds) { travel_time_open_default_ = seconds; }
  void set_travel_time_close_default(uint32_t seconds) { travel_time_close_default_ = seconds; }

  // These are always in HA space (matches what the "Travel Time Open"/
  // "Travel Time Close" number entities display) - GitHub issue #4's own
  // ask, so a user's "open" here always means what HA calls open,
  // regardless of Invert Direction. apply_travel_times_() is what
  // translates that into the raw/motor-space assignment BlindPosition
  // actually needs (see its own comment for why that mapping has to
  // flip when invert_ is on).
  uint32_t get_travel_time_open() const { return travel_time_open_; }
  void set_travel_time_open(uint32_t seconds);
  uint32_t get_travel_time_close() const { return travel_time_close_; }
  void set_travel_time_close(uint32_t seconds);

  // Optional (6/32 hex chars). If both set, the bonded identity comes from
  // YAML/secrets.yaml instead of being randomly generated into this board's
  // own flash - see IOHC::IOHCRemote1W::begin() for why.
  void set_fixed_node(const std::string &node_hex) { fixed_node_hex_ = node_hex; }
  void set_fixed_key(const std::string &key_hex) { fixed_key_hex_ = key_hex; }
  // The cover's own YAML component id (e.g. "garden_shutter_io"), NOT
  // get_object_id() - deliberately decoupled from HA's entity/device naming.
  // Used only to derive a stable NVS namespace for bonded identity + persisted
  // cover state (position/mode). get_object_id() depends on the entity's
  // name/device_id assignment, which is a presentation choice that can change
  // (e.g. switching a cover to its own HA sub-device) without us intending to
  // orphan already-bonded motors or reset their sequence counters.
  void set_nvs_key(const std::string &key) { nvs_key_ = key; }

  IOHC::IOHCRemote1W &remote() { return remote_; }

  Mode get_mode() const { return mode_; }
  void set_mode(Mode mode);

  // Dedicated My button entry point (as opposed to Prog/Identify, which only
  // need to fire the radio command). My also has to update the cover's own
  // displayed position/state, same as the position-slider path in control()
  // does - otherwise the motor moves but Home Assistant keeps showing the
  // stale position. Mirrors the get_stop() branch in control() exactly,
  // just sending Vent instead of Stop.
  void press_my();

  // "Program (2W)" button: laberning's Discover & Pair
  // (IOHC::IOHCController2W::discover_and_pair()). Whichever device answers
  // the discovery is paired; the log names it and the cover that declares it.
  void press_prog2w();

  // The motor is a low-power (battery) receiver: its 2W frames need the
  // wake-up preamble (laberning's per-device `low_power:`).
  void set_low_power_2w(bool low_power) { low_power_2w_ = low_power; }
  bool low_power_2w() const { return low_power_2w_; }
  void set_status_refresh_interval_ms(uint32_t ms) { status_refresh_interval_ms_ = ms; }

  // Real motor address (6 hex chars) - as assigned by Somfy, NOT this
  // bridge's own 1W virtual remote identity (node/key above, which is a
  // separate, locally-generated address). Required for the Program (2W)/Get
  // Name (2W) buttons and Two-Way mode - see README for how to look this up
  // (Overkiz's own unique_id, if you have TaHoma/Connexoon).
  void set_motor_address(const std::string &motor_address_hex);
  // A Two-Way report from IOHCController2W: a status poll reply or a
  // status update, authenticated under this bridge's key (never a passive
  // source, Finding 10). Closures are motor space (0 open, 100 closed);
  // nullopt when the report holds none. `moving` is the controller's verdict
  // (classify_stopped_report()). Sets the position and the direction the way
  // laberning/home_io_control's on_device_update_() does.
  void on_2w_report(std::optional<float> current_closure, std::optional<float> target_closure, bool moving);
  // A Two-Way command did not get through: its predicted movement is withdrawn
  // (rollback_optimistic()); a failed stop gives back the movement it replaced.
  void on_2w_command_failed(bool was_stop);

  // Passive 2W status (cmd 0x04 answer of a motor, overheard while another
  // controller such as TaHoma polls it; see IOHCComponent::on_receive()).
  // Nothing here can reach this cover's own position/state: a passively
  // decoded frame has no integrity check, so it only feeds the two standalone
  // sensors below and the log, never the entity used for control.
  //
  // commander_matches(): the frame's "Commander" field is this cover's own
  // virtual remote address, i.e. our bridge sent the last command the motor
  // obeyed. motor_matches(): the frame was sent by this cover's configured
  // motor_address (needs motor_address, optional).
  bool commander_matches(const uint8_t *commander) const;
  bool motor_matches(const uint8_t *source) const;
  // Closures are 0 = open, 100 = closed. target_valid is false when the
  // target echoes a non-position command (stop, vent, ...); stopped is the
  // motor's own execution state.
  void on_status_heard(float target_closure, bool target_valid, float current_closure, bool current_valid,
                       bool stopped, uint8_t status, bool commander_is_ours, float rssi_dbm, float afc_hz);
  // Any frame sent by this cover's motor_address, decoded or not. RSSI is a
  // property of the reception, so a misdecoded payload cannot corrupt it.
  void update_last_rssi(float rssi_dbm);
  void set_last_rssi_sensor(sensor::Sensor *s) { last_rssi_sensor_ = s; }

 protected:
  void control(const cover::CoverCall &call) override;

  IOHCComponent *parent_{};
  IOHC::IOHCRemote1W remote_;
  // Real defaults (GitHub issue #4 - one fixed 25s constant couldn't fit
  // two differently-geared covers on the same board, or even both
  // directions of the same cover). Purely cosmetic in Mode::POSITION
  // (real commands carry an exact percentage, the motor lands there
  // regardless of this estimate's accuracy) but genuinely affects how
  // closely the displayed position tracks reality in Mode::MY, where
  // Open/Close never carry a percentage at all and this estimate is the
  // only thing driving the displayed position while moving.
  uint32_t travel_time_open_default_{25};
  uint32_t travel_time_close_default_{25};
  uint32_t travel_time_open_{25};
  uint32_t travel_time_close_{25};
  // Pushes travel_time_open_/travel_time_close_ into remote_, swapping
  // which one lands in BlindPosition's raw-space "open" slot vs "close"
  // slot when invert_ is on - travel_time_open_/close_ themselves always
  // stay in HA space (see their own comment), but BlindPosition's
  // startOpening()/startClosing() operate in raw/motor space, same
  // reasoning as target_raw_opening_ below. Called from setup() and
  // whenever invert_, travel_time_open_, or travel_time_close_ change.
  void apply_travel_times_();
  uint8_t type_{0};
  uint8_t manufacturer_{2};
  bool my_pattern_default_extended_{true};
  bool my_pattern_extended_{true};
  bool invert_default_{false};
  bool invert_{false};
  std::string fixed_node_hex_;
  std::string fixed_key_hex_;
  std::string nvs_key_;
  IOHC::address motor_address_{};
  bool has_motor_address_{false};
  bool low_power_2w_{false};
  uint32_t status_refresh_interval_ms_{0};
  // Two-Way direction (platform_cover.cpp): the last reported closure, the
  // predicted target of our own command until a report supersedes it, and
  // the direction a stop replaced.
  float last_2w_closure_{-1.0f};
  float predicted_2w_target_{-1.0f};
  cover::CoverOperation stop_replaced_operation_{cover::COVER_OPERATION_IDLE};
  bool stop_pending_{false};
  // Direction of travelling from one closure to another (operation_toward()).
  cover::CoverOperation operation_toward_(float from_closure, float to_closure) const;
  sensor::Sensor *last_rssi_sensor_{nullptr};

  // Confirm-and-resend (switch "Confirm And Resend (1W)"). A 1W command gets
  // no reply, but the motor's status answer, overheard when TaHoma polls it
  // about 35 s later, carries its position: closure = 100 - the open percent
  // we sent (Open 0, Close 100, Position p -> 100 - p). After an Open, Close
  // or Position in Mode::POSITION the cover waits for an answer that matches;
  // without one it sends the same command again (see check_confirmation_()).
  void arm_confirmation_(IOHC::RemoteButton button, int percent, float expected_closure);
  void check_confirmation_();
  // Takes the motor's real position over as this cover's state (see
  // on_status_heard() for when that is allowed).
  void adopt_real_position_(float closure_percent);
  bool confirm_pending_{false};
  IOHC::RemoteButton pending_button_{IOHC::RemoteButton::Open};
  int pending_percent_{-1};
  float pending_expected_closure_{0.0f};
  uint32_t pending_first_ms_{0};
  uint32_t pending_sent_ms_{0};
  uint8_t pending_resends_{0};

  Mode mode_{Mode::POSITION};
  // Explicitly global-scoped: inside esphome::iohc, unqualified "Preferences"
  // resolves to esphome's OWN esphome::Preferences (aka esp32::ESP32Preferences,
  // pulled in transitively via esphome/core/preferences.h through cover.h) -
  // a completely different class with no begin()/getUChar()/etc. Arduino's
  // global ::Preferences (from <Preferences.h>) is what we actually want,
  // same one iohc_remote1w.h uses without issue since that file lives in the
  // separate global namespace IOHC, not under esphome::, so no collision there.
  ::Preferences cover_prefs_;
  std::string cover_prefs_namespace_;

  // Percent (0-100, ESPHome convention: 100=open) the local BlindPosition
  // estimate should stop at. BlindPosition itself only knows how to run to a
  // full extreme (0 or 100) - upstream's own iohcRemote1W has separate
  // target-tracking logic (remote.targetPosition + updatePositions()) that
  // wasn't ported into IOHCRemote1W, so without this the software estimate
  // would run all the way to 0/100 regardless of what was actually
  // requested, even though the real motor stops correctly on its own.
  // -1 means "no target set" (Stop was called, or nothing requested yet).
  // Only meaningful in Mode::POSITION.
  float target_position_{-1.0f};
  // Which direction the tracker itself is actually moving in raw/motor
  // space - NOT the same as current_operation, which stays in HA space
  // (see control()'s own comments). With Invert Direction on, pressing
  // Open sends Close to the motor, so current_operation correctly says
  // OPENING while the tracker is really counting down - loop()'s
  // reached-target check needs to know the tracker's own real direction,
  // not the HA-facing one, or it picks the wrong comparison and snaps
  // straight to the raw target instead of animating.
  bool target_raw_opening_{true};
};

}  // namespace iohc
}  // namespace esphome
