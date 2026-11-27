#pragma once

// Thin ESPHome wrapper around the vendored io-homecontrol radio stack (the
// other files in this same directory - ported from
// https://github.com/rspaargaren/iohomecontrol, Apache-2.0). Flat layout is
// required: ESPHome's external_components resource discovery only looks at
// a component's own top-level directory, not subdirectories (confirmed in
// esphome/loader.py's Component.resources - "does not look through
// subdirectories"), so a vendor/ subfolder silently doesn't get copied into
// the build at all. This component owns none of the radio logic itself - it
// just starts IOHC::iohcRadio on ESPHome's setup() and logs whatever comes
// in. Phase 0/1 scope: prove real reception, single fixed channel, no
// TX/pairing.

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "iohcRadio.h"
#include "iohc_controller2w.h"
#include "iohc_key_extraction.h"
#include <string>
#include <vector>

namespace esphome {
namespace iohc {

class IOHCCover;

class IOHCComponent : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }

  bool on_receive(IOHC::iohcPacket *packet);

  // Passive 2W status: every cover registers itself in setup() so that a
  // motor's status answer (cmd 0x04), overheard while another controller such
  // as TaHoma polls it, can be matched to a cover by its "Commander" field
  // (our own remote address) or by the motor address. Nothing is decoded
  // unless "Debug Passive Decode (2W)" is on.
  void register_status_cover(IOHCCover *cover) { this->status_covers_.push_back(cover); }
  const std::vector<IOHCCover *> &covers() const { return this->status_covers_; }

  // This bridge's own 2W bonding/control (Phase 3) - one shared instance for
  // the whole bridge, see iohc_controller2w.h. Distinct from the passive
  // decode above (which just reads another already-bonded controller's
  // traffic) - this is the bridge actively bonding with and commanding a
  // motor itself.
  IOHC::IOHCController2W &controller2w() { return controller2w_; }

  // "Recover System Key" responder (see iohc_key_extraction.h) - a
  // different direction from controller2w() above: this emulates a
  // device so the user's OWN hub (TaHoma) can pair to it and hand over
  // the key it already shares with every motor it has bonded.
  IOHC::IOHCKeyExtractor &key_extractor() { return key_extractor_; }

  // Optional, YAML-provided 2W controller identity (see iohc/__init__.py's
  // controller_address/system_key config comment) - mirrors
  // IOHCCover::set_fixed_node()/set_fixed_key()'s exact rationale: leave
  // unset to keep the default random-generate-and-persist-on-device
  // behavior, or set both to survive a board replacement without losing
  // any already-bonded motor's relationship with this bridge.
  void set_fixed_controller_address(const std::string &hex) { fixed_controller_hex_ = hex; }
  void set_fixed_system_key(const std::string &hex) { fixed_system_key_hex_ = hex; }

  // Cooperative 3-channel RX hop, not built on iohcRadio's ISR-driven
  // auto-hop (a hardware interrupt could preempt a retune()-then-send()).
  // It always runs: laberning/home_io_control's idle listening (all three
  // channels, 2.7 ms each, never leaving a channel while a frame arrives
  // there, with the idle RSSI readings the 2W channel access learns the busy
  // limit from). A pairing step, scan or exchange sets its own plan with
  // set_hop_plan() and ends it with release_hop_plan().
  //
  // set_bonding_hop_wanted(): a pairing attempt or scan runs (hop statistics
  // in the log).
  void set_bonding_hop_wanted(bool wanted);

  // Where the bonding-time hop listens. SKIP_FREQ: the two channels other
  // than `freq` (a reply to a broadcast almost never comes back on the
  // channel it was sent on, so dwelling there wastes listening time;
  // laberning's ROTATE_SKIPPING_REQUEST). ALL: all three. HOLD_REQUEST: stay on CH2
  // (every unicast pairing reply comes back on the request channel).
  // HOLD_FREQ: stay on whatever `hold_freq` is, without retuning to CH2 -
  // the key-extraction responder's own reply lands on a randomly chosen
  // channel (any of the three, matching what a real device does answering
  // a broadcast search), and the hub's follow-up frames keep coming back
  // on that same channel for the rest of the session.
  enum class HopPlan : uint8_t { ALL, SKIP_FREQ, HOLD_REQUEST, HOLD_FREQ };
  void set_hop_plan(HopPlan plan, uint32_t freq = 0);
  // Ends a plan set by set_hop_plan(): back to idle listening.
  void release_hop_plan();

  // Requests the same stale-PREAMBLE recovery maybe_hop_() already does for
  // the bonding attempt, for the key-extraction responder's own held
  // channel - it never hops, but the same retune-left-a-latched-flag risk
  // applies to its one retune onto the reply channel. Tracked separately
  // from set_bonding_hop_wanted() so the two features' windows don't
  // interfere with each other's hop_plan_.
  void set_responder_hop_wanted(bool wanted);

  // When false, on_receive() skips ALL processing of received frames
  // (passive 2W status for the standalone cover sensors, controller2w's own
  // bonding/key-sniffer frame handling) right after the packet counter - the
  // bridge still transmits 1W commands normally. It was added on 2026-07-13
  // on the suspicion that continuous decoding collided with TaHoma's polling;
  // that was wrong (the real cause was a loop in this bridge that flooded the
  // IO network, fixed in a later version), so decoding is not a risk. It
  // stays off by default only until the passive status path is proven.
  void set_passive_decode_wanted(bool wanted) { passive_decode_wanted_ = wanted; }
  bool passive_decode_wanted() const { return passive_decode_wanted_; }

  // Confirm-and-resend (see IOHCCover::arm_confirmation_()): while on, received
  // frames are decoded even when the debug switch above is off, because the
  // confirmation of a command is a motor's overheard status answer.
  void set_confirm_resend_wanted(bool wanted) { confirm_resend_wanted_ = wanted; }
  bool confirm_resend_wanted() const { return confirm_resend_wanted_; }
  // True when a motor status answer was heard within the last 6 hours: some
  // controller (TaHoma) polls the motors, so a command that stays
  // unconfirmed means something. Without that, nothing is resent.
  bool status_feedback_available() const;

 protected:
  void maybe_hop_();
  HopPlan hop_plan_{HopPlan::ALL};
  uint32_t hold_freq_{CHANNEL2};
  uint32_t skip_freq_{CHANNEL2};
  uint32_t hop_freq_{CHANNEL2};
  uint8_t hops_since_reading_{0};
  bool plan_override_{false};
  bool preamble_hold_active_{false};
  bool awaiting_redetect_{false};
  uint32_t preamble_slice_start_ms_{0};
  uint32_t preamble_chain_start_ms_{0};
  uint32_t redetect_since_ms_{0};
  uint32_t sync_since_us_{0};
  // ESPHome calls loop() about every 16 ms unless a component asks for a fast
  // loop, which would make the "900 us" hop several times slower. Requested
  // only while a bonding attempt wants the hop.
  HighFrequencyLoopRequester high_freq_loop_;
  // Counters for the once-per-2-s bonding line (see maybe_hop_()).
  uint32_t hop_count_{0};
  uint32_t preamble_count_{0};
  uint32_t stale_recoveries_{0};
  uint32_t hop_stats_ms_{0};
  bool preamble_counted_{false};
  uint32_t last_hop_us_{0};
  bool bonding_hop_wanted_{false};
  bool responder_hop_wanted_{false};
  bool passive_decode_wanted_{false};
  std::string fixed_controller_hex_;
  std::string fixed_system_key_hex_;

  IOHC::IOHCController2W controller2w_;
  IOHC::IOHCKeyExtractor key_extractor_;
  // Total received frame count, any source - only used for the debug log
  // line in on_receive() (frame numbering).
  uint32_t packets_received_{0};
  std::vector<IOHCCover *> status_covers_;
  bool confirm_resend_wanted_{false};
  uint32_t last_status_ms_{0};
};

}  // namespace iohc
}  // namespace esphome
