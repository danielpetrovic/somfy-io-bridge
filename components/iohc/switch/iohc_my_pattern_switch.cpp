#include "iohc_my_pattern_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.my_pattern_switch";

void IOHCMyPatternSwitch::setup() { this->publish_state(cover_->get_my_pattern_extended()); }

void IOHCMyPatternSwitch::write_state(bool state) {
  cover_->set_my_pattern_extended(state);
  this->publish_state(state);
}

void IOHCMyPatternSwitch::dump_config() { LOG_SWITCH("", "Somfy IOHC My Pattern Switch", this); }

}  // namespace iohc
}  // namespace esphome
