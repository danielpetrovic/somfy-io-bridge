#include "iohc_config_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.config_switch";

void IOHCConfigSwitch::setup() {
  bool state = type_ == ConfigSwitchType::MY_PATTERN ? cover_->get_my_pattern_extended() : cover_->get_invert();
  this->publish_state(state);
}

void IOHCConfigSwitch::write_state(bool state) {
  if (type_ == ConfigSwitchType::MY_PATTERN) {
    cover_->set_my_pattern_extended(state);
  } else {
    cover_->set_invert(state);
  }
  this->publish_state(state);
}

void IOHCConfigSwitch::dump_config() {
  LOG_SWITCH("", type_ == ConfigSwitchType::MY_PATTERN ? "Somfy IOHC My Pattern Switch"
                                                         : "Somfy IOHC Invert Direction Switch",
             this);
}

}  // namespace iohc
}  // namespace esphome
