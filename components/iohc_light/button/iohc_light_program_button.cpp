#include "iohc_light_program_button.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc_light {

static const char *const TAG = "iohc_light.button";

void IOHCLightProgramButton::press_action() {
  ESP_LOGI(TAG, "%s pressed - transmitting Prog", this->get_name().c_str());
  light_->remote().cmd(IOHC::RemoteButton::Prog);
}

void IOHCLightProgramButton::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy IOHC Light Program Button '%s'", this->get_name().c_str());
}

}  // namespace iohc_light
}  // namespace esphome
