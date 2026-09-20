#include "iohc_light.h"
#include "esphome/core/log.h"
#include "../../iohc/iohcRadio.h"
#include <algorithm>
#include <cmath>

namespace esphome {
namespace iohc_light {

static const char *const TAG = "iohc_light.light";

void IOHCLight::setup() {
  remote_.set_type(type_);
  remote_.set_manufacturer(manufacturer_);
  // Bonded identity/sequence persist per-light (IOHCRemote1W::begin() hashes
  // nvs_key_ down to a short NVS namespace internally) - same mechanism as
  // IOHCCover, no separate Preferences instance needed here since a light
  // has no extra state of its own to persist (on/off/brightness restore is
  // handled generically by ESPHome's own LightState, not managed here).
  remote_.begin(IOHC::iohcRadio::getInstance(), this->nvs_key_, fixed_node_hex_, fixed_key_hex_);
}

light::LightTraits IOHCLight::get_traits() {
  auto traits = light::LightTraits();
  traits.set_supported_color_modes({light::ColorMode::BRIGHTNESS});
  return traits;
}

void IOHCLight::write_state(light::LightState *state) {
  float brightness;
  state->current_values_as_brightness(&brightness);
  int percent = static_cast<int>(std::lround(brightness * 100.0f));
  percent = std::clamp(percent, 0, 100);
  // Same absolute-value command already confirmed working for
  // IOHCCover::Mode::POSITION (cmd 0x00, (100-p)*2 in main[0]) - real
  // hardware testing (GitHub issue #2) confirmed the receiver accepts this
  // directly and runs its own short fade to it, so nothing further is
  // needed here: no local ramp to maintain, no readback to reconcile.
  remote_.cmd(IOHC::RemoteButton::Position, percent);
}

void IOHCLight::dump_config() {
  ESP_LOGCONFIG(TAG, "Somfy IOHC Light (BETA):");
  ESP_LOGCONFIG(TAG, "  Paired: %s", remote_.is_paired() ? "yes" : "no");
}

}  // namespace iohc_light
}  // namespace esphome
