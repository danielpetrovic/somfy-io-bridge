#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "../light/iohc_light.h"

namespace esphome {
namespace iohc_light {

// Program (1W) - the light's own pairing trigger, same procedure as a
// cover's own Program button (see IOHC::RemoteButton::Prog's own comment in
// iohc_remote1w.h: resolves to Add or Remove internally based on this
// light's own persisted paired_ flag). Deliberately its own tiny class
// rather than reusing iohc's cover-oriented IOHCPairButton - a light only
// ever needs this one action (no My/Identify/Prog2W equivalent has been
// confirmed to make sense for a dimmer yet), and ESPHome only allows one
// button/ platform per component, already taken by IOHCPairButton under
// iohc itself - see this component's own __init__.py for the full reasoning.
class IOHCLightProgramButton : public button::Button, public Component {
 public:
  void dump_config() override;

  void set_light(IOHCLight *light) { light_ = light; }

 protected:
  void press_action() override;

  IOHCLight *light_{};
};

}  // namespace iohc_light
}  // namespace esphome
