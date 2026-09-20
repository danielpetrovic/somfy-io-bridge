#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../cover/iohc_cover.h"

namespace esphome {
namespace iohc {

// Which per-cover boolean this switch instance controls - one C++ class,
// dispatched by type: (same one-class-many-types pattern button/ already
// uses for RemoteButton), rather than a separate class per switch.
enum class ConfigSwitchType : uint8_t { MY_PATTERN, INVERT };

class IOHCConfigSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;
  // Runs after the cover's own setup() (DATA priority) so the cover has
  // already loaded its persisted state from Preferences before this reads
  // it - same ordering as IOHCModeSelect.
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_cover(IOHCCover *cover) { cover_ = cover; }
  void set_switch_type(ConfigSwitchType type) { type_ = type; }

 protected:
  void write_state(bool state) override;

  IOHCCover *cover_{};
  ConfigSwitchType type_{ConfigSwitchType::MY_PATTERN};
};

}  // namespace iohc
}  // namespace esphome
