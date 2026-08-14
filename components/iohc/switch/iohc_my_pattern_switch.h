#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../cover/iohc_cover.h"

namespace esphome {
namespace iohc {

class IOHCMyPatternSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;
  // Runs after the cover's own setup() (DATA priority) so the cover has
  // already loaded its persisted my_pattern_extended_ from Preferences
  // before this reads it - same ordering as IOHCModeSelect.
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_cover(IOHCCover *cover) { cover_ = cover; }

 protected:
  void write_state(bool state) override;

  IOHCCover *cover_{};
};

}  // namespace iohc
}  // namespace esphome
