#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../cover/iohc_cover.h"

namespace esphome {
namespace iohc {

// Which per-cover travel-time value this number instance controls - same
// one-class-many-types dispatch pattern as IOHCConfigSwitch/RemoteButton.
enum class ConfigNumberType : uint8_t { TRAVEL_TIME_OPEN, TRAVEL_TIME_CLOSE };

class IOHCConfigNumber : public number::Number, public Component {
 public:
  void setup() override;
  void dump_config() override;
  // Runs after the cover's own setup() (DATA priority) so the cover has
  // already loaded its persisted state from Preferences before this reads
  // it - same ordering as IOHCConfigSwitch.
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_cover(IOHCCover *cover) { cover_ = cover; }
  void set_number_type(ConfigNumberType type) { type_ = type; }

 protected:
  void control(float value) override;

  IOHCCover *cover_{};
  ConfigNumberType type_{ConfigNumberType::TRAVEL_TIME_OPEN};
};

}  // namespace iohc
}  // namespace esphome
