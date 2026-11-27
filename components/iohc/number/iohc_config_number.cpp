#include "iohc_config_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.config_number";

void IOHCConfigNumber::setup() {
  uint32_t seconds =
      type_ == ConfigNumberType::TRAVEL_TIME_OPEN ? cover_->get_travel_time_open() : cover_->get_travel_time_close();
  this->publish_state(static_cast<float>(seconds));
}

void IOHCConfigNumber::control(float value) {
  auto seconds = static_cast<uint32_t>(value);
  if (type_ == ConfigNumberType::TRAVEL_TIME_OPEN) {
    cover_->set_travel_time_open(seconds);
  } else {
    cover_->set_travel_time_close(seconds);
  }
  this->publish_state(value);
}

void IOHCConfigNumber::dump_config() {
  LOG_NUMBER("", type_ == ConfigNumberType::TRAVEL_TIME_OPEN ? "Somfy IOHC Travel Time Open Number"
                                                              : "Somfy IOHC Travel Time Close Number",
             this);
}

}  // namespace iohc
}  // namespace esphome
