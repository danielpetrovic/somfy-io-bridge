#include "iohc_config_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.config_number";

void IOHCConfigNumber::setup() {
  if (type_ == ConfigNumberType::STATUS_POLL_INTERVAL) {
    this->publish_state(cover_->get_status_poll_interval_ms() / 1000.0f);
    return;
  }
  uint32_t seconds =
      type_ == ConfigNumberType::TRAVEL_TIME_OPEN ? cover_->get_travel_time_open() : cover_->get_travel_time_close();
  this->publish_state(static_cast<float>(seconds));
}

void IOHCConfigNumber::control(float value) {
  if (type_ == ConfigNumberType::STATUS_POLL_INTERVAL) {
    cover_->set_status_poll_interval_ms(static_cast<uint32_t>(value * 1000.0f + 0.5f));
    this->publish_state(value);
    return;
  }
  auto seconds = static_cast<uint32_t>(value);
  if (type_ == ConfigNumberType::TRAVEL_TIME_OPEN) {
    cover_->set_travel_time_open(seconds);
  } else {
    cover_->set_travel_time_close(seconds);
  }
  this->publish_state(value);
}

void IOHCConfigNumber::dump_config() {
  LOG_NUMBER("",
             type_ == ConfigNumberType::TRAVEL_TIME_OPEN    ? "Somfy IOHC Travel Time Open Number"
             : type_ == ConfigNumberType::TRAVEL_TIME_CLOSE ? "Somfy IOHC Travel Time Close Number"
                                                            : "Somfy IOHC Status Poll Interval Number",
             this);
}

}  // namespace iohc
}  // namespace esphome
