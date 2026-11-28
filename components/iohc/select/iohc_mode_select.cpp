#include "iohc_mode_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace iohc {

static const char *const TAG = "iohc.mode_select";

// MODE_OPTIONS (select/__init__.py) is ["One-Way", "Two-Way"],
// no longer index-aligned with IOHCCover::Mode's numeric values (removing
// the old middle option, "Open / My / Close", broke that) - map explicitly
// in both directions instead of casting. By the time this runs, the cover's
// own setup() (which this select depends on, see get_setup_priority()) has
// already migrated any saved Mode::MY away, so get_mode() can only ever
// return POSITION or TWO_WAY here.
static size_t mode_to_index(IOHCCover::Mode mode) { return mode == IOHCCover::Mode::TWO_WAY ? 1 : 0; }
static IOHCCover::Mode index_to_mode(size_t index) {
  return index == 1 ? IOHCCover::Mode::TWO_WAY : IOHCCover::Mode::POSITION;
}

void IOHCModeSelect::setup() { this->publish_state(mode_to_index(cover_->get_mode())); }

void IOHCModeSelect::control(size_t index) {
  cover_->set_mode(index_to_mode(index));
  this->publish_state(index);
}

void IOHCModeSelect::dump_config() { LOG_SELECT("", "Somfy IOHC Mode Select", this); }

}  // namespace iohc
}  // namespace esphome
