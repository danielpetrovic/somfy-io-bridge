#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"
#include "esphome/components/light/light_traits.h"
#include "esphome/components/light/light_state.h"
#include "../../iohc/iohc_remote1w.h"

namespace esphome {
namespace iohc_light {

// BETA (GitHub issue #2) - a dimmable io-homecontrol light behind a
// separate io receiver (not a wall dimmer, not a luminaire with its own
// built-in receiver - see the issue for the real hardware this was built
// against). Brightness is set directly: real-hardware testing (three
// distinct levels, each allowed to settle) confirmed the receiver accepts
// an absolute level and runs its own ~2-3s fade to it, so unlike
// IOHCCover's Position mode there is no local travel-time estimate to
// maintain here at all - write_state() sends the level once and ESPHome's
// own LightState publishes it immediately (optimistic, matches what the
// device actually does).
class IOHCLight : public light::LightOutput, public Component {
 public:
  void setup() override;
  void dump_config() override;
  // Runs after IOHCComponent's own setup() (HARDWARE priority) has started
  // the radio - same ordering as IOHCCover.
  float get_setup_priority() const override { return setup_priority::DATA; }

  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

  void set_type(uint8_t type) { type_ = type; }
  void set_manufacturer(uint8_t manufacturer) { manufacturer_ = manufacturer; }
  // Optional (6/32 hex chars) - same fixed-identity mechanism as
  // IOHCCover's own node/key, see its own comment for why (board
  // replacement without re-pairing).
  void set_fixed_node(const std::string &node_hex) { fixed_node_hex_ = node_hex; }
  void set_fixed_key(const std::string &key_hex) { fixed_key_hex_ = key_hex; }
  // The light's own YAML component id - same role as IOHCCover's
  // nvs_key_ (stable NVS namespace for the bonded identity, deliberately
  // decoupled from HA-facing naming).
  void set_nvs_key(const std::string &key) { nvs_key_ = key; }

  IOHC::IOHCRemote1W &remote() { return remote_; }

 protected:
  IOHC::IOHCRemote1W remote_;
  uint8_t type_{6};  // 6 = "Light" per sDevicesType in iohc_utils.h
  uint8_t manufacturer_{2};
  std::string fixed_node_hex_;
  std::string fixed_key_hex_;
  std::string nvs_key_;
};

}  // namespace iohc_light
}  // namespace esphome
