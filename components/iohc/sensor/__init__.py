"""Somfy IO (io-homecontrol) per-cover diagnostic sensors.

Two `type:` options, both standalone (not tied to the cover entity's own
position/state):

- `target_closure` (default, for backward compatibility with existing YAML
  that omits `type:`): the real, motor-reported closure percentage decoded
  from passively overheard 2W traffic - see IOHCCover::update_real_position()
  in cover/iohc_cover.cpp and README's "Real position feedback" section for
  how this data arrives and what it requires (an existing 2W-bonded
  controller, e.g. TaHoma, already active on the installation - this bridge
  cannot query it alone). Deliberately matches Home Assistant's Overkiz
  integration's own "Target closure" sensor convention exactly (0=open,
  100=closed - the OPPOSITE of this cover's own `position` attribute, which
  follows HA's standard cover convention of 100=open/0=closed) so the two are
  directly comparable. Only ever publishes once real 2W traffic for this
  cover's motor_address has actually been overheard - starts unavailable
  (matching Overkiz's own sensor's behavior on a shutter that's never been
  polled) if none has arrived yet.

- `rssi`: THIS bridge's own radio's signal strength for the last frame
  received from this cover's motor_address - any frame, 1W or 2W, decoded or
  not. Distinct from Overkiz's own "RSSI Level"/"Discrete RSSI Level"
  sensors, which reflect TaHoma's radio, not this board's - see
  IOHCCover::update_last_rssi() in cover/iohc_cover.cpp. Requires
  motor_address to be set on the cover (same requirement as
  target_closure) - without it, this bridge has no way to attribute a
  received frame's RSSI to a specific cover.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_EXPIRE_AFTER,
    CONF_TYPE,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
    UNIT_PERCENT,
)

from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

TYPE_TARGET_CLOSURE = "target_closure"
TYPE_RSSI = "rssi"

# expire_after: neither sensor is pushed on a fixed schedule - both only
# update when a frame happens to be overheard (from an external box, or not
# at all while Passive Decode (2W) is off) - so with no expiry, either would
# silently show a stale-but-plausible-looking last value forever instead of
# going unavailable. This project's own steady-state at-rest polling
# interval was never fully characterized - the ~20s figure elsewhere in this
# codebase is specifically the right-after-a-move backoff, not the at-rest
# cadence, which could plausibly be 30-60min for a shutter nobody's touched.
# 90 minutes gives real margin above that uncertain upper bound so normal
# quiet periods don't falsely flip these to unavailable.
_EXPIRE_AFTER_DEFAULT = "90min"

SENSOR_SCHEMAS = {
    TYPE_TARGET_CLOSURE: sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ).extend({cv.Optional(CONF_EXPIRE_AFTER, default=_EXPIRE_AFTER_DEFAULT): cv.All(cv.positive_time_period_milliseconds)}),
    TYPE_RSSI: sensor.sensor_schema(
        unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ).extend({cv.Optional(CONF_EXPIRE_AFTER, default=_EXPIRE_AFTER_DEFAULT): cv.All(cv.positive_time_period_milliseconds)}),
}

CONFIG_SCHEMA = cv.typed_schema(
    {
        sensor_type: schema.extend(
            {
                cv.Required(CONF_COVER_ID): cv.use_id(IOHCCover),
            }
        )
        for sensor_type, schema in SENSOR_SCHEMAS.items()
    },
    default_type=TYPE_TARGET_CLOSURE,
)


async def to_code(config):
    var = await sensor.new_sensor(config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    if config[CONF_TYPE] == TYPE_RSSI:
        cg.add(cover_var.set_last_rssi_sensor(var))
    else:
        cg.add(cover_var.set_target_closure_sensor(var))
