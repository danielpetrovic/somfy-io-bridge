"""Somfy IO (io-homecontrol) per-cover diagnostic sensors.

- `rssi`: THIS bridge's own radio's signal strength for the last frame
  received from this cover's motor_address - any frame, 1W or 2W, decoded or
  not. Distinct from Overkiz's own "RSSI Level"/"Discrete RSSI Level"
  sensors, which reflect TaHoma's radio, not this board's - see
  IOHCCover::update_last_rssi() in cover/iohc_cover.cpp. Requires
  motor_address to be set on the cover.

The former `target_closure` type is removed: a Two-Way cover reports its real
position itself, and Confirm And Resend takes the overheard current position
over into a Position (1W) cover.
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
)

from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

TYPE_RSSI = "rssi"

# expire_after: the RSSI sensor is not pushed on a fixed schedule - it only
# updates when a frame happens to be overheard - so with no expiry it would
# silently show a stale-but-plausible-looking last value forever instead of
# going unavailable. This project's own steady-state at-rest polling
# interval was never fully characterized - the ~20s figure elsewhere in this
# codebase is specifically the right-after-a-move backoff, not the at-rest
# cadence, which could plausibly be 30-60min for a shutter nobody's touched.
# 90 minutes gives real margin above that uncertain upper bound so normal
# quiet periods don't falsely flip these to unavailable.
_EXPIRE_AFTER_DEFAULT = "90min"

SENSOR_SCHEMAS = {
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
)


async def to_code(config):
    var = await sensor.new_sensor(config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    if config[CONF_TYPE] == TYPE_RSSI:
        cg.add(cover_var.set_last_rssi_sensor(var))
