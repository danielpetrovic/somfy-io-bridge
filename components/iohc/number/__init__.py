"""Somfy IO (io-homecontrol) per-cover travel-time numbers.

Two seconds-valued numbers sharing one generic class (same one-class-many-
types pattern as switch/'s ConfigSwitchType dispatch, rather than a separate
C++ class per number):

- travel_time_open: how long the local BlindPosition estimate takes to run
  from fully closed to fully open, in HA space.
- travel_time_close: the same for closed, in HA space.

One fixed 25s constant couldn't fit two differently-geared covers on the
same board, or even both directions of the same cover (GitHub issue #4 -
real stopwatch data showed errors running in opposite directions on the
same board). Purely cosmetic in Mode::POSITION (the real command always
carries an exact percentage, the motor lands there regardless), but
genuinely affects how closely the displayed position tracks reality in
Mode::MY, where Open/Close never carry a percentage and this estimate is
the only thing driving what HA shows while moving.

Both are seeded on first-ever boot from their own cover/__init__.py YAML
option (travel_time_open / travel_time_close); once changed from Home
Assistant, the persisted NVS value wins over the YAML default on every
subsequent boot. Always HA-space regardless of Invert Direction - see
IOHCCover::apply_travel_times_() for the raw-space mapping.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import CONF_TYPE, UNIT_SECOND

from .. import iohc_ns
from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

IOHCConfigNumber = iohc_ns.class_("IOHCConfigNumber", number.Number, cg.Component)
ConfigNumberType = iohc_ns.enum("ConfigNumberType", is_class=True)

TYPES = {
    "travel_time_open": ConfigNumberType.TRAVEL_TIME_OPEN,
    "travel_time_close": ConfigNumberType.TRAVEL_TIME_CLOSE,
    # Two-Way mode: seconds between position reads while the motor moves
    # (laberning/home_io_control's status_poll_interval, minimum 0.5 s; 3 s
    # equals its default).
    "status_poll_interval": ConfigNumberType.STATUS_POLL_INTERVAL,
}

# min, max, step per type.
RANGES = {
    "travel_time_open": (1, 120, 1),
    "travel_time_close": (1, 120, 1),
    "status_poll_interval": (0.5, 10, 0.5),
}

CONFIG_SCHEMA = number.number_schema(
    IOHCConfigNumber,
    unit_of_measurement=UNIT_SECOND,
).extend(
    {
        cv.Required(CONF_COVER_ID): cv.use_id(IOHCCover),
        cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    min_value, max_value, step = RANGES[config[CONF_TYPE]]
    var = await number.new_number(config, min_value=min_value, max_value=max_value, step=step)
    await cg.register_component(var, config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cover_var))
    cg.add(var.set_number_type(config[CONF_TYPE]))
