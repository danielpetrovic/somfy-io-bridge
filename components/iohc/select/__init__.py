"""Somfy IO (io-homecontrol) per-cover mode select.

Switches one cover between "One-Way" (default: 1W commands as this bridge's
own 1W remote, the position estimated from the travel time) and "Two-Way"
(2W commands to a motor paired to this bridge, confirmed by the motor, with
its real position). The other entities' (1W)/(2W) suffixes name the mode
they belong to. The mode is stored as a number, so renaming these labels
keeps every cover's saved mode. See IOHCCover::Mode in cover/iohc_cover.h.

A third mode, "Open / My / Close" (RTS-bridge-style 3-state discrete
position, no time tracking, no arbitrary percentages), was removed
2026-11-27 as unused - no install, this one included, was ever confirmed
running a cover in it. IOHCCover::Mode keeps its old numeric value (1) only
so a cover whose saved mode is still that byte gets migrated to Position at
boot (see IOHCCover::setup()) rather than silently reinterpreted.

Options list order no longer index-aligns with the Mode enum's numeric
values (removing the middle option broke that) - IOHCModeSelect::control()
and setup() map between the select's index and IOHCCover::Mode explicitly
instead of casting one to the other.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select

from .. import iohc_ns
from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

IOHCModeSelect = iohc_ns.class_("IOHCModeSelect", select.Select, cg.Component)

MODE_OPTIONS = ["One-Way", "Two-Way"]

CONFIG_SCHEMA = select.select_schema(IOHCModeSelect).extend(
    {
        cv.Required(CONF_COVER_ID): cv.use_id(IOHCCover),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await select.new_select(config, options=MODE_OPTIONS)
    await cg.register_component(var, config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cover_var))
