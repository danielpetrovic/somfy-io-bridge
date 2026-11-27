"""Somfy IO (io-homecontrol) per-cover mode select.

Switches one cover between "Position (1W)" (default - the only mode that
sends arbitrary percentage targets to the motor; the displayed position
while moving is a cosmetic local travel-time estimate only, see
IOHCCover::Mode) and "Two-Way (2W)" (real 2W commands to an actually-bonded
motor - the control path is implemented, but this bridge's own bonding
(Program (2W)) has never yet completed successfully against real hardware,
so in practice every command currently gets refused with a warning until
that happens). Labels follow this project's existing Program (1W)/
Program (2W) naming. See IOHCCover::Mode in cover/iohc_cover.h.

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

MODE_OPTIONS = ["Position (1W)", "Two-Way (2W)"]

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
