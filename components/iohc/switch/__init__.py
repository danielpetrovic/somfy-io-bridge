"""Somfy IO (io-homecontrol) per-cover My/Set My pattern switch.

Toggles one cover between the simple (off - single 0xd8 recall frame,
matches two independent real reference implementations, confirmed working
on plain shutters even when 2W-bonded to a TaHoma/Connexoon box) and
extended (on - 16-byte main=0xD200 sequence, needed to reproduce tilt on a
blind, GitHub issue #1) My/Set My wire patterns - see
IOHC::RemoteButton::Vent/SetMy in iohc_remote1w.h for the full picture.

Initial/first-boot state comes from cover/__init__.py's my_pattern YAML
option (auto resolves by device_class); once toggled from Home Assistant,
the chosen state persists in NVS (IOHCCover::set_my_pattern_extended()) and
the YAML default no longer applies on subsequent boots.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch

from .. import iohc_ns
from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

IOHCMyPatternSwitch = iohc_ns.class_("IOHCMyPatternSwitch", switch.Switch, cg.Component)

CONFIG_SCHEMA = switch.switch_schema(IOHCMyPatternSwitch).extend(
    {
        cv.Required(CONF_COVER_ID): cv.use_id(IOHCCover),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cover_var))
