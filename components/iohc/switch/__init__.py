"""Somfy IO (io-homecontrol) per-cover config switches.

Two boolean switches sharing one generic class (same one-class-many-types
pattern as button/'s RemoteButton dispatch, rather than a separate C++ class
per switch):

- my_pattern: toggles one cover between the simple (off - single 0xd8
  recall frame, matches two independent real reference implementations,
  confirmed working on plain shutters even when 2W-bonded to a
  TaHoma/Connexoon box) and extended (on - 16-byte main=0xD200 sequence,
  needed to reproduce tilt on a blind, GitHub issue #1) My/Set My wire
  patterns - see IOHC::RemoteButton::Vent/SetMy in iohc_remote1w.h for the
  full picture.
- invert: swaps which RemoteButton actually gets sent for Open/Close at
  the component boundary, for an installation where HA's own open/close
  convention comes out backwards (GitHub issue #3) - see
  IOHCCover::control()'s own comments for the full picture.

Both are seeded on first-ever boot from their own cover/__init__.py YAML
option (my_pattern / invert); once toggled from Home Assistant, the
persisted NVS value wins over the YAML default on every subsequent boot.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_TYPE

from .. import iohc_ns
from ..cover import IOHCCover

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_COVER_ID = "cover_id"

IOHCConfigSwitch = iohc_ns.class_("IOHCConfigSwitch", switch.Switch, cg.Component)
ConfigSwitchType = iohc_ns.enum("ConfigSwitchType", is_class=True)

TYPES = {
    "my_pattern": ConfigSwitchType.MY_PATTERN,
    "invert": ConfigSwitchType.INVERT,
}

CONFIG_SCHEMA = switch.switch_schema(IOHCConfigSwitch).extend(
    {
        cv.Required(CONF_COVER_ID): cv.use_id(IOHCCover),
        cv.Required(CONF_TYPE): cv.enum(TYPES, lower=True),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await switch.new_switch(config)
    await cg.register_component(var, config)

    cover_var = await cg.get_variable(config[CONF_COVER_ID])
    cg.add(var.set_cover(cover_var))
    cg.add(var.set_switch_type(config[CONF_TYPE]))
