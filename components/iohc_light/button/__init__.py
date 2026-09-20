"""Somfy IO (io-homecontrol) light pairing button - BETA, GitHub issue #2.

Program (1W) only - the one action a light actually needs. See
iohc_light_program_button.h for why this is its own tiny class rather than
reusing iohc's cover-oriented button platform.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button

from .. import iohc_light_ns
from ..light import IOHCLight

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_LIGHT_ID = "light_id"

IOHCLightProgramButton = iohc_light_ns.class_("IOHCLightProgramButton", button.Button, cg.Component)

CONFIG_SCHEMA = button.button_schema(IOHCLightProgramButton).extend(
    {
        cv.Required(CONF_LIGHT_ID): cv.use_id(IOHCLight),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await button.new_button(config)
    await cg.register_component(var, config)

    light_var = await cg.get_variable(config[CONF_LIGHT_ID])
    cg.add(var.set_light(light_var))
