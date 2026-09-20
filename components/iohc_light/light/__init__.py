"""Somfy IO (io-homecontrol) dimmable light platform - BETA, GitHub issue #2.

One IOHCLight = one bonded virtual remote identity (IOHC::IOHCRemote1W),
same command layer IOHCCover already uses, talking to a separate io
dimming receiver rather than a motor. Brightness-only (ColorMode::BRIGHTNESS)
- no color/color-temperature claim, since nothing about that has been
confirmed against real hardware. See iohc_light.h for why there is no local
ramp/travel-time estimate here at all, unlike the cover platform.
"""

import re

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import light
from esphome.const import CONF_OUTPUT_ID

from .. import iohc_light_ns

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

CONF_BROADCAST_TYPE = "broadcast_type"
CONF_MANUFACTURER = "manufacturer"
CONF_NODE = "node"
CONF_KEY = "key"

IOHCLight = iohc_light_ns.class_("IOHCLight", light.LightOutput, cg.Component)


def validate_hex_string(length):
    pattern = re.compile(rf"^[0-9a-fA-F]{{{length}}}$")

    def validator(value):
        value = cv.string_strict(value)
        if value == "":
            return value  # not set - auto-generate and persist on-device instead
        if not pattern.match(value):
            raise cv.Invalid(f"must be exactly {length} hex characters (or empty to auto-generate)")
        return value.lower()

    return validator


CONFIG_SCHEMA = light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(IOHCLight),
        # 6 = "Light" per sDevicesType in iohc_utils.h - the broadcast group
        # this receiver actually listens on, confirmed against real
        # hardware (GitHub issue #2). Overridable in case a future light
        # needs a different group, same reasoning as the cover platform's
        # own broadcast_type option.
        cv.Optional(CONF_BROADCAST_TYPE, default=6): cv.int_range(min=0, max=15),
        cv.Optional(CONF_MANUFACTURER, default=2): cv.int_range(min=0, max=255),
        # Same fixed-identity mechanism as the cover platform's node/key -
        # see IOHCLight::set_fixed_node()'s own comment.
        cv.Optional(CONF_NODE, default=""): validate_hex_string(6),
        cv.Optional(CONF_KEY, default=""): cv.sensitive(validate_hex_string(32)),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)

    # Deliberately the light's own YAML component id, not get_object_id() -
    # same reasoning as IOHCCover's own set_nvs_key() comment.
    cg.add(var.set_nvs_key(str(config[CONF_OUTPUT_ID])))
    cg.add(var.set_type(config[CONF_BROADCAST_TYPE]))
    cg.add(var.set_manufacturer(config[CONF_MANUFACTURER]))
    if config[CONF_NODE]:
        cg.add(var.set_fixed_node(config[CONF_NODE]))
    if config[CONF_KEY]:
        cg.add(var.set_fixed_key(config[CONF_KEY]))

    await light.register_light(var, config)
