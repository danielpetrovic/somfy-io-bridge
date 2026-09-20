"""Somfy IO (io-homecontrol) dimmable light - BETA, GitHub issue #2.

A separate top-level component from `iohc` (not a new subfolder under it),
purely so its own `button/` platform (the Program (1W) pairing trigger a
light needs, same idea as a cover's own Program button) doesn't require
touching `iohc/button/`'s existing, already-released schema - ESPHome only
allows one `__init__.py` per (component, domain) pair, and `iohc`+`button`
is already taken by the cover-oriented IOHCPairButton. Depends on `iohc`
for the actual radio hub (IOHC::iohcRadio) and command layer
(IOHC::IOHCRemote1W) - see `light/iohc_light.h` and
`button/iohc_light_program_button.h`, both of which talk to that shared
C++ layer directly, same as `iohc`'s own cover platform does.

No YAML config of its own - nothing here needs a top-level `iohc_light:`
block, only `light: platform: iohc_light` / `button: platform: iohc_light`
entries (see somfy-io-light.yaml).
"""

import esphome.codegen as cg

CODEOWNERS = ["@danielpetrovic"]
DEPENDENCIES = ["iohc"]

iohc_light_ns = cg.esphome_ns.namespace("iohc_light")
