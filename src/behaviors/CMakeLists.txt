# Copyright (c) 2023 The ZMK Contributors
# SPDX-License-Identifier: MIT

target_sources_ifdef(CONFIG_PAW3395 app PRIVATE behavior_paw3395_calibrate.c)
target_sources_ifdef(CONFIG_PAW3395 app PRIVATE behavior_paw3395_power_mode_toggle.c)
