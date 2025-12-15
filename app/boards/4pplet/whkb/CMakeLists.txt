# Copyright (c) 2025 Stefan Sundin (4pplet)
# SPDX-License-Identifier: MIT

if(CONFIG_BOARD_WHKB)
  add_subdirectory(drivers)
endif()

target_sources(app PRIVATE whkb_led.c)
target_sources(app PRIVATE whkb_buzzer.c)
target_sources_ifdef(CONFIG_WHKB_PRO2_POWER app PRIVATE whkb_pro2_power.c)
