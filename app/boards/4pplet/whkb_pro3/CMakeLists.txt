# Copyright (c) 2025 Stefan Sundin (4pplet)
# SPDX-License-Identifier: MIT

if(CONFIG_BOARD_WHKB_PRO3)
  add_subdirectory(drivers)
endif()

target_sources(app PRIVATE whkb_pro3_led.c)
target_sources(app PRIVATE whkb_pro3_buzzer.c)
