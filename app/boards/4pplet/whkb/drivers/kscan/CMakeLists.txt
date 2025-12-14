# Copyright (c) 2025 Stefan Sundin (4pplet)
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_GPIO_TOPRE)
  zephyr_library()
  zephyr_library_sources(kscan_gpio_topre.c)
endif()
