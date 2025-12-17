# Copyright (c) 2025 Stefan Sundin (4pplet)
# SPDX-License-Identifier: MIT

if(CONFIG_ZMK_KSCAN_TOPRE_ADC)
  zephyr_library()
  zephyr_library_sources(kscan_topre_adc.c)
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
endif()
