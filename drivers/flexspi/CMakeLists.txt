#
# Copyright (c) 2020, Riedo Networks Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_library()

zephyr_library_sources_ifdef(CONFIG_FLEXSPI_IMX flexspi_imx.c)

if(CONFIG_FLEXSPI_NOR_FLASH)
  zephyr_code_relocate(flexspi_imx.c ITCM_TEXT DTCM_DATA DTCM_BSS)
endif()
