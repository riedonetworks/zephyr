#
# Copyright (c) 2020 Riedo Networks Ltd.
#
# SPDX-License-Identifier: Apache-2.0
#

menuconfig FLEXSPI_IMX
	bool "FlexSPI pseudo driver"
	depends on SOC_SERIES_IMX_RT
	help
	  Allow FlexSPI devices to figure out to which controller
	  they are attached to.
