/*
 * Copyright (c) 2020, Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/spi.h>
#include <fsl_flexspi.h>

struct flexspi_imx_config {
	FLEXSPI_Type *base;	/*!< Base address of the FlexSPI controller. */
};

/* Empty API because the driver does not implement any functionality.
   An API is required for device_get_binding to find the device. */
struct flexspi_imx_api {};
