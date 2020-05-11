/*
 * Copyright (c) 2020, Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Public API for FlexSPI drivers.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_FLEXSPI_H_
#define ZEPHYR_INCLUDE_DRIVERS_FLEXSPI_H_

#include <device.h>
#include <fsl_flexspi.h>
#include <zephyr/types.h>

typedef void (*flexspi_api_update_lut)(struct device *dev,
				       unsigned int index,
				       const u32_t *cmd,
				       unsigned int count);

typedef void (*flexspi_api_sw_reset)(struct device *dev);

typedef status_t (*flexspi_api_xfer_blocking)(struct device *dev,
					      flexspi_transfer_t *xfer);

/**
 * @brief FlexSPI driver API.
 * This is the mandatory API any FlexSPI driver needs to expose.
 */
struct flexspi_driver_api {
	flexspi_api_update_lut update_lut;
	flexspi_api_sw_reset sw_reset;
	flexspi_api_xfer_blocking xfer_blocking;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_FLEXSPI_H_ */
