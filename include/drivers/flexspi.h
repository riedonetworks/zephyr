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
#include <sys/types.h>

/*
 *  A P I
 */

typedef void (*flexspi_api_update_lut)(struct device *dev,
				       unsigned int index,
				       const u32_t *cmd,
				       unsigned int count);

typedef void (*flexspi_api_sw_reset)(struct device *dev);

/* TODO Use standard return code (errno) instead of status_t */
typedef status_t (*flexspi_api_xfer_blocking)(struct device *dev,
					      flexspi_transfer_t *xfer);

typedef void (*flexspi_api_ahb_prefetch)(struct device *dev, bool enable);

typedef void (*flexspi_api_invalidate_dcache)(struct device *dev,
					      off_t offset,
					      s32_t size);

/**
 * @brief FlexSPI driver API.
 * This is the mandatory API any FlexSPI driver needs to expose.
 */
struct flexspi_driver_api {
	flexspi_api_update_lut update_lut;
	flexspi_api_sw_reset sw_reset;
	flexspi_api_xfer_blocking xfer_blocking;
	flexspi_api_ahb_prefetch ahb_prefetch;
	flexspi_api_invalidate_dcache invalidate_dcache;
};

#endif /* ZEPHYR_INCLUDE_DRIVERS_FLEXSPI_H_ */
