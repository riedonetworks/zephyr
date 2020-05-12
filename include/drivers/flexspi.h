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

#if defined(CONFIG_USERSPACE)
#error "FlexSPI drivers don't have syscall support and therefore cannot be used with CONFIG_USERSPACE"
/* See https://docs.zephyrproject.org/latest/reference/usermode/syscalls.html */
#endif

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

/*
 *  F U N C T I O N S
 */

/**
 * @brief Update the controller look-up table.
 *
 * @note Each command consists of up to 8 instructions and
 *       occupy 4*32-bit of memory.
 *
 * @param dev FlexSPI device.
 * @param index From which index in the LUT to update.
 * @param cmd Command sequence array.
 * @param count Number of sequences.
 */
static inline void flexspi_update_lut(struct device *dev,
				      unsigned int index,
				      const u32_t *cmd,
				      unsigned int count)
{
	const struct flexspi_driver_api *api = dev->driver_api;

	api->update_lut(dev, index, cmd, count);
}

/**
 * @brief Do a software reset of the FLEXSPI logic.
 *
 * @param dev FlexSPI device.
 */
static inline  void flexspi_sw_reset(struct device *dev)
{
	const struct flexspi_driver_api *api = dev->driver_api;

	api->sw_reset(dev);
}

/**
 * @brief Use IP command to transfer data using a blocking method.
 *
 * @param dev FlexSPI device.
 * @param xfer Transfer description.
 * @return kStatus_Success on success.
 */
static inline status_t flexspi_xfer_blocking(struct device *dev,
					     flexspi_transfer_t *xfer)
{
	const struct flexspi_driver_api *api = dev->driver_api;

	return api->xfer_blocking(dev, xfer);
}

/**
 * @brief Enable or disable AHB read prefetch.
 *
 * @param dev FlexSPI device.
 * @param enable True to enable prefetch, false to disable it.
 */
static inline void flexspi_ahb_prefetch(struct device *dev, bool enable)
{
	const struct flexspi_driver_api *api = dev->driver_api;

	api->ahb_prefetch(dev, enable);
}

/**
 * @brief Invalidate data cache for data located in the FlexSPI memory map.
 *
 * @param dev FlexSPI device.
 * @param offset Offset of the data in the FlexSPI memory map.
 * @param size Number of bytes to invalidate from offset.
 */
static inline void flexspi_invalidate_dcache(struct device *dev,
					     off_t offset,
					     s32_t size)
{
	const struct flexspi_driver_api *api = dev->driver_api;

	api->invalidate_dcache(dev, offset, size);
}

#endif /* ZEPHYR_INCLUDE_DRIVERS_FLEXSPI_H_ */
