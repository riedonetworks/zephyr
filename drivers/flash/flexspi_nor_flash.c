/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/*
 * This is a port of the MCUXpresso SDK
 * "driver_examples/flexspi/nor/polling_transfer" to Zephyr.
 */

#include "flexspi_nor_flash.h"

#if defined(CONFIG_MULTITHREADING)
#warning Multithreading not supported yet (proper locking is missing), use at your own risk!
#endif

/*******************************************************************************
 *  F L A S H   A P I
 ******************************************************************************/

int flexspi_nor_flash_read(struct device *dev, off_t offset,
			   void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	/* Prevent reading outside of the flash */
	if ((offset < 0) || ((offset + len) > dev_cfg->size)) {
		return -ENODEV;
	}

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ;
	flashXfer.data          = data;
	flashXfer.dataSize      = len;

	status = flexspi_xfer_blocking(dev_data->flexspi, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

int flexspi_nor_flash_write_protection_set(struct device *dev, bool enable)
{
	if (enable) {
		/* no-op since protection is automatically set
		   by the flash after any operation needing
		   write enable to be set. */
		return 0;
	}

	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = 0; // TODO Figure out if this can be left to unknown
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

	status = flexspi_xfer_blocking(dev_data->flexspi, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
void flexspi_nor_flash_pages_layout(
	struct device *dev,
	const struct flash_pages_layout **layout,
	size_t *layout_size
)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;

	*layout = &dev_cfg->pages_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
