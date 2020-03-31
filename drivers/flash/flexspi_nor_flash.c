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
*******************************************************************************/

static int flexspi_nor_flash_read(struct device *dev, off_t offset,
				  void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD;
	flashXfer.data          = data;
	flashXfer.dataSize      = len;

	status = FLEXSPI_TransferBlocking(dev_cfg->base, &flashXfer);
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
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = 0; // TODO Figure out if this can be left to unknown
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;

	status = FLEXSPI_TransferBlocking(dev_cfg->base, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
/* FIXME: Valid for W25Q128 only, get it from DTS */
static const struct flash_pages_layout dev_layout = {
	/* "page" means the smallest erasable area on the flash device */
	.pages_count = 4 * 1024,  /* sectors */
	.pages_size  = 4 * 1024,  /* of 4 kiB */
};

static void flexspi_nor_flash_pages_layout(
	struct device *dev,
	const struct flash_pages_layout **layout,
	size_t *layout_size
)
{
	*layout_size = 1;
	*layout = &dev_layout;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static const struct flash_driver_api flexspi_nor_flash_api = {
	.read = flexspi_nor_flash_read,
	.write = flexspi_nor_flash_write,
	.erase = flexspi_nor_flash_erase,
	.write_protection = flexspi_nor_flash_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flexspi_nor_flash_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	.write_block_size = 1,
};

static struct flexspi_nor_flash_dev_data flash_dev_data;

static const struct flexspi_nor_flash_dev_config flash_dev_config = {
	.base = FLEXSPI,         /* FIXME - Get from DTS */
	.port = kFLEXSPI_PortA1, /* FIXME - Get from DTS */
};

DEVICE_AND_API_INIT(flexspi_nor_flash,
		    /* TODO This must be configured at compile time based
		       on DTS. */
		    DT_INST_0_FLEXSPI_NOR_FLASH_LABEL,
		    &flexspi_nor_flash_init,
		    &flash_dev_data,
		    &flash_dev_config,
		    POST_KERNEL,
		    CONFIG_FLEXSPI_NOR_FLASH_INIT_PRIORITY,
		    &flexspi_nor_flash_api);
