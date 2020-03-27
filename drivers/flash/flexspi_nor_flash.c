/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

/*
 * Adapted from MCUXpresso SDK "driver_examples/flexspi/nor/polling_transfer".
 */

#include <drivers/flash.h>

#include "flexspi_nor_flash.h"

/*******************************************************************************
 *  F L A S H   A P I
*******************************************************************************/

static int flexspi_nor_flash_read(struct device *dev, off_t offset,
				  void *data, size_t len)
{
	struct flexspi_flash_data *drv_data =  dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = offset;
	flashXfer.port          = drv_data->port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD;
	flashXfer.data          = data;
	flashXfer.dataSize      = len;
	status = FLEXSPI_TransferBlocking(drv_data->base, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

static int flexspi_nor_flash_write(struct device *dev, off_t offset,
				   const void *data, size_t len)
{
	/* NOT IMPLEMENTED YET */
	return -EPERM;
}

#define SECTOR_SIZE 4096U
#define SECTOR_MASK (4096U - 1U)

static int flexspi_nor_flash_erase(struct device *dev, off_t offset,
				   size_t size)
{
	// /* Offset must be between 0 and flash size */
	// if ((offset < 0) || ((offset + size) > <FLASH SIZE>)) {
	// 	return -ENODEV;
	// }

	/* Offset must correspond to a sector start */
	if (offset & SECTOR_MASK) {
		return -EINVAL;
	}

	struct flexspi_flash_data *drv_data =  dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = offset;
	flashXfer.port          = drv_data->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
	status = FLEXSPI_TransferBlocking(drv_data->base, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	// status = flexspi_nor_wait_bus_busy(base);

	// /* Do software reset. */
	// FLEXSPI_SoftwareReset(base);

	return 0;
}

static int flexspi_nor_flash_write_protection_set(struct device *dev,
						  bool enable)
{
	if (enable) {
		/* no-op since protection is automatically set
		   by the flash after any operation needing
		   write enable to be set. */
		return 0;
	}

	struct flexspi_flash_data *drv_data =  dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.port      = drv_data->port;
	flashXfer.cmdType   = kFLEXSPI_Command;
	flashXfer.SeqNumber = 1;
	flashXfer.seqIndex  = NOR_CMD_LUT_SEQ_IDX_WRITEENABLE;
	status = FLEXSPI_TransferBlocking(drv_data->base, &flashXfer);
	if (status != kStatus_Success) {
		return -EIO;
	}

	return 0;
}

#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static void flexspi_nor_flash_pages_layout(
	struct device *dev,
	const struct flash_pages_layout **layout,
	size_t *layout_size
)
{
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

static struct flexspi_flash_data flexspi_flash_priv_data = {
	.base = FLEXSPI,         /* FIXME - Get from DTS */
	.port = kFLEXSPI_PortA1, /* FIXME - Get from DTS */
};

DEVICE_AND_API_INIT(flexspi_nor_flash,
		    /* TODO This must be configured at compile time based
		       on DTS. */
		    DT_NXP_IMX_FLEXSPI_402A8000_LABEL,
		    &flexspi_nor_flash_init,
		    &flexspi_flash_priv_data,
		    NULL, /* TODO: See if we need this -> cfg_info */
		    POST_KERNEL,
		    CONFIG_FLEXSPI_NOR_FLASH_INIT_PRIORITY,
		    &flexspi_nor_flash_api);
