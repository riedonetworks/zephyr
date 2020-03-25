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
	return 0;
}

static int flexspi_nor_flash_write(struct device *dev, off_t offset,
				   const void *data, size_t len)
{
	return 0;
}

static int flexspi_nor_flash_erase(struct device *dev, off_t offset,
				   size_t size)
{
	return 0;
}

static int flexspi_nor_flash_write_protection_set(struct device *dev,
						  bool enable)
{
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

/*******************************************************************************
 *  I N I T
*******************************************************************************/

static int flexspi_nor_flash_init(struct device *dev)
{
	return 0;
}

static const struct flash_driver_api flexspi_nor_flash_api = {
	.read = flexspi_nor_flash_read,
	.write = flexspi_nor_flash_write,
	.erase = flexspi_nor_flash_erase,
	.write_protection = flexspi_nor_flash_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flexspi_nor_flash_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	.write_block_size = 0,
};

static struct flash_priv flexspi_nor_priv_data;

DEVICE_AND_API_INIT(flexspi_nor_flash,
		    /* TODO This must be configured at compile time based
		       on DTS. */
		    DT_NXP_IMX_FLEXSPI_402A8000_LABEL,
		    &flexspi_nor_flash_init,
		    &flexspi_nor_priv_data,
		    NULL, /* TODO: See if we need this -> cfg_info */
		    POST_KERNEL,
		    CONFIG_FLEXSPI_NOR_FLASH_INIT_PRIORITY,
		    &flexspi_nor_flash_api);
