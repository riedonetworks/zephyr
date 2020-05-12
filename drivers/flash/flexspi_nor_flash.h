/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_
#define ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_

#include <device.h>
#include <drivers/flash.h>
#include <drivers/flexspi.h>

/* SPI access modes, corresponds to "spi-access-mode" property in devicetree. */
#define SPI_ACCESS_MODE_SINGLE 0
#define SPI_ACCESS_MODE_DUAL   1
#define SPI_ACCESS_MODE_QUAD   2
#define SPI_ACCESS_MODE_QPI    3

/* Index of the commands in the FlexSPI LUT. */
enum {
	NOR_CMD_LUT_SEQ_IDX_READ = 0,
	NOR_CMD_LUT_SEQ_IDX_WRITEENABLE,
	NOR_CMD_LUT_SEQ_IDX_ERASESECTOR,
	NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM,
	NOR_CMD_LUT_SEQ_IDX_READJEDECID,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3,
	NOR_CMD_LUT_COUNT,
};

struct flexspi_nor_flash_dev_data {
	/** Handle to the FlexSPI driver to which the flash is attached to. */
	struct device *flexspi;
};

struct flexspi_nor_flash_dev_config {
	const char *bus_name;	/* Name of the parent bus (for device_get_binding). */
	flexspi_port_t port;	/* Port on which the device is connected. */
	size_t size;		/* Size of the flash device in bytes. */
	size_t page_size;	/* Max data size in bytes for "page program" command. */
	const u32_t *lut;	/* FlexSPI LUT. */
	size_t lut_length;	/* Number of element in LUT. */
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	struct flash_pages_layout pages_layout;
#endif
};

/*
 * Flash API functions.
 */
int flexspi_nor_flash_read(struct device *dev, off_t offset,
			   void *data, size_t len);
int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len);
int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t size);
int flexspi_nor_flash_write_protection_set(struct device *dev, bool enable);
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
void flexspi_nor_flash_pages_layout(struct device *dev,
				    const struct flash_pages_layout **layout,
				    size_t *layout_size);
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

/*
 * Driver internal functions.
 */
int flexspi_nor_flash_init(struct device *dev);

#endif /* ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_ */
