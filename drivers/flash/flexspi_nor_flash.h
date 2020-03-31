/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#ifndef ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_
#define ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_

#include <device.h>
#include <fsl_flexspi.h>
#include <drivers/flash.h>

struct flexspi_nor_flash_dev_data {
#if defined(CONFIG_MULTITHREADING)
	struct k_sem wr_er_lock;
#endif
};

struct flexspi_nor_flash_dev_config {
	FLEXSPI_Type *base;
	flexspi_port_t port;
};

status_t flexspi_nor_flash_wait_bus_busy(FLEXSPI_Type *base,
					 flexspi_port_t port);
int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t size);
int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len);
int flexspi_nor_flash_write_protection_set(struct device *dev, bool enable);
int flexspi_nor_flash_init(struct device *dev);

/*******************************************************************************
 * Winbond W25Q128
 ******************************************************************************/

enum {
	NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD = 0,
	NOR_CMD_LUT_SEQ_IDX_WRITEENABLE,
	NOR_CMD_LUT_SEQ_IDX_ERASESECTOR,
	NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD,
	NOR_CMD_LUT_SEQ_IDX_READJEDECID,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG1,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2,
	NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3,
	// NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK,
	// NOR_CMD_LUT_SEQ_IDX_GLOBALSECTORUNLOCK,
	// NOR_CMD_LUT_SEQ_IDX_ENTERQPI,
	// NOR_CMD_LUT_SEQ_IDX_EXITQPI,
	NOR_CMD_LUT_COUNT,
};

#endif /* ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_ */
