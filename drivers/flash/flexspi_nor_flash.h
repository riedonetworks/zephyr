/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#ifndef ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_
#define ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_

#include <device.h>
#include <fsl_flexspi.h>
#include <drivers/flash.h>

struct flexspi_flash_data {
	FLEXSPI_Type *base;
	flexspi_port_t port;
// #if defined(CONFIG_MULTITHREADING)
// 	struct k_sem write_lock;
// #endif
};

status_t flexspi_nor_flash_wait_bus_busy(FLEXSPI_Type *base,
					 flexspi_port_t port);
int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t size);
int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len);
int flexspi_nor_flash_init(struct device *dev);

/*******************************************************************************
 * Winbond W25Q128
 ******************************************************************************/

#define NOR_CMD_LUT_SEQ_IDX_READ_NORMAL         7
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST          13
#define NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD      0
#define NOR_CMD_LUT_SEQ_IDX_READSTATUS          1
#define NOR_CMD_LUT_SEQ_IDX_WRITEENABLE         2
#define NOR_CMD_LUT_SEQ_IDX_ERASESECTOR         3
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE  6
#define NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD    4
#define NOR_CMD_LUT_SEQ_IDX_READID              8
#define NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG      9
#define NOR_CMD_LUT_SEQ_IDX_ENTERQPI           10
#define NOR_CMD_LUT_SEQ_IDX_EXITQPI            11
#define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG      12
#define NOR_CMD_LUT_SEQ_IDX_ERASECHIP           5

#define NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK     14
// #define NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3     15
#define NOR_CMD_LUT_SEQ_IDX_GLOBALSECTORUNLOCK 15

#endif /* ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_ */
