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
 * Winbond W25Q
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

#define CUSTOM_LUT_LENGTH 64
// #define FLASH_QUAD_ENABLE 0x40

static const uint32_t customLUT[CUSTOM_LUT_LENGTH] = {
	/* Normal read mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x03,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_NORMAL + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Fast read mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x0B,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_1PAD, 0x08,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Fast read quad mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04
	),

	/* Read extend parameters */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUS] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x81,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Write Enable */
	[4 * NOR_CMD_LUT_SEQ_IDX_WRITEENABLE] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x06,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Erase Sector */
	[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x20,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),

	/* Page Program - single mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x02,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Page Program - quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Read ID */
	[4 * NOR_CMD_LUT_SEQ_IDX_READID] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Enable Quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x01,
		kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Enter QPI mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_ENTERQPI] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Exit QPI mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_EXITQPI] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_4PAD, 0xF5,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Read status register 1 */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Erase whole chip */
	[4 * NOR_CMD_LUT_SEQ_IDX_ERASECHIP] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xC7,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Read sector lock status */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x3D,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Read status register 3 */
// 	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3] = FLEXSPI_LUT_SEQ(
// 		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x15,
// 		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
// 	),

        /* Global Block/Sector Unlock */
	[4 * NOR_CMD_LUT_SEQ_IDX_GLOBALSECTORUNLOCK] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x98,
		kFLEXSPI_Command_STOP, 0, 0
	),
};

#endif /* ZEPHYR_DRIVERS_FLEXSPI_NOR_FLASH_H_ */
