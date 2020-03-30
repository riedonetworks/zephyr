/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 */

#include "flexspi_nor_flash.h"

#define LOG_MODULE_NAME flexspi_nor_flash
#define LOG_LEVEL CONFIG_FLASH_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#if !defined(CONFIG_CODE_DATA_RELOCATION)
#error CONFIG_CODE_DATA_RELOCATION must be enable to use FLEXSPI_NOR_FLASH.
#endif

/*******************************************************************************
 * Winbond W25Q128 LUT
 ******************************************************************************/

#define W25Q128_LUT_LENGTH (8 * 4)

static const uint32_t w25q128LUT[W25Q128_LUT_LENGTH] = {
	/* Fast read quad mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_DUMMY_SDR, kFLEXSPI_4PAD, 0x06,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_4PAD, 0x04
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
	// [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] = FLEXSPI_LUT_SEQ(
	// 	kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32,
	// 	kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	// ),
	// [4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] = FLEXSPI_LUT_SEQ(
	// 	kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04,
	// 	kFLEXSPI_Command_STOP, 0, 0
	// ),

	/* Read JEDEC ID */
	[4 * NOR_CMD_LUT_SEQ_IDX_READJEDECID] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Read status register 1 */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x05,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Read status register 2 */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x35,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Read status register 3 */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x15,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Read sector lock status */
	// [4 * NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK] = FLEXSPI_LUT_SEQ(
	// 	kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x3D,
	// 	kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	// ),
	// [4 * NOR_CMD_LUT_SEQ_IDX_READSECTORLOCK + 1] = FLEXSPI_LUT_SEQ(
	// 	kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04,
	// 	kFLEXSPI_Command_STOP, 0, 0
	// ),

	/* Global Block/Sector Unlock */
	// [4 * NOR_CMD_LUT_SEQ_IDX_GLOBALSECTORUNLOCK] = FLEXSPI_LUT_SEQ(
	// 	kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x98,
	// 	kFLEXSPI_Command_STOP, 0, 0
	// ),
};

/*******************************************************************************
 *  H E L P E R S
*******************************************************************************/

#define REG_STATUS_BIT_BUSY 1U

status_t flexspi_nor_flash_wait_bus_busy(FLEXSPI_Type *base,
					 flexspi_port_t port)
{
	flexspi_transfer_t flashXfer;
	status_t status;
	u32_t reg;
	bool isBusy;

	flashXfer.deviceAddress = 0;
	flashXfer.port          = port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READSTATUSREG1;
	flashXfer.data          = &reg;
	flashXfer.dataSize      = 1;

	do {
		status = FLEXSPI_TransferBlocking(base, &flashXfer);
		if (status != kStatus_Success) {
			return status;
		}

		if (reg & REG_STATUS_BIT_BUSY) {
			isBusy = true;
		}
		else {
			isBusy = false;
		}
	} while (isBusy);

	return status;
}

/*******************************************************************************
 *  F L A S H   A P I
*******************************************************************************/

/* FIXME: Get it from DTS or page_layout */
#define SECTOR_SIZE 4096U
#define SECTOR_MASK (4096U - 1U)

int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t size)
{
	// /* Offset must be between 0 and flash size */
	// if ((offset < 0) || ((offset + size) > <FLASH SIZE>)) {
	// 	return -ENODEV;
	// }

	/* Offset must correspond to a sector start */
	if (offset & SECTOR_MASK) {
		return -EINVAL;
	}

	struct flexspi_flash_data *drv_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;

	int retval = 0;

	k_sched_lock();
	unsigned int key = irq_lock();

	/* Erase sector */
	flashXfer.deviceAddress = offset;
	flashXfer.port          = drv_data->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;
	status = FLEXSPI_TransferBlocking(drv_data->base, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(drv_data->base,
						 drv_data->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	// /* Do software reset. */
	// FLEXSPI_SoftwareReset(base);

done:
	irq_unlock(key);
	k_sched_unlock();
	return retval;
}

/* FIXME: Get it from DTS */
#define PAGE_SIZE 256U

int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	// TODO: Allow to write more than a page, by splitting writes
	//       into page size chunks.
	if (len > PAGE_SIZE) {
		return -EINVAL;
	}

	struct flexspi_flash_data *drv_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;
	int retval = 0;

	k_sched_lock();
	unsigned int key = irq_lock();

	flashXfer.deviceAddress = offset;
	flashXfer.port          = drv_data->port;
	flashXfer.cmdType       = kFLEXSPI_Write;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_SINGLE;
	flashXfer.data          = (u32_t *)data;
	flashXfer.dataSize      = len;
	status = FLEXSPI_TransferBlocking(drv_data->base, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(drv_data->base,
						 drv_data->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	/* Do software reset. */
	// FLEXSPI_SoftwareReset(base);

done:
	irq_unlock(key);
	k_sched_unlock();
	return retval;
}

/*******************************************************************************
 *  I N I T
*******************************************************************************/

/* TODO: Verify if it is actually required to have this function in RAM */
int flexspi_nor_flash_init(struct device *dev)
{
	struct flexspi_flash_data *drv_data = dev->driver_data;

	/* Update LUT table. */
	FLEXSPI_UpdateLUT(drv_data->base, 0, w25q128LUT, W25Q128_LUT_LENGTH);

	/* Do software reset. */
	FLEXSPI_SoftwareReset(drv_data->base);

	return 0;
}
