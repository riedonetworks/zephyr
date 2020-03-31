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

	/* Page Program - quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

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

#if CONFIG_FLASH_LOG_LEVEL >= 4
static status_t flexspi_nor_flash_get_reg(FLEXSPI_Type *base,
					  flexspi_port_t port,
					  u8_t cmd_idx,
					  void *reg,
					  size_t len)
{
	flexspi_transfer_t flashXfer;
	status_t status;

	flashXfer.deviceAddress = 0;
	flashXfer.port          = port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = cmd_idx;
	flashXfer.data          = reg;
	flashXfer.dataSize      = len;

	status = FLEXSPI_TransferBlocking(base, &flashXfer);

	return status;
}
#endif

/*******************************************************************************
 *  F L A S H   A P I
*******************************************************************************/

/* FIXME: Get it from DTS or page_layout */
#define SECTOR_SIZE 4096U
#define SECTOR_MASK (4096U - 1U)

static int flexspi_nor_flash_sector_erase(struct device *dev, off_t offset)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	flexspi_transfer_t flashXfer;
	status_t status;
	int retval = 0;

	k_sched_lock();
	unsigned int key = irq_lock();

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;

	status = FLEXSPI_TransferBlocking(dev_cfg->base, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(dev_cfg->base, dev_cfg->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

done:
	irq_unlock(key);
	k_sched_unlock();
	return retval;
}

int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t len)
{
	// /* Offset must be between 0 and flash size */
	// if ((offset < 0) || ((offset + size) > <FLASH SIZE>)) {
	// 	return -ENODEV;
	// }

	/* Can only erase full sector(s) */
	if ((offset & SECTOR_MASK) || (len & SECTOR_MASK)) {
		return -EINVAL;
	}

	// TODO: Improve performance by using block erase 32 kiB ot 64 kiB
	//       when len is big enough.

	off_t sector;
	int retval;

	for (
		sector = offset;
		sector < offset + len - SECTOR_SIZE;
		sector += SECTOR_SIZE
	) {
		retval = flexspi_nor_flash_sector_erase(dev, sector);
		if (retval) {
			return retval;
		}

		retval = flexspi_nor_flash_write_protection_set(dev, false);
		if (retval) {
			return retval;
		}
	}

	return flexspi_nor_flash_sector_erase(dev, sector);
}

/* FIXME: Get it from DTS */
#define PAGE_SIZE 256U
#define PAGE_MASK (256U - 1U)

static int flexspi_nor_flash_page_program(struct device *dev, off_t offset,
					  const void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	flexspi_transfer_t flashXfer;
	status_t status;
	int retval = 0;

	k_sched_lock();
	unsigned int key = irq_lock();

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Write;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM_QUAD;
	flashXfer.data          = (void *)data;
	flashXfer.dataSize      = len;

	status = FLEXSPI_TransferBlocking(dev_cfg->base, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(dev_cfg->base, dev_cfg->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

done:
	irq_unlock(key);
	k_sched_unlock();
	return retval;
}

int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	// /* Offset must be between 0 and flash size */
	// if ((offset < 0) || ((offset + size) > <FLASH SIZE>)) {
	// 	return -ENODEV;
	// }

	// /* Offset + len must not exceed flash size */
	// if (offset + len > <FLASH SIZE>)
	// 	return -EINVAL;
	// }

	/* Cast data  to prevent  void * arithmetic */
	const u8_t *data_ptr = data;
	off_t next_page_offset;
	size_t chunk_len;
	int retval;

	while (len > 0) {
		next_page_offset = (offset & ~PAGE_MASK) + PAGE_SIZE;
		chunk_len = next_page_offset - offset;
		if (chunk_len > len) {
			chunk_len = len;
		}

		retval = flexspi_nor_flash_page_program(dev, offset,
							data_ptr, chunk_len);
		if (retval) {
			return retval;
		}

		len      -= chunk_len;
		offset   += chunk_len;
		data_ptr += chunk_len;

		if (len > 0) {
			retval = flexspi_nor_flash_write_protection_set(dev,
									false);
			if (retval) {
				return retval;
			}
		}
	}

	return 0;
}

/*******************************************************************************
 *  I N I T
*******************************************************************************/

/* TODO: Verify if it is actually required to have this function in RAM */
int flexspi_nor_flash_init(struct device *dev)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;

	FLEXSPI_UpdateLUT(dev_cfg->base, 0, w25q128LUT, W25Q128_LUT_LENGTH);

	FLEXSPI_SoftwareReset(dev_cfg->base);

#if CONFIG_FLASH_LOG_LEVEL >= 4
	volatile u32_t *lut = drv_data->base->LUT;

	for (size_t i = 0; i < 16; i++) {
		LOG_DBG("%s LUT %02d: 0x%08x 0x%08x 0x%08x 0x%08x",
			dev->config->name, i, lut[0], lut[1], lut[2], lut[3]);
		lut += 4;
	}

	u8_t reg[3];
	status_t status;

	status = flexspi_nor_flash_get_reg(dev_cfg->base, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READJEDECID, reg, 3);
	if (status != kStatus_Success) {
		LOG_WRN("Reading JEDEC ID failed (0x%x)", status);
	} else {
		LOG_DBG("JEDEC ID %02x %02x %02x",
			reg[0], reg[1], reg[2]);
	}

	status = flexspi_nor_flash_get_reg(dev_cfg->base, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG1, reg, 1);
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 1 failed (0x%x)", status);
	} else {
		LOG_DBG("Status register 1 0x%02x", reg[0]);
	}

	status = flexspi_nor_flash_get_reg(dev_cfg->base, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2, reg, 1);
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 2 failed (0x%x)", status);
	} else {
		LOG_DBG("Status register 2 0x%02x", reg[0]);
	}

	status = flexspi_nor_flash_get_reg(dev_cfg->base, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3, reg, 1);
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 3 failed (0x%x)", status);
	} else {
		LOG_DBG("Status register 3 0x%02x", reg[0]);
	}
#endif

	return 0;
}
