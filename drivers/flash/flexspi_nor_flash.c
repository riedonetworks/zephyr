/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
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

#if !defined(CONFIG_CODE_DATA_RELOCATION) && !defined(CONFIG_BOOTLOADER_MCUBOOT)
#error CONFIG_CODE_DATA_RELOCATION must be enable to use FLEXSPI_NOR_FLASH.
#endif

#if defined(CONFIG_MULTITHREADING)
#warning Multithreading not supported yet (proper locking is missing), use at your own risk!
#endif

/*******************************************************************************
 *  H E L P E R S
 ******************************************************************************/

/**
 * Mark the beginning of a critical section.
 * @warning Calling this function disable all interrupts.
 *          No operations allowing another thread to run (e.g. sleeping)
 *          must be called inside the critical section,
 *          see https://docs.zephyrproject.org/latest/reference/kernel/other/interrupts.html#preventing-interruptions
 *
 * @param flexspi FlexSPI device driver handle.
 * @return irq_lock key.
 */
static ALWAYS_INLINE unsigned int critical_section_enter(struct device *flexspi)
{
	/* TODO If flash is not used for executing code (XIP)
	        this function must be a no-op. */

	unsigned int key;

	key = irq_lock();
	flexspi_ahb_prefetch(flexspi, false);

	return key;
}

/**
 * Mark the end of a critical section.
 * Calling this function re-enable all interrupts.
 *
 * @param flexspi FlexSPI device driver handle.
 * @param irq_lock key.
 */
static ALWAYS_INLINE void critical_section_leave(struct device *flexspi,
						 unsigned int key)
{
	/* TODO If flash is not used for executing code (XIP)
	        this function must be a no-op. */

	flexspi_ahb_prefetch(flexspi, true);
	irq_unlock(key);
}

#define REG_STATUS_BIT_BUSY 0x01U

status_t flexspi_nor_flash_wait_bus_busy(struct device *flexspi,
					 off_t dev_addr,
					 flexspi_port_t port)
{
	flexspi_transfer_t flashXfer;
	status_t status;
	u32_t reg;
	bool isBusy;

	flashXfer.deviceAddress = dev_addr;
	flashXfer.port          = port;
	flashXfer.cmdType       = kFLEXSPI_Read;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_READSTATUSREG;
	flashXfer.data          = &reg;
	flashXfer.dataSize      = 1;

	do {
		status = flexspi_xfer_blocking(flexspi, &flashXfer);
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

static status_t flexspi_nor_flash_get_reg(struct device *flexspi,
					  off_t dev_addr,
					  flexspi_port_t port,
					  u8_t cmd_idx,
					  void *reg,
					  size_t len)
{
	u32_t buffer[(len + (sizeof(u32_t) - 1)) / sizeof(u32_t)];

	flexspi_transfer_t flashXfer = {
		.deviceAddress = dev_addr,
		.port          = port,
		.cmdType       = kFLEXSPI_Read,
		.SeqNumber     = 1,
		.seqIndex      = cmd_idx,
		.data          = buffer,
		.dataSize      = len,
	};

	status_t status = flexspi_xfer_blocking(flexspi, &flashXfer);

	u8_t *dst = reg;
	u8_t *src = (u8_t *)buffer;
	for (int i = 0; i < len; ++i) {
		*dst++ = *src++;
	}

	return status;
}

static status_t flexspi_nor_flash_set_reg(struct device *flexspi,
					  off_t dev_addr,
					  flexspi_port_t port,
					  u8_t cmd_idx,
					  void *reg,
					  size_t len)
{
	u32_t buffer[(len + (sizeof(u32_t) - 1)) / sizeof(u32_t)];

	u8_t *dst = (u8_t *)buffer;
	u8_t *src = reg;
	for (int i = 0; i < len; ++i) {
		*dst++ = *src++;
	}

	flexspi_transfer_t flashXfer = {
		.deviceAddress = dev_addr,
		.port          = port,
		.cmdType       = kFLEXSPI_Write,
		.SeqNumber     = 1,
		.seqIndex      = cmd_idx,
		.data          = buffer,
		.dataSize      = len,
	};

	return flexspi_xfer_blocking(flexspi, &flashXfer);
}

#define REG_STATUS2_BIT_QE 0x02U

/**
 * Set quad enable if not yet set
 *
 * @todo:
 * - Only set QE bit if spi-access-mode = "quad" or "qpi" in devicetree
 * - Use a config function which is specific to the flash model
 *
 * @return 0 on success or a negative error code on failure.
 */
static int flexspi_nor_flash_set_quad_enable(struct device *dev)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	status_t status;
	u8_t stat_reg_2;

	status = flexspi_nor_flash_get_reg(
		dev_data->flexspi, dev_data->mem_offset, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2,
		&stat_reg_2, sizeof(stat_reg_2)
	);
	if (status != kStatus_Success) {
		LOG_ERR("Reading status register 2 failed (%d)", status);
		return -EIO;
	}

	LOG_DBG("%s: status register 2 0x%02x\n",
		dev->config->name, stat_reg_2);

	if ((stat_reg_2 & REG_STATUS2_BIT_QE) == 0) {
		if (flexspi_nor_flash_write_protection_set(dev, false)) {
			return -EIO;
		}

		LOG_INF("%s: setting QE bit in status register 2\n",
			dev->config->name);

		stat_reg_2 |= REG_STATUS2_BIT_QE;
		status = flexspi_nor_flash_set_reg(
			dev_data->flexspi, dev_data->mem_offset, dev_cfg->port,
			NOR_CMD_LUT_SEQ_IDX_WRITESTATUSREG2,
			&stat_reg_2, sizeof(stat_reg_2)
		);
		if (status != kStatus_Success) {
			LOG_ERR("Writing status register 2 failed (%d)",
				status);
			return -EIO;
		}

		status = flexspi_nor_flash_wait_bus_busy(dev_data->flexspi,
							 dev_data->mem_offset,
							 dev_cfg->port);
		if (status != kStatus_Success) {
			LOG_ERR("Waiting while bus is busy failed (%d)",
				status);
			return -EIO;
		}
	}

	return 0;
}

static int flexspi_nor_flash_check_jedec_id(struct device *dev)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	status_t status;
	u8_t id[JEDEC_ID_LEN];

	status = flexspi_nor_flash_get_reg(
		dev_data->flexspi, dev_data->mem_offset, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READJEDECID, id, ARRAY_SIZE(id)
	);
	if (status != kStatus_Success) {
		LOG_ERR("Reading JEDEC ID failed (%d)", status);
		return -EIO;
	}

	if (memcmp(id, dev_cfg->jedec_id, JEDEC_ID_LEN) != 0) {
		LOG_ERR("Wrong JEDEC ID: expected %02x %02x %02x but got %02x %02x %02x",
			dev_cfg->jedec_id[0], dev_cfg->jedec_id[1],
			dev_cfg->jedec_id[2], id[0], id[1], id[2]);
		return -ENODEV;
	}

	LOG_DBG("%s JEDEC ID: %02x %02x %02x",
		dev->config->name, id[0], id[1], id[2]);

	return 0;
}

/*******************************************************************************
 *  F L A S H   A P I
 ******************************************************************************/

int flexspi_nor_flash_read(struct device *dev, off_t offset,
			   void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;

	/* Prevent reading outside of the flash */
	if ((offset < 0) || ((offset + len) > dev_cfg->size)) {
		return -ENODEV;
	}

	offset += dev_data->mem_offset;

	flexspi_mem_read(dev_data->flexspi, offset, data, len);

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

	flashXfer.deviceAddress = dev_data->mem_offset;
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

static int flexspi_nor_flash_sector_erase(struct device *dev, off_t offset)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;
	int retval = 0;

	__ASSERT((offset & (dev_cfg->pages_layout.pages_size - 1U)) == 0,
		 "Offset not on sector boundary");

	unsigned int key = critical_section_enter(dev_data->flexspi);

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Command;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_ERASESECTOR;

	status = flexspi_xfer_blocking(dev_data->flexspi, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(dev_data->flexspi,
						 dev_data->mem_offset,
						 dev_cfg->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

done:
	flexspi_sw_reset(dev_data->flexspi);
	flexspi_invalidate_dcache(dev_data->flexspi,
				  offset,
				  dev_cfg->pages_layout.pages_size);
	critical_section_leave(dev_data->flexspi, key);
	return retval;
}

int flexspi_nor_flash_erase(struct device *dev, off_t offset, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	const size_t SECTOR_SIZE = dev_cfg->pages_layout.pages_size;
	const size_t SECTOR_MASK = SECTOR_SIZE - 1U;

	/* Prevent erasing outside of the flash */
	if ((offset < 0) || ((offset + len) > dev_cfg->size)) {
		return -ENODEV;
	}

	/* Can only erase full sector(s) */
	if ((offset & SECTOR_MASK) || (len & SECTOR_MASK)) {
		return -EINVAL;
	}

	offset += dev_data->mem_offset;

	/* TODO: Improve performance by using block erase 32 kiB ot 64 kiB
	         when len is big enough. */

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

static int flexspi_nor_flash_page_program(struct device *dev, off_t offset,
					  const void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	flexspi_transfer_t flashXfer;
	status_t status;
	int retval = 0;

	__ASSERT(len <= dev_cfg->page_size, "Length is above page size");

	memcpy(dev_data->bounce_buffer, data, len);

	unsigned int key = critical_section_enter(dev_data->flexspi);

	flashXfer.deviceAddress = offset;
	flashXfer.port          = dev_cfg->port;
	flashXfer.cmdType       = kFLEXSPI_Write;
	flashXfer.SeqNumber     = 1;
	flashXfer.seqIndex      = NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM;
	flashXfer.data          = dev_data->bounce_buffer;
	flashXfer.dataSize      = len;

	status = flexspi_xfer_blocking(dev_data->flexspi, &flashXfer);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

	status = flexspi_nor_flash_wait_bus_busy(dev_data->flexspi,
						 dev_data->mem_offset,
						 dev_cfg->port);
	if (status != kStatus_Success) {
		retval = -EIO;
		goto done;
	}

done:
	flexspi_sw_reset(dev_data->flexspi);
	flexspi_invalidate_dcache(dev_data->flexspi, offset, len);
	critical_section_leave(dev_data->flexspi, key);
	return retval;
}

int flexspi_nor_flash_write(struct device *dev, off_t offset,
			    const void *data, size_t len)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	const size_t PAGE_SIZE = dev_cfg->page_size;
	const size_t PAGE_MASK = PAGE_SIZE - 1U;

	/* Prevent writing outside of the flash */
	if ((offset < 0) || ((offset + len) > dev_cfg->size)) {
		return -ENODEV;
	}

	offset += dev_data->mem_offset;

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
 ******************************************************************************/

int flexspi_nor_flash_init(struct device *dev)
{
	const struct flexspi_nor_flash_dev_config *dev_cfg =
		dev->config->config_info;
	struct flexspi_nor_flash_dev_data *dev_data = dev->driver_data;
	int err;

	/*
	 * FlexSPI controller binding
	 */
	dev_data->flexspi = device_get_binding(dev_cfg->bus_name);
	if (!dev_data->flexspi) {
		LOG_ERR("Failed to get FlexSPI bus for %s", dev->config->name);
		return -ENODEV;
	}

	LOG_DBG("%s bound to FlexSPI controller %s",
		dev->config->name, dev_data->flexspi->config->name);

	/*
	 * Get memory offset
	 */
	dev_data->mem_offset = flexspi_get_mem_offset(dev_data->flexspi,
						      dev_cfg->port);
	if (dev_data->mem_offset == -1) {
		LOG_ERR("Failed to get mem offset for %s", dev->config->name);
		return -ENODEV;
	}

	/*
	 * Configure LUT
	 */
	unsigned int key = critical_section_enter(dev_data->flexspi);

	err = flexspi_update_lut(dev_data->flexspi,
				 0,
				 dev_cfg->lut,
				 dev_cfg->lut_length);
	if (err) {
		LOG_ERR("Failed to configure LUT for %s", dev->config->name);
		return err;
	}

	/* TODO Check why software reset is after FLEXSPI_UpdateLUT in SDK
	        while in AN12564 examples it is before. */
	flexspi_sw_reset(dev_data->flexspi);

	critical_section_leave(dev_data->flexspi, key);

	/*
	 * Other config
	 */
	err = flexspi_nor_flash_set_quad_enable(dev);
	if (err) {
		LOG_ERR("Failed to enable quad I/O for %s", dev->config->name);
		return err;
	}

	err = flexspi_nor_flash_check_jedec_id(dev);
	if (err) {
		return err;
	}

#if CONFIG_FLASH_LOG_LEVEL >= 4
	u8_t reg;
	status_t status;

	status = flexspi_nor_flash_get_reg(dev_data->flexspi,
		dev_data->mem_offset, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG, &reg, size(reg));
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 1 failed (%d)", status);
	} else {
		LOG_DBG("Status register 1 0x%02x", reg);
	}

	status = flexspi_nor_flash_get_reg(dev_data->flexspi,
		dev_data->mem_offset, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG2, &reg, size(reg));
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 2 failed (%d)", status);
	} else {
		LOG_DBG("Status register 2 0x%02x", reg);
	}

	status = flexspi_nor_flash_get_reg(dev_data->flexspi,
		dev_data->mem_offset, dev_cfg->port,
		NOR_CMD_LUT_SEQ_IDX_READSTATUSREG3, &reg, size(reg));
	if (status != kStatus_Success) {
		LOG_WRN("Reading status register 3 failed (%d)", status);
	} else {
		LOG_DBG("Status register 3 0x%02x", reg);
	}
#endif

	return 0;
}
