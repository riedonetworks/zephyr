/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "flexspi_nor_flash.h"

#if defined(DT_INST_0_FLEXSPI_WINBOND_W25Q128)

#define W25Q128_LUT_LENGTH (8U * 4U)

#if DT_INST_0_FLEXSPI_WINBOND_W25Q128_SPI_ACCESS_MODE_ENUM == SPI_ACCESS_MODE_QUAD
static const u32_t w25q128LUT[W25Q128_LUT_LENGTH] = {
	/* Fast read quad mode - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEB,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_READ + 1] = FLEXSPI_LUT_SEQ(
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
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x32,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x18
	),
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM + 1] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_WRITE_SDR, kFLEXSPI_4PAD, 0x04,
		kFLEXSPI_Command_STOP, 0, 0
	),

	/* Read JEDEC ID */
	[4 * NOR_CMD_LUT_SEQ_IDX_READJEDECID] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x9F,
		kFLEXSPI_Command_READ_SDR, kFLEXSPI_1PAD, 0x04
	),

	/* Read status register 1 */
	[4 * NOR_CMD_LUT_SEQ_IDX_READSTATUSREG] = FLEXSPI_LUT_SEQ(
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
};
#else
#error SPI access mode not supported by FlexSPI winbond W25Q128 driver
#endif

#define NB_SECTOR    (4U * 1024U)
#define SECTOR_SIZE  (4U * 1024U)

static const struct flexspi_nor_flash_dev_config w25q128_config = {
	.bus_name   = DT_INST_0_FLEXSPI_WINBOND_W25Q128_BUS_NAME,
	.port       = DT_INST_0_FLEXSPI_WINBOND_W25Q128_BASE_ADDRESS,
	.size       = NB_SECTOR * SECTOR_SIZE,
	.page_size  = DT_INST_0_FLEXSPI_WINBOND_W25Q128_PAGE_SIZE,
	.lut        = w25q128LUT,
	.lut_length = W25Q128_LUT_LENGTH,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.pages_layout = {
		/* "page" means the smallest erasable area */
		.pages_count = NB_SECTOR,
		.pages_size  = SECTOR_SIZE,
	},
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

static const struct flash_driver_api w25q128_flash_api = {
	.read = flexspi_nor_flash_read,
	.write = flexspi_nor_flash_write,
	.erase = flexspi_nor_flash_erase,
	.write_protection = flexspi_nor_flash_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flexspi_nor_flash_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	.write_block_size = 1,
};

/* Bounce buffer is used for writing at most a full page.
   Round the size up to be sure to have enough space. */
static u32_t w25q128_bounce_buffer[
	(DT_INST_0_FLEXSPI_WINBOND_W25Q128_PAGE_SIZE / sizeof(u32_t)) +
	(DT_INST_0_FLEXSPI_WINBOND_W25Q128_PAGE_SIZE % sizeof(u32_t))
];

static struct flexspi_nor_flash_dev_data w25q128_data = {
	.flexspi       = NULL,
	.bounce_buffer = w25q128_bounce_buffer,
};

/* Instance of the flexspi nor flash driver for winbond W25Q128 chip */
DEVICE_AND_API_INIT(flexspi_nor_flash_w25q128,
		    DT_INST_0_FLEXSPI_WINBOND_W25Q128_LABEL,
		    &flexspi_nor_flash_init,
		    &w25q128_data,
		    &w25q128_config,
		    POST_KERNEL,
		    CONFIG_FLEXSPI_NOR_FLASH_INIT_PRIORITY,
		    &w25q128_flash_api);

#endif /* DT_INST_0_FLEXSPI_WINBOND_W25Q128 */
