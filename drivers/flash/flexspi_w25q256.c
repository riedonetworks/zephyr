/*
 * Copyright (c) 2020 Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

#include "flexspi_nor_flash.h"

#if defined(DT_INST_0_FLEXSPI_WINBOND_W25Q256)

#define W25Q256_LUT_LENGTH (8U * 4U)

#if DT_INST_0_FLEXSPI_WINBOND_W25Q256_SPI_ACCESS_MODE_ENUM == SPI_ACCESS_MODE_QUAD
#warning W25Q256 LUT NOT TESTED
static const u32_t w25q256LUT[W25Q256_LUT_LENGTH] = {
	/* Fast read quad mode 4-byte address - SDR */
	[4 * NOR_CMD_LUT_SEQ_IDX_READ] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0xEC,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_4PAD, 0x20
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

	/* Erase Sector 4-byte address */
	[4 * NOR_CMD_LUT_SEQ_IDX_ERASESECTOR] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x21,
		kFLEXSPI_Command_RADDR_SDR, kFLEXSPI_1PAD, 0x20
	),

	/* Page Program 4-byte address - quad mode */
	[4 * NOR_CMD_LUT_SEQ_IDX_PAGEPROGRAM] = FLEXSPI_LUT_SEQ(
		kFLEXSPI_Command_SDR, kFLEXSPI_1PAD, 0x34,
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
#else
#error SPI access mode not supported by FlexSPI winbond W25Q128 driver
#endif

static const struct flexspi_nor_flash_dev_config w25q256_config = {
	.bus_name   = DT_INST_0_FLEXSPI_WINBOND_W25Q256_BUS_NAME,
	.port      = DT_INST_0_FLEXSPI_WINBOND_W25Q256_BASE_ADDRESS,
	.size      = 32U * 1024U * 1024U,
	.page_size = 256U,
	.lut        = w25q256LUT,
	.lut_length = W25Q256_LUT_LENGTH,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.pages_layout = {
		/* "page" means the smallest erasable area on the flash device */
		.pages_count = 8 * 1024,  /* sectors */
		.pages_size  = 4 * 1024,  /* of 4 kiB */
	},
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
};

static const struct flash_driver_api w25q256_flash_api = {
	.read = flexspi_nor_flash_read,
	.write = flexspi_nor_flash_write,
	.erase = flexspi_nor_flash_erase,
	.write_protection = flexspi_nor_flash_write_protection_set,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flexspi_nor_flash_pages_layout,
#endif /* CONFIG_FLASH_PAGE_LAYOUT */
	.write_block_size = 1,
};

static struct flexspi_nor_flash_dev_data w25q256_data;

/* Instance of the flexspi nor flash driver for winbond W25Q256 chip */
DEVICE_AND_API_INIT(flexspi_nor_flash_w25q256,
		    DT_INST_0_FLEXSPI_WINBOND_W25Q256_LABEL,
		    &flexspi_nor_flash_init,
		    &w25q256_data,
		    &w25q256_config,
		    POST_KERNEL,
		    CONFIG_FLEXSPI_NOR_FLASH_INIT_PRIORITY,
		    &w25q256_flash_api);

#endif /* DT_INST_0_FLEXSPI_WINBOND_W25Q256 */