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
// #error CONFIG_CODE_DATA_RELOCATION must be enable to use FLEXSPI_NOR_FLASH.
#warning CONFIG_CODE_DATA_RELOCATION must be enable to use FLEXSPI_NOR_FLASH.
#endif

/*******************************************************************************
 *  F L A S H   A P I
*******************************************************************************/



/*******************************************************************************
 *  I N I T
*******************************************************************************/

/*
 * FIXME - Most of the settings must come from DTS
 */
flexspi_device_config_t deviceconfig = {
    .flexspiRootClk       = 133000000,
    .flashSize            = DT_FLASH_SIZE,
    .CSIntervalUnit       = kFLEXSPI_CsIntervalUnit1SckCycle,
    .CSInterval           = 2,
    .CSHoldTime           = 3,
    .CSSetupTime          = 3,
    .dataValidTime        = 0,
    .columnspace          = 0,
    .enableWordAddress    = 0,
    .AWRSeqIndex          = 0,
    .AWRSeqNumber         = 0,
    .ARDSeqIndex          = NOR_CMD_LUT_SEQ_IDX_READ_FAST_QUAD,
    .ARDSeqNumber         = 1,
    .AHBWriteWaitUnit     = kFLEXSPI_AhbWriteWaitUnit2AhbCycle,
    .AHBWriteWaitInterval = 0,
};

static inline void flexspi_clock_init(void)
{
// #if defined(XIP_EXTERNAL_FLASH) && (XIP_EXTERNAL_FLASH == 1)
//     CLOCK_SetMux(kCLOCK_FlexspiMux, 0x2); /* Choose PLL2 PFD2 clock as flexspi source clock. 396M */
//     CLOCK_SetDiv(kCLOCK_FlexspiDiv, 2);   /* flexspi clock 133M. */
// #else
//     const clock_usb_pll_config_t g_ccmConfigUsbPll = {.loopDivider = 0U};
//     CLOCK_InitUsb1Pll(&g_ccmConfigUsbPll);
//     CLOCK_InitUsb1Pfd(kCLOCK_Pfd0, 24);   /* Set PLL3 PFD0 clock 360MHZ. */
//     CLOCK_SetMux(kCLOCK_FlexspiMux, 0x3); /* Choose PLL3 PFD0 clock as flexspi source clock. */
//     CLOCK_SetDiv(kCLOCK_FlexspiDiv, 2);   /* flexspi clock 120M. */
// #endif
}

int flexspi_nor_flash_init(struct device *dev)
{
	struct flexspi_flash_data *drv_data =  dev->driver_data;
	flexspi_config_t config;

	// flexspi_clock_init();

	/* Get FLEXSPI default settings and configure the flexspi. */
	FLEXSPI_GetDefaultConfig(&config);

	/* Set AHB buffer size for reading data through AHB bus. */
	config.ahbConfig.enableAHBPrefetch    = true;
	config.ahbConfig.enableAHBBufferable  = true;
	config.ahbConfig.enableReadAddressOpt = true;
	config.ahbConfig.enableAHBCachable    = true;
	config.rxSampleClock                  =
		kFLEXSPI_ReadSampleClkLoopbackFromDqsPad;
	// FLEXSPI_Init(drv_data->base, &config);

	/* Configure flash settings according to serial flash feature. */
	// FLEXSPI_SetFlashConfig(drv_data->base, &deviceconfig, drv_data->port);

	/* Update LUT table. */
	FLEXSPI_UpdateLUT(drv_data->base, 0, customLUT, CUSTOM_LUT_LENGTH);

	/* Do software reset. */
	FLEXSPI_SoftwareReset(drv_data->base);

	return 0;
}
