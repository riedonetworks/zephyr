/*
 * Copyright (c) 2017 Linaro Limited.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <sys/slist.h>
#include <arch/arm/aarch32/cortex_m/mpu/arm_mpu.h>

#include "arm_mpu_mem_cfg.h"

static const struct arm_mpu_region mpu_regions[] = {
	/* Region 0 */
	MPU_REGION_ENTRY("FLASH_0",
			 CONFIG_FLASH_BASE_ADDRESS,
#if defined(CONFIG_ARMV8_M_BASELINE) || defined(CONFIG_ARMV8_M_MAINLINE)
			 REGION_FLASH_ATTR(CONFIG_FLASH_BASE_ADDRESS, \
				 CONFIG_FLASH_SIZE * 1024)),
#else
			 REGION_FLASH_ATTR(REGION_FLASH_SIZE)),
#endif
	/* Region 1 */
	MPU_REGION_ENTRY("SRAM_0",
			 CONFIG_SRAM_BASE_ADDRESS,
#if defined(CONFIG_ARMV8_M_BASELINE) || defined(CONFIG_ARMV8_M_MAINLINE)
			 REGION_RAM_ATTR(CONFIG_SRAM_BASE_ADDRESS, \
				 CONFIG_SRAM_SIZE * 1024)),
#else
			 REGION_RAM_ATTR(REGION_SRAM_SIZE)),
#endif
#if defined(CONFIG_SOC_MIMXRT1062)
	/* Region 2 - ITCM for NULL pointer detection */
	MPU_REGION_ENTRY("NULL",
			 0x0,
			 REGION_NA_ATTR(REGION_64K)), /* FIXME - Remove hard-coded value */
	/* Region 3 - ITCM for code */
	MPU_REGION_ENTRY("ITCM",
			 DT_INST_0_NXP_IMX_ITCM_BASE_ADDRESS,
			 REGION_ITCM_ATTR(REGION_ITCM_SIZE)),
	/* Region 4 - DTCM */
	MPU_REGION_ENTRY("DTCM",
			 DT_INST_0_NXP_IMX_DTCM_BASE_ADDRESS,
			 REGION_DTCM_ATTR(REGION_DTCM_SIZE)),
#endif /* CONFIG_SOC_MIMXRT1062 */
};

const struct arm_mpu_config mpu_config = {
	.num_regions = ARRAY_SIZE(mpu_regions),
	.mpu_regions = mpu_regions,
};
