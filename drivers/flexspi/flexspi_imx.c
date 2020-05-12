/*
 * Copyright (c) 2020, Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/flexspi.h>

struct flexspi_imx_data {
	/** Base address of the FlexSPI controller. */
	FLEXSPI_Type * const base;
	/** Base address in the CPU memory map. */
	u32_t * const mem_addr;
};

/*******************************************************************************
 *  A P I
 ******************************************************************************/

/**
 * Wrapper for FLEXSPI_UpdateLUT.
 */
void flexspi_imx_update_lut(struct device *dev, unsigned int index,
			    const u32_t *cmd, unsigned int count)
{
	const struct flexspi_imx_data *dev_data = dev->driver_data;

	FLEXSPI_UpdateLUT(dev_data->base, index, cmd, count);
}

/**
 * Wrapper for FLEXSPI_SoftwareReset.
 */
void flexspi_imx_sw_reset(struct device *dev)
{
	const struct flexspi_imx_data *dev_data = dev->driver_data;

	FLEXSPI_SoftwareReset(dev_data->base);
}

/**
 * Wrapper for FLEXSPI_TransferBlocking.
 */
status_t flexspi_imx_xfer_blocking(struct device *dev,
				   flexspi_transfer_t *xfer)
{
	const struct flexspi_imx_data *dev_data = dev->driver_data;

	return FLEXSPI_TransferBlocking(dev_data->base, xfer);
}

/**
 * Enable or disable AHB read prefetch.
 */
void flexspi_imx_ahb_prefetch(struct device *dev, bool enable)
{
	const struct flexspi_imx_data *dev_data = dev->driver_data;

	if (enable) {
		dev_data->base->AHBCR |= FLEXSPI_AHBCR_PREFETCHEN_MASK;
	} else {
		dev_data->base->AHBCR &= ~FLEXSPI_AHBCR_PREFETCHEN_MASK;
	}
}

/**
 * Wrapper to SCB_InvalidateDCache_by_Addr.
 */
void flexspi_imx_invalidate_dcache(struct device *dev,
				   off_t offset,
				   s32_t size)
{
	const struct flexspi_imx_data *dev_data = dev->driver_data;

	/* Using ARM specific function since Zephyr doesn't have
	   a generic cache management API. */
	SCB_InvalidateDCache_by_Addr(dev_cfg->mem_addr + offset, size);
}

/*******************************************************************************
 *  I N I T
 ******************************************************************************/

int flexspi_imx_init(struct device *dev)
{
	return 0;
}

static const struct flexspi_driver_api flexspi_imx_api = {
	.update_lut = flexspi_imx_update_lut,
	.sw_reset = flexspi_imx_sw_reset,
	.xfer_blocking = flexspi_imx_xfer_blocking,
	.ahb_prefetch = flexspi_imx_ahb_prefetch,
	.invalidate_dcache = flexspi_imx_invalidate_dcache,
};

/*******************************************************************************
 * FlexSPI first controller
 ******************************************************************************/

#if defined(DT_INST_0_NXP_IMX_FLEXSPI)

#if defined(DT_INST_0_NXP_IMX_FLEXSPI_BASE_ADDRESS)
#define FLEXSPI_BASE_ADDRESS DT_INST_0_NXP_IMX_FLEXSPI_BASE_ADDRESS
#elif defined(DT_INST_0_NXP_IMX_FLEXSPI_BASE_ADDRESS_0)
#define FLEXSPI_BASE_ADDRESS DT_INST_0_NXP_IMX_FLEXSPI_BASE_ADDRESS_0
#else
#error FlexSPI controller base address not defined
#endif

static struct flexspi_imx_data flexspi0_data = {
	.base = (FLEXSPI_Type *)FLEXSPI_BASE_ADDRESS,
	.mem_addr = (u32_t *)FlexSPI_AMBA_BASE,
};

DEVICE_AND_API_INIT(flexspi0_controller,
		    DT_INST_0_NXP_IMX_FLEXSPI_LABEL,
		    &flexspi_imx_init,
		    &flexspi0_data,
		    NULL,
		    POST_KERNEL,
		    CONFIG_FLEXSPI_INIT_PRIORITY,
		    &flexspi_imx_api);

#endif /* DT_INST_0_NXP_IMX_FLEXSPI */

/*******************************************************************************
 * FlexSPI second controller
 ******************************************************************************/

#if defined(DT_INST_1_NXP_IMX_FLEXSPI)

#if defined(DT_INST_1_NXP_IMX_FLEXSPI_BASE_ADDRESS)
#define FLEXSPI2_BASE_ADDRESS DT_INST_1_NXP_IMX_FLEXSPI_BASE_ADDRESS
#elif defined(DT_INST_1_NXP_IMX_FLEXSPI_BASE_ADDRESS_0)
#define FLEXSPI2_BASE_ADDRESS DT_INST_1_NXP_IMX_FLEXSPI_BASE_ADDRESS_0
#else
#error FlexSPI2 controller base address not defined
#endif

static struct flexspi_imx_data flexspi1_data = {
	.base = (FLEXSPI_Type *)FLEXSPI2_BASE_ADDRESS,
	.mem_addr = (u32_t *)FlexSPI2_AMBA_BASE,
};

DEVICE_AND_API_INIT(flexspi1_controller,
		    DT_INST_1_NXP_IMX_FLEXSPI_LABEL,
		    &flexspi_imx_init,
		    &flexspi1_data,
		    NULL,
		    POST_KERNEL,
		    CONFIG_FLEXSPI_INIT_PRIORITY,
		    &flexspi_imx_api);

#endif /* DT_INST_1_NXP_IMX_FLEXSPI */
