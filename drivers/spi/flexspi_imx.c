/*
 * Copyright (c) 2020, Riedo Networks Ltd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "flexspi_imx.h"

int flexspi_imx_init(struct device *dev)
{
	return 0;
}

static const struct flexspi_imx_api empty_api;

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

static const struct flexspi_imx_config flexspi0_config = {
	.base = (FLEXSPI_Type *)FLEXSPI_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(flexspi0_controller,
		    DT_INST_0_NXP_IMX_FLEXSPI_LABEL,
		    &flexspi_imx_init,
		    NULL,
		    &flexspi0_config,
		    POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY,
		    &empty_api);

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

static const struct flexspi_imx_config flexspi1_config = {
	.base = (FLEXSPI_Type *)FLEXSPI2_BASE_ADDRESS,
};

DEVICE_AND_API_INIT(flexspi1_controller,
		    DT_INST_1_NXP_IMX_FLEXSPI_LABEL,
		    &flexspi_imx_init,
		    NULL,
		    &flexspi1_config,
		    POST_KERNEL,
		    CONFIG_SPI_INIT_PRIORITY,
		    &empty_api);

#endif /* DT_INST_1_NXP_IMX_FLEXSPI */
