/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <drivers/uart.h>
#include <drivers/clock_control.h>
#include <fsl_flexio_uart.h>
#include <soc.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(flexio_uart);

struct mcux_flexio_uart_config {
	FLEXIO_UART_Type *base;
	char *clock_controller;
	clock_control_subsys_t clock_subsys;
	u32_t baud_rate;
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	void (*irq_config_func)(struct device *dev);
#endif
};

struct mcux_flexio_uart_data {
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
};

static int mcux_flexio_uart_poll_in(struct device *dev, unsigned char *c)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = FLEXIO_UART_GetStatusFlags(config->base);
	int ret = -1;

	if (flags & kFLEXIO_UART_RxDataRegFullFlag) {
		FLEXIO_UART_ReadByte(config->base, c);
		ret = 0;
	}

	return ret;
}

static void mcux_flexio_uart_poll_out(struct device *dev, unsigned char c)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;

	while (!(FLEXIO_UART_GetStatusFlags(config->base)
		& kFLEXIO_UART_TxDataRegEmptyFlag)) {
	}

	LOG_DBG("Writing 0x%02X to %p", c, config->base->flexioBase);

	FLEXIO_UART_WriteByte(config->base, &c);
}

static int mcux_flexio_uart_err_check(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = FLEXIO_UART_GetStatusFlags(config->base);
	int err = 0;

	if (flags & kFLEXIO_UART_RxOverRunFlag) {
		err |= UART_ERROR_OVERRUN;
	}

	FLEXIO_UART_ClearStatusFlags(config->base, kFLEXIO_UART_RxOverRunFlag);

	return err;
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int mcux_flexio_uart_fifo_fill(struct device *dev, const u8_t *tx_data,
			       int len)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u8_t num_tx = 0U;

	while ((len - num_tx > 0) &&
	       (LPUART_GetStatusFlags(config->base)
		& kLPUART_TxDataRegEmptyFlag)) {

		LPUART_WriteByte(config->base, tx_data[num_tx++]);
	}

	return num_tx;
}

static int mcux_flexio_uart_fifo_read(struct device *dev, u8_t *rx_data,
			       const int len)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u8_t num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (LPUART_GetStatusFlags(config->base)
		& kLPUART_RxDataRegFullFlag)) {

		rx_data[num_rx++] = LPUART_ReadByte(config->base);
	}

	return num_rx;
}

static void mcux_flexio_uart_irq_tx_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_TxDataRegEmptyInterruptEnable;

	LPUART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_tx_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_TxDataRegEmptyInterruptEnable;

	LPUART_DisableInterrupts(config->base, mask);
}

static int mcux_flexio_uart_irq_tx_complete(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = LPUART_GetStatusFlags(config->base);

	return (flags & kLPUART_TxDataRegEmptyFlag) != 0U;
}

static int mcux_flexio_uart_irq_tx_ready(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_TxDataRegEmptyInterruptEnable;

	return (LPUART_GetEnabledInterrupts(config->base) & mask)
		&& mcux_flexio_uart_irq_tx_complete(dev);
}

static void mcux_flexio_uart_irq_rx_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_RxDataRegFullInterruptEnable;

	LPUART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_rx_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_RxDataRegFullInterruptEnable;

	LPUART_DisableInterrupts(config->base, mask);
}

static int mcux_flexio_uart_irq_rx_full(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = LPUART_GetStatusFlags(config->base);

	return (flags & kLPUART_RxDataRegFullFlag) != 0U;
}

static int mcux_flexio_uart_irq_rx_ready(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_RxDataRegFullInterruptEnable;

	return (LPUART_GetEnabledInterrupts(config->base) & mask)
		&& mcux_flexio_uart_irq_rx_full(dev);
}

static void mcux_flexio_uart_irq_err_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_NoiseErrorInterruptEnable |
			kLPUART_FramingErrorInterruptEnable |
			kLPUART_ParityErrorInterruptEnable;

	LPUART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_err_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kLPUART_NoiseErrorInterruptEnable |
			kLPUART_FramingErrorInterruptEnable |
			kLPUART_ParityErrorInterruptEnable;

	LPUART_DisableInterrupts(config->base, mask);
}

static int mcux_flexio_uart_irq_is_pending(struct device *dev)
{
	return (mcux_flexio_uart_irq_tx_ready(dev)
		|| mcux_flexio_uart_irq_rx_ready(dev));
}

static int mcux_flexio_uart_irq_update(struct device *dev)
{
	return 1;
}

static void mcux_flexio_uart_irq_callback_set(struct device *dev,
				       uart_irq_callback_user_data_t cb,
				       void *cb_data)
{
	struct mcux_flexio_uart_data *data = dev->driver_data;

	data->callback = cb;
	data->cb_data = cb_data;
}

static void mcux_flexio_uart_isr(void *arg)
{
	struct device *dev = arg;
	struct mcux_flexio_uart_data *data = dev->driver_data;

	if (data->callback) {
		data->callback(data->cb_data);
	}
}
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

static int mcux_flexio_uart_init(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	flexio_uart_config_t uart_config;
	struct device *clock_dev;
	u32_t clock_freq;

	clock_dev = device_get_binding(config->clock_controller);
	if (clock_dev == NULL) {
		return -EINVAL;
	}

	if (clock_control_get_rate(clock_dev, config->clock_subsys,
				   &clock_freq)) {
		return -EINVAL;
	}
	LOG_INF("Clock freq: %d", clock_freq);
	LOG_DBG("target baudrade: %d", config->baud_rate);
	LOG_DBG("TX pin: %d", config->base->TxPinIndex);
	LOG_DBG("RX pin: %d", config->base->RxPinIndex);
	LOG_DBG("FlexIO: %p", config->base->flexioBase);

	if(clock_freq == 0)
	{
		return -EINVAL;
	}

    FLEXIO_UART_GetDefaultConfig(&uart_config);
    uart_config.baudRate_Bps = config->baud_rate;
    uart_config.enableUart   = true;

    if( FLEXIO_UART_Init(config->base, &uart_config, clock_freq) != kStatus_Success)
	{
		printk("Failed to set baudrate!");
		return -EINVAL;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	config->irq_config_func(dev);
#endif

	return 0;
}

static const struct uart_driver_api mcux_flexio_uart_driver_api = {
	.poll_in = mcux_flexio_uart_poll_in,
	.poll_out = mcux_flexio_uart_poll_out,
	.err_check = mcux_flexio_uart_err_check,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = mcux_flexio_uart_fifo_fill,
	.fifo_read = mcux_flexio_uart_fifo_read,
	.irq_tx_enable = mcux_flexio_uart_irq_tx_enable,
	.irq_tx_disable = mcux_flexio_uart_irq_tx_disable,
	.irq_tx_complete = mcux_flexio_uart_irq_tx_complete,
	.irq_tx_ready = mcux_flexio_uart_irq_tx_ready,
	.irq_rx_enable = mcux_flexio_uart_irq_rx_enable,
	.irq_rx_disable = mcux_flexio_uart_irq_rx_disable,
	.irq_rx_ready = mcux_flexio_uart_irq_rx_ready,
	.irq_err_enable = mcux_flexio_uart_irq_err_enable,
	.irq_err_disable = mcux_flexio_uart_irq_err_disable,
	.irq_is_pending = mcux_flexio_uart_irq_is_pending,
	.irq_update = mcux_flexio_uart_irq_update,
	.irq_callback_set = mcux_flexio_uart_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_MCUX_FLEXIO_UART_1

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void mcux_flexio_uart_config_func_0(struct device *dev);
#endif

static FLEXIO_UART_Type uart_0_flexio_config = {
	
    .flexioBase      = FLEXIO1,
    .TxPinIndex      = DT_INST_0_NXP_IMXRT_FLEXIO_UART_TXD_SIGNAL, 
    .RxPinIndex      = DT_INST_0_NXP_IMXRT_FLEXIO_UART_RXD_SIGNAL,
    .shifterIndex[0] = 0U,
    .shifterIndex[1] = 1U,
    .timerIndex[0]   = 0U,
    .timerIndex[1]   = 1U
};

static const struct mcux_flexio_uart_config mcux_flexio_uart_0_config = {
	.base = &uart_0_flexio_config,
	.clock_controller = DT_INST_0_NXP_IMXRT_FLEXIO_UART_CLOCK_CONTROLLER,
	.clock_subsys =
		(clock_control_subsys_t)DT_INST_0_NXP_IMXRT_FLEXIO_UART_CLOCK_NAME,
	.baud_rate = DT_INST_0_NXP_IMXRT_FLEXIO_UART_CURRENT_SPEED,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = mcux_flexio_uart_config_func_0,
#endif
};

static struct mcux_flexio_uart_data mcux_flexio_uart_0_data;

DEVICE_AND_API_INIT(flexio_uart_0, DT_INST_0_NXP_IMXRT_FLEXIO_UART_LABEL,
		    &mcux_flexio_uart_init,
		    &mcux_flexio_uart_0_data, &mcux_flexio_uart_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_flexio_uart_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void mcux_flexio_uart_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_UART_MCUX_LPUART_0_IRQ_0,
		    DT_UART_MCUX_LPUART_0_IRQ_0_PRI,
		    mcux_flexio_uart_isr, DEVICE_GET(uart_0), 0);

	irq_enable(DT_UART_MCUX_LPUART_0_IRQ_0);

#ifdef DT_UART_MCUX_LPUART_0_IRQ_1
	IRQ_CONNECT(DT_UART_MCUX_LPUART_0_IRQ_1,
		    DT_UART_MCUX_LPUART_0_IRQ_1_PRI,
		    mcux_flexio_uart_isr, DEVICE_GET(uart_0), 0);

	irq_enable(DT_UART_MCUX_LPUART_0_IRQ_1);
#endif /* DT_UART_MCUX_LPUART_0_IRQ_1 */
}
#endif

#endif /* CONFIG_UART_MCUX_FLEXIO_UART_1 */

#ifdef CONFIG_UART_MCUX_FLEXIO_UART_2

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void mcux_flexio_uart_config_func_0(struct device *dev);
#endif

static const FLEXIO_UART_Type uart_1_flexio_config = {
	
    .flexioBase      = FLEXIO2,
    .TxPinIndex      = DT_INST_1_NXP_IMXRT_FLEXIO_UART_TXD_SIGNAL, 
    .RxPinIndex      = DT_INST_1_NXP_IMXRT_FLEXIO_UART_RXD_SIGNAL,
    .shifterIndex[0] = 0U,
    .shifterIndex[1] = 1U,
    .timerIndex[0]   = 0U,
    .timerIndex[1]   = 1U
};

static const struct mcux_flexio_uart_config mcux_flexio_uart_1_config = {
	.base = &uart_1_flexio_config,
	.clock_controller = DT_INST_1_NXP_IMXRT_FLEXIO_UART_CLOCK_CONTROLLER,
	.clock_subsys =
		(clock_control_subsys_t)DT_INST_1_NXP_IMXRT_FLEXIO_UART_CLOCK_NAME,
	.baud_rate = DT_INST_1_NXP_IMXRT_FLEXIO_UART_CURRENT_SPEED,
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.irq_config_func = mcux_flexio_uart_config_func_0,
#endif
};

static struct mcux_flexio_uart_data mcux_flexio_uart_1_data;

DEVICE_AND_API_INIT(flexio_uart_1, DT_INST_1_NXP_IMXRT_FLEXIO_UART_LABEL,
		    &mcux_flexio_uart_init,
		    &mcux_flexio_uart_1_data, &mcux_flexio_uart_1_config,
		    PRE_KERNEL_2, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_flexio_uart_driver_api);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static void mcux_flexio_uart_config_func_0(struct device *dev)
{
	IRQ_CONNECT(DT_UART_MCUX_LPUART_0_IRQ_0,
		    DT_UART_MCUX_LPUART_0_IRQ_0_PRI,
		    mcux_flexio_uart_isr, DEVICE_GET(uart_0), 0);

	irq_enable(DT_UART_MCUX_LPUART_0_IRQ_0);

#ifdef DT_UART_MCUX_LPUART_0_IRQ_1
	IRQ_CONNECT(DT_UART_MCUX_LPUART_0_IRQ_1,
		    DT_UART_MCUX_LPUART_0_IRQ_1_PRI,
		    mcux_flexio_uart_isr, DEVICE_GET(uart_0), 0);

	irq_enable(DT_UART_MCUX_LPUART_0_IRQ_1);
#endif /* DT_UART_MCUX_LPUART_0_IRQ_1 */
}
#endif

#endif /* CONFIG_UART_MCUX_FLEXIO_UART_2 */
