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


#include <logging/log.h>
#define LOG_LEVEL LOG_LEVEL_DBG
LOG_MODULE_REGISTER(flexio_uart);

struct mcux_flexio_uart_config {
	FLEXIO_UART_Type *base;
	uint8_t tx_clk_pin_index;
	uint8_t rx_clk_pin_index;
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

	//LOG_DBG("Writing 0x%02X to %p", c, config->base->flexioBase);

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
	LOG_DBG("Clock freq: %d", clock_freq);
	LOG_INF("target baudrade: %d", config->baud_rate);
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

	// Monkey patch the Tx timer for TX clk if used
	if( config->tx_clk_pin_index != 0xFF)
	{
		LOG_INF("Using sychronous TX mode!");

	    /*2. Configure the timer 0 for tx. */
		flexio_timer_config_t timerConfig;
		uint16_t timerDiv = 0;
		uint16_t timerCmp = 0;
		timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(config->base->shifterIndex[0]);
		timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
		timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
		timerConfig.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
		timerConfig.pinSelect       = config->tx_clk_pin_index;//config->base->TxPinIndex;
		timerConfig.pinPolarity     = kFLEXIO_PinActiveLow;
		timerConfig.timerMode       = kFLEXIO_TimerModeDual8BitBaudBit;
		timerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
		timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
		timerConfig.timerReset      = kFLEXIO_TimerResetNever;
		timerConfig.timerDisable    = kFLEXIO_TimerDisableOnTimerCompare;
		timerConfig.timerEnable     = kFLEXIO_TimerEnableOnPinRisingEdgeTriggerHigh;//kFLEXIO_TimerEnableOnTriggerHigh;
		timerConfig.timerStop       = kFLEXIO_TimerStopBitEnableOnTimerDisable;
		timerConfig.timerStart      = kFLEXIO_TimerStartBitEnabled;

		timerDiv = clock_freq / config->baud_rate;
		timerDiv = timerDiv / 2 - 1;

		LOG_DBG("timer div = %d", timerDiv);

		if (timerDiv > 0xFFU)
		{
			LOG_ERR("Failed to set timer!");
		}

		timerCmp = ((uint32_t)(uart_config.bitCountPerChar * 2 - 1)) << 8U;
		timerCmp |= timerDiv;

		timerConfig.timerCompare = timerCmp;

		FLEXIO_SetTimerConfig(config->base->flexioBase, config->base->timerIndex[0], &timerConfig);

	    /*Extra: Configurate a spare timer to generate the clock signal */

		timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_SHIFTnSTAT(config->base->shifterIndex[0]);
		timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
		timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
		timerConfig.pinConfig       = kFLEXIO_PinConfigOutput;
		timerConfig.pinSelect       = config->tx_clk_pin_index;
		timerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
		timerConfig.timerMode       = kFLEXIO_TimerModeDual8BitBaudBit;
		timerConfig.timerOutput     = kFLEXIO_TimerOutputOneNotAffectedByReset;
		timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnFlexIOClockShiftTimerOutput;
		timerConfig.timerReset      = kFLEXIO_TimerResetNever;
		timerConfig.timerDisable    = kFLEXIO_TimerDisableNever; //kFLEXIO_TimerDisableOnTimerCompare;
		timerConfig.timerEnable     = kFLEXIO_TimerEnabledAlways;//kFLEXIO_TimerEnableOnTriggerHigh;
		timerConfig.timerStop       = kFLEXIO_TimerStopBitDisabled;// kFLEXIO_TimerStopBitEnableOnTimerDisable;
		timerConfig.timerStart      = kFLEXIO_TimerStartBitDisabled; // kFLEXIO_TimerStartBitEnabled



		timerDiv = clock_freq / config->baud_rate;
		timerDiv = timerDiv / 2 - 1;

		if (timerDiv > 0xFFU)
		{
			LOG_ERR("Failed to set timer!");
		}

		timerCmp = ((uint32_t)(9 * 2 - 1)) << 8U;
		timerCmp |= timerDiv;

		timerConfig.timerCompare = timerCmp;

    	FLEXIO_SetTimerConfig(config->base->flexioBase, 2, &timerConfig);

	}
	
	if( config->rx_clk_pin_index != 0xFF )
	{

		/* 4. Configure the Rx timer for Sychronous RX */
		flexio_timer_config_t timerConfig;
		timerConfig.triggerSelect   = FLEXIO_TIMER_TRIGGER_SEL_PININPUT(config->base->RxPinIndex);
		timerConfig.triggerPolarity = kFLEXIO_TimerTriggerPolarityActiveLow;
		timerConfig.triggerSource   = kFLEXIO_TimerTriggerSourceInternal;
		timerConfig.pinConfig       = kFLEXIO_PinConfigOutputDisabled;
		timerConfig.pinSelect       = config->rx_clk_pin_index;
		timerConfig.pinPolarity     = kFLEXIO_PinActiveHigh;
		timerConfig.timerMode       = kFLEXIO_TimerModeSingle16Bit;
		timerConfig.timerOutput     = kFLEXIO_TimerOutputZeroNotAffectedByReset;
		timerConfig.timerDecrement  = kFLEXIO_TimerDecSrcOnPinInputShiftPinInput;
		timerConfig.timerReset      = kFLEXIO_TimerResetNever;
		timerConfig.timerDisable    = kFLEXIO_TimerDisableOnTimerCompare;
		timerConfig.timerEnable     = kFLEXIO_TimerEnableOnTriggerRisingEdge;
		timerConfig.timerStop       = kFLEXIO_TimerStopBitEnableOnTimerDisable;
		timerConfig.timerStart      = kFLEXIO_TimerStartBitEnabled;

		timerConfig.timerCompare = uart_config.bitCountPerChar * 2 - 1U;

		FLEXIO_SetTimerConfig(config->base->flexioBase, config->base->timerIndex[1], &timerConfig);
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

static FLEXIO_UART_Type uart_1_flexio_config = {
	
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
#ifdef DT_INST_1_NXP_IMXRT_FLEXIO_UART_TXCLK_SIGNAL
	.tx_clk_pin_index = DT_INST_1_NXP_IMXRT_FLEXIO_UART_TXCLK_SIGNAL,
#else
	.tx_clk_pin_index =	 0xFF,
#endif
#ifdef DT_INST_1_NXP_IMXRT_FLEXIO_UART_RXCLK_SIGNAL
	.rx_clk_pin_index = DT_INST_1_NXP_IMXRT_FLEXIO_UART_RXCLK_SIGNAL,
#else
	.rx_clk_pin_index =	 0xFF,
#endif
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
