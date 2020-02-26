/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <device.h>
#include <logging/log_ctrl.h>
#include <drivers/uart.h>
#include <drivers/clock_control.h>
#include <fsl_flexio_uart.h>
#ifdef CONFIG_UART_ASYNC_API
//#include <fsl_flexio_uart_edma.h>
#include <fsl_dmamux.h>
#include <fsl_edma.h>
#endif
#include <soc.h>


#include <logging/log.h>
LOG_MODULE_REGISTER(uart_mcux_flexio, LOG_LEVEL_INF);

struct mcux_flexio_uart_config {
	FLEXIO_UART_Type *base;
	uint8_t tx_clk_pin_index;
	uint8_t rx_clk_pin_index;
	char *clock_controller;
	clock_control_subsys_t clock_subsys;
	u32_t baud_rate;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	void (*irq_config_func)(struct device *dev);
#endif
#ifdef CONFIG_UART_ASYNC_API
	u32_t tx_dma_channel;
	u32_t tx_dma_source;
	u32_t rx_dma_channel;
	u32_t rx_dma_source;
#endif
};

#define TCD_QUEUE_SIZE 3

struct mcux_flexio_uart_data {

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uart_irq_callback_user_data_t callback;
	void *cb_data;
#endif
#ifdef CONFIG_UART_ASYNC_API
	// Common
	const struct mcux_flexio_uart_config* cfg;

	uart_callback_t async_cb;
	void*           async_cb_data;

	// Tx
	edma_handle_t tx_dma_handle;
	struct k_delayed_work tx_timeout_work;
	flexio_uart_transfer_t tx_xfer;

	////// Rx
	edma_handle_t rx_dma_handle;
	struct k_delayed_work rx_timeout_work;
	s32_t rx_timeout_time;
	s32_t rx_timeout_chunk;
	u32_t rx_timeout_start;

	flexio_uart_transfer_t rx_xfer;
	flexio_uart_transfer_t rx_next_xfer;
	size_t rx_processed_len;
	bool rx_waiting_for_irq;
	bool rx_irq_req_buf;
	bool rx_timeout_from_isr;

	__attribute__((aligned(sizeof(edma_tcd_t)))) edma_tcd_t tcd_pool[TCD_QUEUE_SIZE];


#endif
};

const flexio_uart_transfer_t NULL_XFER = {NULL, 0};

#ifdef CONFIG_UART_ASYNC_API

static void debug_dma()
{
	LOG_DBG("Channel 0 status : 0x%x", 
	EDMA_GetChannelStatusFlags(DMA0, 0));
	LOG_DBG("Channel 1 status : 0x%x", 
	EDMA_GetChannelStatusFlags(DMA0, 1));
	LOG_DBG("Channel 2 status : 0x%x", 
	EDMA_GetChannelStatusFlags(DMA0, 2));
	LOG_DBG("Channel 3 status : 0x%x", 
	EDMA_GetChannelStatusFlags(DMA0, 3));
	LOG_DBG("DMA0.ERR = 0x%08x", DMA0->ERR);
	LOG_DBG("DMA0.ES = 0x%08x", DMA0->ES);
}

static bool  mcux_flexio_uartnotify_rx_processed(struct mcux_flexio_uart_data *dev_data,
					  size_t processed)
{


	if (dev_data->rx_processed_len == processed) {
		LOG_DBG("All byte alread processed, returning");
		return false;
	}

	if (!dev_data->async_cb) {
		return true;
	}

	struct uart_event evt = {
		.type = UART_RX_RDY,
		.data.rx = {
			.buf = dev_data->rx_xfer.data,
			.offset = dev_data->rx_processed_len,
			.len = processed - dev_data->rx_processed_len,
		},
	};

	dev_data->rx_processed_len = processed;

	dev_data->async_cb(&evt, dev_data->async_cb_data);

	return true;
}

static void mcux_flexio_uart_dma_tx_cb(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	struct mcux_flexio_uart_data* data = (struct mcux_flexio_uart_data*)param;


	if (transferDone)
    {
		LOG_DBG("DMA call-back: TxIdle");

		    /* Disable UART TX EDMA. */
		FLEXIO_UART_EnableTxDMA(data->cfg->base, false);

		/* Stop transfer. */
		EDMA_AbortTransfer(&data->tx_dma_handle);

		// Cancel the timeout
		k_delayed_work_cancel(&data->tx_timeout_work);

		// Build an event
		struct uart_event evt = {
			.type = UART_TX_DONE,
			.data.tx = {
				.buf = data->tx_xfer.data,
				.len = data->tx_xfer.dataSize,
			},
		};

		// Clear internal state
		data->tx_xfer.data = NULL;
		data->tx_xfer.dataSize = 0;

		// notify user application
		if (evt.data.tx.len != 0U && data->async_cb) {
			data->async_cb(&evt, data->async_cb_data);
		}
    }
	else
	{
		LOG_ERR("Tx tranfers error");
	}
}

static void mcux_flexio_uart_dma_rx_cb(edma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
	struct mcux_flexio_uart_data* data = (struct mcux_flexio_uart_data*)param;

	LOG_DBG("DMA call-back: Rx done (done=%d, %d tcds)", transferDone, tcds);

	// end of the buffer. If received lenght match buffer, 
	// Notify the application about received data
	mcux_flexio_uartnotify_rx_processed(data, data->rx_xfer.dataSize);

	if (data->async_cb) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = data->rx_xfer.data,
			},
		};

		data->async_cb(&evt, data->async_cb_data);
	}

	// No next buffer, so end the transfer 
	if (!data->rx_next_xfer.dataSize) {
		data->rx_xfer.data = NULL;
		data->rx_xfer.dataSize = 0U;

		// Stop DMA
		FLEXIO_UART_EnableRxDMA(data->cfg->base, false);
		EDMA_AbortTransfer(&data->rx_dma_handle);
		// Disable IRQ
		FLEXIO_UART_DisableInterrupts(data->cfg->base, 
			kFLEXIO_UART_RxDataRegFullInterruptEnable);

		// Canceld timeout
		k_delayed_work_cancel(&data->rx_timeout_work);


		if (data->async_cb) {
			struct uart_event evt = {
				.type = UART_RX_DISABLED,
			};

			data->async_cb(&evt, data->async_cb_data);
		}

		return;
	}

	data->rx_xfer = data->rx_next_xfer;
	data->rx_processed_len = 0;
	data->rx_next_xfer = NULL_XFER;

	/* Request next buffer*/
	if (data->async_cb) 
	{
		struct uart_event evt = {
			.type = UART_RX_BUF_REQUEST,
		};

		data->async_cb(&evt, data->async_cb_data);
	}

	data->rx_waiting_for_irq = true;
	data->rx_irq_req_buf = true;
	FLEXIO_UART_EnableInterrupts(data->cfg->base,
			kFLEXIO_UART_RxDataRegFullInterruptEnable);


	LOG_DBG("End of RxIdle");
}

static int mcux_flex_io_uart_tx_halt(struct mcux_flexio_uart_data *data)
{
	int key = irq_lock();
	size_t tx_active = data->tx_xfer.dataSize;

	struct uart_event evt = {
		.type = UART_TX_ABORTED,
		.data.tx = {
			.buf = data->tx_xfer.data,
			.len = 0U,
		},
	};

	data->tx_xfer.data = NULL;
	data->tx_xfer.dataSize = 0U;

	FLEXIO_UART_EnableTxDMA(data->cfg->base, false);

    /* Stop transfer. */
    EDMA_AbortTransfer(&data->tx_dma_handle);

	irq_unlock(key);

	
	size_t sent_count = tx_active - \
	EDMA_GetRemainingMajorLoopCount(data->tx_dma_handle.base, 
									data->tx_dma_handle.channel);

	evt.data.tx.len = sent_count;

	if (tx_active) {
		if (data->async_cb) {
			data->async_cb(&evt, data->async_cb_data);
		}
	} else {
		return -EINVAL;
	}

	return 0;
}

static void mcux_flex_io_uart_tx_timeout(struct k_work *work)
{
	struct mcux_flexio_uart_data *data = CONTAINER_OF(work,
							   struct mcux_flexio_uart_data, tx_timeout_work);

	LOG_WRN("Tx Timeout occured!");
	mcux_flex_io_uart_tx_halt(data);
}

static void mcux_flex_io_uart_rx_timeout(struct k_work *work)
{
	struct mcux_flexio_uart_data *data = CONTAINER_OF(work,
							struct mcux_flexio_uart_data, rx_timeout_work);

	int key = irq_lock();

	if (data->rx_xfer.dataSize == 0)
	{
		LOG_DBG("Rx Timeout, but transfer ended.");
		irq_unlock(key);
		return;
	}

	size_t pending_length = data->rx_xfer.dataSize - 
		EDMA_GetRemainingMajorLoopCount(data->rx_dma_handle.base, 
										data->rx_dma_handle.channel);

		
	//LOG_DBG("Rx Timeout occured (received = %d)", pending_length);


	// If received count is equal to buffer size, then the DMA ISR is pending. 
	// Let it porcess the work for us!
	if( pending_length == data->rx_xfer.dataSize)
	{
		LOG_DBG("Discarding Rx timout with recevied count of %d", pending_length);
		k_delayed_work_submit(&data->rx_timeout_work,
				      data->rx_timeout_chunk);
		irq_unlock(key);
		return;
	}


	/*
	 * Never do a notify on a timeout started from the ISR: timing
	 * granularity means the first timeout can be in the middle
	 * of reception but still have the total elapsed time exhausted.
	 * So we require a timeout chunk with no data seen at all
	 * (i.e. no ISR entry).
	 */
	if (data->rx_timeout_from_isr) {
		data->rx_timeout_from_isr = false;
		k_delayed_work_submit(&data->rx_timeout_work,
				      data->rx_timeout_chunk);
		irq_unlock(key);
		return;
	}

	u32_t now = k_uptime_get_32();
	u32_t elapsed = now - data->rx_timeout_start;

	if (elapsed >= data->rx_timeout_time) {
		/*
		 * No time left, so call the handler, and let the ISR
		 * restart the timeout when it sees data.
		 */
		
		if( (pending_length - data->rx_processed_len) != 0)
		{
			k_delayed_work_submit(&data->rx_timeout_work,
				      data->rx_timeout_chunk);
		}
		else
		{
			LOG_DBG("Re-eanble IRQ");
			// Re-eanble IRQ to start new Rx
			data->rx_waiting_for_irq = true;
			FLEXIO_UART_EnableInterrupts(data->cfg->base,
			kFLEXIO_UART_RxDataRegFullInterruptEnable);
		}
		LOG_DBG("Notifiying %d bytes", pending_length - data->rx_processed_len);
		mcux_flexio_uartnotify_rx_processed(data, pending_length);
	} else {
		/*
		 * Still have time left, so start another timeout.
		 */
		u32_t remaining = MIN(data->rx_timeout_time - elapsed,
				      data->rx_timeout_chunk);

		k_delayed_work_submit(&data->rx_timeout_work, remaining);
		//LOG_DBG("Re-arming timeout, (remaining=%d)", data->rx_timeout_time - elapsed);
	}

	irq_unlock(key);
}

static int mcux_flexio_uart_callback_set(struct device *dev, uart_callback_t callback,void *user_data)
{
	struct mcux_flexio_uart_data *data = dev->driver_data;

	LOG_DBG("callback: %p", callback);
	LOG_DBG("user_data: %p", user_data);

	data->async_cb = callback;
	data->async_cb_data = user_data;

	return 0;
}

static int mcux_flexio_uart_tx(struct device *dev, const u8_t *buf, size_t len, 
			u32_t timeout)
{
	struct mcux_flexio_uart_data *data = dev->driver_data;
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	int retval = 0;

	int key = irq_lock();

	if (data->tx_xfer.dataSize != 0U) {
		retval = -EBUSY;
		LOG_WRN("device busy, remaining %d", data->tx_xfer.dataSize);
		goto err;
	}


	data->tx_xfer.data = (u8_t*)buf;
	data->tx_xfer.dataSize = len;

	LOG_DBG("%s: Sending %d bytes from %p", dev->config->name,
			data->tx_xfer.dataSize, data->tx_xfer.data
	);

	if(timeout != K_FOREVER)
	{
		LOG_DBG("Setting timeout of %d", timeout);
		k_delayed_work_submit(&data->tx_timeout_work, timeout);
	}

	edma_transfer_config_t xferConfig;
	EDMA_PrepareTransfer(&xferConfig, 
							data->tx_xfer.data, 
							sizeof(uint8_t),
                            (void *)FLEXIO_UART_GetTxDataRegisterAddress(config->base), 
							sizeof(uint8_t),
							sizeof(uint8_t),
                            data->tx_xfer.dataSize, 
							kEDMA_MemoryToPeripheral);

	EDMA_SubmitTransfer(&data->tx_dma_handle, &xferConfig);
	EDMA_StartTransfer(&data->tx_dma_handle);
	FLEXIO_UART_EnableTxDMA(config->base, true);

err:
	irq_unlock(key);
	return retval;
}

static int mcux_flexio_uart_tx_abort(struct device *dev)
{
	struct mcux_flexio_uart_data * data = dev->driver_data;

	k_delayed_work_cancel(&data->tx_timeout_work);

	return mcux_flex_io_uart_tx_halt(data);
}

static int mcux_flexio_uart_poll_in(struct device *dev, unsigned char *c);

static int mcux_flexio_uart_rx_enable(struct device *dev, u8_t *buf, size_t len,u32_t timeout)
{
	struct mcux_flexio_uart_data *data = dev->driver_data;
	const struct mcux_flexio_uart_config *config = dev->config->config_info;

	int key = irq_lock();

	if (data->rx_xfer.dataSize != 0U) {
		irq_unlock(key);
		return -EBUSY;
	}

	
	// store arguments
	data->rx_xfer.data = buf;
	data->rx_xfer.dataSize = len;
	data->rx_timeout_time = timeout;
	data->rx_timeout_chunk = MAX(timeout / 4U, 1);
	data->rx_processed_len = 0U;
	
	data->rx_timeout_from_isr = true;


	LOG_DBG("Setup async receive of %d bytes to %p", 
			data->rx_xfer.dataSize, data->rx_xfer.data
	);

	// purge the last char received
	u8_t dummy;
	mcux_flexio_uart_poll_in(dev, &dummy);

	irq_unlock(key);

	// We will use the first ISR to trigger the Rx timeout timer
	data->rx_waiting_for_irq = true;
	data->rx_irq_req_buf = true;

	// Clear status and interrupt flags
	FLEXIO_UART_ClearStatusFlags(data->cfg->base,
			kFLEXIO_UART_RxDataRegFullInterruptEnable);
	FLEXIO_UART_EnableInterrupts(config->base, 
		kFLEXIO_UART_RxDataRegFullInterruptEnable);

	// start DMA
	edma_transfer_config_t xferConfig;
	EDMA_PrepareTransfer(&xferConfig, 
							(void *)FLEXIO_UART_GetRxDataRegisterAddress(config->base),
							sizeof(uint8_t),
							data->rx_xfer.data, 
							sizeof(uint8_t), 
							sizeof(uint8_t), 
							data->rx_xfer.dataSize, 
							kEDMA_PeripheralToMemory);

	EDMA_SubmitTransfer(&data->rx_dma_handle, &xferConfig);
	EDMA_StartTransfer(&data->rx_dma_handle);
	FLEXIO_UART_EnableRxDMA(config->base, true);

	return 0;
}

static int mcux_flexio_uart_rx_buf_rsp(struct device *dev, u8_t *buf, size_t len)
{
	struct mcux_flexio_uart_data *data = dev->driver_data;

	int key = irq_lock();
	int retval = 0;

	if (data->rx_xfer.dataSize == 0U) {
		retval = -EACCES;
		goto err;
	}

	if (data->rx_next_xfer.dataSize != 0U) {
		retval = -EBUSY;
		LOG_ERR("Overrinding next buffer!");
		goto err;
	}

	data->rx_next_xfer.data = buf;
	data->rx_next_xfer.dataSize= len;

err:
	irq_unlock(key);
	return retval;
}

static int mcux_flexio_uart_rx_disable(struct device *dev)
{	
	struct mcux_flexio_uart_data *data = dev->driver_data;
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	int retval;

	// Canceld timeout
	k_delayed_work_cancel(&data->rx_timeout_work);

	int key = irq_lock();

	// disabling Rx while no transfer is scheduled is invalid
	if (data->rx_xfer.dataSize == 0U) {
		retval = -EINVAL;
		goto err;
	}

	// Stop DMA
	FLEXIO_UART_EnableRxDMA(config->base, false);
    EDMA_AbortTransfer(&data->rx_dma_handle);

	// Disable IRQ
	FLEXIO_UART_DisableInterrupts(data->cfg->base, 
		kFLEXIO_UART_RxDataRegFullInterruptEnable);

	// If chaind buffer is used, notify for release
	if (data->rx_next_xfer.dataSize) {
		struct uart_event evt = {
			.type = UART_RX_BUF_RELEASED,
			.data.rx_buf = {
				.buf = data->rx_xfer.data,
			},
		};

		data->rx_next_xfer.data = NULL;
		data->rx_next_xfer.dataSize = 0U;

		if (data->async_cb) {
			data->async_cb(&evt, data->async_cb_data);
		}
	}

	// Notify for the data already received!
	size_t pending_length = data->rx_xfer.dataSize - 
	EDMA_GetRemainingMajorLoopCount(data->rx_dma_handle.base, 
									data->rx_dma_handle.channel);

	mcux_flexio_uartnotify_rx_processed(data, pending_length);

	// notify for the current buffer to be released
	struct uart_event evt = {
		.type = UART_RX_BUF_RELEASED,
		.data.rx_buf = {
			.buf = data->rx_xfer.data,
		},
	};
	data->rx_xfer.data = NULL;
	data->rx_xfer.dataSize = 0U;
	if (data->async_cb) {
		data->async_cb(&evt, data->async_cb_data);
	}

	// Nofitfy that the Rx has been disabled!
	evt.type = UART_RX_DISABLED;
	if (data->async_cb) {
		data->async_cb(&evt, data->async_cb_data);
	}



err:
	irq_unlock(key);
	return retval;
}


static void mcux_flexio_dma_tx_isr(void *arg)
{
	struct device *dev = arg;
	struct mcux_flexio_uart_data *data = dev->driver_data;

	LOG_DBG("DMA Tx ISR: channel: %d", data->rx_dma_handle.channel);


	EDMA_HandleIRQ(&data->tx_dma_handle);

}

static void mcux_flexio_dma_rx_isr(void *arg)
{
	struct device *dev = arg;
	struct mcux_flexio_uart_data *data = dev->driver_data;

	LOG_DBG("DMA Rx ISR:channel: %d", data->rx_dma_handle.channel);

	EDMA_HandleIRQ(&data->rx_dma_handle);
}

#endif

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
	       (FLEXIO_UART_GetStatusFlags(config->base)
		& kFLEXIO_UART_TxDataRegEmptyFlag)) {

		FLEXIO_UART_WriteByte(config->base, &tx_data[num_tx++]);
	}

	return num_tx;
}

static int mcux_flexio_uart_fifo_read(struct device *dev, u8_t *rx_data,
			       const int len)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u8_t num_rx = 0U;

	while ((len - num_rx > 0) &&
	       (FLEXIO_UART_GetStatusFlags(config->base)
		& kFLEXIO_UART_RxDataRegFullFlag)) {

		FLEXIO_UART_ReadByte(config->base, &rx_data[num_rx++]);
	}

	return num_rx;
}

static void mcux_flexio_uart_irq_tx_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_TxDataRegEmptyInterruptEnable;

	FLEXIO_UART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_tx_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_TxDataRegEmptyInterruptEnable;

	FLEXIO_UART_DisableInterrupts(config->base, mask);
}

static int mcux_flexio_uart_irq_tx_complete(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = FLEXIO_UART_GetStatusFlags(config->base);

	return (flags & kFLEXIO_UART_TxDataRegEmptyFlag) != 0U;
}

static int mcux_flexio_uart_irq_tx_ready(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_TxDataRegEmptyInterruptEnable;

	return (config->base->flexioBase->SHIFTEIEN & mask)
		&& mcux_flexio_uart_irq_tx_complete(dev);
}

static void mcux_flexio_uart_irq_rx_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_RxDataRegFullInterruptEnable;

	FLEXIO_UART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_rx_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_RxDataRegFullInterruptEnable;

	FLEXIO_UART_DisableInterrupts(config->base, mask);
}

static int mcux_flexio_uart_irq_rx_full(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t flags = FLEXIO_UART_GetStatusFlags(config->base);

	return (flags & kFLEXIO_UART_RxDataRegFullFlag) != 0U;
}

static int mcux_flexio_uart_irq_rx_ready(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_RxDataRegFullInterruptEnable;

	return (config->base->flexioBase->SHIFTEIEN & mask)
		&& mcux_flexio_uart_irq_rx_full(dev);
}

static void mcux_flexio_uart_irq_err_enable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_RxOverRunFlag;

	FLEXIO_UART_EnableInterrupts(config->base, mask);
}

static void mcux_flexio_uart_irq_err_disable(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	u32_t mask = kFLEXIO_UART_RxOverRunFlag;

	FLEXIO_UART_DisableInterrupts(config->base, mask);
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

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#if CONFIG_UART_INTERRUPT_DRIVEN || CONFIG_UART_ASYNC_API

static void mcux_flexio_uart_isr(void *arg)
{
	struct device *dev = arg;
	struct mcux_flexio_uart_data *data = dev->driver_data;



#if CONFIG_UART_INTERRUPT_DRIVEN

	if (data->callback) {
		data->callback(data->cb_data);
	}
#endif

#if CONFIG_UART_ASYNC_API
	
	if (data->rx_xfer.dataSize && 
		data->rx_waiting_for_irq) 
	{
		LOG_DBG("FlexIO RX ISR");
		data->rx_waiting_for_irq = false;
		// Disable Rx IRQ, the reset will be handled by the timeout & DMA IRQ
		FLEXIO_UART_DisableInterrupts(data->cfg->base,
			kFLEXIO_UART_RxDataRegFullInterruptEnable);

		FLEXIO_UART_ClearStatusFlags(data->cfg->base,
			kFLEXIO_UART_RxDataRegFullInterruptEnable);

		if (data->rx_irq_req_buf) 
		{
			data->rx_irq_req_buf = false;
			/* Receive started, so request the next buffer */
			if (data->rx_next_xfer.dataSize == 0U && data->async_cb) {
				LOG_DBG("ISR: Requesting new buffer");
				struct uart_event evt = {
					.type = UART_RX_BUF_REQUEST,
				};
				data->async_cb(&evt, data->async_cb_data);


			}

			if (data->rx_next_xfer.dataSize != 0U )
			{
					LOG_DBG("ISR: Loading DMA");
				edma_transfer_config_t xferConfig;
				EDMA_PrepareTransfer(&xferConfig, 
										(void *)FLEXIO_UART_GetRxDataRegisterAddress(data->cfg->base),
										sizeof(uint8_t),
										data->rx_next_xfer.data, 
										sizeof(uint8_t), 
										sizeof(uint8_t), 
										data->rx_next_xfer.dataSize, 
										kEDMA_PeripheralToMemory);

				status_t s = EDMA_SubmitTransfer(&data->rx_dma_handle, &xferConfig);
				if( s!= kStatus_Success)
				{
					LOG_ERR("Failed EDMA_SubmitTransfer()=%d", s);
				}
				
				// temporary diagnositc
				if( DMA0->ES != 0UL || DMA0->ERR != 0uL)
				{
					LOG_ERR("DMA in error state!");
					debug_dma();
				}
			}
		}

		// If work is already submitted, this will reset the timer
		if (data->rx_timeout_time != K_FOREVER) 
		{
			LOG_DBG("ISR: Starint timeout");
			data->rx_timeout_from_isr = true;
			data->rx_timeout_start = k_uptime_get_32();
			k_delayed_work_submit(&data->rx_timeout_work,
						data->rx_timeout_chunk);
		}

		LOG_DBG("ISR: END");
	}

#endif
}
#endif



bool dma_initized = false;

static int mcux_flexio_uart_init(struct device *dev)
{
	const struct mcux_flexio_uart_config *config = dev->config->config_info;
	struct mcux_flexio_uart_data *data = dev->driver_data;
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
	LOG_INF("Initalizing FlexIO-UART \"%s\"", dev->config->name);
	LOG_DBG("Clock freq: %d MHz", clock_freq/1000000);
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
		LOG_INF("Using sychronous TX mode (TxClkpin: %d!)", config->tx_clk_pin_index);

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

		LOG_INF("Using sychronous RX mode (RxClkpin: %d!)", config->rx_clk_pin_index);

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

	FLEXIO_UART_EnableInterrupts(config->base, 
			kFLEXIO_UART_TxDataRegEmptyInterruptEnable | kFLEXIO_UART_RxDataRegFullInterruptEnable );
	config->irq_config_func(dev);
#endif

#ifdef CONFIG_UART_ASYNC_API
	k_delayed_work_init(&data->tx_timeout_work, mcux_flex_io_uart_tx_timeout);
	k_delayed_work_init(&data->rx_timeout_work, mcux_flex_io_uart_rx_timeout);

	data->rx_irq_req_buf = false;


	// Intialize the DMA & DMAMUX
	edma_config_t edma_config;
	if (!dma_initized)
	{
		DMAMUX_Init(DMAMUX);
		EDMA_GetDefaultConfig(&edma_config);
		edma_config.enableDebugMode = true; 
		EDMA_Init(DMA0, &edma_config);
		dma_initized = true;
	}

	// Tx Mux and handle
	LOG_DBG("%s: Assign source %d to channel %d", dev->config->name, config->tx_dma_source, config->tx_dma_channel);
	DMAMUX_SetSource(DMAMUX, config->tx_dma_channel, config->tx_dma_source);
    DMAMUX_EnableChannel(DMAMUX, config->tx_dma_channel);
    EDMA_CreateHandle(&data->tx_dma_handle, DMA0, config->tx_dma_channel);
	EDMA_SetCallback(&data->tx_dma_handle, mcux_flexio_uart_dma_tx_cb, data);

	// Rx Mux and handle
	LOG_DBG("%s: Assign source %d to channel %d", dev->config->name, config->rx_dma_source, config->rx_dma_channel);
    DMAMUX_SetSource(DMAMUX, config->rx_dma_channel, config->rx_dma_source);
    DMAMUX_EnableChannel(DMAMUX, config->rx_dma_channel);
    EDMA_CreateHandle(&data->rx_dma_handle, DMA0, config->rx_dma_channel);
	EDMA_SetCallback(&data->rx_dma_handle, mcux_flexio_uart_dma_rx_cb, data);

	EDMA_InstallTCDMemory(&data->rx_dma_handle, data->tcd_pool, TCD_QUEUE_SIZE);

	uint32_t dma_error_flags = EDMA_GetErrorStatusFlags(DMA0);
	if(dma_error_flags)
	{
		LOG_ERR("DMA configuration error: 0x%08x", dma_error_flags);
		return -EIO;
	}


	// Attache the IRQ to the ISR
	config->irq_config_func(dev);


	data->cfg = config;

#endif

	return 0;
}

static const struct uart_driver_api mcux_flexio_uart_driver_api = {

#ifdef CONFIG_UART_ASYNC_API
	.callback_set = mcux_flexio_uart_callback_set,
	.tx = mcux_flexio_uart_tx,
	.tx_abort = mcux_flexio_uart_tx_abort,
	.rx_enable = mcux_flexio_uart_rx_enable,
	.rx_buf_rsp = mcux_flexio_uart_rx_buf_rsp,
	.rx_disable = mcux_flexio_uart_rx_disable,
#endif
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

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void mcux_flexio_uart_config_func_0(struct device *dev);
#endif

static FLEXIO_UART_Type uart_0_flexio_config = {
	
    .flexioBase      = FLEXIO1,
    .TxPinIndex      = DT_INST_0_NXP_IMXRT_FLEXIO_UART_TXD_SIGNAL, 
    .RxPinIndex      = DT_INST_0_NXP_IMXRT_FLEXIO_UART_RXD_SIGNAL,
    .shifterIndex[0] = 0U,
    .shifterIndex[1] = 2U,
    .timerIndex[0]   = 0U,
    .timerIndex[1]   = 1U,
};

static const struct mcux_flexio_uart_config mcux_flexio_uart_0_config = {
	.base = &uart_0_flexio_config,
#ifdef DT_INST_0_NXP_IMXRT_FLEXIO_UART_TXCLK_SIGNAL
	.tx_clk_pin_index = DT_INST_0_NXP_IMXRT_FLEXIO_UART_TXCLK_SIGNAL,
#else
	.tx_clk_pin_index =	 0xFF,
#endif
#ifdef DT_INST_0_NXP_IMXRT_FLEXIO_UART_RXCLK_SIGNAL
	.rx_clk_pin_index = DT_INST_0_NXP_IMXRT_FLEXIO_UART_RXCLK_SIGNAL,
#else
	.rx_clk_pin_index =	 0xFF,
#endif
	.clock_controller = DT_INST_0_NXP_IMXRT_FLEXIO_UART_CLOCK_CONTROLLER,
	.clock_subsys =
		(clock_control_subsys_t)DT_INST_0_NXP_IMXRT_FLEXIO_UART_CLOCK_NAME,
	.baud_rate = DT_INST_0_NXP_IMXRT_FLEXIO_UART_CURRENT_SPEED,
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	.irq_config_func = mcux_flexio_uart_config_func_0,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.tx_dma_channel = 0, 
	.tx_dma_source = kDmaRequestMuxFlexIO1Request0Request1, 
	.rx_dma_channel = 1, 
	.rx_dma_source = kDmaRequestMuxFlexIO1Request2Request3, 
#endif
};

static struct mcux_flexio_uart_data mcux_flexio_uart_0_data;

DEVICE_AND_API_INIT(flexio_uart_0, DT_INST_0_NXP_IMXRT_FLEXIO_UART_LABEL,
		    &mcux_flexio_uart_init,
		    &mcux_flexio_uart_0_data, &mcux_flexio_uart_0_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_flexio_uart_driver_api);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void mcux_flexio_uart_config_func_0(struct device *dev)
{
	LOG_DBG("Inst0: Connecting IRQs");
	IRQ_CONNECT(DT_INST_0_NXP_IMXRT_FLEXIO_UART_IRQ_0,
		    DT_INST_0_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_uart_isr, DEVICE_GET(flexio_uart_0), 0);

	irq_enable(DT_INST_0_NXP_IMXRT_FLEXIO_UART_IRQ_0);

#ifdef CONFIG_UART_ASYNC_API
	IRQ_CONNECT(0,
		    DT_INST_0_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_dma_tx_isr, DEVICE_GET(flexio_uart_0), 0);

	irq_enable(0);

	IRQ_CONNECT(1,
		    DT_INST_0_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_dma_rx_isr, DEVICE_GET(flexio_uart_0), 0);

	irq_enable(1);
#endif
}
#endif

#endif /* CONFIG_UART_MCUX_FLEXIO_UART_1 */

#ifdef CONFIG_UART_MCUX_FLEXIO_UART_2

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void mcux_flexio_uart_config_func_1(struct device *dev);
#endif

static FLEXIO_UART_Type uart_1_flexio_config = {
	
    .flexioBase      = FLEXIO2,
    .TxPinIndex      = DT_INST_1_NXP_IMXRT_FLEXIO_UART_TXD_SIGNAL, 
    .RxPinIndex      = DT_INST_1_NXP_IMXRT_FLEXIO_UART_RXD_SIGNAL,
    .shifterIndex[0] = 0U,
    .shifterIndex[1] = 2U,
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
#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
	.irq_config_func = mcux_flexio_uart_config_func_1,
#endif
#ifdef CONFIG_UART_ASYNC_API
	.tx_dma_channel = 2, 
	.tx_dma_source = kDmaRequestMuxFlexIO2Request0Request1, 
	.rx_dma_channel = 3, 
	.rx_dma_source = kDmaRequestMuxFlexIO2Request2Request3, 
#endif
};

static struct mcux_flexio_uart_data mcux_flexio_uart_1_data;

DEVICE_AND_API_INIT(flexio_uart_1, DT_INST_1_NXP_IMXRT_FLEXIO_UART_LABEL,
		    &mcux_flexio_uart_init,
		    &mcux_flexio_uart_1_data, &mcux_flexio_uart_1_config,
		    PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
		    &mcux_flexio_uart_driver_api);

#if defined(CONFIG_UART_INTERRUPT_DRIVEN) || defined(CONFIG_UART_ASYNC_API)
static void mcux_flexio_uart_config_func_1(struct device *dev)
{
	LOG_DBG("Inst1: Connecting IRQs");
	IRQ_CONNECT(DT_INST_1_NXP_IMXRT_FLEXIO_UART_IRQ_0,
		    DT_INST_1_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_uart_isr, DEVICE_GET(flexio_uart_1), 0);

	irq_enable(DT_INST_1_NXP_IMXRT_FLEXIO_UART_IRQ_0);

#ifdef CONFIG_UART_ASYNC_API
	IRQ_CONNECT(2,
		    DT_INST_1_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_dma_tx_isr, DEVICE_GET(flexio_uart_1), 0);

	irq_enable(2);

	IRQ_CONNECT(3,
		    DT_INST_1_NXP_IMXRT_FLEXIO_UART_IRQ_0_PRIORITY,
		    mcux_flexio_dma_rx_isr, DEVICE_GET(flexio_uart_1), 0);

	irq_enable(3);
#endif
}
#endif

#endif /* CONFIG_UART_MCUX_FLEXIO_UART_2 */
