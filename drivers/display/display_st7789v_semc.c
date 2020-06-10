/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2019 Nordic Semiconductor ASA
 * Copyright (c) 2019 Marc Reilly
 * Copyright (c) 2019 PHYTEC Messtechnik GmbH
 * Copyright (c) 2020 Riedo Networks
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_st7789v.h"

#include <device.h>
#include <drivers/spi.h>
#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/display.h>

#include <fsl_semc.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_st7789v);

#define ST7789V_RESET_PIN	DT_INST_0_SITRONIX_ST7789V_PARALLEL_RESET_GPIOS_PIN

struct st7789v_data {
	volatile u64_t* data_reg;
	volatile u64_t* cmd_reg;

#if defined(DT_INST_0_SITRONIX_ST7789V_RESET_GPIOS_CONTROLLER) || defined(DT_INST_0_SITRONIX_ST7789V_PARALLEL_RESET_GPIOS_PIN)
	struct device *reset_gpio;
#endif
	struct device *cmd_data_gpio;

	u16_t height;
	u16_t width;
	u16_t x_offset;
	u16_t y_offset;
};

#ifdef CONFIG_ST7789V_RGB565
#define ST7789V_PIXEL_SIZE 2u
#else
#define ST7789V_PIXEL_SIZE 3u
#endif

static void st7789v_configure_semc()
{
	/* Init SEMC peripheral */
	semc_config_t semc_config;
	SEMC_GetDefaultConfig(&semc_config);
	//semc_config.dqsMode = kSEMC_Loopbackinternal; /* For more accurate timing. */
	//semc_config.cmdTimeoutCycles = 0;
	//semc_config.busTimeoutCycles = 0;
	SEMC_Init(SEMC, &semc_config);

	/* Configure the SEMC module to interface to the display, "DBI" mode */
	status_t config_res;
	semc_dbi_config_t dbi_config;
	dbi_config.csxPinMux = kSEMC_MUXA8;
	dbi_config.address = DT_INST_0_SITRONIX_ST7789V_PARALLEL_BASE_ADDRESS;
	dbi_config.memsize_kbytes = 128;
	dbi_config.columnAddrBitNum = kSEMC_Dbi_Colum_9bit;
	dbi_config.burstLen = kSEMC_Dbi_BurstLen1;
	dbi_config.portSize = kSEMC_PortSize8Bit;
	dbi_config.tCsxSetup_Ns = 15;
	dbi_config.tCsxHold_Ns = 10;
	dbi_config.tWexLow_Ns = 35;
	dbi_config.tWexHigh_Ns = 35;
	dbi_config.tRdxLow_Ns = 250;
	dbi_config.tRdxHigh_Ns = 250;
	dbi_config.tCsxInterval_Ns = 0;
	config_res = SEMC_ConfigureDBI(SEMC, &dbi_config, CLOCK_GetFreq(kCLOCK_SemcClk));
	if(config_res != kStatus_Success) {
		LOG_ERR("Error: Failed to initialize SEMC DBI!");
	} else {
		LOG_DBG("SMEC-DBI initialized!");
	}
}

static void st7789v_write_cmd_byte(struct st7789v_data *data, u8_t cmd)
{
#if 0
	u64_t cmd_64 = ((u64_t)cmd)<<56;
	SCB_DisableDCache();
	__DMB();
	*(data->cmd_reg) = cmd_64;
	SCB_EnableDCache();
#endif
	//uint8_t dataSize  = SEMC->DBICR0 & SEMC_DBICR0_PS_MASK;
	//SEMC_ConfigureIPCommand(SEMC, 1);

	int result = SEMC_SendIPCommand(
		SEMC, kSEMC_MemType_8080,
		(uint32_t)data->cmd_reg, kSEMC_NORDBICM_Write,
		cmd, NULL
	);
	if (result != kStatus_Success) {
		LOG_ERR("Error on IPCommand!");
	}

	//SEMC_ConfigureIPCommand(base, dataSize);
}

/* Use one of the define below to work around the problem with
 * cache management. */
#define NO_64BIT_ACCESS 1
#define USE_IRQ_LOCK 0

static void st7789v_write_data(struct st7789v_data *data, void *tx_data, int tx_len)
{
	int result;

	u8_t* data_ptr8 = tx_data;

	// set via IP bus until data is 64 bit aligned
	while( ((int)data_ptr8 & 0x00000003) != 0 && tx_len)
	{
		result = SEMC_SendIPCommand(
			SEMC, kSEMC_MemType_8080,
			(uint32_t)data->data_reg,
			kSEMC_NORDBICM_Write,
			*data_ptr8++, NULL
		);
		if (result != kStatus_Success) {
			LOG_ERR("Error on IPCommand!");
		} else {
			tx_len--;
		}
	}

#if !NO_64BIT_ACCESS
	// Switch to 64bit AXI bus
	u64_t* data_ptr64 = (u64_t*)data_ptr8;

	if( tx_len > sizeof(u64_t))
	{
		LOG_DBG("Writing %d bytes over AXI bus", tx_len);
#if USE_IRQ_LOCK
		int key = irq_lock();
#endif
		SCB_DisableDCache();
		__DMB();
		while(tx_len > sizeof(u64_t))
		{
			__DMB();
			*(data->data_reg) = *data_ptr64++;
			tx_len -= sizeof(u64_t);
		}
		SCB_EnableDCache();
#if USE_IRQ_LOCK
		irq_unlock(key);
#endif
	}
#endif /* !NO_64BIT_ACCESS */

	// If required, finish on IP bus
	if (tx_len)
	{
		LOG_DBG("Writing %d bytes over IP bus", tx_len);
		// Send out, 8 bit mode
#if !NO_64BIT_ACCESS
		u8_t* data_ptr8 = (u8_t*)data_ptr64;
#endif
		while(tx_len)
		{
			result = SEMC_SendIPCommand(
				SEMC, kSEMC_MemType_8080,
				(uint32_t)data->data_reg,
				kSEMC_NORDBICM_Write,
				*data_ptr8++, NULL
			);
			if (result != kStatus_Success) {
				LOG_ERR("Error on IPCommand!");
			} else {
				tx_len--;
			}
		}
	}
}

void st7789v_transmit(struct st7789v_data *data, u8_t cmd,
		u8_t *tx_data, size_t tx_count)
{
	LOG_DBG("cmd: 0x%02X, size: %d", cmd, tx_count);

	//SEMC_IPCommandNorWrite(SEMC, data->cmd_reg, &cmd, 1);
	//SEMC_IPCommandNorWrite(SEMC, data->data_reg, tx_data, tx_len);
	
	st7789v_write_cmd_byte(data, cmd);
	st7789v_write_data(data, tx_data, tx_count);
}

static void st7789v_exit_sleep(struct st7789v_data *data)
{
	st7789v_transmit(data, ST7789V_CMD_SLEEP_OUT, NULL, 0);
	k_sleep(K_MSEC(120));
}

static void st7789v_reset_display(struct st7789v_data *data)
{
	LOG_DBG("Resetting display");
#if defined(DT_INST_0_SITRONIX_ST7789V_RESET_GPIOS_CONTROLLER) || defined(DT_INST_0_SITRONIX_ST7789V_PARALLEL_RESET_GPIOS_CONTROLLER)
	k_sleep(K_MSEC(1));
	gpio_pin_set(data->reset_gpio, ST7789V_RESET_PIN, 1);
	k_sleep(K_MSEC(6));
	gpio_pin_set(data->reset_gpio, ST7789V_RESET_PIN, 0);
	k_sleep(K_MSEC(20));
#else
	st7789v_transmit(p_st7789v, ST7789V_CMD_SW_RESET, NULL, 0);
	k_sleep(K_MSEC(5));
#endif
}

int st7789v_cmd_read8(struct st7789v_data *data, int cmd, u8_t *pRet)
{
	SEMC_SendIPCommand(SEMC, kSEMC_MemType_8080,
			  (uint32_t)data->cmd_reg, kSEMC_NORDBICM_Write,
			  cmd, NULL);
	uint32_t read_val;
	int ret = SEMC_SendIPCommand(SEMC, kSEMC_MemType_8080, (uint32_t)data->data_reg, kSEMC_NORDBICM_Read, 0, &read_val);
	*pRet = (u8_t)read_val & 0x000000FF;

	return ret;
}

static int st7789v_blanking_on(const struct device *dev)
{
	struct st7789v_data *driver = (struct st7789v_data *)dev->driver_data;

	st7789v_transmit(driver, ST7789V_CMD_DISP_OFF, NULL, 0);
	return 0;
}

static int st7789v_blanking_off(const struct device *dev)
{
	struct st7789v_data *driver = (struct st7789v_data *)dev->driver_data;

	st7789v_transmit(driver, ST7789V_CMD_DISP_ON, NULL, 0);
	return 0;
}

static int st7789v_read(const struct device *dev,
			const u16_t x,
			const u16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	return -ENOTSUP;
}

static void st7789v_set_mem_area(struct st7789v_data *data, const u16_t x,
				 const u16_t y, const u16_t w, const u16_t h)
{
	u16_t spi_data[2];

	u16_t ram_x = x + data->x_offset;
	u16_t ram_y = y + data->y_offset;

	spi_data[0] = sys_cpu_to_be16(ram_x);
	spi_data[1] = sys_cpu_to_be16(ram_x + w - 1);
	st7789v_transmit(data, ST7789V_CMD_CASET, (u8_t *)&spi_data[0], 4);

	spi_data[0] = sys_cpu_to_be16(ram_y);
	spi_data[1] = sys_cpu_to_be16(ram_y + h - 1);
	st7789v_transmit(data, ST7789V_CMD_RASET, (u8_t *)&spi_data[0], 4);
}

static int st7789v_write(const struct device *dev,
			 const u16_t x,
			 const u16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	struct st7789v_data *data = (struct st7789v_data *)dev->driver_data;
	const u8_t *write_data_start = (u8_t *) buf;
	u16_t write_cnt;
	u16_t nbr_of_writes;
	u16_t write_h;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT((desc->pitch * ST7789V_PIXEL_SIZE * desc->height) <= desc->buf_size,
			"Input buffer to small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)",
			desc->width, desc->height, x, y);
	st7789v_set_mem_area(data, x, y, desc->width, desc->height);

	if (desc->pitch > desc->width) {
		write_h = 1U;
		nbr_of_writes = desc->height;
	} else {
		write_h = desc->height;
		nbr_of_writes = 1U;
	}

	LOG_DBG("number of writes: %d", nbr_of_writes);

	st7789v_transmit(data, ST7789V_CMD_RAMWR,
			 (void *) write_data_start,
			 desc->width * ST7789V_PIXEL_SIZE * write_h);
	write_data_start += (desc->pitch * ST7789V_PIXEL_SIZE);
	for (write_cnt = 1U; write_cnt < nbr_of_writes; ++write_cnt) {
		LOG_DBG("Write %d", write_cnt);
		//SEMC_IPCommandNorWrite(SEMC, 0x10000, write_data_start, desc->width * ST7789V_RGB_SIZE * write_h);
		
		st7789v_write_data(data, (void*)write_data_start, desc->width * ST7789V_PIXEL_SIZE * write_h);
		write_data_start += (desc->pitch * ST7789V_PIXEL_SIZE);
	}

	return 0;
}

static void *st7789v_get_framebuffer(const struct device *dev)
{
	return NULL;
}

static int st7789v_set_brightness(const struct device *dev,
			   const u8_t brightness)
{
	return -ENOTSUP;
}

static int st7789v_set_contrast(const struct device *dev,
			 const u8_t contrast)
{
	return -ENOTSUP;
}

static void st7789v_get_capabilities(const struct device *dev,
			      struct display_capabilities *capabilities)
{
	struct st7789v_data *data = (struct st7789v_data *)dev->driver_data;

	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = data->width;
	capabilities->y_resolution = data->height;

#ifdef CONFIG_ST7789V_RGB565
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
#else
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
#endif
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

static int st7789v_set_pixel_format(const struct device *dev,
			     const enum display_pixel_format pixel_format)
{
#ifdef CONFIG_ST7789V_RGB565
	if (pixel_format == PIXEL_FORMAT_RGB_565) {
#else
	if (pixel_format == PIXEL_FORMAT_RGB_888) {
#endif
		return 0;
	}
	LOG_ERR("Pixel format change not implemented");
	return -ENOTSUP;
}

static int st7789v_set_orientation(const struct device *dev,
			    const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

int st7789v_init(struct device *dev)
{
	struct st7789v_data *data = (struct st7789v_data *)dev->driver_data;

#ifdef DT_INST_0_SITRONIX_ST7789V_PARALLEL_RESET_GPIOS_CONTROLLER
	data->reset_gpio = device_get_binding(
		DT_INST_0_SITRONIX_ST7789V_PARALLEL_RESET_GPIOS_CONTROLLER);
	if (data->reset_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for display reset");
		return -EPERM;
	}

	if (gpio_pin_configure(data->reset_gpio, ST7789V_RESET_PIN,
			       GPIO_OUTPUT)) {
		LOG_ERR("Couldn't configure reset pin");
		return -EIO;
	}
#endif

	st7789v_configure_semc();
	data->cmd_reg = (void*)(DT_INST_0_SITRONIX_ST7789V_PARALLEL_BASE_ADDRESS+0x10000);
	data->data_reg = (void*)DT_INST_0_SITRONIX_ST7789V_PARALLEL_BASE_ADDRESS;

	LOG_DBG("data register %p", data->data_reg);
	LOG_DBG("command register %p", data->cmd_reg);

	LOG_HEXDUMP_DBG((void*)&SEMC->MCR, 4, "SMEC->MCR");
	LOG_HEXDUMP_DBG((void*)&SEMC->IOCR, 4, "SMEC->IOCR");
	LOG_HEXDUMP_DBG((void*)&SEMC->BR[7], 4, "SMEC->BR[7]");
	LOG_HEXDUMP_DBG((void*)&SEMC->DBICR0, 4, "SMEC->DBICR0");
	LOG_HEXDUMP_DBG((void*)&SEMC->DBICR1, 4, "SMEC->DBICR1");

	st7789v_reset_display(data);

	st7789v_blanking_on(dev);

	st7789v_lcd_init(data);

	st7789v_exit_sleep(data);

	return 0;
}

static const struct display_driver_api st7789v_api = {
	.blanking_on = st7789v_blanking_on,
	.blanking_off = st7789v_blanking_off,
	.write = st7789v_write,
	.read = st7789v_read,
	.get_framebuffer = st7789v_get_framebuffer,
	.set_brightness = st7789v_set_brightness,
	.set_contrast = st7789v_set_contrast,
	.get_capabilities = st7789v_get_capabilities,
	.set_pixel_format = st7789v_set_pixel_format,
	.set_orientation = st7789v_set_orientation,
};

static struct st7789v_data st7789v_data = {
#ifdef DT_INST_0_SITRONIX_ST7789V_WIDTH
	.width = DT_INST_0_SITRONIX_ST7789V_WIDTH,
#endif
#ifdef DT_INST_0_SITRONIX_ST7789V_HEIGHT
	.height = DT_INST_0_SITRONIX_ST7789V_HEIGHT,
#endif
#ifdef DT_INST_0_SITRONIX_ST7789V_X_OFFSET
	.x_offset = DT_INST_0_SITRONIX_ST7789V_X_OFFSET,
#endif
#ifdef DT_INST_0_SITRONIX_ST7789V_Y_OFFSET
	.y_offset = DT_INST_0_SITRONIX_ST7789V_Y_OFFSET,
#endif
};

DEVICE_AND_API_INIT(st7789v, DT_INST_0_SITRONIX_ST7789V_PARALLEL_LABEL, &st7789v_init,
		    &st7789v_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &st7789v_api);
