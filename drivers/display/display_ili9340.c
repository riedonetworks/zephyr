/*
 * Copyright (c) 2017 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "display_ili9340.h"
#include <drivers/display.h>

#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_ili9340);

#include <drivers/gpio.h>
#include <sys/byteorder.h>
#include <drivers/spi.h>
#include <string.h>
#include <arch/cpu.h>
#include <soc.h>


#if defined(CONFIG_ILI9340_SPI)
#warning ILI9340 SPI mode
#elif defined(CONFIG_ILI9340_PARALLEL)
#warning ILI9340 Parallel mode
#include <fsl_semc.h>
#else
#error ILI9340 mode undefined!
#endif

struct ili9340_data {
#if defined(DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_CONTROLLER) || defined(DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_CONTROLLER)
	struct device *reset_gpio;
#endif
#ifdef CONFIG_ILI9340_SPI
	struct device *command_data_gpio;
	struct device *spi_dev;
	struct spi_config spi_config;
#endif

#ifdef CONFIG_ILI9340_PARALLEL
	volatile u8_t* data_reg;
	volatile u64_t* cmd_reg;
#endif

#ifdef DT_INST_0_ILITEK_ILI9340_CS_GPIOS_CONTROLLER
	struct spi_cs_control cs_ctrl;
#endif
};

#define ILI9340_CMD_DATA_PIN_COMMAND 0
#define ILI9340_CMD_DATA_PIN_DATA 1

/* The number of bytes taken by a RGB pixel */
#ifdef CONFIG_ILI9340_RGB565
#define ILI9340_RGB_SIZE 2U
#else
#define ILI9340_RGB_SIZE 3U
#endif

static void ili9340_exit_sleep(struct ili9340_data *data)
{
	ili9340_transmit(data, ILI9340_CMD_EXIT_SLEEP, NULL, 0);
	k_sleep(K_MSEC(120));
}

static void ili9340_configure_semc()
{
	/* Init SEMC perfieral */
	semc_config_t semc_config;
	SEMC_GetDefaultConfig(&semc_config);
    //semc_config.dqsMode = kSEMC_Loopbackinternal; /* For more accurate timing. */
	//semc_config.cmdTimeoutCycles = 0;
    //semc_config.busTimeoutCycles = 0;
    SEMC_Init(SEMC, &semc_config);

	/* Configure the SEMC module to inteface to the display, "DBI" mode */
	status_t config_res;
	semc_dbi_config_t dbi_config;
	dbi_config.csxPinMux = kSEMC_MUXA8;
	dbi_config.address = DT_INST_0_ILITEK_ILI9340_PARALLEL_BASE_ADDRESS;
	dbi_config.memsize_kbytes = 128;
	dbi_config.columnAddrBitNum = kSEMC_Dbi_Colum_9bit;
	dbi_config.burstLen = kSEMC_Dbi_BurstLen1;
	dbi_config.portSize = kSEMC_PortSize8Bit;
	dbi_config.tCsxSetup_Ns = 100;//50;
	dbi_config.tCsxHold_Ns = 100;//50;
	dbi_config.tWexLow_Ns = 100;//40;
	dbi_config.tWexHigh_Ns = 100;//40;
	dbi_config.tRdxLow_Ns = 100;//90;
	dbi_config.tRdxHigh_Ns = 100;//70;
	dbi_config.tCsxInterval_Ns = 100;//50;
	config_res = SEMC_ConfigureDBI(SEMC, &dbi_config, CLOCK_GetFreq(kCLOCK_SemcClk));
	if(config_res != kStatus_Success)
	{
		LOG_ERR("Error: Failed to initialize SEMC DBI!");
	}
	else
	{
		LOG_DBG("SMEC-DBI initialized!");
	}
}

static void ili9340_write_cmd_byte(struct ili9340_data *data, u8_t cmd)
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

	int result = SEMC_SendIPCommand(SEMC, kSEMC_MemType_8080, data->cmd_reg, kSEMC_NORDBICM_Write, cmd, NULL);
	if (result != kStatus_Success)
	{
		LOG_ERR("Error on IPCommand!");
	}


	//SEMC_ConfigureIPCommand(base, dataSize);

}

static void ili9340_write_data(struct ili9340_data *data, void *tx_data, int tx_len)
{
	#if 0
	SCB_DisableDCache();
	__DMB();
	u8_t* data_ptr = tx_data;
	for(int i=0; i< tx_len; i++)
	{
		__DMB();
		*(data->data_reg) = *data_ptr++;
	}
	SCB_EnableDCache();
	#endif
	int result;
	u8_t* data_ptr = tx_data;
	for(int i=0; i< tx_len; i++)
	{
		result = SEMC_SendIPCommand(SEMC, kSEMC_MemType_8080, data->data_reg, kSEMC_NORDBICM_Write, data_ptr[i], NULL);
		if (result != kStatus_Success)
		{
			LOG_ERR("Error on IPCommand!");
		}
	}

}

static int ili9340_init(struct device *dev)
{
	struct ili9340_data *data = (struct ili9340_data *)dev->driver_data;

	LOG_DBG("Initializing display driver");

#ifdef CONFIG_ILI9340_SPI

	data->spi_dev = device_get_binding(DT_INST_0_ILITEK_ILI9340_BUS_NAME);
	if (data->spi_dev == NULL) {
		LOG_ERR("Could not get SPI device for ILI9340");
		return -EPERM;
	}

	data->spi_config.frequency = DT_INST_0_ILITEK_ILI9340_SPI_MAX_FREQUENCY;
	data->spi_config.operation = SPI_OP_MODE_MASTER | SPI_WORD_SET(8);
	data->spi_config.slave = DT_INST_0_ILITEK_ILI9340_BASE_ADDRESS;


#ifdef DT_INST_0_ILITEK_ILI9340_CS_GPIOS_CONTROLLER
	data->cs_ctrl.gpio_dev =
		device_get_binding(DT_INST_0_ILITEK_ILI9340_CS_GPIOS_CONTROLLER);
	data->cs_ctrl.gpio_pin = DT_INST_0_ILITEK_ILI9340_CS_GPIOS_PIN;
	data->cs_ctrl.delay = 0U;
	data->spi_config.cs = &(data->cs_ctrl);
#else
	data->spi_config.cs = NULL;
#endif
#endif

#ifdef CONFIG_ILI9340_PARALLEL
	ili9340_configure_semc();
	data->cmd_reg = (void*)(DT_INST_0_ILITEK_ILI9340_PARALLEL_BASE_ADDRESS+0x10000);
	data->data_reg = (u8_t*)DT_INST_0_ILITEK_ILI9340_PARALLEL_BASE_ADDRESS;

	LOG_DBG("data register %p", data->data_reg);
	LOG_DBG("command register %p", data->cmd_reg);

	LOG_HEXDUMP_DBG((void*)&SEMC->MCR, 4, "SMEC->MCR");
	LOG_HEXDUMP_DBG((void*)&SEMC->IOCR, 4, "SMEC->IOCR");
	LOG_HEXDUMP_DBG((void*)&SEMC->BR[7], 4, "SMEC->BR[7]");
	LOG_HEXDUMP_DBG((void*)&SEMC->DBICR0, 4, "SMEC->DBICR0");
	LOG_HEXDUMP_DBG((void*)&SEMC->DBICR1, 4, "SMEC->DBICR1");
#endif


#ifdef DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_CONTROLLER
	data->reset_gpio =
		device_get_binding(DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_CONTROLLER);
	if (data->reset_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for ILI9340 reset");
		return -EPERM;
	}

	gpio_pin_configure(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_PIN,
			   GPIO_DIR_OUT);
#endif

#ifdef DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_CONTROLLER
	data->reset_gpio =
		device_get_binding(DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_CONTROLLER);
	if (data->reset_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for ILI9340 reset");
		return -EPERM;
	}

	gpio_pin_configure(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_PIN,
			   GPIO_DIR_OUT);
#endif

#ifdef CONFIG_ILI9340_SPI

	data->command_data_gpio =
		device_get_binding(DT_INST_0_ILITEK_ILI9340_CMD_DATA_GPIOS_CONTROLLER);
	if (data->command_data_gpio == NULL) {
		LOG_ERR("Could not get GPIO port for ILI9340 command/data");
		return -EPERM;
	}

	gpio_pin_configure(data->command_data_gpio, DT_INST_0_ILITEK_ILI9340_CMD_DATA_GPIOS_PIN,
			   GPIO_DIR_OUT);
#endif

#ifdef DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_CONTROLLER
	LOG_DBG("Resetting display driver");
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_PIN, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_PIN, 0);
	k_sleep(K_MSEC(1));
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_RESET_GPIOS_PIN, 1);
	k_sleep(K_MSEC(5));
#endif

#ifdef DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_CONTROLLER
	LOG_DBG("Resetting display driver");
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_PIN, 1);
	k_sleep(1);
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_PIN, 0);
	k_sleep(1);
	gpio_pin_write(data->reset_gpio, DT_INST_0_ILITEK_ILI9340_PARALLEL_RESET_GPIOS_PIN, 1);
	k_sleep(5);
#endif

	LOG_DBG("Initializing LCD");
	ili9340_lcd_init(data);

	LOG_DBG("Exiting sleep mode");
	ili9340_exit_sleep(data);

	return 0;
}

static void ili9340_set_mem_area(struct ili9340_data *data, const u16_t x,
				 const u16_t y, const u16_t w, const u16_t h)
{
	u16_t spi_data[2];

	spi_data[0] = sys_cpu_to_be16(x);
	spi_data[1] = sys_cpu_to_be16(x + w - 1);
	ili9340_transmit(data, ILI9340_CMD_COLUMN_ADDR, &spi_data[0], 4);

	spi_data[0] = sys_cpu_to_be16(y);
	spi_data[1] = sys_cpu_to_be16(y + h - 1);
	ili9340_transmit(data, ILI9340_CMD_PAGE_ADDR, &spi_data[0], 4);
}

static int ili9340_write(const struct device *dev, const u16_t x,
			 const u16_t y,
			 const struct display_buffer_descriptor *desc,
			 const void *buf)
{
	struct ili9340_data *data = (struct ili9340_data *)dev->driver_data;
	const u8_t *write_data_start = (u8_t *) buf;
#ifdef CONFIG_ILI9340_SPI
	struct spi_buf tx_buf;
	struct spi_buf_set tx_bufs;
#endif
	u16_t write_cnt;
	u16_t nbr_of_writes;
	u16_t write_h;

	__ASSERT(desc->width <= desc->pitch, "Pitch is smaller then width");
	__ASSERT((desc->pitch * ILI9340_RGB_SIZE * desc->height) <= desc->bu_size,
			"Input buffer to small");

	LOG_DBG("Writing %dx%d (w,h) @ %dx%d (x,y)", desc->width, desc->height,
			x, y);
	ili9340_set_mem_area(data, x, y, desc->width, desc->height);

	if (desc->pitch > desc->width) {
		write_h = 1U;
		nbr_of_writes = desc->height;
	} else {
		write_h = desc->height;
		nbr_of_writes = 1U;
	}

	ili9340_transmit(data, ILI9340_CMD_MEM_WRITE,
			 (void *) write_data_start,
			 desc->width * ILI9340_RGB_SIZE * write_h);

#ifdef CONFIG_ILI9340_SPI
	tx_bufs.buffers = &tx_buf;
	tx_bufs.count = 1;

	write_data_start += (desc->pitch * ILI9340_RGB_SIZE);
	for (write_cnt = 1U; write_cnt < nbr_of_writes; ++write_cnt) {
		tx_buf.buf = (void *)write_data_start;
		tx_buf.len = desc->width * ILI9340_RGB_SIZE * write_h;
		spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
		write_data_start += (desc->pitch * ILI9340_RGB_SIZE);
	}
#endif
#ifdef CONFIG_ILI9340_PARALLEL
	
	write_data_start += (desc->pitch * ILI9340_RGB_SIZE);
	for (write_cnt = 1U; write_cnt < nbr_of_writes; ++write_cnt) {
		LOG_DBG("Write %d", write_cnt);
		//SEMC_IPCommandNorWrite(SEMC, 0x10000, write_data_start, desc->width * ILI9340_RGB_SIZE * write_h);
		
		ili9340_write_data(data, write_data_start, desc->width * ILI9340_RGB_SIZE * write_h);
		write_data_start += (desc->pitch * ILI9340_RGB_SIZE);
	}
#endif

	return 0;
}

static int ili9340_read(const struct device *dev, const u16_t x,
			const u16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Reading not supported");
	return -ENOTSUP;
}

static void *ili9340_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Direct framebuffer access not supported");
	return NULL;
}

static int ili9340_display_blanking_off(const struct device *dev)
{
	struct ili9340_data *data = (struct ili9340_data *)dev->driver_data;

	LOG_DBG("Turning display blanking off");
	ili9340_transmit(data, ILI9340_CMD_DISPLAY_ON, NULL, 0);
	return 0;
}

static int ili9340_display_blanking_on(const struct device *dev)
{
	struct ili9340_data *data = (struct ili9340_data *)dev->driver_data;

	LOG_DBG("Turning display blanking on");
	ili9340_transmit(data, ILI9340_CMD_DISPLAY_OFF, NULL, 0);
	return 0;
}

static int ili9340_set_brightness(const struct device *dev,
				  const u8_t brightness)
{
	LOG_WRN("Set brightness not implemented");
	return -ENOTSUP;
}

static int ili9340_set_contrast(const struct device *dev, const u8_t contrast)
{
	LOG_ERR("Set contrast not supported");
	return -ENOTSUP;
}

static int ili9340_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format
				    pixel_format)
{
#ifdef CONFIG_ILI9340_RGB565
	if (pixel_format == PIXEL_FORMAT_RGB_565) {
#else
	if (pixel_format == PIXEL_FORMAT_RGB_888) {
#endif
		return 0;
	}
	LOG_ERR("Pixel format change not implemented");
	return -ENOTSUP;
}

static int ili9340_set_orientation(const struct device *dev,
				   const enum display_orientation orientation)
{
	if (orientation == DISPLAY_ORIENTATION_NORMAL) {
		return 0;
	}
	LOG_ERR("Changing display orientation not implemented");
	return -ENOTSUP;
}

static void ili9340_get_capabilities(const struct device *dev,
				     struct display_capabilities *capabilities)
{
	memset(capabilities, 0, sizeof(struct display_capabilities));
	capabilities->x_resolution = 320U;
	capabilities->y_resolution = 240U;
#ifdef CONFIG_ILI9340_RGB565
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_565;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_565;
#else
	capabilities->supported_pixel_formats = PIXEL_FORMAT_RGB_888;
	capabilities->current_pixel_format = PIXEL_FORMAT_RGB_888;
#endif
	capabilities->current_orientation = DISPLAY_ORIENTATION_NORMAL;
}

void ili9340_transmit(struct ili9340_data *data, u8_t cmd, void *tx_data,
		      size_t tx_len)
{
#ifdef CONFIG_ILI9340_SPI
	struct spi_buf tx_buf = { .buf = &cmd, .len = 1 };
	struct spi_buf_set tx_bufs = { .buffers = &tx_buf, .count = 1 };

	gpio_pin_write(data->command_data_gpio, DT_INST_0_ILITEK_ILI9340_CMD_DATA_GPIOS_PIN,
		       ILI9340_CMD_DATA_PIN_COMMAND);
	spi_write(data->spi_dev, &data->spi_config, &tx_bufs);

	if (tx_data != NULL) {
		tx_buf.buf = tx_data;
		tx_buf.len = tx_len;
		gpio_pin_write(data->command_data_gpio,
			       DT_INST_0_ILITEK_ILI9340_CMD_DATA_GPIOS_PIN,
			       ILI9340_CMD_DATA_PIN_DATA);
		spi_write(data->spi_dev, &data->spi_config, &tx_bufs);
	}
#endif
#ifdef CONFIG_ILI9340_PARALLEL
	LOG_DBG("cmd: 0x%02X, size: %d", cmd, tx_len);

	//SEMC_IPCommandNorWrite(SEMC, data->cmd_reg, &cmd, 1);
	//SEMC_IPCommandNorWrite(SEMC, data->data_reg, tx_data, tx_len);
	
	ili9340_write_cmd_byte(data, cmd);
	ili9340_write_data(data, tx_data, tx_len);
#endif
}

static const struct display_driver_api ili9340_api = {
	.blanking_on = ili9340_display_blanking_on,
	.blanking_off = ili9340_display_blanking_off,
	.write = ili9340_write,
	.read = ili9340_read,
	.get_framebuffer = ili9340_get_framebuffer,
	.set_brightness = ili9340_set_brightness,
	.set_contrast = ili9340_set_contrast,
	.get_capabilities = ili9340_get_capabilities,
	.set_pixel_format = ili9340_set_pixel_format,
	.set_orientation = ili9340_set_orientation,
};

static struct ili9340_data ili9340_data;

#ifdef DT_INST_0_ILITEK_ILI9340_LABEL
DEVICE_AND_API_INIT(ili9340, DT_INST_0_ILITEK_ILI9340_LABEL, &ili9340_init,
		    &ili9340_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &ili9340_api);
#endif

#ifdef DT_INST_0_ILITEK_ILI9340_PARALLEL_LABEL
DEVICE_AND_API_INIT(ili9340, DT_INST_0_ILITEK_ILI9340_PARALLEL_LABEL, &ili9340_init,
		    &ili9340_data, NULL, APPLICATION,
		    CONFIG_APPLICATION_INIT_PRIORITY, &ili9340_api);
#endif
