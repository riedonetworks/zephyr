/*
 * Copyright (c) 2019 Riedo Networks Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/display.h>
#include "display_ili9340.h"
#define LOG_LEVEL CONFIG_DISPLAY_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(display_ili9340_TM024);


void ili9340_lcd_init(struct ili9340_data *p_ili9340)
{
	u8_t cmd;
	u8_t data[15];

	LOG_DBG("Intializing TM024HDH49");

	cmd = 0xCF;
	data[0] = 0x00;
	data[1] = 0xDB;
	data[2] = 0x30;
	ili9340_transmit(p_ili9340, cmd, data, 3);

	cmd = 0xED;
	data[0] = 0x64;
	data[1] = 0x03;
	data[2] = 0x12;
	data[3] = 0x81;
	ili9340_transmit(p_ili9340, cmd, data, 4);

	cmd = 0xE8;
	data[0] = 0x85;
	data[1] = 0x10;
	data[2] = 0x7A;
	ili9340_transmit(p_ili9340, cmd, data, 3);

	cmd = 0xCB;
	data[0] = 0x39;
	data[1] = 0x2C;
	data[2] = 0x00;
	data[3] = 0x34;
	data[4] = 0x02;
	ili9340_transmit(p_ili9340, cmd, data, 5);

	cmd = 0xF7;
	data[0] = 0x20;
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xEA;
	data[0] = 0x00;
	data[1] = 0x00;
	ili9340_transmit(p_ili9340, cmd, data, 2);

	cmd = 0xC0;
	data[0] = 0x21; // Power Control , VRH[5:0]
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xC1;
	data[0] = 0x11;
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xC5;
	data[0] = 0x28; // VCOM Control
	data[1] = 0x25;
	ili9340_transmit(p_ili9340, cmd, data, 2);

	cmd = 0xC7;
	data[0] = 0xc4; //b2
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0x36;
	data[0] = 0x40; // Memory Access Control
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xB1;
	data[0] = 0x00; //XIN
	data[1] = 0x1F;
	ili9340_transmit(p_ili9340, cmd, data, 2);

	cmd = 0xB6;
	data[0] = 0x0A; //XIN
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xF2;
	data[0] = 0x00; // 3Gamma Function Disable
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0x26;
	data[0] = 0x01; //Gamma curve selected
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xE0;
	data[0] = 0x00; //Set Gamma
	data[1] = 0x21;
	data[2] = 0x1D;
	data[3] = 0x0A;
	data[4] = 0x10;
	data[5] = 0x09;
	data[6] = 0x4B;
	data[7] = 0xA9;
	data[8] = 0x3A;
	data[9] = 0x0A;
	data[10] = 0x10;
	data[11] = 0x04;
	data[12] = 0x14;
	data[13] = 0x15;
	data[14] = 0x00;
	ili9340_transmit(p_ili9340, cmd, data, 15);

	cmd = 0xE1;
	data[0] = 0x0F;
	data[1] = 0x1C;
	data[2] = 0x22;
	data[3] = 0x05;
	data[4] = 0x11;
	data[5] = 0x06;
	data[6] = 0x35;
	data[7] = 0x56;
	data[8] = 0x46;
	data[9] = 0x05;
	data[10] = 0x0F;
	data[11] = 0x0B;
	data[12] = 0x29;
	data[13] = 0x3A;
	data[14] = 0x0F;
	ili9340_transmit(p_ili9340, cmd, data, 15);

	cmd = 0x11;    //Exit Sleep
	ili9340_transmit(p_ili9340, cmd, NULL, 0);
	k_sleep(120);

	cmd = 0x29;              //Display on
	ili9340_transmit(p_ili9340, cmd, NULL, 0);

	cmd = 0x36;
	//data[0] = 0x40;
	data[0] = 0x28;
	ili9340_transmit(p_ili9340, cmd, data, 1);

#ifdef CONFIG_ILI9340_RGB565
	data[0] = ILI9340_DATA_PIXEL_FORMAT_MCU_16_BIT |
		     ILI9340_DATA_PIXEL_FORMAT_RGB_16_BIT;
#else
	data[0] = ILI9340_DATA_PIXEL_FORMAT_MCU_18_BIT |
		     ILI9340_DATA_PIXEL_FORMAT_RGB_18_BIT;
#endif
	ili9340_transmit(p_ili9340, ILI9340_CMD_PIXEL_FORMAT_SET, data, 1);

	cmd = 0x2A;
	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = 0x00;
	data[3] = 0xEF;
	ili9340_transmit(p_ili9340, cmd, data, 4);

	cmd = 0x2B;
	data[0] = 0x00;
	data[1] = 0x00;
	data[2] = 0x01;
	data[3] = 0x3F;
	ili9340_transmit(p_ili9340, cmd, data, 4);

	cmd = 0x2C;
	ili9340_transmit(p_ili9340, cmd, NULL, 0);
}
