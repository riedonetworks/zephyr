/*
 * Copyright (c) 2019 Riedo Networks Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/display.h>
#include "display_ili9340.h"
#include <logging/log.h>


void ili9340_lcd_init(struct ili9340_data *p_ili9340)
{
	u8_t cmd;
	u8_t data[15];

	///=================ILI9341V================================
	cmd = 0x11;    //Sleep out
	ili9340_transmit(p_ili9340, cmd, NULL, 0);
	k_sleep(150);          //Delay 120ms

	cmd = 0x3A;    // Pixxel  fromat set
	data[0] = (0x55);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0x26;    // Gamma Set
	data[0] = (0x01);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	///=================ILI9341V================================
	cmd = 0xb0;    //Porch Setting
	data[0] = (0X80|(1<<0)|(1<<1));
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xb1;    //Frame Rate Control (In Normal Mode/Full Colors)
	data[0] = (0);
	data[1] = 0x1D;//(0x1B);
	ili9340_transmit(p_ili9340, cmd, data, 2);

	cmd = 0xb6;    // Display Function control
	data[0] = (0x0a);
	data[1] = (0x02);//black
	data[2] = (0x27);
	data[3] = (0x04);
	ili9340_transmit(p_ili9340, cmd, data, 4);

	cmd = 0xb7;    //Entry Mode Set,
	data[0] = (0x06);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xc0;    //power1 GVDD
	data[0] = (0x35);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xc1;    //power2 /AVDDVGH/VGL 
	data[0] = (0x10);		//10
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0xC5;    //VCOM control 1, VCOMH/VCOML
	data[0] = (0x20);		//20
	data[1] = (0x21);		//21
	ili9340_transmit(p_ili9340, cmd, data, 2);

	cmd = 0xC7;    // VCOM control 2
	data[0] = (0x80|0x40);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0x55;    //Write Content Adaptive Brightness Control and Color Enhancement
	data[0] = (0x90);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	cmd = 0x34;  // Tearing Effect line off
	ili9340_transmit(p_ili9340, cmd, NULL, 0);  

	cmd = 0x36;    //Memory Data Access Control
	data[0] = ILI9340_DATA_MEM_ACCESS_CTRL_MX 
		| ILI9340_DATA_MEM_ACCESS_CTRL_MY 
		| ILI9340_DATA_MEM_ACCESS_CTRL_MV 
		| ILI9340_DATA_MEM_ACCESS_CTRL_BGR;       //D3=¡¥1¡¦ =BGR color filter panel)	rgb/ bgr
	ili9340_transmit(p_ili9340, cmd, data, 1);

#ifdef CONFIG_ILI9340_RGB565
	data[0] = ILI9340_DATA_PIXEL_FORMAT_MCU_16_BIT |
		     ILI9340_DATA_PIXEL_FORMAT_RGB_16_BIT;
#else
	data[0] = ILI9340_DATA_PIXEL_FORMAT_MCU_18_BIT |
		     ILI9340_DATA_PIXEL_FORMAT_RGB_18_BIT;
#endif
	ili9340_transmit(p_ili9340, ILI9340_CMD_PIXEL_FORMAT_SET, data, 1);

	cmd = 0x26;    //Set Gamma
	data[0] = (0x01);
	ili9340_transmit(p_ili9340, cmd, data, 1);

	//ILI9341 gamma setting

	cmd = 0xE0;       //Set Gamma +
	data[0] = (0x0F); 
	data[1] = (0x35); 
	data[2] = (0x31); 
	data[3] = (0x0B); 
	data[4] = (0x0E); 
	data[5] = (0x06); 
	data[6] = (0x49); 
	data[7] = (0xA7); 
	data[8] = (0x33); 
	data[9] = (0x07); 
	data[10] = (0x0F); 
	data[11] = (0x03); 
	data[12] = (0x0C); 
	data[13] = (0x0A); 
	data[14] = (0x00); 
	ili9340_transmit(p_ili9340, cmd, data, 15);

	cmd = 0XE1;       //Set Gamma  -
	data[0] = (0x00); 
	data[1] = (0x0A); 
	data[2] = (0x0F); 
	data[3] = (0x04); 
	data[4] = (0x11); 
	data[5] = (0x08); 
	data[6] = (0x36); 
	data[7] = (0x58); 
	data[8] = (0x4D); 
	data[9] = (0x07); 
	data[10] = (0x10); 
	data[11] = (0x0C); 
	data[12] = (0x32); 
	data[13] = (0x34); 
	data[14] = (0x0F); 
	ili9340_transmit(p_ili9340, cmd, data, 15);


	cmd = 0x29;    //Display on
	ili9340_transmit(p_ili9340, cmd, NULL, 0);

}
