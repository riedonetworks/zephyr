/*
 * Copyright (c) 2019 Riedo Networks
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "display_st7789v.h"
#include <zephyr.h>
#include <stddef.h>

void st7789v_lcd_init(struct st7789v_data *p_st7789v)
{
    u8_t cmd;
	u8_t data[14];

   //---------------------------------------------------------------------------------------------------//
    cmd = 0x11;
    st7789v_transmit(p_st7789v, cmd, NULL, 0);
    k_sleep(K_MSEC(120));
    //------------------------------display and color format setting--------------------------------//
    cmd = 0x36;
    data[0] = (0x20 | ST7789V_MADCTL_MX_RIGHT_TO_LEFT);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0x3a;
    data[0] = 0x06;
    st7789v_transmit(p_st7789v, cmd, data, 1);
    //--------------------------------ST7789V Frame rate setting----------------------------------//
    cmd = 0xb2;
    data[0] = 0x0c;
    data[1] = 0x0c;
    data[2] = 0x00;
    data[3] = 0x33;
    data[4] = 0x33;
    st7789v_transmit(p_st7789v, cmd, data, 5);

    cmd = 0xb7;
    data[0] = 0x35;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    //---------------------------------ST7789V Power setting--------------------------------------//
    cmd = 0xbb;
    data[0] = 0x20;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xc0;
    data[0] = 0x2c;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xc2;
    data[0] = 0x01;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xc3;
    data[0] = 0x0b;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xc4;
    data[0] = 0x20;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xc6;
    data[0] = 0x0f;
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = 0xd0;
    data[0] = 0xa4;
    data[1] = 0xa1;
    st7789v_transmit(p_st7789v, cmd, data, 2);

    cmd = 0x21;
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

    //--------------------------------ST7789V gamma setting---------------------------------------//
    cmd = 0xe0;
    data[0] = 0xd0;
    data[1] = 0x00;
    data[2] = 0x03;
    data[3] = 0x08;
    data[4] = 0x0a;
    data[5] = 0x17;
    data[6] = 0x2e;
    data[7] = 0x44;
    data[8] = 0x3f;
    data[9] = 0x29;
    data[10] = 0x10;
    data[11] = 0x0e;
    data[12] = 0x14;
    data[13] = 0x18;
    st7789v_transmit(p_st7789v, cmd, data, 14);

    cmd = 0xe1;
    data[0] = 0xd0;
    data[1] = 0x00;
    data[2] = 0x03;
    data[3] = 0x08;
    data[4] = 0x07;
    data[5] = 0x27;
    data[6] = 0x2b;
    data[7] = 0x44;
    data[8] = 0x41;
    data[9] = 0x3c;
    data[10] = 0x1b;
    data[11] = 0x1d;
    data[12] = 0x14;
    data[13] = 0x18;
    st7789v_transmit(p_st7789v, cmd, data, 14);

    cmd = 0x29;
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

    cmd = 0x2C;
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

}
