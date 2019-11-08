/*
 * Copyright (c) 2019 Riedo Networks
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "display_st7789v.h"
#include <zephyr.h>
#include <stddef.h>


#warning FG0204L0DSSWMG01 init
void st7789v_lcd_init(struct st7789v_data *p_st7789v)
{
    u8_t cmd;
	u8_t data[14];

    k_sleep(120);

    cmd = (0x11);     //Sleep out
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

    k_sleep(120);

    cmd = (0x36);       // MADCTL
    data[0] = (0x20 | ST7789V_MADCTL_MX_RIGHT_TO_LEFT);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0x3A);
    data[0] = (0x06);    //18bit
    st7789v_transmit(p_st7789v, cmd, data, 1);

    ///////////////////////////////////////////

    cmd = (0xB2);
    data[0] = (0x0C);
    data[1] = (0x0C);
    data[2] = (0x00);
    data[3] = (0x33);
    data[4] = (0x33);
    st7789v_transmit(p_st7789v, cmd, data, 5);

    cmd = (0xB7);     // VGH and VGL setting
    data[0] = (0x75);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xBB);     //VCOMS setting
    data[0] = (0x2C);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xC0);
    data[0] = (0x2C);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xC2);
    data[0] = (0x01);
    data[1] = (0xFF);
    st7789v_transmit(p_st7789v, cmd, data, 2);

    cmd = (0xC3);     //GVDD, GVCL setting
    data[0] = (0x20);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xC4);
    data[0] = (0x20);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xC6);
    data[0] = (0x0F);
    st7789v_transmit(p_st7789v, cmd, data, 1);

    cmd = (0xD0);
    data[0] = (0xA4);
    data[1] = (0xA1);
    st7789v_transmit(p_st7789v, cmd, data, 2);

    cmd = (0xE0);     //Positive Voltage Gamma Control
    data[0] = (0xD0);
    data[1] = (0x16);
    data[2] = (0x1B);
    data[3] = (0x0B);
    data[4] = (0x0B);
    data[5] = (0x26);
    data[6] = (0x3C);
    data[7] = (0x43);
    data[8] = (0x4F);
    data[9] = (0x28);
    data[10] = (0x13);
    data[11] = (0x13);
    data[12] = (0x2E);
    data[13] = (0x33);
    st7789v_transmit(p_st7789v, cmd, data, 14);

    cmd = (0xE1);     //Negative Voltage Gamma Control
    data[0] = (0xD0);
    data[1] = (0x16);
    data[2] = (0x1B);
    data[3] = (0x0B);
    data[4] = (0x0A);
    data[5] = (0x26);
    data[6] = (0x3B);
    data[7] = (0x44);
    data[8] = (0x4E);
    data[9] = (0x27);
    data[10] = (0x13);
    data[11] = (0x12);
    data[12] = (0x2E);
    data[13] = (0x33);
    st7789v_transmit(p_st7789v, cmd, data, 14);

    cmd = (0x21);
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

    cmd = (0x29);     //Display ON
    st7789v_transmit(p_st7789v, cmd, NULL, 0);

}
