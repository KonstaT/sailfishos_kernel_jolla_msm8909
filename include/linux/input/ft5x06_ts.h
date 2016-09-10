/**********uniscope-driver-modify-file-on-qualcomm-platform*****************/
/*
 *
 * FocalTech ft5x06 TouchScreen driver header file.
 *
 * Copyright (c) 2010  Focal tech Ltd.
 * Copyright (c) 2012-2014, The Linux Foundation. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#ifndef __LINUX_FT5X06_TS_H__
#define __LINUX_FT5X06_TS_H__

//#define TPD_AUTO_UPGRADE   //liguowei@uniscope.com 20141208 force update TP FW

#define FT5X06_ID		0x55
#define FT5X16_ID		0x0A
#define FT5X36_ID		0x14
#define FT6X06_ID		0x06
#define FT6X36_ID       0x36

struct fw_upgrade_info {
	bool auto_cal;
	u16 delay_aa;
	u16 delay_55;
	u8 upgrade_id_1;
	u8 upgrade_id_2;
	u16 delay_readid;
	u16 delay_erase_flash;
};

struct ft5x06_ts_platform_data {
	struct fw_upgrade_info info;
	const char *name;
	const char *fw_name;
	u32 irqflags;
	u32 irq_gpio;
	u32 irq_gpio_flags;
	u32 reset_gpio;
	u32 reset_gpio_flags;
#ifdef UNISCOPE_DRIVER_DTV //liguowei@uniscope 20140722 support siano dtv
   u32 wlt_gpio;
	u32 wlt_gpio_flags;
 u32 vcc18_gpio;
	u32 vcc18_gpio_flags;
 u32 vccio_gpio;
	u32 vccio_gpio_flags;
 u32 dtvreset_gpio;
	u32 dtvreset_gpio_flags;
 u32 dtvinterrupt_gpio;
	u32 dtvinterrupt_gpio_flags;	
#endif	
	u32 family_id;
	u32 x_max;
	u32 y_max;
	u32 x_min;
	u32 y_min;
	u32 panel_minx;
	u32 panel_miny;
	u32 panel_maxx;
	u32 panel_maxy;
	u32 group_id;
	u32 hard_rst_dly;
	u32 soft_rst_dly;
	u32 num_max_touches;
	bool fw_vkey_support;
	bool no_force_update;
	bool i2c_pull_up;
	bool ignore_id_check;
#ifndef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	int (*power_init) (bool);
	int (*power_on) (bool);
#endif
};

/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
#if defined TPD_AUTO_UPGRADE
struct Upgrade_Info
{
    u8 CHIP_ID;
    u8 FTS_NAME[20];
    u8 TPD_MAX_POINTS;
    u8 AUTO_CLB;
    u16 delay_aa;       /*delay of write FT_UPGRADE_AA */
    u16 delay_55;       /*delay of write FT_UPGRADE_55 */
    u8 upgrade_id_1;    /*upgrade id 1 */
    u8 upgrade_id_2;    /*upgrade id 2 */
    u16 delay_readid;   /*delay of read id */
    u16 delay_earse_flash; /*delay of earse flash*/
};
#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/
#endif
