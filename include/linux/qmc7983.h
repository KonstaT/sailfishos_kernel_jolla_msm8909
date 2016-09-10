/**********uniscope-driver-modify-file-on-qualcomm-platform*****************/
/****************************************************************************
*   This software is licensed under the terms of the GNU General Public License version 2,
*   as published by the Free Software Foundation, and may be copied, distributed, and
*   modified under those terms.
*   This program is distributed in the hope that it will be useful, but WITHOUT ANY 
*   WARRANTY; without even the implied warranty of MERCHANTABILITY or
*   FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License 
*   for more details.
*
*   Copyright (C) 2012 by QST(Shanghai XiRui Keji) Corporation 
****************************************************************************/

#ifndef __QMC7983_H__
#define __QMC7983_H__

#include <linux/ioctl.h>  /* For IOCTL macros */

#define QMC7983_IOCTL_BASE 'm'
/* The following define the IOCTL command values via the ioctl macros */
#define QMC7983_SET_MODE			  _IOW(QMC7983_IOCTL_BASE, 1, int)
#define QMC7983_SET_RANGE		      _IOW(QMC7983_IOCTL_BASE, 2, int)
#define QMC7983_READ_MAGN_XYZ	      _IOR(QMC7983_IOCTL_BASE, 3, char *)
#define QMC7983_SET_OUTPUT_DATA_RATE  _IOR(QMC7983_IOCTL_BASE, 4, char *)
#define QMC7983_SELF_TEST	   		  _IOWR(QMC7983_IOCTL_BASE, 5, char *)
#define QMC7983_SET_OVERSAMPLE_RATIO  _IOWR(QMC7983_IOCTL_BASE, 6, char *)

#define QMC7983_READ_ACC_XYZ	_IOR(QMC7983_IOCTL_BASE, 21, short[3])
#define QMC7983_READ_QMC_XYZ	_IOR(QMC7983_IOCTL_BASE, 22, short[4])
#define QMC7983_READ_ORI_XYZ	_IOR(QMC7983_IOCTL_BASE, 23, short[4])
#define QMC7983_READ_ALL_REG	_IOR(QMC7983_IOCTL_BASE, 25, short[10])



//increase part
#define QMC7983_READ_CONTROL_REGISTER_ONE _IOR(QMC7983_IOCTL_BASE, 26, char *)
#define QMC7983_READ_CONTROL_REGISTER_TWO _IOR(QMC7983_IOCTL_BASE, 27, char *)
#define QMC7983_READ_MODE_REGISTER      	   _IOR(QMC7983_IOCTL_BASE, 28, char *)
#define QMC7983_READ_DATAOUTPUT_REGISTER       _IOR(QMC7983_IOCTL_BASE, 29, char *)
#define QMC7983_READ_STATUS_REGISTER_ONE 	       _IOR(QMC7983_IOCTL_BASE, 30, char *)
#define QMC7983_READ_STATUS_REGISTER_TWO 	       _IOR(QMC7983_IOCTL_BASE, 31, char *)
#define QMC7983_READ_TEMPERATURE_OUTPUT_MSB_REGISTER _IOR(QMC7983_IOCTL_BASE, 35, char *)
#define QMC7983_READ_TEMPERATURE_OUTPUT_LSB_REGISTER _IOR(QMC7983_IOCTL_BASE, 36, char *)

/************************************************/
/* 	Magnetometer section defines	 	*/
/************************************************/

/* Magnetic Sensor Operating Mode */
#define QMC7983_STANDBY_MODE	0x00
#define QMC7983_CC_MODE			0x01
#define QMC7983_SELFTEST_MODE	0x02
#define QMC7983_RESERVE_MODE	0x03


/* Magnetometer output data rate  */
#define QMC7983_ODR_10		0x00	/* 0.75Hz output data rate */
#define QMC7983_ODR_50		0x01	/* 1.5Hz output data rate */
#define QMC7983_ODR_100		0x02	/* 3Hz output data rate */
#define QMC7983_ODR7_200	0x03	/* 7.5Hz output data rate */


/* Magnetometer full scale  */
#define QMC7983_RNG_2G		0x00
#define QMC7983_RNG_8G		0x01
#define QMC7983_RNG_12G		0x02
#define QMC7983_RNG_20G		0x03

#define RNG_2G		2
#define RNG_8G		8
#define RNG_12G		12
#define RNG_20G		20

/*data output rate HZ*/
#define DATA_OUTPUT_RATE_10HZ 	0x00
#define DATA_OUTPUT_RATE_50HZ 	0x01
#define DATA_OUTPUT_RATE_100HZ 	0x02
#define DATA_OUTPUT_RATE_200HZ 	0x03

/*oversample Ratio */
#define OVERSAMPLE_RATIO_512 	0x00
#define OVERSAMPLE_RATIO_256 	0x01
#define OVERSAMPLE_RATIO_128 	0x02
#define OVERSAMPLE_RATIO_64 	0x03

#ifdef __KERNEL__

struct QMC7983_platform_data {

	u8 h_range;

	u8 axis_map_x;
	u8 axis_map_y;
	u8 axis_map_z;
	char	layout;
	u8 negate_x;
	u8 negate_y;
	u8 negate_z;

	int (*init)(void);
	void (*exit)(void);
	int (*power_on)(void);
	int (*power_off)(void);
	//char	layout;
	int	gpio_rstn;
	unsigned int auto_report;
	unsigned int delay;

};
#endif /* __KERNEL__ */


#endif  /* __QMC7983_H__ */
