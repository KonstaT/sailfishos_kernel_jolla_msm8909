/**********uniscope-driver-modify-file-on-qualcomm-platform*****************/
/*
 *
 * FocalTech ft5x06 TouchScreen driver.
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

#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/firmware.h>
#include <linux/debugfs.h>
#include <linux/input/ft5x06_ts.h>

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
#include <linux/uaccess.h>
#include <linux/irq.h>
#endif

#if defined(CONFIG_FB)
#include <linux/notifier.h>
#include <linux/fb.h>

#elif defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
/* Early-suspend level */
#define FT_SUSPEND_LEVEL 1
#endif

#define FT_DRIVER_VERSION	0x02

#define FT_META_REGS		3
#define FT_ONE_TCH_LEN		6
#define FT_TCH_LEN(x)		(FT_META_REGS + FT_ONE_TCH_LEN * x)

#define FT_PRESS		0x7F
#define FT_MAX_ID		0x0F
#define FT_TOUCH_X_H_POS	3
#define FT_TOUCH_X_L_POS	4
#define FT_TOUCH_Y_H_POS	5
#define FT_TOUCH_Y_L_POS	6
#define FT_TD_STATUS		2
#define FT_TOUCH_EVENT_POS	3
#define FT_TOUCH_ID_POS		5
#define FT_TOUCH_DOWN		0
#define FT_TOUCH_CONTACT	2

/*register address*/
#define FT_REG_DEV_MODE		0x00
#define FT_DEV_MODE_REG_CAL	0x02
#define FT_REG_ID		0xA3
#define FT_REG_PMODE		0xA5
#define FT_REG_FW_VER		0xA6
#define FT_REG_FW_VENDOR_ID	0xA8
#define FT_REG_POINT_RATE	0x88
#define FT_REG_THGROUP		0x80
#define FT_REG_ECC		0xCC
#define FT_REG_RESET_FW		0x07
#define FT_REG_FW_MIN_VER	0xB2
#define FT_REG_FW_SUB_MIN_VER	0xB3

/* power register bits*/
#define FT_PMODE_ACTIVE		0x00
#define FT_PMODE_MONITOR	0x01
#define FT_PMODE_STANDBY	0x02
#define FT_PMODE_HIBERNATE	0x03
#define FT_FACTORYMODE_VALUE	0x40
#define FT_WORKMODE_VALUE	0x00
#define FT_RST_CMD_REG1		0xFC
#define FT_RST_CMD_REG2		0xBC
#define FT_READ_ID_REG		0x90
#define FT_ERASE_APP_REG	0x61
#define FT_ERASE_PANEL_REG	0x63
#define FT_FW_START_REG		0xBF

#define FT_STATUS_NUM_TP_MASK	0x0F

#define FT_VTG_MIN_UV		2600000
#define FT_VTG_MAX_UV		3300000
#define FT_I2C_VTG_MIN_UV	1800000
#define FT_I2C_VTG_MAX_UV	1800000

#define FT_COORDS_ARR_SIZE	4
#define MAX_BUTTONS		4

#define FT_8BIT_SHIFT		8
#define FT_4BIT_SHIFT		4
#define FT_FW_NAME_MAX_LEN	50

#define FT5316_ID		0x0A
#define FT5306I_ID		0x55
#define FT6X06_ID		0x06
#define FT6X36_ID		0x36

#define FT_UPGRADE_AA		0xAA
#define FT_UPGRADE_55		0x55

#define FT_FW_MIN_SIZE		8
#define FT_FW_MAX_SIZE		32768

/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
/* This macro is used for TP auto upgrade ,if TP is died in using ,
and upgrade other version is no use, we can open it and test,now ,the 
defaule states is closed*/
#if defined TPD_AUTO_UPGRADE

#define CTP_IC_TYPE 0x14
#define TPD_MAX_POINTS_5    5
#define TPD_MAX_POINTS_2    2
#define AUTO_CLB_NEED   1
#define AUTO_CLB_NONEED     0
static u8 is_ic_update_crash = 0;
static struct i2c_client *update_client = NULL;
struct Upgrade_Info fts_updateinfo[] =
{
    {0x55,"FT5x06",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x08,"FT5606",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x06, 100, 2000},
    {0x0a,"FT5x16",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x07, 1, 1500},
    {0x05,"FT6208",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,60, 30, 0x79, 0x05, 10, 2000},
    {0x06,"FT6x06",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x08, 10, 2000},
    {0x36,"FT6x36",TPD_MAX_POINTS_2,AUTO_CLB_NONEED,100, 30, 0x79, 0x18, 10, 2000},//CHIP ID error
    {0x55,"FT5x06i",TPD_MAX_POINTS_5,AUTO_CLB_NEED,50, 30, 0x79, 0x03, 1, 2000},
    {0x14,"FT5336",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x13,"FT3316",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x12,"FT5436i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
    {0x11,"FT5336i",TPD_MAX_POINTS_5,AUTO_CLB_NONEED,30, 30, 0x79, 0x11, 10, 2000},
};

#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/
/* Firmware file is not supporting minor and sub minor so use 0 */
#define FT_FW_FILE_MAJ_VER(x)	((x)->data[(x)->size - 2])
#define FT_FW_FILE_MIN_VER(x)	0
#define FT_FW_FILE_SUB_MIN_VER(x) 0
#define FT_FW_FILE_VENDOR_ID(x)	((x)->data[(x)->size - 1])

#define FT_FW_FILE_MAJ_VER_FT6X36(x)	((x)->data[0x10a])
#define FT_FW_FILE_VENDOR_ID_FT6X36(x)	((x)->data[0x108])

/**
* Application data verification will be run before upgrade flow.
* Firmware image stores some flags with negative and positive value
* in corresponding addresses, we need pick them out do some check to
* make sure the application data is valid.
*/
#define FT_FW_CHECK(x, ts_data) \
	(ts_data->family_id == FT6X36_ID ? \
	(((x)->data[0x104] ^ (x)->data[0x105]) == 0xFF \
	&& ((x)->data[0x106] ^ (x)->data[0x107]) == 0xFF) : \
	(((x)->data[(x)->size - 8] ^ (x)->data[(x)->size - 6]) == 0xFF \
	&& ((x)->data[(x)->size - 7] ^ (x)->data[(x)->size - 5]) == 0xFF \
	&& ((x)->data[(x)->size - 3] ^ (x)->data[(x)->size - 4]) == 0xFF))

#define FT_MAX_TRIES		5
#define FT_RETRY_DLY		20

#define FT_MAX_WR_BUF		10
#define FT_MAX_RD_BUF		2
#define FT_FW_PKT_LEN		128
#define FT_FW_PKT_META_LEN	6
#define FT_FW_PKT_DLY_MS	20
#define FT_FW_LAST_PKT		0x6ffa
#define FT_EARSE_DLY_MS		100
#define FT_55_AA_DLY_NS		5000

#define FT_UPGRADE_LOOP		30
#define FT_CAL_START		0x04
#define FT_CAL_FIN		0x00
#define FT_CAL_STORE		0x05
#define FT_CAL_RETRY		100
#define FT_REG_CAL		0x00
#define FT_CAL_MASK		0x70

#define FT_INFO_MAX_LEN		512

#define FT_BLOADER_SIZE_OFF	12
#define FT_BLOADER_NEW_SIZE	30
#define FT_DATA_LEN_OFF_OLD_FW	8
#define FT_DATA_LEN_OFF_NEW_FW	14
#define FT_FINISHING_PKT_LEN_OLD_FW	6
#define FT_FINISHING_PKT_LEN_NEW_FW	12
#define FT_MAGIC_BLOADER_Z7	0x7bfa
#define FT_MAGIC_BLOADER_LZ4	0x6ffa
#define FT_MAGIC_BLOADER_GZF_30	0x7ff4
#define FT_MAGIC_BLOADER_GZF	0x7bf4
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
#define PINCTRL_STATE_ACTIVE	"pmx_ts_active"
#define PINCTRL_STATE_SUSPEND	"pmx_ts_suspend"
#define PINCTRL_STATE_RELEASE	"pmx_ts_release"
#endif
enum {
	FT_BLOADER_VERSION_LZ4 = 0,
	FT_BLOADER_VERSION_Z7 = 1,
	FT_BLOADER_VERSION_GZF = 2,
};

enum {
	FT_FT5336_FAMILY_ID_0x11 = 0x11,
	FT_FT5336_FAMILY_ID_0x12 = 0x12,
	FT_FT5336_FAMILY_ID_0x13 = 0x13,
	FT_FT5336_FAMILY_ID_0x14 = 0x14,
};

#define FT_STORE_TS_INFO(buf, id, name, max_tch, group_id, fw_vkey_support, \
			fw_name, fw_maj, fw_min, fw_sub_min) \
			snprintf(buf, FT_INFO_MAX_LEN, \
				"controller\t= focaltech\n" \
				"model\t\t= 0x%x\n" \
				"name\t\t= %s\n" \
				"max_touches\t= %d\n" \
				"drv_ver\t\t= 0x%x\n" \
				"group_id\t= 0x%x\n" \
				"fw_vkey_support\t= %s\n" \
				"fw_name\t\t= %s\n" \
				"fw_ver\t\t= %d.%d.%d\n", id, name, \
				max_tch, FT_DRIVER_VERSION, group_id, \
				fw_vkey_support, fw_name, fw_maj, fw_min, \
				fw_sub_min)

#define FT_DEBUG_DIR_NAME	"ts_debug"

#ifdef UNISCOPE_DRIVER_QC8909  
static struct workqueue_struct *ft5x06_wq;
int uniscope_debug_flag = 0;
#define UNISCOPE_DEBUG(fmt,arg...)          do{\
                                         if(uniscope_debug_flag)\
                                         pr_err("DATANG [%d]"fmt"\n",__LINE__, ##arg);\
                                       }while(0)
#endif

#ifdef UNISCOPE_DRIVER_L700  //liguowei@uniscope.com 20141126
int uniscope_imobile_charger_flag = 0;
#endif

struct ft5x06_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct ft5x06_ts_platform_data *pdata;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	struct regulator *avdd;	
	struct work_struct  work;	
#endif
	struct regulator *vdd;
	struct regulator *vcc_i2c;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	spinlock_t irq_lock;
	s32 irq_is_disabled;
	s32 use_irq;
#endif
	char fw_name[FT_FW_NAME_MAX_LEN];
	bool loading_fw;
	u8 family_id;
	struct dentry *dir;
	u16 addr;
	bool suspended;
	char *ts_info;
	u8 *tch_data;
	u32 tch_data_len;
	u8 fw_ver[3];
	u8 fw_vendor_id;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	bool power_on;
#endif
#if defined(CONFIG_FB)
	struct notifier_block fb_notif;
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
#endif
	struct pinctrl *ts_pinctrl;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	struct pinctrl_state *pinctrl_state_active;
	struct pinctrl_state *pinctrl_state_suspend;
	struct pinctrl_state *pinctrl_state_release;
#endif
};

static int ft5x06_i2c_read(struct i2c_client *client, char *writebuf,
			   int writelen, char *readbuf, int readlen)
{
	int ret;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = 0,
				 .len = writelen,
				 .buf = writebuf,
			 },
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 2);
		if (ret < 0)
			dev_err(&client->dev, "%s: i2c read error.\n",
				__func__);
	} else {
		struct i2c_msg msgs[] = {
			{
				 .addr = client->addr,
				 .flags = I2C_M_RD,
				 .len = readlen,
				 .buf = readbuf,
			 },
		};
		ret = i2c_transfer(client->adapter, msgs, 1);
		if (ret < 0)
			dev_err(&client->dev, "%s:i2c read error.\n", __func__);
	}
	return ret;
}

static int ft5x06_i2c_write(struct i2c_client *client, char *writebuf,
			    int writelen)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			 .addr = client->addr,
			 .flags = 0,
			 .len = writelen,
			 .buf = writebuf,
		 },
	};
	ret = i2c_transfer(client->adapter, msgs, 1);
	if (ret < 0)
		dev_err(&client->dev, "%s: i2c write error.\n", __func__);

	return ret;
}

static int ft5x0x_write_reg(struct i2c_client *client, u8 addr, const u8 val)
{
	u8 buf[2] = {0};

	buf[0] = addr;
	buf[1] = val;

	return ft5x06_i2c_write(client, buf, sizeof(buf));
}

static int ft5x0x_read_reg(struct i2c_client *client, u8 addr, u8 *val)
{
	return ft5x06_i2c_read(client, &addr, 1, val, 1);
}

static void ft5x06_update_fw_vendor_id(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VENDOR_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_vendor_id, 1);
	if (err < 0)
		dev_err(&client->dev, "fw vendor id read failed");
}

static void ft5x06_update_fw_ver(struct ft5x06_ts_data *data)
{
	struct i2c_client *client = data->client;
	u8 reg_addr;
	int err;

	reg_addr = FT_REG_FW_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[0], 1);
	if (err < 0)
		dev_err(&client->dev, "fw major version read failed");

	reg_addr = FT_REG_FW_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[1], 1);
	if (err < 0)
		dev_err(&client->dev, "fw minor version read failed");

	reg_addr = FT_REG_FW_SUB_MIN_VER;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &data->fw_ver[2], 1);
	if (err < 0)
		dev_err(&client->dev, "fw sub minor version read failed");

	dev_info(&client->dev, "Firmware version = %d.%d.%d\n",
		data->fw_ver[0], data->fw_ver[1], data->fw_ver[2]);
}

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
/*******************************************************
Function:
	Disable irq function
Input:
	ts: ft5x06 i2c_client private data
Output:
	None.
*********************************************************/
void ft5x06_irq_disable(struct ft5x06_ts_data *ts)
{
	unsigned long irqflags;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (!ts->irq_is_disabled) {
		ts->irq_is_disabled = true;
		disable_irq_nosync(ts->client->irq);
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

/*******************************************************
Function:
	Enable irq function
Input:
	ts: ft5x06 i2c_client private data
Output:
	None.
*********************************************************/
void ft5x06_irq_enable(struct ft5x06_ts_data *ts)
{
	unsigned long irqflags = 0;

	spin_lock_irqsave(&ts->irq_lock, irqflags);
	if (ts->irq_is_disabled) {
		enable_irq(ts->client->irq);
		ts->irq_is_disabled = false;
	}
	spin_unlock_irqrestore(&ts->irq_lock, irqflags);
}

#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
static int reg_set_optimum_mode_check(struct regulator *reg, int load_uA)
{
	return (regulator_count_voltages(reg) > 0) ?
		regulator_set_optimum_mode(reg, load_uA) : 0;
}
#endif

#ifdef UNISCOPE_DRIVER_QC8909 //liguowei@uniscope.com 20140827 for up issue 
extern bool uni_charger_status_using_ft5x06; //  Jason charge 20140901

static void ft5x0x_read_Touchdata_Work(struct work_struct *work)
{
       struct ft5x06_ts_data *data = NULL;
	struct input_dev *ip_dev;
	static u8 last_touchpoint=0;  // add by zhuqy for up issue @20140529
	static u8 Retry_iic=0;  // add by JasonLv for up issue @20140529
        static u8 usb_charge_flag=0;  //  Jason charge 20140901
        static  u8 charger_ftstatus =0;
        int ret=0;  //error 0; ok 1 
	int rc, i;
	u32 id, x, y, status, num_touches;
	u8 touch_cnums;
	u8 reg = 0x00, *buf;
	char txbuf[2];        //  Jason charge 20140901
	bool update_input = false;

	 data = container_of(work, struct ft5x06_ts_data, work);
	 
        UNISCOPE_DEBUG("%s \n",__func__);

	if (!data) {
		pr_err("%s: Invalid data\n", __func__);
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
		if (data->use_irq)
		     ft5x06_irq_enable(data);	
#endif
		return ;
	}
	/* please   check  usb_charge_flag firstly*/ // if(usb_charge_onoff==1)&&(usb_charge_onoff!=charge_ft)  
	if(uni_charger_status_using_ft5x06) 
		charger_ftstatus = 0x01;
	else
		charger_ftstatus = 0;

	if((charger_ftstatus==1) && (usb_charge_flag!=charger_ftstatus ))
	{
		txbuf[0] = 0x8B;
		txbuf[1] = 0x01;
		ret=ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
		if(ret==1)//  Jason charge flag 20140901
	          {
                  	txbuf[0] = 0x8B;
		        txbuf[1] = 0x01;
			ret=ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
			usb_charge_flag=charger_ftstatus;
	           }
        }
        else if((charger_ftstatus==0) && (usb_charge_flag!=charger_ftstatus ))//  Jason charge flag 20140901
        {
		txbuf[0] = 0x8B;
	        txbuf[1] = 0x00;//report 80 times per second
		ret=ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	       if (ret>0)  
	          usb_charge_flag=0;
	   }

	ip_dev = data->input_dev;
	buf = data->tch_data;

	rc = ft5x06_i2c_read(data->client, &reg, 1,
			buf, data->tch_data_len);
	if (rc < 0) {
		dev_err(&data->client->dev, "%s: read data fail\n", __func__);
		 Retry_iic++;
	}
	else 
	{
		if (buf[FT_TD_STATUS]==0 && buf[15]<0xff )
		   {
		   	Retry_iic++;
			}
		 else 
		 	Retry_iic=0;
        }
	if (Retry_iic>=3) //liguowei@uniscope.com 20141016 decrease tp sleep time
	{
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
		
	 if (gpio_is_valid(data->pdata->reset_gpio)) 
	 {
	 	for (i = 0; i <  data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	 }
	 }
	msleep(260);//liguowei@uniscope.com 20141224  tp sleep time
	UNISCOPE_DEBUG("Retry_iic=%d   reset_gpio  0-->1   %s\n", Retry_iic,__func__);

	for (i = 0; i <  data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	     }
	input_mt_report_pointer_emulation(ip_dev, false);
	input_sync(ip_dev);

	 Retry_iic=0;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	  if(data->use_irq)
		     ft5x06_irq_enable(data);	
#endif
         return ;
     }
	
	
	touch_cnums=buf[2]&0x7;  // read point num from  TP

	for (i = 0; i < data->pdata->num_max_touches; i++) {
		id = (buf[FT_TOUCH_ID_POS + FT_ONE_TCH_LEN * i]) >> 4;         // point id  

		if (id >= FT_MAX_ID)                                 // 	if ((pointid >= FT_MAX_ID)&&(eventid==3))
              
		break;
		update_input = true;

		x = (buf[FT_TOUCH_X_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_X_L_POS + FT_ONE_TCH_LEN * i]);
		y = (buf[FT_TOUCH_Y_H_POS + FT_ONE_TCH_LEN * i] & 0x0F) << 8 |
			(buf[FT_TOUCH_Y_L_POS + FT_ONE_TCH_LEN * i]);

		status = buf[FT_TOUCH_EVENT_POS + FT_ONE_TCH_LEN * i] >> 6;    // event id
                num_touches = buf[FT_TD_STATUS] & FT_STATUS_NUM_TP_MASK;  // buf[2]
		/* invalid combination */
		if (!num_touches && !status && !id)   	
		break;

		input_mt_slot(ip_dev, id);
		if (status == FT_TOUCH_DOWN || status == FT_TOUCH_CONTACT) 
		{
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 1);
			input_report_abs(ip_dev, ABS_MT_POSITION_X, x);
			input_report_abs(ip_dev, ABS_MT_POSITION_Y, y);
		} 
		else 
		{
			input_mt_report_slot_state(ip_dev, MT_TOOL_FINGER, 0);
		}
//  请在这里把 X,Y, id, status 4 个值，打印出来，做Debug 追踪	????	
		        UNISCOPE_DEBUG("id=%d status=%d x=%d y=%d     %s\n", id,status, x, y,__func__);
	}
	
	 if((last_touchpoint>0)&&(touch_cnums==0))    // add by zhuqy for up issue @20140529
	{	/* release all touches */ 
	for (i = 0; i <  data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	last_touchpoint=0;

        } 
	

	if (update_input) 
	{
		input_mt_report_pointer_emulation(ip_dev, false);
		input_sync(ip_dev);
	}
      
        // 请在这里last_touchpoint = %d,touch_cnums = %d 打印出来，做Debug 追踪	
        //  
	UNISCOPE_DEBUG("last_touchpoint=%d touch_cnums=%d    %s\n", last_touchpoint,touch_cnums,__func__);
		
        last_touchpoint= touch_cnums; // add by zhuqy for up issue @20140529
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
        if(data->use_irq)
		     ft5x06_irq_enable(data);	
#endif
   	return ;

} 

static irqreturn_t ft5x06_ts_interrupt(int irq, void *dev_id)
{

	struct ft5x06_ts_data *data = dev_id;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
   ft5x06_irq_disable(data);
#endif
   UNISCOPE_DEBUG("%s interrupt is really comming\n",__func__);
	
    queue_work(ft5x06_wq, &data->work);
	return IRQ_HANDLED;

}
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
#define ft5x06_VDD_LOAD_MAX_UA	10000 //liguowei

static int ft5x06_power_on(struct ft5x06_ts_data *data)
{
	int ret;

	if (data->power_on) {
		dev_info(&data->client->dev,
				"Device already power on\n");
		return 0;
	}
	if (!IS_ERR(data->avdd)) {
		ret = reg_set_optimum_mode_check(data->avdd,
			ft5x06_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Regulator avdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_avdd;
		}
		ret = regulator_enable(data->avdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		} 
	}
	data->power_on=true;
	return 0;

       err_enable_avdd:
       err_set_opt_avdd:
	data->power_on=false;
	return ret;
}

static int ft5x06_power_off(struct ft5x06_ts_data *data)
{
	int ret;
	if (data->power_on==0) {
		dev_err(&data->client->dev,
				"Device already power off\n");
		return 0;
	}
	if (!IS_ERR(data->avdd)) {
		ret = reg_set_optimum_mode_check(data->avdd,
			ft5x06_VDD_LOAD_MAX_UA);
		if (ret < 0) {
			dev_err(&data->client->dev,
				"Regulator avdd set_opt failed rc=%d\n", ret);
			goto err_set_opt_avdd;
		}
		ret = regulator_disable(data->avdd);
		if (ret) {
			dev_err(&data->client->dev,
				"Regulator avdd enable failed ret=%d\n", ret);
			goto err_enable_avdd;
		} 
	}
	data->power_on=false;
	return 0;

       err_enable_avdd:
       err_set_opt_avdd:
	data->power_on=true;
	return ret;
}
#else
static int ft5x06_power_on(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd enable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_enable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c enable failed rc=%d\n", rc);
		regulator_disable(data->vdd);
	}

	return rc;

power_off:
	rc = regulator_disable(data->vdd);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vdd disable failed rc=%d\n", rc);
		return rc;
	}

	rc = regulator_disable(data->vcc_i2c);
	if (rc) {
		dev_err(&data->client->dev,
			"Regulator vcc_i2c disable failed rc=%d\n", rc);
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
		}
	}

	return rc;
}
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
static int ft5x06_power_init(struct ft5x06_ts_data *data )
{
	int ret;

	data->avdd = regulator_get(&data->client->dev, "avdd");
	if (IS_ERR(data->avdd)) {
		ret = PTR_ERR(data->avdd);
		dev_info(&data->client->dev,
			"Regulator get failed avdd ret=%d\n", ret);
	}

	return 0;
}

#else
static int ft5x06_power_init(struct ft5x06_ts_data *data, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;

	data->vdd = regulator_get(&data->client->dev, "vdd");
	if (IS_ERR(data->vdd)) {
		rc = PTR_ERR(data->vdd);
		dev_err(&data->client->dev,
			"Regulator get failed vdd rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(data->vdd) > 0) {
		rc = regulator_set_voltage(data->vdd, FT_VTG_MIN_UV,
					   FT_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator set_vtg failed vdd rc=%d\n", rc);
			goto reg_vdd_put;
		}
	}

	data->vcc_i2c = regulator_get(&data->client->dev, "vcc_i2c");
	if (IS_ERR(data->vcc_i2c)) {
		rc = PTR_ERR(data->vcc_i2c);
		dev_err(&data->client->dev,
			"Regulator get failed vcc_i2c rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(data->vcc_i2c) > 0) {
		rc = regulator_set_voltage(data->vcc_i2c, FT_I2C_VTG_MIN_UV,
					   FT_I2C_VTG_MAX_UV);
		if (rc) {
			dev_err(&data->client->dev,
			"Regulator set_vtg failed vcc_i2c rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}

	return 0;

reg_vcc_i2c_put:
	regulator_put(data->vcc_i2c);
reg_vdd_set_vtg:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, FT_VTG_MAX_UV);

	regulator_put(data->vdd);

	if (regulator_count_voltages(data->vcc_i2c) > 0)
		regulator_set_voltage(data->vcc_i2c, 0, FT_I2C_VTG_MAX_UV);

	regulator_put(data->vcc_i2c);
	return 0;
}
#endif

static int ft5x06_ts_pinctrl_init(struct ft5x06_ts_data *ft5x06_data)
{
	int retval;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	/* Get pinctrl if target uses pinctrl */
	ft5x06_data->ts_pinctrl = devm_pinctrl_get(&(ft5x06_data->client->dev));
	if (IS_ERR_OR_NULL(ft5x06_data->ts_pinctrl)) {
		retval = PTR_ERR(ft5x06_data->ts_pinctrl);
		dev_dbg(&ft5x06_data->client->dev,
			"Target does not use pinctrl %d\n", retval);
		goto err_pinctrl_get;
	}

	ft5x06_data->pinctrl_state_active
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
				PINCTRL_STATE_ACTIVE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_active)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_active);
		dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_ACTIVE, retval);
		goto err_pinctrl_lookup;
	}

	ft5x06_data->pinctrl_state_suspend
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_SUSPEND);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_suspend)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_suspend);
			dev_err(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_SUSPEND, retval);
		goto err_pinctrl_lookup;
		}

	ft5x06_data->pinctrl_state_release
		= pinctrl_lookup_state(ft5x06_data->ts_pinctrl,
			PINCTRL_STATE_RELEASE);
	if (IS_ERR_OR_NULL(ft5x06_data->pinctrl_state_release)) {
		retval = PTR_ERR(ft5x06_data->pinctrl_state_release);
		dev_dbg(&ft5x06_data->client->dev,
			"Can not lookup %s pinstate %d\n",
			PINCTRL_STATE_RELEASE, retval);
	}

	return 0;

err_pinctrl_lookup:
	devm_pinctrl_put(ft5x06_data->ts_pinctrl);
err_pinctrl_get:
	ft5x06_data->ts_pinctrl = NULL;
	return retval;
#endif
}

#ifdef CONFIG_PM
static int ft5x06_ts_suspend(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	char txbuf[2], i;
	int err;

	if (data->loading_fw) {
		dev_info(dev, "Firmware loading in process...\n");
		return 0;
	}

	if (data->suspended) {
		dev_info(dev, "Already in suspend state\n");
		return 0;
	}

      UNISCOPE_DEBUG("%s \n",__func__);
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes	  
      if(data->use_irq)
	 ft5x06_irq_disable(data);
#endif
	/* Added by JZZ(zhizhang) for L600 TP leak 3mA current */
#if defined(UNISCOPE_DRIVER_L600)	
	gpio_direction_output(data->pdata->irq_gpio, 1); 
#endif	

	/* release all touches */
	for (i = 0; i < data->pdata->num_max_touches; i++) {
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, 0);
	}
	input_mt_report_pointer_emulation(data->input_dev, false);
	input_sync(data->input_dev);

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		txbuf[0] = FT_REG_PMODE;
		txbuf[1] = FT_PMODE_HIBERNATE;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	}
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	if (data->power_on) {
                ft5x06_power_off(data);
		err = data->power_on=false;
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_off(data);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}
#else
	if (data->pdata->power_on) {
		err = data->pdata->power_on(false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	} else {
		err = ft5x06_power_on(data, false);
		if (err) {
			dev_err(dev, "power off failed");
			goto pwr_off_fail;
		}
	}
#endif

	data->suspended = true;

	return 0;

pwr_off_fail:
	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}
	enable_irq(data->client->irq);
	return err;
}

static int ft5x06_ts_resume(struct device *dev)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	int err;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com Jason charge 20140901
        char txbuf[2];        
#endif
	if (!data->suspended) {
		dev_dbg(dev, "Already in awake state\n");
		return 0;
	}

       UNISCOPE_DEBUG("%s \n",__func__);
	
	/* Added by JZZ(zhizhang) for L600 TP leak 3mA current */
#if defined(UNISCOPE_DRIVER_L600)		
	err = gpio_direction_input(data->pdata->irq_gpio); 
#endif	
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	if (data->power_on) {
		err = data->power_on=true;
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
#else
	if (data->pdata->power_on) {
		err = data->pdata->power_on(true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(dev, "power on failed");
			return err;
		}
	}
#endif

	if (gpio_is_valid(data->pdata->reset_gpio)) {
		gpio_set_value_cansleep(data->pdata->reset_gpio, 0);
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

	msleep(260);
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	 if (data->use_irq)
		     ft5x06_irq_enable(data);	
#endif
	data->suspended = false;
	msleep(20);	
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com Jason charge 20140901
/* please   check  usb_charge_flag firstly*/
	if(uni_charger_status_using_ft5x06==1){  //  Jason charge 20140901
	   	txbuf[0] = 0x8B;
		txbuf[1] = 0x01;
		ft5x06_i2c_write(data->client, txbuf, sizeof(txbuf));
	   }
#endif

	return 0;
}

static const struct dev_pm_ops ft5x06_ts_pm_ops = {
#if (!defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND))
	.suspend = ft5x06_ts_suspend,
	.resume = ft5x06_ts_resume,
#endif
};

#else
static int ft5x06_ts_suspend(struct device *dev)
{
	return 0;
}

static int ft5x06_ts_resume(struct device *dev)
{
	return 0;
}

#endif

#if defined(CONFIG_FB)
static int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data)
{
	struct fb_event *evdata = data;
	int *blank;
	struct ft5x06_ts_data *ft5x06_data =
		container_of(self, struct ft5x06_ts_data, fb_notif);

	if (evdata && evdata->data && event == FB_EVENT_BLANK &&
			ft5x06_data && ft5x06_data->client) {
		blank = evdata->data;
		if (*blank == FB_BLANK_UNBLANK)
			ft5x06_ts_resume(&ft5x06_data->client->dev);
		else if (*blank == FB_BLANK_POWERDOWN)
			ft5x06_ts_suspend(&ft5x06_data->client->dev);
	}

	return 0;
}
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void ft5x06_ts_early_suspend(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_suspend(&data->client->dev);
}

static void ft5x06_ts_late_resume(struct early_suspend *handler)
{
	struct ft5x06_ts_data *data = container_of(handler,
						   struct ft5x06_ts_data,
						   early_suspend);

	ft5x06_ts_resume(&data->client->dev);
}
#endif

static int ft5x06_auto_cal(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	u8 temp = 0, i;

	/* set to factory mode */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* start calibration */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_START);
	msleep(2 * data->pdata->soft_rst_dly);
	for (i = 0; i < FT_CAL_RETRY; i++) {
		ft5x0x_read_reg(client, FT_REG_CAL, &temp);
		/*return to normal mode, calibration finish */
		if (((temp & FT_CAL_MASK) >> FT_4BIT_SHIFT) == FT_CAL_FIN)
			break;
	}

	/*calibration OK */
	msleep(2 * data->pdata->soft_rst_dly);
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_FACTORYMODE_VALUE);
	msleep(data->pdata->soft_rst_dly);

	/* store calibration data */
	ft5x0x_write_reg(client, FT_DEV_MODE_REG_CAL, FT_CAL_STORE);
	msleep(2 * data->pdata->soft_rst_dly);

	/* set to normal mode */
	ft5x0x_write_reg(client, FT_REG_DEV_MODE, FT_WORKMODE_VALUE);
	msleep(2 * data->pdata->soft_rst_dly);

	return 0;
}

static int ft5x06_fw_upgrade_start(struct i2c_client *client,
			const u8 *data, u32 data_len)
{
	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
	struct fw_upgrade_info info = ts_data->pdata->info;
	u8 reset_reg;
	u8 w_buf[FT_MAX_WR_BUF] = {0}, r_buf[FT_MAX_RD_BUF] = {0};
	u8 pkt_buf[FT_FW_PKT_LEN + FT_FW_PKT_META_LEN];
	int i, j, temp;
	u32 pkt_num, pkt_len;
	u8 is_5336_new_bootloader = false;
	u8 is_5336_fwsize_30 = false;
	u8 fw_ecc;
	
/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
#if defined TPD_AUTO_UPGRADE
	u8 reg_addr;
	u8 chip_id = 0x00;

	#if 1//READ IC INFO
	reg_addr = FT_REG_ID;
	temp = ft5x06_i2c_read(client, &reg_addr, 1, &chip_id, 1);
	if (temp < 0)
	{
		dev_err(&client->dev, "version read failed");
	}

	if (is_ic_update_crash)
	{
		chip_id = CTP_IC_TYPE;
	}
	for(i=0; i<sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info); i++)
	{
		if(chip_id==fts_updateinfo[i].CHIP_ID)
		{
			info.auto_cal = fts_updateinfo[i].AUTO_CLB;
			info.delay_55 = fts_updateinfo[i].delay_55;
			info.delay_aa = fts_updateinfo[i].delay_aa;
			info.delay_erase_flash = fts_updateinfo[i].delay_earse_flash;
			info.delay_readid = fts_updateinfo[i].delay_readid;
			info.upgrade_id_1 = fts_updateinfo[i].upgrade_id_1;
			info.upgrade_id_2 = fts_updateinfo[i].upgrade_id_2;

			break;
		}
	}

	ts_data->family_id = chip_id;

	if(i >= sizeof(fts_updateinfo)/sizeof(struct Upgrade_Info))
	{
		info.auto_cal = fts_updateinfo[0].AUTO_CLB;
		info.delay_55 = fts_updateinfo[0].delay_55;
		info.delay_aa = fts_updateinfo[0].delay_aa;
		info.delay_erase_flash = fts_updateinfo[0].delay_earse_flash;
		info.delay_readid = fts_updateinfo[0].delay_readid;
		info.upgrade_id_1 = fts_updateinfo[0].upgrade_id_1;
		info.upgrade_id_2 = fts_updateinfo[0].upgrade_id_2;
	}
	#endif

	dev_err(&client->dev, "id1 = 0x%x id2 = 0x%x family_id=0x%x\n",
	info.upgrade_id_1, info.upgrade_id_2, ts_data->family_id);
#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/

	/* determine firmware size */
	if (*(data + data_len - FT_BLOADER_SIZE_OFF) == FT_BLOADER_NEW_SIZE)
		is_5336_fwsize_30 = true;
	else
		is_5336_fwsize_30 = false;

	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++) {
		msleep(FT_EARSE_DLY_MS);
		/* reset - write 0xaa and 0x55 to reset register */
		if (ts_data->family_id == FT6X06_ID
			|| ts_data->family_id == FT6X36_ID)
			reset_reg = FT_RST_CMD_REG2;
		else
			reset_reg = FT_RST_CMD_REG1;

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_AA);
		msleep(info.delay_aa);

		ft5x0x_write_reg(client, reset_reg, FT_UPGRADE_55);
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(info.delay_55 + i * 3);
		else
			msleep(info.delay_55 - (i - (FT_UPGRADE_LOOP / 2)) * 2);

		/* Enter upgrade mode */
		w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, w_buf, 1);
		usleep(FT_55_AA_DLY_NS);
		w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, w_buf, 1);

		/* check READ_ID */
		msleep(info.delay_readid);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
		#if defined TPD_AUTO_UPGRADE
        	pr_err("%s:r_buf[0]=%x, r_buf[0]=%x\n",__func__, r_buf[0], r_buf[1]);
		r_buf[0] = 0x79;
		r_buf[1] = 0x11;
		#endif
		/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/


		if (r_buf[0] != info.upgrade_id_1
			|| r_buf[1] != info.upgrade_id_2) {
			dev_err(&client->dev, "Upgrade ID mismatch(%d), IC=0x%x 0x%x, info=0x%x 0x%x\n",
				i, r_buf[0], r_buf[1],
				info.upgrade_id_1, info.upgrade_id_2);
		} else
			break;
	}

	if (i >= FT_UPGRADE_LOOP) {
		dev_err(&client->dev, "Abort upgrade\n");
		return -EIO;
	}

	w_buf[0] = 0xcd;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);

	if (r_buf[0] <= 4)
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;
	else if (r_buf[0] == 7)
		is_5336_new_bootloader = FT_BLOADER_VERSION_Z7;
	else if (r_buf[0] >= 0x0f &&
		((ts_data->family_id == FT_FT5336_FAMILY_ID_0x11) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x12) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x13) ||
		(ts_data->family_id == FT_FT5336_FAMILY_ID_0x14)))
		is_5336_new_bootloader = FT_BLOADER_VERSION_GZF;
	else
		is_5336_new_bootloader = FT_BLOADER_VERSION_LZ4;

	dev_dbg(&client->dev, "bootloader type=%d, r_buf=0x%x, family_id=0x%x\n",
		is_5336_new_bootloader, r_buf[0], ts_data->family_id);
	/* is_5336_new_bootloader = FT_BLOADER_VERSION_GZF; */

	/* erase app and panel paramenter area */
	w_buf[0] = FT_ERASE_APP_REG;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(info.delay_erase_flash);

	if (is_5336_fwsize_30) {
		w_buf[0] = FT_ERASE_PANEL_REG;
		ft5x06_i2c_write(client, w_buf, 1);
	}
	msleep(FT_EARSE_DLY_MS);

	/* program firmware */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4
		|| is_5336_new_bootloader == FT_BLOADER_VERSION_Z7)
		data_len = data_len - FT_DATA_LEN_OFF_OLD_FW;
	else
		data_len = data_len - FT_DATA_LEN_OFF_NEW_FW;

	pkt_num = (data_len) / FT_FW_PKT_LEN;
	pkt_len = FT_FW_PKT_LEN;
	pkt_buf[0] = FT_FW_START_REG;
	pkt_buf[1] = 0x00;
	fw_ecc = 0;

	for (i = 0; i < pkt_num; i++) {
		temp = i * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		pkt_buf[4] = (u8) (pkt_len >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) pkt_len;

		for (j = 0; j < FT_FW_PKT_LEN; j++) {
			pkt_buf[6 + j] = data[i * FT_FW_PKT_LEN + j];
			fw_ecc ^= pkt_buf[6 + j];
		}

		ft5x06_i2c_write(client, pkt_buf,
				FT_FW_PKT_LEN + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send remaining bytes */
	if ((data_len) % FT_FW_PKT_LEN > 0) {
		temp = pkt_num * FT_FW_PKT_LEN;
		pkt_buf[2] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[3] = (u8) temp;
		temp = (data_len) % FT_FW_PKT_LEN;
		pkt_buf[4] = (u8) (temp >> FT_8BIT_SHIFT);
		pkt_buf[5] = (u8) temp;

		for (i = 0; i < temp; i++) {
			pkt_buf[6 + i] = data[pkt_num * FT_FW_PKT_LEN + i];
			fw_ecc ^= pkt_buf[6 + i];
		}

		ft5x06_i2c_write(client, pkt_buf, temp + FT_FW_PKT_META_LEN);
		msleep(FT_FW_PKT_DLY_MS);
	}

	/* send the finishing packet */
	if (is_5336_new_bootloader == FT_BLOADER_VERSION_LZ4 ||
		is_5336_new_bootloader == FT_BLOADER_VERSION_Z7) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_OLD_FW; i++) {
			if (is_5336_new_bootloader  == FT_BLOADER_VERSION_Z7)
				temp = FT_MAGIC_BLOADER_Z7 + i;
			else if (is_5336_new_bootloader ==
						FT_BLOADER_VERSION_LZ4)
				temp = FT_MAGIC_BLOADER_LZ4 + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);
		}
	} else if (is_5336_new_bootloader == FT_BLOADER_VERSION_GZF) {
		for (i = 0; i < FT_FINISHING_PKT_LEN_NEW_FW; i++) {
			if (is_5336_fwsize_30)
				temp = FT_MAGIC_BLOADER_GZF_30 + i;
			else
				temp = FT_MAGIC_BLOADER_GZF + i;
			pkt_buf[2] = (u8)(temp >> 8);
			pkt_buf[3] = (u8)temp;
			temp = 1;
			pkt_buf[4] = (u8)(temp >> 8);
			pkt_buf[5] = (u8)temp;
			pkt_buf[6] = data[data_len + i];
			fw_ecc ^= pkt_buf[6];

			ft5x06_i2c_write(client,
				pkt_buf, temp + FT_FW_PKT_META_LEN);
			msleep(FT_FW_PKT_DLY_MS);

		}
	}

	/* verify checksum */
	w_buf[0] = FT_REG_ECC;
	ft5x06_i2c_read(client, w_buf, 1, r_buf, 1);
	if (r_buf[0] != fw_ecc) {
		dev_err(&client->dev, "ECC error! dev_ecc=%02x fw_ecc=%02x\n",
					r_buf[0], fw_ecc);
		return -EIO;
	}

	/* reset */
	w_buf[0] = FT_REG_RESET_FW;
	ft5x06_i2c_write(client, w_buf, 1);
	msleep(ts_data->pdata->soft_rst_dly);

	dev_info(&client->dev, "Firmware upgrade successful\n");

	return 0;
}

/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
#if defined TPD_AUTO_UPGRADE
static unsigned char CTPM_FW[]=
{
#include "Uniscope_V08_L700_5336_0x5A_20141014_app.i"
};

static unsigned char CTPM_FW2[]=
{
#include "Uniscope_V08_L700_5336_0x5A_20141014_app.i"
};


u8 fts_ctpm_update_project_setting(struct i2c_client *client)
{
    u8 buf[128];
	u8 w_buf[4], r_buf[2];
    u32 i = 0,j=0;
    	struct ft5x06_ts_data *ts_data = i2c_get_clientdata(client);
    //int i_ret;
    //
	for (i = 0, j = 0; i < FT_UPGRADE_LOOP; i++)
	{
		msleep(FT_EARSE_DLY_MS);

		//reset tp
		if(gpio_is_valid(ts_data->pdata->reset_gpio))
		{
			gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 0);
			msleep(ts_data->pdata->hard_rst_dly);
			gpio_set_value_cansleep(ts_data->pdata->reset_gpio, 1);
		}
		
		if (i <= (FT_UPGRADE_LOOP / 2))
			msleep(30 + i * 3);
		else
			msleep(30 - (i - (FT_UPGRADE_LOOP / 2)) * 2);
		
		/* Enter upgrade mode */
			w_buf[0] = FT_UPGRADE_55;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		usleep(FT_55_AA_DLY_NS);
			w_buf[0] = FT_UPGRADE_AA;
		ft5x06_i2c_write(client, &w_buf[0], 1);
		
		/* check READ_ID */
		msleep(10);
		w_buf[0] = FT_READ_ID_REG;
		w_buf[1] = 0x00;
		w_buf[2] = 0x00;
		w_buf[3] = 0x00;

		ft5x06_i2c_read(client, w_buf, 4, r_buf, 2);

		if (r_buf[0] != 0x79 || r_buf[1] != 0x11)
		{
			continue;
		}
		else
			break;
	}
    buf[0] = 0x03;
    buf[1] = 0x00;
    buf[2] = (u8)(0x07b0 >> 8);
    buf[3] = (u8)(0x07b0);

    ft5x06_i2c_read(client, buf, 4, buf, 128);
    msleep(10);

    ft5x0x_write_reg(client, 0xfc, 0xaa);
    msleep(30);
    ft5x0x_write_reg(client, 0xfc, 0x55);
    msleep(200);

    is_ic_update_crash = 1;
    return buf[4];
}
int fts_ctpm_fw_upgrade_with_i_file(struct ft5x06_ts_data *data)
{
    struct i2c_client *client = data->client;
    int  flag_TPID=0;
    u8*     pbt_buf = 0x0;
    int rc = 0,fw_len = 0;
    u8 uc_host_fm_ver,uc_tp_fm_ver,vendor_id, ic_type;
    u8 reg_addr;

    //=========FW upgrade========================*/
    pbt_buf = CTPM_FW;
    fw_len = sizeof(CTPM_FW);
    pr_err("update firmware size:%d", fw_len);

    if (sizeof(CTPM_FW) < 8 || sizeof(CTPM_FW) > 32 * 1024 || sizeof(CTPM_FW2) < 8 || sizeof(CTPM_FW2) > 32 * 1024)
    {
        pr_err("FW length error\n");
        return -1;
    }

	reg_addr = 0xA6;
	ft5x06_i2c_read(client, &reg_addr, 1, &uc_tp_fm_ver, 1);
	reg_addr = 0xA8;
	ft5x06_i2c_read(client, &reg_addr, 1, &vendor_id, 1);
	reg_addr = 0xA3;
	ft5x06_i2c_read(client, &reg_addr, 1, &ic_type, 1);

        pr_err(" %s:ic_type is %x,CTP_IC_TYPE is %x;\n",__func__,ic_type,CTP_IC_TYPE);

	if(ic_type != CTP_IC_TYPE)
	{
		pr_err("IC type dismatch, please check");
	}
	
	if(vendor_id == 0xA8 || vendor_id == 0x00 || ic_type == 0xA3 || ic_type == 0x00)
	{
		pr_err("vend_id read error,need project");
		vendor_id = fts_ctpm_update_project_setting(client);
		flag_TPID = 1;
	}

	if(vendor_id == 0x5A)//truly
	{
		pbt_buf = CTPM_FW;
		fw_len = sizeof(CTPM_FW);
		pr_err("update firmware size1:%d", fw_len);
	}
	else if(vendor_id == 0x5B)//mudong
	{
		pbt_buf = CTPM_FW2;
		fw_len = sizeof(CTPM_FW2);
		pr_err("update firmware size2:%d", fw_len);
	}
	else
	{
		pr_err("read vendor_id fail");
		return -1;
	}

    if ((pbt_buf[fw_len - 8] ^ pbt_buf[fw_len - 6]) == 0xFF
        && (pbt_buf[fw_len - 7] ^ pbt_buf[fw_len - 5]) == 0xFF
        && (pbt_buf[fw_len - 3] ^ pbt_buf[fw_len - 4]) == 0xFF)
    {

		if(vendor_id != pbt_buf[fw_len-1])
		{
			pr_err("vendor_id dismatch, ic:%x, file:%x", vendor_id, pbt_buf[fw_len-1]);
			return -1;
		}

        uc_host_fm_ver = pbt_buf[fw_len - 2];
        pr_err("[FTS] uc_tp_fm_ver = %d.\n", uc_tp_fm_ver);
        pr_err("[FTS] uc_host_fm_ver = %d.\n", uc_host_fm_ver);

        if((uc_tp_fm_ver < uc_host_fm_ver)||(is_ic_update_crash==1))
        {
            rc = ft5x06_fw_upgrade_start(update_client, pbt_buf, fw_len);
            if (rc != 0)
            {
                pr_err("[FTS]  upgrade failed rc = %d.\n", rc);
            }
            else
            {
                pr_err("[FTS] upgrade successfully.\n");
            }
        }
    }

    return rc;
}
#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/
static int ft5x06_fw_upgrade(struct device *dev, bool force)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	const struct firmware *fw = NULL;
	int rc;
	u8 fw_file_maj, fw_file_min, fw_file_sub_min, fw_file_vendor_id;
	bool fw_upgrade = false;

	if (data->suspended) {
		dev_err(dev, "Device is in suspend state: Exit FW upgrade\n");
		return -EBUSY;
	}

	rc = request_firmware(&fw, data->fw_name, dev);
	if (rc < 0) {
		dev_err(dev, "Request firmware failed - %s (%d)\n",
						data->fw_name, rc);
		return rc;
	}

	if (fw->size < FT_FW_MIN_SIZE || fw->size > FT_FW_MAX_SIZE) {
		dev_err(dev, "Invalid firmware size (%zu)\n", fw->size);
		rc = -EIO;
		goto rel_fw;
	}

	if (data->family_id == FT6X36_ID) {
		fw_file_maj = FT_FW_FILE_MAJ_VER_FT6X36(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID_FT6X36(fw);
	} else {
		fw_file_maj = FT_FW_FILE_MAJ_VER(fw);
		fw_file_vendor_id = FT_FW_FILE_VENDOR_ID(fw);
	}
	fw_file_min = FT_FW_FILE_MIN_VER(fw);
	fw_file_sub_min = FT_FW_FILE_SUB_MIN_VER(fw);

	dev_info(dev, "Current firmware: %d.%d.%d", data->fw_ver[0],
				data->fw_ver[1], data->fw_ver[2]);
	dev_info(dev, "New firmware: %d.%d.%d", fw_file_maj,
				fw_file_min, fw_file_sub_min);

	if (force)
		fw_upgrade = true;
	else if ((data->fw_ver[0] < fw_file_maj) &&
		data->fw_vendor_id == fw_file_vendor_id)
		fw_upgrade = true;

	if (!fw_upgrade) {
		dev_info(dev, "Exiting fw upgrade...\n");
		rc = -EFAULT;
		goto rel_fw;
	}

	/* start firmware upgrade */
	if (FT_FW_CHECK(fw, data)) {
		rc = ft5x06_fw_upgrade_start(data->client, fw->data, fw->size);
		if (rc < 0)
			dev_err(dev, "update failed (%d). try later...\n", rc);
		else if (data->pdata->info.auto_cal)
			ft5x06_auto_cal(data->client);
	} else {
		dev_err(dev, "FW format error\n");
		rc = -EIO;
	}

	ft5x06_update_fw_ver(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);
rel_fw:
	release_firmware(fw);
	return rc;
}

static ssize_t ft5x06_update_fw_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, 2, "%d\n", data->loading_fw);
}

static ssize_t ft5x06_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	if (data->suspended) {
		dev_info(dev, "In suspend state, try again later...\n");
		return size;
	}

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, false);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_update_fw_store);

static ssize_t ft5x06_force_update_fw_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int rc;

	if (size > 2)
		return -EINVAL;

	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;

	mutex_lock(&data->input_dev->mutex);
	if (!data->loading_fw  && val) {
		data->loading_fw = true;
		ft5x06_fw_upgrade(dev, true);
		data->loading_fw = false;
	}
	mutex_unlock(&data->input_dev->mutex);

	return size;
}

static DEVICE_ATTR(force_update_fw, 0664, ft5x06_update_fw_show,
				ft5x06_force_update_fw_store);

static ssize_t ft5x06_fw_name_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);
	return snprintf(buf, FT_FW_NAME_MAX_LEN - 1, "%s\n", data->fw_name);
}

static ssize_t ft5x06_fw_name_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct ft5x06_ts_data *data = dev_get_drvdata(dev);

	if (size > FT_FW_NAME_MAX_LEN - 1)
		return -EINVAL;

	strlcpy(data->fw_name, buf, size);
	if (data->fw_name[size-1] == '\n')
		data->fw_name[size-1] = 0;

	return size;
}

static DEVICE_ATTR(fw_name, 0664, ft5x06_fw_name_show, ft5x06_fw_name_store);

static bool ft5x06_debug_addr_is_valid(int addr)
{
	if (addr < 0 || addr > 0xFF) {
		pr_err("FT reg address is invalid: 0x%x\n", addr);
		return false;
	}

	return true;
}

static int ft5x06_debug_data_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		dev_info(&data->client->dev,
			"Writing into FT registers not supported\n");

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_data_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;
	int rc;
	u8 reg;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr)) {
		rc = ft5x0x_read_reg(data->client, data->addr, &reg);
		if (rc < 0)
			dev_err(&data->client->dev,
				"FT read register 0x%x failed (%d)\n",
				data->addr, rc);
		else
			*val = reg;
	}

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_data_fops, ft5x06_debug_data_get,
			ft5x06_debug_data_set, "0x%02llX\n");

static int ft5x06_debug_addr_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	if (ft5x06_debug_addr_is_valid(val)) {
		mutex_lock(&data->input_dev->mutex);
		data->addr = val;
		mutex_unlock(&data->input_dev->mutex);
	}

	return 0;
}

static int ft5x06_debug_addr_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (ft5x06_debug_addr_is_valid(data->addr))
		*val = data->addr;

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_addr_fops, ft5x06_debug_addr_get,
			ft5x06_debug_addr_set, "0x%02llX\n");

static int ft5x06_debug_suspend_set(void *_data, u64 val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);

	if (val)
		ft5x06_ts_suspend(&data->client->dev);
	else
		ft5x06_ts_resume(&data->client->dev);

	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

static int ft5x06_debug_suspend_get(void *_data, u64 *val)
{
	struct ft5x06_ts_data *data = _data;

	mutex_lock(&data->input_dev->mutex);
	*val = data->suspended;
	mutex_unlock(&data->input_dev->mutex);

	return 0;
}

DEFINE_SIMPLE_ATTRIBUTE(debug_suspend_fops, ft5x06_debug_suspend_get,
			ft5x06_debug_suspend_set, "%lld\n");

static int ft5x06_debug_dump_info(struct seq_file *m, void *v)
{
	struct ft5x06_ts_data *data = m->private;

	seq_printf(m, "%s\n", data->ts_info);

	return 0;
}

static int debugfs_dump_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, ft5x06_debug_dump_info, inode->i_private);
}

static const struct file_operations debug_dump_info_fops = {
	.owner		= THIS_MODULE,
	.open		= debugfs_dump_info_open,
	.read		= seq_read,
	.release	= single_release,
};

#ifdef CONFIG_OF
static int ft5x06_get_dt_coords(struct device *dev, char *name,
				struct ft5x06_ts_platform_data *pdata)
{
	u32 coords[FT_COORDS_ARR_SIZE];
	struct property *prop;
	struct device_node *np = dev->of_node;
	int coords_size, rc;

	prop = of_find_property(np, name, NULL);
	if (!prop)
		return -EINVAL;
	if (!prop->value)
		return -ENODATA;

	coords_size = prop->length / sizeof(u32);
	if (coords_size != FT_COORDS_ARR_SIZE) {
		dev_err(dev, "invalid %s\n", name);
		return -EINVAL;
	}

	rc = of_property_read_u32_array(np, name, coords, coords_size);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read %s\n", name);
		return rc;
	}

	if (!strcmp(name, "focaltech,panel-coords")) {
		pdata->panel_minx = coords[0];
		pdata->panel_miny = coords[1];
		pdata->panel_maxx = coords[2];
		pdata->panel_maxy = coords[3];
	} else if (!strcmp(name, "focaltech,display-coords")) {
		pdata->x_min = coords[0];
		pdata->y_min = coords[1];
		pdata->x_max = coords[2];
		pdata->y_max = coords[3];
	} else {
		dev_err(dev, "unsupported property %s\n", name);
		return -EINVAL;
	}

	return 0;
}

static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	int rc;
	struct device_node *np = dev->of_node;
	struct property *prop;
	u32 temp_val, num_buttons;
	u32 button_map[MAX_BUTTONS];

	pdata->name = "focaltech";
	rc = of_property_read_string(np, "focaltech,name", &pdata->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read name\n");
		return rc;
	}

	rc = ft5x06_get_dt_coords(dev, "focaltech,panel-coords", pdata);
	if (rc && (rc != -EINVAL))
		return rc;

	rc = ft5x06_get_dt_coords(dev, "focaltech,display-coords", pdata);
	if (rc)
		return rc;

	pdata->i2c_pull_up = of_property_read_bool(np,
						"focaltech,i2c-pull-up");

	pdata->no_force_update = of_property_read_bool(np,
						"focaltech,no-force-update");
	/* reset, irq gpio info */
	pdata->reset_gpio = of_get_named_gpio_flags(np, "focaltech,reset-gpio",
				0, &pdata->reset_gpio_flags);
	if (pdata->reset_gpio < 0)
		return pdata->reset_gpio;

	pdata->irq_gpio = of_get_named_gpio_flags(np, "focaltech,irq-gpio",
				0, &pdata->irq_gpio_flags);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
#ifdef UNISCOPE_DRIVER_DTV //liguowei@uniscope 20140722 support siano dtv
	pdata->wlt_gpio = of_get_named_gpio_flags(np, "focaltech,wlt-gpio",
				0, &pdata->wlt_gpio_flags);
	if (pdata->wlt_gpio < 0)
		return pdata->wlt_gpio;

	pdata->vcc18_gpio = of_get_named_gpio_flags(np, "focaltech,vcc18-gpio",
				0, &pdata->vcc18_gpio_flags);
	if (pdata->vcc18_gpio < 0)
		return pdata->vcc18_gpio;

	pdata->vccio_gpio = of_get_named_gpio_flags(np, "focaltech,vccio-gpio",
				0, &pdata->vccio_gpio_flags);
	if (pdata->vccio_gpio < 0)
		return pdata->vccio_gpio;

	pdata->dtvreset_gpio = of_get_named_gpio_flags(np, "focaltech,dtvreset-gpio",
				0, &pdata->dtvreset_gpio_flags);
	if (pdata->dtvreset_gpio < 0)
		return pdata->dtvreset_gpio;
	pdata->dtvinterrupt_gpio = of_get_named_gpio_flags(np, "focaltech,dtvinterrupt-gpio",
				0, &pdata->dtvinterrupt_gpio_flags);
	if (pdata->dtvinterrupt_gpio < 0)
		return pdata->dtvinterrupt_gpio;
#endif
	pdata->fw_name = "ft_fw.bin";
	rc = of_property_read_string(np, "focaltech,fw-name", &pdata->fw_name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw name\n");
		return rc;
	}

	rc = of_property_read_u32(np, "focaltech,group-id", &temp_val);
	if (!rc)
		pdata->group_id = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,hard-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->hard_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,soft-reset-delay-ms",
							&temp_val);
	if (!rc)
		pdata->soft_rst_dly = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,num-max-touches", &temp_val);
	if (!rc)
		pdata->num_max_touches = temp_val;
	else
		return rc;

	rc = of_property_read_u32(np, "focaltech,fw-delay-aa-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay aa\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_aa =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-55-ms", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay 55\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_55 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id1", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id1\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_1 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-upgrade-id2", &temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw upgrade id2\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.upgrade_id_2 =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-readid-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay read id\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_readid =  temp_val;

	rc = of_property_read_u32(np, "focaltech,fw-delay-era-flsh-ms",
							&temp_val);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "Unable to read fw delay erase flash\n");
		return rc;
	} else if (rc != -EINVAL)
		pdata->info.delay_erase_flash =  temp_val;

	pdata->info.auto_cal = of_property_read_bool(np,
					"focaltech,fw-auto-cal");

	pdata->fw_vkey_support = of_property_read_bool(np,
						"focaltech,fw-vkey-support");

	pdata->ignore_id_check = of_property_read_bool(np,
						"focaltech,ignore-id-check");

	rc = of_property_read_u32(np, "focaltech,family-id", &temp_val);
	if (!rc)
		pdata->family_id = temp_val;
	else
		return rc;

	prop = of_find_property(np, "focaltech,button-map", NULL);
	if (prop) {
		num_buttons = prop->length / sizeof(temp_val);
		if (num_buttons > MAX_BUTTONS)
			return -EINVAL;

		rc = of_property_read_u32_array(np,
			"focaltech,button-map", button_map,
			num_buttons);
		if (rc) {
			dev_err(dev, "Unable to read key codes\n");
			return rc;
		}
	}

	return 0;
}
#else
static int ft5x06_parse_dt(struct device *dev,
			struct ft5x06_ts_platform_data *pdata)
{
	return -ENODEV;
}
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
static ssize_t  log_tp_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 2, "%d \n", uniscope_debug_flag);
}

static ssize_t log_tp_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long  val = 0;
	int rc =0;
	
	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;
	pr_err("%s code = %lu  \n",__func__,val);
	if(val > 0)
		uniscope_debug_flag = 1;
	else
		uniscope_debug_flag = 0;
	return count;
}

static DEVICE_ATTR(log_tp, 0664, log_tp_show, log_tp_store);
#endif

#ifdef UNISCOPE_DRIVER_L700  //liguowei@uniscope.com 20141126
static ssize_t  uni_chg_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	return snprintf(buf, 2, "%d \n", uniscope_imobile_charger_flag);
}

static ssize_t uni_chg_store(struct device *dev,
				struct device_attribute *attr,
				const char *buf, size_t count)
{
	unsigned long  val = 0;
	int rc =0;
	
	rc = kstrtoul(buf, 10, &val);
	if (rc != 0)
		return rc;
	pr_err("%s code = %lu  \n",__func__,val);
	if(val > 0)
		uniscope_imobile_charger_flag = 1;
	else
		uniscope_imobile_charger_flag = 0;
	return count;
}

static DEVICE_ATTR(uni_chg, 0664, uni_chg_show, uni_chg_store);
#endif
/* Added by JZZ(zhizhang)@uniscope_drv 20140918 */
#if defined(UNISCOPE_DRIVER_TP_AUTOMATCH_RESOLUTION)
extern void get_mdss_dsi_panel_resolution(u32 *x, u32 *y);
#endif
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
/*******************************************************
Function:
	Request interrupt.
Input:
	ts: private data.
Output:
	Executive outcomes.
	0: succeed, -1: failed.
*******************************************************/
static int ft5x06_request_irq(struct ft5x06_ts_data *ts)
{
	int ret = 0;

	ret = request_threaded_irq(ts->client->irq, NULL,
			ft5x06_ts_interrupt,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			ts->client->dev.driver->name, ts);
	if (ret) {
		ts->use_irq = false;
		return ret;
	} else {
		ft5x06_irq_disable(ts);
		ts->use_irq = true;
		return ret;
	}
}
#endif

static int ft5x06_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct ft5x06_ts_platform_data *pdata;
	struct ft5x06_ts_data *data;
	struct input_dev *input_dev;
	struct dentry *temp;
	u8 reg_value;
	u8 reg_addr;
	int err, len;

#if defined TPD_AUTO_UPGRADE
    int ret_auto_upgrade = 0;
    int i;
    update_client = client;
#endif
	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_platform_data), GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		err = ft5x06_parse_dt(&client->dev, pdata);
		if (err) {
			dev_err(&client->dev, "DT parsing failed\n");
			return err;
		}
	} else
		pdata = client->dev.platform_data;

	if (!pdata) {
		dev_err(&client->dev, "Invalid pdata\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "I2C not supported\n");
		return -ENODEV;
	}

	data = devm_kzalloc(&client->dev,
			sizeof(struct ft5x06_ts_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	if (pdata->fw_name) {
		len = strlen(pdata->fw_name);
		if (len > FT_FW_NAME_MAX_LEN - 1) {
			dev_err(&client->dev, "Invalid firmware name\n");
			return -EINVAL;
		}

		strlcpy(data->fw_name, pdata->fw_name, len + 1);
	}

	data->tch_data_len = FT_TCH_LEN(pdata->num_max_touches);
	data->tch_data = devm_kzalloc(&client->dev,
				data->tch_data_len, GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Not enough memory\n");
		return -ENOMEM;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev, "failed to allocate input device\n");
		return -ENOMEM;
	}

	data->input_dev = input_dev;
	data->client = client;
	data->pdata = pdata;

	input_dev->name = "ft5x06_ts";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes	
	/* For 2.6.39 & later use spin_lock_init(&ts->irq_lock)
	 * For 2.6.39 & before, use ts->irq_lock = SPIN_LOCK_UNLOCKED
	 */
	spin_lock_init(&data->irq_lock);
#endif

	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, pdata->num_max_touches, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);

	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev, "Input device registration failed\n");
		goto free_inputdev;
	}
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	data->power_on = false;
	err = ft5x06_power_init(data);
	if (err) {
		dev_err(&client->dev, "ft5x06 power init failed\n");
			goto unreg_inputdev;
	}

	err = ft5x06_power_on(data);
	if (err) {
		dev_err(&client->dev, "ft5x06 power on failed\n");
			goto pwr_deinit;
	}
#else
	if (pdata->power_init) {
		err = pdata->power_init(true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	} else {
		err = ft5x06_power_init(data, true);
		if (err) {
			dev_err(&client->dev, "power init failed");
			goto unreg_inputdev;
		}
	}

	if (pdata->power_on) {
		err = pdata->power_on(true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	} else {
		err = ft5x06_power_on(data, true);
		if (err) {
			dev_err(&client->dev, "power on failed");
			goto pwr_deinit;
		}
	}
#endif
	err = ft5x06_ts_pinctrl_init(data);
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	if (!err && data->ts_pinctrl) {
		err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_active);
		if (err < 0) {
			dev_err(&client->dev,
				"failed to select pin to active state");
			goto pinctrl_deinit;
		}
	} else {
			goto pwr_off;
	}

	if (gpio_is_valid(pdata->irq_gpio)) {
		err = gpio_request(pdata->irq_gpio, "ft5x06_irq_gpio");
		if (err) {
			dev_err(&client->dev, "irq gpio request failed");
			goto err_gpio_req;
		}
		err = gpio_direction_input(pdata->irq_gpio);
		if (err) {
			dev_err(&client->dev,
				"set_direction for irq gpio failed\n");
			goto free_irq_gpio;
		}
	}
#endif
	if (gpio_is_valid(pdata->reset_gpio)) {
		err = gpio_request(pdata->reset_gpio, "ft5x06_reset_gpio");
		if (err) {
			dev_err(&client->dev, "reset gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->reset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_reset_gpio;
		}
		msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->reset_gpio, 1);
	}

#ifdef UNISCOPE_DRIVER_DTV //liguowei@uniscope 20140722 support siano dtv
if (gpio_is_valid(pdata->wlt_gpio)) {
		err = gpio_request(pdata->wlt_gpio, "ft5x06_wlt_gpio");
		if (err) {
			dev_err(&client->dev, "wlt gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->wlt_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_wlt_gpio;
		}
		//leep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->wlt_gpio, 0);
	}

if (gpio_is_valid(pdata->vcc18_gpio)) {
		err = gpio_request(pdata->vcc18_gpio, "ft5x06_vcc18_gpio");
		if (err) {
			dev_err(&client->dev, "wlt gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->vcc18_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_vcc18_gpio;
		}
		//leep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->vcc18_gpio, 0);
	}
if (gpio_is_valid(pdata->vccio_gpio)) {
		err = gpio_request(pdata->vccio_gpio, "ft5x06_vccio_gpio");
		if (err) {
			dev_err(&client->dev, "wlt gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->vccio_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_vccio_gpio;
		}
		//leep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->vccio_gpio, 0);
	}
if (gpio_is_valid(pdata->dtvreset_gpio)) {
		err = gpio_request(pdata->dtvreset_gpio, "ft5x06_dtvreset_gpio");
		if (err) {
			dev_err(&client->dev, "wlt gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->dtvreset_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for reset gpio failed\n");
			goto free_dtvreset_gpio;
		}
		//msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->dtvreset_gpio, 0);
	}
if (gpio_is_valid(pdata->dtvinterrupt_gpio)) {
		err = gpio_request(pdata->dtvinterrupt_gpio, "ft5x06_dtvinterrupt_gpio");
		if (err) {
			dev_err(&client->dev, "dtvinterrupt gpio request failed");
			goto free_irq_gpio;
		}

		err = gpio_direction_output(pdata->dtvinterrupt_gpio, 0);
		if (err) {
			dev_err(&client->dev,
				"set_direction for dtvinterrupt gpio  failed\n");
			goto free_dtvinterrupt_gpio;
		}
		//msleep(data->pdata->hard_rst_dly);
		gpio_set_value_cansleep(data->pdata->dtvinterrupt_gpio, 1);
	}

  if(gpio_is_valid(971)) //gpio69 connect to SD gpio_cd gpio38
    {
        gpio_set_value(971,1);
        dev_err(&client->dev,  "dtvinterrupt_gpio   gpio \n");
    }
#if 0 //byj 

dev_err(&client->dev, "zzzzzz   pdata->reset_gpio = %d \n", pdata->irq_gpio); //13   -915

     /*byj */
    if(gpio_is_valid(938)) //vcc_1.8
    {
    gpio_set_value(938,1);
dev_err(&client->dev, "zzzzzz    938\n");
    }
    
 
    if(gpio_is_valid(953)) //vcc_1.8
    {
    gpio_set_value(953,1);
dev_err(&client->dev,  "zzzzzz    953\n");
    }
   
    
    if(gpio_is_valid(954)) //vcc_1.8
    {
    gpio_set_value(954,1);
dev_err(&client->dev,  "zzzzzz    954\n");
    }
    
    if(gpio_is_valid(1020)) //vcc_1.8
    {
        gpio_set_value(1020,1);
dev_err(&client->dev, "zzzzzz    1020 \n");
        gpio_set_value(1020,0);
            msleep(50);
        gpio_set_value(1020,1);
    }

#endif

#endif
	/* make sure CTP already finish startup process */
	msleep(data->pdata->soft_rst_dly);

	/* check the controller id */
	reg_addr = FT_REG_ID;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0) {
		dev_err(&client->dev, "version read failed");
		goto free_reset_gpio;
	}
	/* Added by JZZ(zhizhang)@uniscope_drv 20140918 begin */
#if defined(UNISCOPE_DRIVER_TP_AUTOMATCH_RESOLUTION)
	{
		u32 disp_x=0;
		u32 disp_y=0;
		get_mdss_dsi_panel_resolution(&disp_x,&disp_y);
		pdata->x_max = disp_x;
		pdata->y_max = disp_y;	
		input_set_abs_params(input_dev, ABS_MT_POSITION_X, pdata->x_min,
			     pdata->x_max, 0, 0);
		input_set_abs_params(input_dev, ABS_MT_POSITION_Y, pdata->y_min,
			     pdata->y_max, 0, 0);	
	}
#endif
	/* Added by JZZ(zhizhang)@uniscope_drv 20140918 end */
	dev_info(&client->dev, "Device ID = 0x%x\n", reg_value);

	if ((pdata->family_id != reg_value) && (!pdata->ignore_id_check)) {
		dev_err(&client->dev, "%s:Unsupported controller\n", __func__);
		goto free_reset_gpio;
	}

	data->family_id = pdata->family_id;
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	err = ft5x06_request_irq(data);
	if (err)
		{
		dev_err(&client->dev, "ft5x06 request irq failed %d.\n", err);
		goto free_reset_gpio;
	}
	else
		dev_err(&client->dev, "ft5x06 works in interrupt mode.\n");

       if (data->use_irq)
		     ft5x06_irq_enable(data);	
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
      ft5x06_wq = create_singlethread_workqueue("ft5x06_wq");
      INIT_WORK(&data->work, ft5x0x_read_Touchdata_Work);	

	err = device_create_file(&client->dev, &dev_attr_log_tp);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}  
#endif

#ifdef UNISCOPE_DRIVER_L700  //liguowei@uniscope.com 20141126
	err = device_create_file(&client->dev, &dev_attr_uni_chg);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}  
#endif

	err = device_create_file(&client->dev, &dev_attr_fw_name);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto irq_free;
	}

	err = device_create_file(&client->dev, &dev_attr_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_fw_name_sys;
	}

	err = device_create_file(&client->dev, &dev_attr_force_update_fw);
	if (err) {
		dev_err(&client->dev, "sys file creation failed\n");
		goto free_update_fw_sys;
	}

	data->dir = debugfs_create_dir(FT_DEBUG_DIR_NAME, NULL);
	if (data->dir == NULL || IS_ERR(data->dir)) {
		pr_err("debugfs_create_dir failed(%ld)\n", PTR_ERR(data->dir));
		err = PTR_ERR(data->dir);
		goto free_force_update_fw_sys;
	}

	temp = debugfs_create_file("addr", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_addr_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("data", S_IRUSR | S_IWUSR, data->dir, data,
				   &debug_data_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("suspend", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_suspend_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	temp = debugfs_create_file("dump_info", S_IRUSR | S_IWUSR, data->dir,
					data, &debug_dump_info_fops);
	if (temp == NULL || IS_ERR(temp)) {
		pr_err("debugfs_create_file failed: rc=%ld\n", PTR_ERR(temp));
		err = PTR_ERR(temp);
		goto free_debug_dir;
	}

	data->ts_info = devm_kzalloc(&client->dev,
				FT_INFO_MAX_LEN, GFP_KERNEL);
	if (!data->ts_info) {
		dev_err(&client->dev, "Not enough memory\n");
		goto free_debug_dir;
	}

	/*get some register information */
	reg_addr = FT_REG_POINT_RATE;
	ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "report rate read failed");

	dev_info(&client->dev, "report rate = %dHz\n", reg_value * 10);

	reg_addr = FT_REG_THGROUP;
	err = ft5x06_i2c_read(client, &reg_addr, 1, &reg_value, 1);
	if (err < 0)
		dev_err(&client->dev, "threshold read failed");

	dev_dbg(&client->dev, "touch threshold = %d\n", reg_value * 4);

	ft5x06_update_fw_ver(data);
	ft5x06_update_fw_vendor_id(data);

	FT_STORE_TS_INFO(data->ts_info, data->family_id, data->pdata->name,
			data->pdata->num_max_touches, data->pdata->group_id,
			data->pdata->fw_vkey_support ? "yes" : "no",
			data->pdata->fw_name, data->fw_ver[0],
			data->fw_ver[1], data->fw_ver[2]);

#if defined(CONFIG_FB)
	data->fb_notif.notifier_call = fb_notifier_callback;

	err = fb_register_client(&data->fb_notif);

	if (err)
		dev_err(&client->dev, "Unable to register fb_notifier: %d\n",
			err);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN +
						    FT_SUSPEND_LEVEL;
	data->early_suspend.suspend = ft5x06_ts_early_suspend;
	data->early_suspend.resume = ft5x06_ts_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade begin*/
#if defined TPD_AUTO_UPGRADE
    {
        pr_err("********************Enter CTP Auto Upgrade********************\n");
        msleep(50);
        i = 0;
        do
        {
            ret_auto_upgrade = fts_ctpm_fw_upgrade_with_i_file(data);
            i++;
            if(ret_auto_upgrade < 0)
            {
                pr_err(" ctp upgrade fail err = %d \n",ret_auto_upgrade);
            }
        }
        while((ret_auto_upgrade < 0)&&(i<3));
    }
#endif
/*kangyan@uniscope_drv 20141125 add tp auto upgrade end*/
	return 0;

free_debug_dir:
	debugfs_remove_recursive(data->dir);
free_force_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
free_update_fw_sys:
	device_remove_file(&client->dev, &dev_attr_update_fw);
free_fw_name_sys:
	device_remove_file(&client->dev, &dev_attr_fw_name);
irq_free:
	free_irq(client->irq, data);
free_reset_gpio:
	if (gpio_is_valid(pdata->reset_gpio))
		gpio_free(pdata->reset_gpio);
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
free_irq_gpio:
	if (gpio_is_valid(pdata->irq_gpio))
		gpio_free(pdata->irq_gpio);
err_gpio_req:
pinctrl_deinit:
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			err = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
			if (err)
				pr_err("failed to select relase pinctrl state\n");
		}
	}
#endif
#ifdef UNISCOPE_DRIVER_DTV //liguowei@uniscope 20140722 support siano dtv
free_wlt_gpio:
	if (gpio_is_valid(pdata->wlt_gpio))
		gpio_free(pdata->wlt_gpio);
free_vcc18_gpio:
	if (gpio_is_valid(pdata->vcc18_gpio))
		gpio_free(pdata->vcc18_gpio);
free_vccio_gpio:
	if (gpio_is_valid(pdata->vccio_gpio))
		gpio_free(pdata->vccio_gpio);
free_dtvreset_gpio:
	if (gpio_is_valid(pdata->dtvreset_gpio))
		gpio_free(pdata->dtvreset_gpio);
free_dtvinterrupt_gpio:
	if (gpio_is_valid(pdata->dtvinterrupt_gpio))
		gpio_free(pdata->dtvinterrupt_gpio);
#endif
#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
pwr_off:
		ft5x06_power_on(data);
pwr_deinit:
		ft5x06_power_init(data);
#else
pwr_off:
	if (pdata->power_on)
		pdata->power_on(false);
	else
		ft5x06_power_on(data, false);
pwr_deinit:
	if (pdata->power_init)
		pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
#endif
unreg_inputdev:
	input_unregister_device(input_dev);
	input_dev = NULL;
free_inputdev:
	input_free_device(input_dev);
	return err;
}

static int ft5x06_ts_remove(struct i2c_client *client)
{
	struct ft5x06_ts_data *data = i2c_get_clientdata(client);
	int retval;

	debugfs_remove_recursive(data->dir);
	device_remove_file(&client->dev, &dev_attr_force_update_fw);
	device_remove_file(&client->dev, &dev_attr_update_fw);
	device_remove_file(&client->dev, &dev_attr_fw_name);

#if defined(CONFIG_FB)
	if (fb_unregister_client(&data->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#elif defined(CONFIG_HAS_EARLYSUSPEND)
	unregister_early_suspend(&data->early_suspend);
#endif
	free_irq(client->irq, data);

	if (gpio_is_valid(data->pdata->reset_gpio))
		gpio_free(data->pdata->reset_gpio);

	if (gpio_is_valid(data->pdata->irq_gpio))
		gpio_free(data->pdata->irq_gpio);
#ifdef UNISCOPE_DRIVER_DTV //liguowei@uniscope 20140722 support siano dtv
	if (gpio_is_valid(data->pdata->wlt_gpio))
		gpio_free(data->pdata->wlt_gpio);
        if (gpio_is_valid(data->pdata->vcc18_gpio))
		gpio_free(data->pdata->vcc18_gpio);
        if (gpio_is_valid(data->pdata->vccio_gpio))
		gpio_free(data->pdata->vccio_gpio);
        if (gpio_is_valid(data->pdata->dtvreset_gpio))
		gpio_free(data->pdata->dtvreset_gpio);
        if (gpio_is_valid(data->pdata->dtvinterrupt_gpio))
		gpio_free(data->pdata->dtvinterrupt_gpio);
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20150107 tp can not work sometimes
	if (data->ts_pinctrl) {
		if (IS_ERR_OR_NULL(data->pinctrl_state_release)) {
			devm_pinctrl_put(data->ts_pinctrl);
			data->ts_pinctrl = NULL;
		} else {
			retval = pinctrl_select_state(data->ts_pinctrl,
					data->pinctrl_state_release);
		if (retval < 0)
				pr_err("failed to select release pinctrl state\n");
	}
	}
#endif

#ifdef UNISCOPE_DRIVER_QC8909  //liguowei@uniscope.com 20140703
	ft5x06_power_on(data);

	ft5x06_power_init(data);
#else
	if (data->pdata->power_on)
		data->pdata->power_on(false);
	else
		ft5x06_power_on(data, false);

	if (data->pdata->power_init)
		data->pdata->power_init(false);
	else
		ft5x06_power_init(data, false);
#endif
	input_unregister_device(data->input_dev);

	return 0;
}

static const struct i2c_device_id ft5x06_ts_id[] = {
	{"ft5x06_ts", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ft5x06_ts_id);

#ifdef CONFIG_OF
static struct of_device_id ft5x06_match_table[] = {
	{ .compatible = "focaltech,5x06",},
	{ },
};
#else
#define ft5x06_match_table NULL
#endif

static struct i2c_driver ft5x06_ts_driver = {
	.probe = ft5x06_ts_probe,
	.remove = ft5x06_ts_remove,
	.driver = {
		   .name = "ft5x06_ts",
		   .owner = THIS_MODULE,
		.of_match_table = ft5x06_match_table,
#ifdef CONFIG_PM
		   .pm = &ft5x06_ts_pm_ops,
#endif
		   },
	.id_table = ft5x06_ts_id,
};

static int __init ft5x06_ts_init(void)
{
	return i2c_add_driver(&ft5x06_ts_driver);
}
module_init(ft5x06_ts_init);

static void __exit ft5x06_ts_exit(void)
{
	i2c_del_driver(&ft5x06_ts_driver);
}
module_exit(ft5x06_ts_exit);

MODULE_DESCRIPTION("FocalTech ft5x06 TouchScreen driver");
MODULE_LICENSE("GPL v2");
