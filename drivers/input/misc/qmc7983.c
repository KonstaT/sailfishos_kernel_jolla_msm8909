/**********uniscope-driver-modify-file-on-qualcomm-platform*****************/
/*   This software is licensed under the terms of the GNU General Public License version 2,
*   as published by the Free Software Foundation,
*	and may be copied, distributed, and
*   modified under those terms.
*	This program is distributed in the hope that it will be useful,
*	but WITHOUT ANY
*   WARRANTY; without even the implied warranty of MERCHANTABILITY or
*	FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
*   for more details.
*
*	Copyright (C) 2012-2014 by QST(Shanghai XiRui Keji) Corporation
****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/uaccess.h>
#include <linux/input.h>
#include <linux/errno.h>
#include <linux/completion.h>
#include <linux/workqueue.h>
#include <linux/qmc7983.h>
#include <linux/miscdevice.h>
#include <linux/delay.h>
#include <linux/sensors.h>
#include <linux/regulator/consumer.h>
#include <linux/of_gpio.h>

#define QMC7983_MAJOR	101
#define QMC7983_MINOR	4

/* Magnetometer registers */
#define CTL_REG_ONE	0x09  /* Contrl register one */
#define CTL_REG_TWO	0x0a  /* Contrl register two */

/* Output register start address*/
#define OUT_X_REG		0x00

/*Status registers */
#define STA_REG_ONE    0x06
#define STA_REG_TWO    0x0c

/* Temperature registers */
#define TEMP_H_REG		0x08
#define TEMP_L_REG		0x07

/*different from qmc7983,the ratio register*/
#define RATIO_REG		0x0b

/* POWER SUPPLY VOLTAGE RANGE */
#define QMC7983_VDD_MIN_UV	2160000
#define QMC7983_VDD_MAX_UV	3600000
#define QMC7983_VIO_MIN_UV	1650000
#define QMC7983_VIO_MAX_UV	3600000

#define QCOM_PLATFORM
#define	QMC7983_BUFSIZE		0x20

#define ACC_OFFSET     0
#define QMC_OFFSET     3
#define QMC_CAL_OFFSET 7
#define ORI_OFFSET     11
short all_data[15];
short reg_data[15];
char qst_layout = 0;
/*
 * QMC7983 magnetometer data
 * brief Structure containing magnetic field values for x,y and z-axis in
 * signed short
*/

struct QMC7983_t {
	short	x, /**< x-axis magnetic field data. Range -8000 to 8000. */
			y, /**< y-axis magnetic field data. Range -8000 to 8000. */
			z; /**< z-axis magnetic filed data. Range -8000 to 8000. */
};

/* Save last device state for power down */
struct qmc_sensor_state {
	bool power_on;
	uint8_t mode;
};

struct QMC7983_data {
	struct i2c_client				*client;
	struct QMC7983_platform_data	*pdata;
	short						xy_sensitivity;
	short						z_sensitivity;
	struct mutex					lock;
	struct delayed_work			work;
	struct input_dev				*input;
	struct miscdevice				qmc_misc;
	struct sensors_classdev			cdev;
	int							delay_ms;
	int							enabled;
	struct completion				data_updated;
	wait_queue_head_t				state_wq;
	struct regulator				*vdd;
	struct regulator				*vio;
	struct qmc_sensor_state			state;
	int enable_sensor;
};

static struct class *qmc_mag_dev_class;

static struct QMC7983_data *qmc7983;

static struct sensors_classdev sensors_cdev = {
	.name = "qmc7983-mag",
	.vendor = "QST",
	.version = 1,
	.handle = SENSORS_MAGNETIC_FIELD_HANDLE,
	.type = SENSOR_TYPE_MAGNETIC_FIELD,
	.max_range = "1228.8",
	.resolution = "1.0",
	.sensor_power = "0.35",
	.min_delay = 10000,
	.fifo_reserved_event_count = 0,
	.fifo_max_event_count = 0,
	.enabled = 0,
	.delay_msec = 10000,
	.sensors_enable = NULL,
	.sensors_poll_delay = NULL,
};


static char QMC7983_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len);

static char QMC7983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len);

static int QMC7983_read_mag_xyz(struct QMC7983_t *data);
static int qmc7983_power_set(struct QMC7983_data *data, bool on);
static int qmc7983_power_init(struct QMC7983_data *data, bool on);

/* 7983 Self Test data collection */
int QMC7983_self_test(char mode, short *buf)
{
	return 0;
}

/* X,Y and Z-axis magnetometer data readout
 * param *mag pointer to \ref QMC7983_t structure for x,y,z data readout
 * note data will be read by multi-byte protocol into a 6 byte structure
 */
int QMC7983_read_mag_xyz(struct QMC7983_t *data)
{
	int res;
	unsigned char mag_data[6];
	int hw_d[3] = { 0 };
	short tmp;

	res = QMC7983_i2c_read(OUT_X_REG, mag_data, 6);

	hw_d[0] = (short) (((mag_data[1]) << 8) | mag_data[0]);
	hw_d[1] = (short) (((mag_data[3]) << 8) | mag_data[2]);
	hw_d[2] = (short) (((mag_data[5]) << 8) | mag_data[4]);


	hw_d[0] = hw_d[0] * 1000 / qmc7983->xy_sensitivity;
	hw_d[1] = hw_d[1] * 1000 / qmc7983->xy_sensitivity;
	hw_d[2] = hw_d[2] * 1000 / qmc7983->z_sensitivity;

	data->x = hw_d[0];
	data->y = hw_d[1];
	data->z = hw_d[2];


	switch (qst_layout) {
	case 0:
	case 1:
		tmp = data->x;
		data->x = -data->y;
		data->y = tmp;
		break;
	case 2:
		break;
	case 3:
		tmp = data->x;
		data->x =  data->y;
		data->y = -tmp;
		break;
	case 4:

		data->x = -data->x;
		data->y = -data->y;
		break;
		
	case 5:
		tmp = data->x;
		data->x = data->y;
		data->y = tmp;
		data->z = -data->z;
		break;
	case 6:
		
		data->y = -data->y;
		data->z = -data->z;
		break;
	case 7:
		tmp = data->x;
		data->x = -data->y;
		data->y = -tmp;
		data->z = -data->z;
		break;
	case 8:

		data->x = -data->x;
		data->z = -data->z;
		break;
	}
	//pr_err("%s: data x y z is %d, %d, %d  , layout  = %d \n", __func__,data->x,data->y,data->z,qst_layout);

	return res;
}

/* Set the Gain range */
int QMC7983_set_range(short range)
{
	int err = 0;
	int ran;
	switch (range) {
	case QMC7983_RNG_2G:
		ran = RNG_2G;
		break;
	case QMC7983_RNG_8G:
		ran = RNG_8G;
		break;
	case QMC7983_RNG_12G:
		ran = RNG_12G;
		break;
	case QMC7983_RNG_20G:
		ran = RNG_20G;
		break;
	default:
		return -EINVAL;
	}
	qmc7983->xy_sensitivity = 16000/ran;
	qmc7983->z_sensitivity = 16000/ran;

	return err;
}

/* Set the sensor mode */
int QMC7983_set_mode(char mode)
{
	int err = 0;
	unsigned char data;

	QMC7983_i2c_read(CTL_REG_ONE, &data, 1);
	data &= 0xfc;
	data |= mode;
	err = QMC7983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

int QMC7983_set_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	err = QMC7983_i2c_write(RATIO_REG, &data, 1);
	return err;
}

int QMC7983_set_output_data_rate(char rate)
{
	int err = 0;
	unsigned char data;

	data = rate;
	data &= 0xf3;
	data |= (rate << 2);
	err = QMC7983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

int QMC7983_set_oversample_ratio(char ratio)
{
	int err = 0;
	unsigned char data;

	data = ratio;
	data &= 0x3f;
	data |= (ratio << 6);
	err = QMC7983_i2c_write(CTL_REG_ONE, &data, 1);
	return err;
}

/*  i2c write routine for qmc7983 magnetometer */
static char QMC7983_i2c_write(unsigned char reg_addr,
				    unsigned char *data,
				    unsigned char len)
{
	int dummy;
	int i;

	if (qmc7983->client == NULL)  /*  No global client pointer? */
		return -ENODEV;
	for (i = 0; i < len; i++) {
		dummy = i2c_smbus_write_byte_data(qmc7983->client,
						  reg_addr++, data[i]);
		if (dummy) {
			pr_info("%s:i2c write error\n", __func__);
			return dummy;
		}
	}
	return 0;
}

/*  i2c read routine for QMC7983 magnetometer */
static char QMC7983_i2c_read(unsigned char reg_addr,
				   unsigned char *data,
				   unsigned char len)
{
	char dummy = 0;
	int i = 0;

	if (qmc7983->client == NULL)  /*  No global client pointer? */
		return -ENODEV;

	while (i < len) {
		dummy = i2c_smbus_read_byte_data(qmc7983->client,
				reg_addr++);
		if (dummy >= 0) {
			data[i] = dummy;
			i++;
		} else
		{
			return dummy;
		}
		dummy = len;
	}
	return dummy;
}

static void qmc7983_start_measure(struct QMC7983_data *qmc7983)
{
	int err = 0;
	unsigned char data;

	data = 0x0c;
	err = QMC7983_i2c_write(0x0a, &data, 1);

	data = 0x7f;
	err = QMC7983_i2c_write(0x20, &data, 1);


	data = 0x80;
	err = QMC7983_i2c_write(0x29, &data, 1);

	
	data = 0x1d;
	err = QMC7983_i2c_write(CTL_REG_ONE, &data, 1);
}

static void qmc7983_stop_measure(struct QMC7983_data *qmc7983)
{
	unsigned char data;
	data = 0x1c;
	QMC7983_i2c_write(CTL_REG_ONE, &data, 1);
}

static int qmc7983_enable(struct QMC7983_data *qmc7983)
{
	/* qmc7983_power_set(qmc7983, true); */
	qmc7983_start_measure(qmc7983);
	QMC7983_set_range(QMC7983_RNG_20G);
	QMC7983_set_ratio(1);
	schedule_delayed_work(&qmc7983->work,
		msecs_to_jiffies(qmc7983->delay_ms));
	return 0;
}

static int qmc7983_disable(struct QMC7983_data *qmc7983)
{
	dev_dbg(&qmc7983->client->dev, "stop measure!\n");
	/* qmc7983_power_set(qmc7983, false); */
	qmc7983_stop_measure(qmc7983);
	cancel_delayed_work(&qmc7983->work);

	return 0;
}


static void qmc7983_work(struct work_struct *work)
{
	int ret;
	struct QMC7983_data *qmc7983 = container_of((struct delayed_work *)work,
						struct QMC7983_data, work);
	unsigned char data[6];
	int needRetry = 0;
	mutex_lock(&qmc7983->lock);

	if (!qmc7983->enabled)
		goto out;

	ret = QMC7983_read_mag_xyz((struct QMC7983_t *)data);
	if (ret < 0)
		dev_err(&qmc7983->client->dev, "error read data\n");

/*TODO: Some times, sensor could send bad data
  *(x&y&z read 255 in same time) for hardware reason
  *Because we don't know the gain, so we just check
  *and retry when x==y==z
  */
	if ((((struct QMC7983_t *)data)->x ==
		((struct QMC7983_t *)data)->y)
		&& (((struct QMC7983_t *)data)->x
		== ((struct QMC7983_t *)data)->y)) {
			needRetry = 1;
		} else {
			//dev_err(&qmc7983->client->dev, "%s: report start\n",__func__);
			
			input_report_abs(qmc7983->input, ABS_X,
				((struct QMC7983_t *)data)->x);
			input_report_abs(qmc7983->input, ABS_Y,
				((struct QMC7983_t *)data)->y);
			input_report_abs(qmc7983->input, ABS_Z,
				((struct QMC7983_t *)data)->z);

		//dev_err(&qmc7983->client->dev, "%s: report x y z is  %d,%d,%d\n",__func__,((struct QMC7983_t *)data)->x,((struct QMC7983_t *)data)->y,((struct QMC7983_t *)data)->z);
				
			input_sync(qmc7983->input);
		}

	schedule_delayed_work(&qmc7983->work,
		msecs_to_jiffies(qmc7983->delay_ms));

out:
	mutex_unlock(&qmc7983->lock);
}

static int qmc7983_input_init(struct QMC7983_data *qmc7983)
{
	struct input_dev *dev;
	int ret;

	dev = input_allocate_device();
	if (!dev)
		return -ENOMEM;

	dev->name = "compass";
	dev->id.bustype = BUS_I2C;

	set_bit(EV_ABS, dev->evbit);
	/* Magnetic field (limited to 16bit) */
	input_set_abs_params(dev, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(dev, ABS_Z, -32768, 32767, 0, 0);
	ret = input_register_device(dev);

	input_set_drvdata(dev, qmc7983);
	if (ret < 0) {
		input_free_device(dev);
		return ret;
	}

	qmc7983->input = dev;
	return 0;
}

static ssize_t
attr_get_poll(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	int pollms;

	mutex_lock(&qmc7983->lock);
	pollms = qmc7983->delay_ms;
	mutex_unlock(&qmc7983->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", pollms);
}

#define QMC7983_MAX_RATE 75

static ssize_t attr_set_poll(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	unsigned long pollms = 0;

	if (kstrtoul(buf, 10, &pollms) || pollms <= 0)
		return -EINVAL;

	if (pollms < (1000 / QMC7983_MAX_RATE))
		pollms = 1000 / QMC7983_MAX_RATE + 5;

	mutex_lock(&qmc7983->lock);
	qmc7983->delay_ms = pollms;
	mutex_unlock(&qmc7983->lock);

	return size;
}
static DEVICE_ATTR(poll, S_IRUGO | S_IWUSR, attr_get_poll, attr_set_poll);

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qmc7983->lock);
	val = qmc7983->enabled;
	mutex_unlock(&qmc7983->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&qmc7983->lock);
	if (val)
		qmc7983_enable(qmc7983);
	else
		qmc7983_disable(qmc7983);

	qmc7983->enabled = val;
	mutex_unlock(&qmc7983->lock);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, attr_get_enable, attr_set_enable);

static int id = -1;
static ssize_t attr_get_idchk(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	int val;

	mutex_lock(&qmc7983->lock);
	val = id;
	mutex_unlock(&qmc7983->lock);

	return snprintf(buf, PAGE_SIZE, "%d\n", val);
}

static ssize_t attr_set_idchk(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 10, &val))
		return -EINVAL;

	dev_dbg(&qmc7983->client->dev, "val=%lu\n", val);

	mutex_lock(&qmc7983->lock);
	id = i2c_smbus_read_word_data(qmc7983->client, (int)val);
	if ((id & 0x00FF) == 0x0048)
		pr_info("%s:I2C driver registered!\n", __func__);

	id = id & 0x00FF;
	mutex_unlock(&qmc7983->lock);

	return size;
}

static DEVICE_ATTR(idchk, S_IRUGO | S_IWUSR, attr_get_idchk, attr_set_idchk);


static int qmc7983_poll_delay_set(struct sensors_classdev *sensors_cdev,
		unsigned int delay_msec)
{
	struct QMC7983_data *data = container_of(sensors_cdev,
			struct QMC7983_data, cdev);

	mutex_lock(&data->lock);
	data->delay_ms = (int)delay_msec;
	mutex_unlock(&data->lock);

	return 0;
}

static int qmc7983_enable_sensor(struct i2c_client *client, unsigned int enable)
{
	struct QMC7983_data *data = i2c_get_clientdata(client);
	int err;

	mutex_lock(&data->lock);
	if (enable) {
		if (data->enable_sensor == 0) {
			data->enable_sensor = 1;
			err = qmc7983_power_set(data, true);
			if (err < 0)
				goto exit;
			qmc7983_enable(data);
		}
	} else {
		if (data->enable_sensor == 1) {
			data->enable_sensor = 0;
			qmc7983_disable(data);
			err = qmc7983_power_set(data, false);
			if (err < 0)
				goto exit;
		}
	}

	mutex_unlock(&data->lock);

exit:
	return 0;
}
  
static int qmc7983_enable_set(struct sensors_classdev *sensors_cdev,
		unsigned int enable)
{
	struct QMC7983_data *data = container_of(sensors_cdev,
			struct QMC7983_data, cdev);

	if ((enable != 0) && (enable != 1)) {
		pr_err("%s: invalid value(%d)\n", __func__, enable);
		return -EINVAL;
	}

	return qmc7983_enable_sensor(data->client, enable);
}

/******************liufa add 20140605*************/
#ifdef QCOM_PLATFORM
static unsigned char regbuf[2] = {0};
static ssize_t show_temperature_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char strbuf[QMC7983_BUFSIZE];
	unsigned char mag_temperature[2];
	unsigned char data;
	int hw_temperature = 0;
	unsigned char rdy = 0;
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);

	/* Check status register for data availability */
	int t1 = 0;
	while (!(rdy & 0x07) && t1 < 3) {
		QMC7983_i2c_read(STA_REG_ONE, &data, 1);
		rdy = data;
		t1++;
	}

	mutex_lock(&qmc7983->lock);
	QMC7983_i2c_read(TEMP_L_REG, &data, 1);
	mag_temperature[0] = data;
	QMC7983_i2c_read(TEMP_H_REG, &data, 1);
	mag_temperature[1] = data;
	mutex_unlock(&qmc7983->lock);
	hw_temperature = ((mag_temperature[1]) << 8) | mag_temperature[0];

	snprintf(strbuf, PAGE_SIZE, "temperature = %d\n", hw_temperature);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static DEVICE_ATTR(temperature, S_IRUGO, show_temperature_value, NULL);

static ssize_t show_WRregisters_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char strbuf[QMC7983_BUFSIZE];
	unsigned char data;
	unsigned char rdy = 0;
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);

	/* Check status register for data availability */
	int t1 = 0;
	while (!(rdy & 0x07) && t1 < 3) {
		QMC7983_i2c_read(STA_REG_ONE, &data, 1);
		rdy = data;
		t1++;
	}

	mutex_lock(&qmc7983->lock);
	QMC7983_i2c_read(regbuf[0], &data, 1);
	mutex_unlock(&qmc7983->lock);
	snprintf(strbuf, PAGE_SIZE, "hw_registers = 0x%02x\n", data);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}

static ssize_t store_WRregisters_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	int err = 0;
	unsigned char data;
	if (NULL == qmc7983)
		return 0;

	mutex_lock(&qmc7983->lock);
	data = *buf;
	err = QMC7983_i2c_write(regbuf[0], &data, 1);
	mutex_unlock(&qmc7983->lock);

	return count;
}

static DEVICE_ATTR(WRregisters, S_IRUGO | S_IWUSR | S_IWGRP |
	S_IWOTH, show_WRregisters_value, store_WRregisters_value);

static ssize_t show_registers_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char strbuf[QMC7983_BUFSIZE];
	snprintf(strbuf, PAGE_SIZE, "hw_registers = 0x%02x\n", regbuf[0]);

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
static ssize_t store_registers_value(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);
	if (NULL == qmc7983)
		return 0;

	regbuf[0] = *buf;

	return count;
}
static DEVICE_ATTR(registers,   S_IRUGO | S_IWUSR | S_IWGRP |
	S_IWOTH, show_registers_value, store_registers_value);

static ssize_t show_dumpallreg_value(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	char strbuf[300];
	char tempstrbuf[24];
	unsigned char data;
	int length = 0;
	unsigned char rdy = 0;
	int i;

	/* Check status register for data availability */
	int t1 = 0;
	while (!(rdy & 0x07) && t1 < 3) {
		QMC7983_i2c_read(STA_REG_ONE, &data, 1);
		rdy = data;
		t1++;
	}

	for (i = 0; i < 12; i++) {
		QMC7983_i2c_read(i, &data, 1);
		length = snprintf(tempstrbuf, PAGE_SIZE,
			"reg[0x%2x] =  0x%2x\n", i, data);
		snprintf(strbuf+length*i, PAGE_SIZE, "%s\n", tempstrbuf);
	}

	return snprintf(buf, PAGE_SIZE, "%s\n", strbuf);
}
static DEVICE_ATTR(dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL);
#endif
/********	END	  **********/

static struct attribute *qmc7983_attributes[] = {
/*****liufa add at 20140605**************/
#ifdef QCOM_PLATFORM
	&dev_attr_dumpallreg.attr,
	&dev_attr_WRregisters.attr,
	&dev_attr_registers.attr,
	&dev_attr_temperature.attr,
#endif
/*************end********************/
	&dev_attr_enable.attr,
	&dev_attr_poll.attr,
	&dev_attr_idchk.attr,
	NULL
};

static struct attribute_group qmc7983_attr_group = {
	.name = "qmc7983",
	.attrs = qmc7983_attributes
};


static struct device_attribute attributes[] = {
/*****liufa add at 20140605**************/
#ifdef QCOM_PLATFORM
	__ATTR(dumpallreg,  S_IRUGO , show_dumpallreg_value, NULL),
	__ATTR(WRregisters, S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		show_WRregisters_value, store_WRregisters_value),
	__ATTR(registers,   S_IRUGO | S_IWUSR | S_IWGRP | S_IWOTH,
		show_registers_value, store_registers_value),
	__ATTR(temperature, S_IRUGO, show_temperature_value, NULL),
#endif
/*************end****************/
	__ATTR(pollrate_ms, S_IRUGO | S_IWUSR, attr_get_poll, attr_set_poll),
	__ATTR(enable, S_IRUGO | S_IWUSR, attr_get_enable, attr_set_enable),
	__ATTR(idchk, S_IRUGO | S_IWUSR, attr_get_idchk, attr_set_idchk),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++) {
		if (device_create_file(dev, attributes + i))
			goto error;
	}
	return 0;

error:
	for (; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -ENODEV;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

/*  ioctl command for QMC7983 device file */
static long qmc_misc_ioctl(struct file *file,
	unsigned int cmd, unsigned long arg)
{
	int err = 0;
	unsigned char data[6];

	/* check QMC7983_client */
	if (&qmc7983->client == NULL)
		return -EFAULT;

	switch (cmd) {
	case QMC7983_SET_RANGE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0)
			return -EFAULT;

		err = QMC7983_set_range(*data);
		return err;

	case QMC7983_SET_MODE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0)
			return -EFAULT;
		err = QMC7983_set_mode(data[0]);
		return err;

	case QMC7983_READ_MAGN_XYZ:
		err = QMC7983_read_mag_xyz((struct QMC7983_t *)data);
		if (copy_to_user((struct hmc5883_t *)arg,
			(struct hmc5883_t *)data, 6) != 0)
			return -EFAULT;

		return err;

	case QMC7983_SET_OUTPUT_DATA_RATE:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0)
			return -EFAULT;
		err = QMC7983_set_output_data_rate(data[0]);
		return err;

	case QMC7983_SET_OVERSAMPLE_RATIO:
		if (copy_from_user(data, (unsigned char *)arg, 1) != 0)
			return -EFAULT;

		err = QMC7983_set_oversample_ratio(data[0]);
		return err;

	case QMC7983_SELF_TEST:
		return err;

	default:
		return 0;
	}

}

static const struct file_operations qmc_misc_fops = {
	.owner = THIS_MODULE,
	.open = nonseekable_open,
	.unlocked_ioctl = qmc_misc_ioctl,
};

static int qmc7983_parse_dt(struct device *dev,
		struct QMC7983_platform_data *pdata)
{
	struct device_node *np = dev->of_node;
	const char *tmp;
	int rc = 0;
	unsigned int delay;
	u32 temp_val;

	/* set functions of platform data */
/*	pdata->init = sensor_platform_hw_init;
	pdata->exit = sensor_platform_hw_exit;
	pdata->power_on = sensor_platform_hw_power_on;
*/



	rc = of_property_read_u32(np, "qst,layout", &temp_val);
	if (rc && (rc != -EINVAL)) 
	{
		dev_err(dev, "Unable to read qst,layout\n");
		return rc;
	}
	else 
	{
		qst_layout =  temp_val;
	}
	

	rc = of_property_read_string(np, "qst,dir", &tmp);

	/* does not have a value or the string is not null-terminated */
	if (rc && (rc != -EINVAL)) {
		pr_info("%s,Unable to read qst,dir\n", __func__);
		return rc;
	}

	if (of_property_read_bool(np, "qst,auto-report"))
		pdata->auto_report = 1;
	else
		pdata->auto_report = 0;

	rc = of_property_read_u32(np, "qst,delay_ms", &delay);
	 if (rc) {
		dev_err(dev, "Unable to read ps hysteresis threshold\n");
		return rc;
	}
	pdata->delay = delay;

	//kangyan add  test
	pdata->gpio_rstn = of_get_named_gpio_flags(dev->of_node,
			"qst,gpio_rstn", 0, NULL);

	if (!gpio_is_valid(pdata->gpio_rstn)) {
		dev_err(dev, "gpio reset pin %d is invalid.\n",
			pdata->gpio_rstn);
		return -EINVAL;
	}


	return 0;
}

static int QMC7983_probe(struct i2c_client *client,
	const struct i2c_device_id *devid)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct QMC7983_data *data;
	struct QMC7983_platform_data *pdata;
	int err = 0;
	unsigned char temp_data[2];//kangyan add test 
	pr_debug("%s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		err = -EIO;
		goto exit;
	}
	
	

	if (client->dev.of_node) {
		pdata = devm_kzalloc(&client->dev,
				sizeof(struct QMC7983_platform_data),
				GFP_KERNEL);
		if (!pdata) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		client->dev.platform_data = pdata;
		err = qmc7983_parse_dt(&client->dev, pdata);
		if (err) {
			pr_err("%s: sensor_parse_dt() err\n", __func__);
			return err;
		}
	} else {
		pdata = client->dev.platform_data;
		if (!pdata) {
			dev_err(&client->dev, "No platform data\n");
			return -ENODEV;
		}
	}

	data = kzalloc(sizeof(struct QMC7983_data), GFP_KERNEL);
	if (!data) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto exit;
	}

	qmc7983 = data;
	data->client = client;
	data->delay_ms = pdata->delay;

	i2c_set_clientdata(client, data);
	data->client = client;

	mutex_init(&data->lock);
	INIT_DELAYED_WORK(&data->work, qmc7983_work);

	/* check connection */
	err = qmc7983_power_init(data, true);
	if (err < 0)
		goto exit_kfree;

	err = qmc7983_power_set(data, true);
	if (err < 0)
		goto err_compass_pwr_init;
		
	//kangyan add begin
	err = QMC7983_i2c_read(0x0d, temp_data, 1);
	if(err<0)
	{
		pr_err("%s: i2c read test err !\n", __func__);
		goto exit_kfree;
	}
	if (temp_data[0] != 0x31) //0xff ->qmc6983 0x31->qmc7983
	{
		pr_err("%s: i2c read chip ID err !\n", __func__);
		goto exit_kfree;
	}

	//kangyan add end

	/* Create input device for qmc7983 */
	err = qmc7983_input_init(data);
	if (err < 0) {
		dev_err(&client->dev, "error init input dev interface\n");
		goto exit_kfree;
	}

	err = sysfs_create_group(&client->dev.kobj, &qmc7983_attr_group);
	if (err < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto exit_kfree_input;
	}

	err = create_sysfs_interfaces(&data->input->dev);
	if (err < 0) {
		dev_err(&client->dev, "sysfs register failed\n");
		goto exit_kfree_input;
	}

	data->cdev = sensors_cdev;
	data->cdev.sensors_enable = qmc7983_enable_set;
	data->cdev.sensors_poll_delay = qmc7983_poll_delay_set;

	err = sensors_classdev_register(&client->dev, &data->cdev);
	if (err) {
		dev_err(&client->dev, "class device create failed: %d\n", err);
		goto remove_sysfs;
	}

	qmc_mag_dev_class = class_create(THIS_MODULE, "compass");
	if (IS_ERR(qmc_mag_dev_class)) {
		err = PTR_ERR(qmc_mag_dev_class);
		goto exit_class_create_failed;
	}

	data->qmc_misc.minor = MISC_DYNAMIC_MINOR;
	data->qmc_misc.name = "qmc7983";
	data->qmc_misc.fops = &qmc_misc_fops;

	err = misc_register(&data->qmc_misc);
	if (err)
		dev_err(&client->dev, "misc register failed\n");

	init_completion(&data->data_updated);
	init_waitqueue_head(&data->state_wq);

	dev_info(&client->dev, "QMC7983_probe --- 12\n");

	err = qmc7983_power_set(data, false);
	if (err)
		dev_err(&client->dev,
			"Fail to disable power after probe: %d\n", err);
			
	dev_info(&client->dev, "QMC7983_probe --- success\n");

	return 0;

remove_sysfs:
	remove_sysfs_interfaces(&data->input->dev);
exit_kfree_input:
	input_unregister_device(data->input);
exit_kfree:
	kfree(data);
err_compass_pwr_init:
	qmc7983_power_init(data, false);
exit_class_create_failed:
	qmc_mag_dev_class = NULL;
exit:
	return err;
}

static int QMC7983_remove(struct i2c_client *client)
{
	struct QMC7983_data *dev = i2c_get_clientdata(client);

	if (qmc7983_power_set(dev, false))
		dev_err(&client->dev, "power off failed.\n");
	if (qmc7983_power_init(dev, false))
		dev_err(&client->dev, "power deinit failed.\n");

	device_destroy(qmc_mag_dev_class, MKDEV(QMC7983_MAJOR, 0));
	class_destroy(qmc_mag_dev_class);
	unregister_chrdev(QMC7983_MAJOR, "QMC7983");

	kfree(dev);
	qmc7983 = NULL;

	return 0;
}

static int qmc7983_power_set(struct QMC7983_data *data, bool on)
{
	int rc = 0;

	if (!on && data->enabled) {
		rc = regulator_disable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd disable failed rc=%d\n", rc);
			goto err_vdd_disable;
		}

		rc = regulator_disable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio disable failed rc=%d\n", rc);
			goto err_vio_disable;
		}
		data->enabled = false;
		return rc;
	} else if (on && !data->enabled) {
		rc = regulator_enable(data->vdd);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vdd enable failed rc=%d\n", rc);
			goto err_vdd_enable;
		}

		rc = regulator_enable(data->vio);
		if (rc) {
			dev_err(&data->client->dev,
				"Regulator vio enable failed rc=%d\n", rc);
			goto err_vio_enable;
		}
		data->enabled = true;

		/*
		 * The max time for the power supply rise time is 50ms.
		 * Use 80ms to make sure it meets the requirements.
		 */
		msleep(50);
		return rc;
	} else {
		dev_warn(&data->client->dev,
				"Power on=%d. enabled=%d\n",
				on, data->enabled);
		return rc;
	}

err_vio_enable:
	regulator_disable(data->vio);
err_vdd_enable:
	return rc;

err_vio_disable:
	if (regulator_enable(data->vdd))
		dev_warn(&data->client->dev, "Regulator vdd enable failed\n");
err_vdd_disable:
	return rc;
}

static int qmc7983_power_init(struct QMC7983_data *data, bool on)
{
	int rc;

	if (!on) {
		if (regulator_count_voltages(data->vdd) > 0)
			regulator_set_voltage(data->vdd, 0,
				QMC7983_VDD_MAX_UV);

		regulator_put(data->vdd);

		if (regulator_count_voltages(data->vio) > 0)
			regulator_set_voltage(data->vio, 0,
				QMC7983_VIO_MAX_UV);

		regulator_put(data->vio);

	} else {
		data->vdd = regulator_get(&data->client->dev, "vdd");
		if (IS_ERR(data->vdd)) {
			rc = PTR_ERR(data->vdd);
			dev_err(&data->client->dev,
				"Regulator get failed vdd rc=%d\n", rc);
			return rc;
		}

		if (regulator_count_voltages(data->vdd) > 0) {
			rc = regulator_set_voltage(data->vdd,
				QMC7983_VDD_MIN_UV, QMC7983_VDD_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
					"Regulator set failed vdd rc=%d\n",
					rc);
				goto reg_vdd_put;
			}
		}

		data->vio = regulator_get(&data->client->dev, "vio");
		if (IS_ERR(data->vio)) {
			rc = PTR_ERR(data->vio);
			dev_err(&data->client->dev,
				"Regulator get failed vio rc=%d\n", rc);
			goto reg_vdd_set;
		}

		if (regulator_count_voltages(data->vio) > 0) {
			rc = regulator_set_voltage(data->vio,
				QMC7983_VIO_MIN_UV, QMC7983_VIO_MAX_UV);
			if (rc) {
				dev_err(&data->client->dev,
				"Regulator set failed vio rc=%d\n", rc);
				goto reg_vio_put;
			}
		}
	}

	return 0;

reg_vio_put:
	regulator_put(data->vio);
reg_vdd_set:
	if (regulator_count_voltages(data->vdd) > 0)
		regulator_set_voltage(data->vdd, 0, QMC7983_VDD_MAX_UV);
reg_vdd_put:
	regulator_put(data->vdd);
	return rc;
}

static int QMC7983_suspend(struct device *dev)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);

	pr_info("%s:QMC7983_suspend\n", __func__);

	qmc7983->state.power_on = qmc7983->enable_sensor;

	if (qmc7983->state.power_on)
		qmc7983_enable_sensor(qmc7983->client, 0);

	return 0;
}

static int QMC7983_resume(struct device *dev)
{
	struct QMC7983_data *qmc7983 = dev_get_drvdata(dev);

	pr_info("%s:QMC7983_resume\n", __func__);

	if (qmc7983->state.power_on)
		qmc7983_enable_sensor(qmc7983->client, 0);

	return 0;
}

static const struct i2c_device_id QMC7983_id[] = {
	{ "QMC7983", 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, QMC7983_id);

/* fae support modify */
static const struct dev_pm_ops qmc_compass_pm_ops = {
	.suspend = QMC7983_suspend,
	.resume = QMC7983_resume,
};

static struct of_device_id qmc7983_match_table[] = {
	{ .compatible = "qst,qmc7983", },
	{ },
};
/* fae support modify end */
static struct i2c_driver QMC7983_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = QMC7983_probe,
	.remove = QMC7983_remove,
	.id_table = QMC7983_id,
	.driver = {
		.owner = THIS_MODULE,
		.name = "qmc7983",
		.of_match_table = qmc7983_match_table,
		.pm		= &qmc_compass_pm_ops,
	},
	/*
	.detect = QMC7983_detect,
	*/
};

static int __init QMC7983_init(void)
{
	int ret;

	/* add i2c driver for QMC7983 magnetometer */
	ret = i2c_add_driver(&QMC7983_driver);

	return ret;
}

static void __exit QMC7983_exit(void)
{
	i2c_del_driver(&QMC7983_driver);
	return;
}

module_init(QMC7983_init);
module_exit(QMC7983_exit);

MODULE_DESCRIPTION("QMC7983 magnetometer driver");
MODULE_AUTHOR("QST");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.1");
