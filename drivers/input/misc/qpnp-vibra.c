/* Copyright (c) 2013-2014, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015 Jolla Ltd.
 *
 * Contact: Kalle Jokiniemi <kalle.jokiniemi@jolla.com>
 *
 * This is ffmemless vibra driver for Qualcomm QPNP PMIC is based on
 * qpnp-vibrator.c Android timed output vibrator driver.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/hrtimer.h>
#include <linux/of_device.h>
#include <linux/spmi.h>
#include <linux/input.h>
#include <linux/qpnp/pwm.h>
#include <linux/err.h>
#include <linux/wakelock.h>
#include "../../staging/android/timed_output.h"

#define QPNP_VIB_VTG_CTL(base)		(base + 0x41)
#define QPNP_VIB_EN_CTL(base)		(base + 0x46)

#define QPNP_VIB_MAX_LEVEL		31
#define QPNP_VIB_MIN_LEVEL		12
#define QPNP_VIB_MAX_LEVELS		(QPNP_VIB_MAX_LEVEL - \
					QPNP_VIB_MIN_LEVEL)
#define MAX_FF_SPEED			0xff

#define QPNP_VIB_DEFAULT_VTG_LVL	3100

#define QPNP_VIB_EN			BIT(7)
#define QPNP_VIB_VTG_SET_MASK		0x1F
#define QPNP_VIB_LOGIC_SHIFT		4

enum qpnp_vib_mode {
	QPNP_VIB_MANUAL,
	QPNP_VIB_DTEST1,
	QPNP_VIB_DTEST2,
	QPNP_VIB_DTEST3,
};

struct qpnp_pwm_info {
	struct pwm_device *pwm_dev;
	u32 pwm_channel;
	u32 duty_us;
	u32 period_us;
};

struct qpnp_vib {
	struct spmi_device *spmi;
	struct work_struct work;
	struct qpnp_pwm_info pwm_info;
	struct input_dev *input_dev;
	enum   qpnp_vib_mode mode;

	u8  reg_vtg_ctl;
	u8  reg_en_ctl;
	u8  active_low;
	u16 base;
	int speed;
	int vtg_level;
	struct wake_lock wake_lock;
};

static int qpnp_vib_read_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_readl(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error reading address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vib_write_u8(struct qpnp_vib *vib, u8 *data, u16 reg)
{
	int rc;

	rc = spmi_ext_register_writel(vib->spmi->ctrl, vib->spmi->sid,
							reg, data, 1);
	if (rc < 0)
		dev_err(&vib->spmi->dev,
			"Error writing address: %X - ret %X\n", reg, rc);

	return rc;
}

static int qpnp_vibrator_config(struct qpnp_vib *vib)
{
	u8 reg = 0;
	int rc;

	/* Configure the VTG CTL regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg &= ~QPNP_VIB_VTG_SET_MASK;
	reg |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_VTG_CTL(vib->base));
	if (rc)
		return rc;
	vib->reg_vtg_ctl = reg;

	/* Configure the VIB ENABLE regiser */
	rc = qpnp_vib_read_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	reg |= (!!vib->active_low) << QPNP_VIB_LOGIC_SHIFT;
	if (vib->mode != QPNP_VIB_MANUAL) {
		vib->pwm_info.pwm_dev = pwm_request(vib->pwm_info.pwm_channel,
								 "qpnp-vib");
		if (IS_ERR_OR_NULL(vib->pwm_info.pwm_dev)) {
			dev_err(&vib->spmi->dev, "vib pwm request failed\n");
			return -ENODEV;
		}

		rc = pwm_config(vib->pwm_info.pwm_dev, vib->pwm_info.duty_us,
						vib->pwm_info.period_us);
		if (rc < 0) {
			dev_err(&vib->spmi->dev, "vib pwm config failed\n");
			pwm_free(vib->pwm_info.pwm_dev);
			return -ENODEV;
		}

		reg |= BIT(vib->mode - 1);
	}

	rc = qpnp_vib_write_u8(vib, &reg, QPNP_VIB_EN_CTL(vib->base));
	if (rc < 0)
		return rc;
	vib->reg_en_ctl = reg;

	return rc;
}

static int qpnp_vib_set(struct qpnp_vib *vib, int on)
{
	int rc = 0;
	u8 vol, en;

	if (on) {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_enable(vib->pwm_info.pwm_dev);
		else {
			vol &= ~QPNP_VIB_VTG_SET_MASK;
			vol |= (vib->vtg_level & QPNP_VIB_VTG_SET_MASK);
			rc = qpnp_vib_write_u8(vib, &vol, QPNP_VIB_VTG_CTL(vib->base));
			if (rc) {
				dev_err(&vib->spmi->dev,
					"Couldn't set voltage level\n");
				goto failure;
			}
			vib->vtg_level = vol;

			en = vib->reg_en_ctl;
			en |= QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &en,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0) {
				dev_err(&vib->spmi->dev,
					"Couldn't set enable bit\n");
				goto failure;
			}
			vib->reg_en_ctl = en;
		}
	} else {
		if (vib->mode != QPNP_VIB_MANUAL)
			pwm_disable(vib->pwm_info.pwm_dev);
		else {
			en = vib->reg_en_ctl;
			en &= ~QPNP_VIB_EN;
			rc = qpnp_vib_write_u8(vib, &en,
					QPNP_VIB_EN_CTL(vib->base));
			if (rc < 0)
				goto failure;
			vib->reg_en_ctl = en;
		}
	}

failure:
	/* Allow suspend when vibra is off / in case of errors */
	if( !on || rc )
		wake_unlock(&vib->wake_lock);
	return rc;
}

static int qpnp_vib_play(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{
	struct qpnp_vib *vib = input_get_drvdata(dev);

	int speed = effect->u.rumble.strong_magnitude >> 8;
	if (!speed)
		speed = effect->u.rumble.weak_magnitude >> 9;

	/* Block suspend until qpnp_vib_set(0) gets called
	 * either from qpnp_vib_work() or qpnp_vib_stop() */
	wake_lock(&vib->wake_lock);

	vib->speed = speed;
	schedule_work(&vib->work);

	return 0;
}

static void qpnp_vib_work(struct work_struct *work)
{
	struct qpnp_vib *vib = container_of(work, struct qpnp_vib,
					 work);

	/*
	 * qpnp vibrator supports voltage ranges from 1.2 to 3.1V, so
	 * scale the level to fit into these ranges.
	 */
	int speed = vib->speed;
	if (speed) {
		vib->vtg_level = ((QPNP_VIB_MAX_LEVELS * speed) /
				MAX_FF_SPEED) + QPNP_VIB_MIN_LEVEL;
	} else {
		vib->vtg_level = QPNP_VIB_MIN_LEVEL;
	}

	qpnp_vib_set(vib, speed);
}

static int qpnp_vib_stop(struct qpnp_vib *vib)
{
	cancel_work_sync(&vib->work);
	return qpnp_vib_set(vib, 0);
}

static void qpnp_vib_close(struct input_dev *dev)
{
	struct qpnp_vib *vib = input_get_drvdata(dev);

	qpnp_vib_stop(vib);
}

#ifdef CONFIG_PM
static int qpnp_vibrator_suspend(struct device *dev)
{
	struct qpnp_vib *vib = dev_get_drvdata(dev);

	return qpnp_vib_stop(vib);
}

static SIMPLE_DEV_PM_OPS(qpnp_vibrator_pm_ops, qpnp_vibrator_suspend, NULL);
#endif

static int qpnp_vib_parse_dt(struct qpnp_vib *vib)
{
	struct spmi_device *spmi = vib->spmi;
	int rc;
	const char *mode = "NOTHING";
	u32 temp_val;

	vib->vtg_level = QPNP_VIB_DEFAULT_VTG_LVL;
	rc = of_property_read_u32(spmi->dev.of_node,
			"qcom,vib-vtg-level-mV", &temp_val);
	if (!rc) {
		vib->vtg_level = temp_val;
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read vtg level\n");
		return rc;
	}

	vib->vtg_level /= 100;
	if (vib->vtg_level < QPNP_VIB_MIN_LEVEL)
		vib->vtg_level = QPNP_VIB_MIN_LEVEL;
	else if (vib->vtg_level > QPNP_VIB_MAX_LEVEL)
		vib->vtg_level = QPNP_VIB_MAX_LEVEL;

	vib->mode = QPNP_VIB_MANUAL;
	rc = of_property_read_string(spmi->dev.of_node, "qcom,mode", &mode);
	if (!rc) {
		if (strcmp(mode, "manual") == 0)
			vib->mode = QPNP_VIB_MANUAL;
		else if (strcmp(mode, "dtest1") == 0)
			vib->mode = QPNP_VIB_DTEST1;
		else if (strcmp(mode, "dtest2") == 0)
			vib->mode = QPNP_VIB_DTEST2;
		else if (strcmp(mode, "dtest3") == 0)
			vib->mode = QPNP_VIB_DTEST3;
		else {
			dev_err(&spmi->dev, "Invalid mode\n");
			return -EINVAL;
		}
	} else if (rc != -EINVAL) {
		dev_err(&spmi->dev, "Unable to read mode\n");
		return rc;
	}

	if (vib->mode != QPNP_VIB_MANUAL) {
		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,pwm-channel", &temp_val);
		if (!rc)
			vib->pwm_info.pwm_channel = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,period-us", &temp_val);
		if (!rc)
			vib->pwm_info.period_us = temp_val;
		else
			return rc;

		rc = of_property_read_u32(spmi->dev.of_node,
				"qcom,duty-us", &temp_val);
		if (!rc)
			vib->pwm_info.duty_us = temp_val;
		else
			return rc;
	}

	vib->active_low = of_property_read_bool(spmi->dev.of_node,
				"qcom,active-low");

	return 0;
}

static int qpnp_vibrator_probe(struct spmi_device *spmi)
{
	struct qpnp_vib *vib;
	struct resource *vib_resource;
	struct input_dev *input_dev;
	int rc;

	vib = devm_kzalloc(&spmi->dev, sizeof(*vib), GFP_KERNEL);
	if (!vib)
		return -ENOMEM;

	vib->spmi = spmi;

	vib_resource = spmi_get_resource(spmi, 0, IORESOURCE_MEM, 0);
	if (!vib_resource) {
		dev_err(&spmi->dev, "Unable to get vibrator base address\n");
		return -EINVAL;
	}
	vib->base = vib_resource->start;

	rc = qpnp_vib_parse_dt(vib);
	if (rc) {
		dev_err(&spmi->dev, "DT parsing failed\n");
		goto probe_err1;
	}

	rc = qpnp_vibrator_config(vib);
	if (rc) {
		dev_err(&spmi->dev, "vib config failed\n");
		goto probe_err1;
	}

	input_dev = input_allocate_device();
	if (!vib || !input_dev) {
		dev_err(&spmi->dev, "couldn't allocate input device\n");
		rc = -ENOMEM;
		goto probe_err1;
	}

	wake_lock_init(&vib->wake_lock, WAKE_LOCK_SUSPEND, "qpnp_vibra");
	INIT_WORK(&vib->work, qpnp_vib_work);

	input_dev->name = "qpnp_vibra_ffmemless";
	input_dev->id.version = 1;
	input_dev->dev.parent = &spmi->dev;
	input_dev->close = qpnp_vib_close;
	input_set_drvdata(input_dev, vib);
	input_set_capability(input_dev, EV_FF, FF_RUMBLE);

	rc = input_ff_create_memless(input_dev, NULL, qpnp_vib_play);
	if (rc) {
		dev_err(&spmi->dev,
			"couldn't create ffmemless device\n");
		goto probe_err2;
	}

	vib->input_dev = input_dev;

	rc = input_register_device(input_dev);
	if (rc) {
		dev_err(&spmi->dev, "couldn't register input device\n");
		goto probe_err2;
	}

	dev_set_drvdata(&spmi->dev, vib);

	dev_info(&spmi->dev, "Probe success: vtg_level = %u, mode = %u, "
			"pwm_channel = %u, period_us = %u, duty_us = %u, "
			"active_low = %u\n",
			vib->vtg_level, vib->mode, vib->pwm_info.pwm_channel,
			vib->pwm_info.period_us, vib->pwm_info.duty_us,
			vib->active_low);

	return rc;

probe_err2:
	input_free_device(input_dev);
	wake_lock_destroy(&vib->wake_lock);
probe_err1:
	kfree(vib);
	return rc;
}

static int qpnp_vibrator_remove(struct spmi_device *spmi)
{
	struct qpnp_vib *vib = dev_get_drvdata(&spmi->dev);

	cancel_work_sync(&vib->work);
	input_unregister_device(vib->input_dev);
	wake_lock_destroy(&vib->wake_lock);
	kfree(vib);

	return 0;
}

static struct of_device_id spmi_match_table[] = {
	{	.compatible = "qcom,qpnp-vibrator",
	},
	{}
};

static struct spmi_driver qpnp_vibrator_driver = {
	.driver		= {
		.name	= "qcom,qpnp-vibrator",
		.of_match_table = spmi_match_table,
#ifdef CONFIG_PM
		.pm	= &qpnp_vibrator_pm_ops,
#endif
	},
	.probe		= qpnp_vibrator_probe,
	.remove		= qpnp_vibrator_remove,
};

static int __init qpnp_vibrator_init(void)
{
	return spmi_driver_register(&qpnp_vibrator_driver);
}
module_init(qpnp_vibrator_init);

static void __exit qpnp_vibrator_exit(void)
{
	return spmi_driver_unregister(&qpnp_vibrator_driver);
}
module_exit(qpnp_vibrator_exit);

MODULE_DESCRIPTION("qpnp ffmemless vibra driver");
MODULE_LICENSE("GPL v2");
