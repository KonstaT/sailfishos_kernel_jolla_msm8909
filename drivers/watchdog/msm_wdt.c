/* Copyright (c) 2010-2013, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/watchdog.h>
#include <linux/init.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <linux/irq.h>
#include <linux/percpu.h>
#include <linux/io.h>
#include <linux/pm.h>
#include <linux/of.h>
#include <mach/msm_iomap.h>
#include <asm/mach-types.h>
#include <linux/platform_device.h>
#include <linux/suspend.h>
#include <../msm_watchdog.h>  // located in arch/arm/mach-msm/msm_watchdog.h

#define MODULE_NAME "msm_watchdog"

#define WDT_RST        0x04
#define WDT_EN         0x08
#define WDT_STS        0x0C
#define WDT_BARK_TIME  0x10
#define WDT_BITE_TIME  0x14


#define WDT_HZ		32768
#define WDT_MAXTIMEOUT  0x3FFF

static void __iomem *msm_wdt_base;
static void pet_watchdog_work(struct work_struct *work);
static DEFINE_SPINLOCK(wdt_lock);
static DECLARE_DELAYED_WORK(dogwork_struct, pet_watchdog_work);

static atomic_t enable = ATOMIC_INIT(0);

struct msm_watchdog_data {
        unsigned int __iomem phys_base;
        size_t size;
        void __iomem *base;
        void __iomem *wdog_absent_base;
        struct device *dev;
        unsigned int pet_time;
        unsigned int bark_time;
        unsigned int bark_irq;
        unsigned int bite_irq;
        bool do_ipi_ping;
        unsigned long long last_pet;
        unsigned min_slack_ticks;
        unsigned long long min_slack_ns;
        void *scm_regsave;
        cpumask_t alive_mask;
        struct mutex disable_lock;
        struct work_struct init_dogwork_struct;
        struct delayed_work dogwork_struct;
        bool irq_ppi;
        struct msm_watchdog_data __percpu **wdog_cpu_dd;
        struct notifier_block panic_blk;
        bool enabled;
};

static void msm_wdt_enable(void)
{
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_EN);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	mb();
	atomic_set(&enable,1);
	spin_unlock(&wdt_lock);
}

static void msm_wdt_disable(void)
{
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	__raw_writel(0, msm_wdt_base + WDT_EN);
	mb();
	atomic_set(&enable,0);
	spin_unlock(&wdt_lock);
}

static int msm_wdt_settimeout(struct watchdog_device *wdd, unsigned int time)
{
	unsigned long timeout = (time * WDT_HZ);
	spin_lock(&wdt_lock);
	__raw_writel(timeout, msm_wdt_base + WDT_BARK_TIME);
	__raw_writel(timeout + 3*WDT_HZ, msm_wdt_base + WDT_BITE_TIME);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	spin_unlock(&wdt_lock);
	wdd->timeout = time;
	return 0;
}

static void msm_wdt_keepalive(void)
{
	spin_lock(&wdt_lock);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	spin_unlock(&wdt_lock);
}

void pet_watchdog(void)
{
	msm_wdt_keepalive();
}

static int msm_wdt_start (struct watchdog_device *wdd)
{
	msm_wdt_enable();
	return 0;
}

static int msm_wdt_stop (struct watchdog_device *wdd)
{
	msm_wdt_disable();
	return 0;
}

static int msm_wdt_ping (struct watchdog_device *wdd)
{
	msm_wdt_keepalive();
	return 0;
}

static int msm_wdt_suspend(struct device *dev)
{
	if (!atomic_read(&enable))
		return 0;

	__raw_writel(1, msm_wdt_base + WDT_RST);
	__raw_writel(0, msm_wdt_base + WDT_EN);
	mb();
	return 0;
}

static int msm_wdt_resume(struct device *dev)
{
	if (!atomic_read(&enable))
		return 0;

	__raw_writel(1, msm_wdt_base + WDT_EN);
	__raw_writel(1, msm_wdt_base + WDT_RST);
	mb();
	return 0;
}

static const struct watchdog_info msm_wdt_info = {
	.identity	= "msm-watchdog",
	.options	= WDIOF_SETTIMEOUT
			  | WDIOF_KEEPALIVEPING
			  #ifdef CONFIG_WATCHDOG_NOWAYOUT
			  | WDIOF_MAGICCLOSE
			  #endif
			  ,
};

static const struct watchdog_ops  msm_wdt_fops = {
	.owner		= THIS_MODULE,
	.start		= msm_wdt_start,
	.stop		= msm_wdt_stop,
	.ping		= msm_wdt_ping,
	.set_timeout	= msm_wdt_settimeout,
};

static struct watchdog_device msm_wdt_dev = {
	.info =		&msm_wdt_info,
	.ops =		&msm_wdt_fops,
	.min_timeout =	1,
	.max_timeout =	WDT_MAXTIMEOUT,
};



static void pet_watchdog_work(struct work_struct *work)
{
	pet_watchdog();
	if (atomic_read(&enable)) {
		schedule_delayed_work_on(0, &dogwork_struct, msm_wdt_dev.timeout / 2 * 100);
	}
}



static int msm_wdt_pm_notifier(struct notifier_block *nb,
				unsigned long event, void *unused)
{
	switch (event) {
	case PM_SUSPEND_PREPARE:
		pet_watchdog();
		if (atomic_read(&enable)) {
			schedule_delayed_work_on(0, &dogwork_struct, msm_wdt_dev.timeout / 2 * 100);
		}
		break;

	case PM_POST_SUSPEND:
		pet_watchdog();
		if (atomic_read(&enable)) {
			cancel_delayed_work_sync (&dogwork_struct);
		}
		break;
	}
	return NOTIFY_DONE;
}

static struct notifier_block msm_wdt_pm_nb = {
	.notifier_call = msm_wdt_pm_notifier,
};

static void dump_pdata(struct msm_watchdog_data *pdata)
{
        dev_dbg(pdata->dev, "wdog bark_time %d", pdata->bark_time);
        dev_dbg(pdata->dev, "wdog pet_time %d", pdata->pet_time);
        dev_dbg(pdata->dev, "wdog perform ipi ping %d", pdata->do_ipi_ping);
        dev_dbg(pdata->dev, "wdog base address is 0x%lx\n", (unsigned long)
                                                                pdata->base);
}

static int msm_wdog_dt_to_pdata(struct platform_device *pdev,
                                        struct msm_watchdog_data *pdata)
{
        struct device_node *node = pdev->dev.of_node;
        struct resource *res;
        int ret;

        res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wdt-base");
        if (!res)
                return -ENODEV;
        pdata->size = resource_size(res);
        pdata->phys_base = res->start;
        if (unlikely(!(devm_request_mem_region(&pdev->dev, pdata->phys_base,
                                               pdata->size, "msm-watchdog")))) {

                dev_err(&pdev->dev, "%s cannot reserve watchdog region\n",
                                                                __func__);
                return -ENXIO;
        }
        pdata->base  = devm_ioremap(&pdev->dev, pdata->phys_base,
                                                        pdata->size);
        if (!pdata->base) {
                dev_err(&pdev->dev, "%s cannot map wdog register space\n",
                                __func__);
                return -ENXIO;
        }

        pdata->bark_irq = platform_get_irq(pdev, 0);
        pdata->bite_irq = platform_get_irq(pdev, 1);
        ret = of_property_read_u32(node, "qcom,bark-time", &pdata->bark_time);
        if (ret) {
                dev_err(&pdev->dev, "reading bark time failed\n");
                return -ENXIO;
        }
        ret = of_property_read_u32(node, "qcom,pet-time", &pdata->pet_time);
        if (ret) {
                dev_err(&pdev->dev, "reading pet time failed\n");
                return -ENXIO;
        }
        pdata->do_ipi_ping = of_property_read_bool(node, "qcom,ipi-ping");
        if (!pdata->bark_time) {
                dev_err(&pdev->dev, "%s watchdog bark time not setup\n",
                                                                __func__);
                return -ENXIO;
        }
        if (!pdata->pet_time) {
                dev_err(&pdev->dev, "%s watchdog pet time not setup\n",
                                                                __func__);
                return -ENXIO;
        }
        pdata->irq_ppi = irq_is_percpu(pdata->bark_irq);
        dump_pdata(pdata);
        return 0;
}

static int msm_wdt_probe(struct platform_device *pdev)
{
	int retval;
        int ret;
        struct msm_watchdog_data *wdog_dd;


        if (!pdev->dev.of_node)
	{
		printk(KERN_ERR "MSM Watchdog Init Failed\n");
                return -ENODEV;
	}
        wdog_dd = kzalloc(sizeof(struct msm_watchdog_data), GFP_KERNEL);
        if (!wdog_dd)
                return -EIO;
        ret = msm_wdog_dt_to_pdata(pdev, wdog_dd);

        wdog_dd->dev = &pdev->dev;
	msm_wdt_base = wdog_dd->base;

        __raw_writel(10 * WDT_HZ, msm_wdt_base + WDT_BARK_TIME); // set default timeout to prevent
	__raw_writel(13 * WDT_HZ, msm_wdt_base + WDT_BITE_TIME); // immediately dog bark once dog enable

        retval = watchdog_register_device(&msm_wdt_dev);
	if (retval)
		printk(KERN_ERR "%s: watchdog register failed %d\n", __func__, retval);

	retval = register_pm_notifier(&msm_wdt_pm_nb);
	if (retval)
		printk(KERN_ERR "%s: power state notif error %d\n", __func__, retval);

	printk(KERN_INFO "MSM Watchdog Initialized\n");
	return 0;
}

static void msm_wdt_shutdown(struct platform_device *dev)
{
        msm_wdt_disable();
}

static int msm_wdt_remove(struct platform_device *pdev)
{
	watchdog_unregister_device(&msm_wdt_dev);
	return 0;
}

static const struct dev_pm_ops msm_wdt_dev_pm_ops = {
	.suspend_noirq = msm_wdt_suspend,
	.resume_noirq = msm_wdt_resume,
};

static struct of_device_id msm_wdog_match_table[] = {
        { .compatible = "qcom,msm-watchdog" },
        {}
};

static struct platform_driver msm_wdt_driver = {
	.probe = msm_wdt_probe,
	.remove = msm_wdt_remove,
	.shutdown = msm_wdt_shutdown,
	.driver = {
		.name = MODULE_NAME,
		.owner = THIS_MODULE,
		.pm = &msm_wdt_dev_pm_ops,
                .of_match_table = msm_wdog_match_table,
	},
};

module_platform_driver(msm_wdt_driver);

MODULE_LICENSE("GPL");
