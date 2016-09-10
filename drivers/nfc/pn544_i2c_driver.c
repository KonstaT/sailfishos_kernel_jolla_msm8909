/**********uniscope-driver-modify-file-on-qualcomm-platform*****************/
/*
 * Copyright (C) 2010 Trusted Logic S.A.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

//ALERT:relocate pn544.c under .\kernel\drivers\misc

/*
* Makefile//TODO:Here is makefile reference
* obj-$(CONFIG_PN544)+= pn544.o
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "pn544_i2c_driver.h"
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include <linux/clk.h>
//TODO:replace and include corresponding head file for VEN/IRQ/FIRM I/O configuration


//#define pr_err pr_err
//#define pr_debug pr_err
//#define pr_warning pr_err




#define MAX_BUFFER_SIZE	512

struct pn544_dev	
{
	wait_queue_head_t	read_wq;
	struct mutex		read_mutex;
	struct i2c_client	*client;
	struct miscdevice	pn544_device;
	unsigned int 		ven_gpio;
	unsigned int 		firm_gpio;
	unsigned int		irq_gpio;
	bool				irq_enabled;
	spinlock_t			irq_enabled_lock;
	unsigned int clkreq_gpio;
	struct	clk		*s_clk;
	bool			clk_run;
};

static void pn544_disable_irq(struct pn544_dev *pn544_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&pn544_dev->irq_enabled_lock, flags);
	if (pn544_dev->irq_enabled) 
	{
		disable_irq_nosync(pn544_dev->client->irq);
		pn544_dev->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn544_dev->irq_enabled_lock, flags);
}

static irqreturn_t pn544_dev_irq_handler(int irq, void *dev_id)
{
	struct pn544_dev *pn544_dev = dev_id;

	pn544_disable_irq(pn544_dev);
	/* Wake up waiting readers */
	wake_up(&pn544_dev->read_wq);

	return IRQ_HANDLED;
}

static ssize_t pn544_dev_read(struct file *filp, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev *pn544_dev = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	pr_err("[pn547] %s : reading %zu bytes.\n", __func__, count);

	mutex_lock(&pn544_dev->read_mutex);

	if (!gpio_get_value(pn544_dev->irq_gpio)) 
	{
		if (filp->f_flags & O_NONBLOCK) 
		{
			ret = -EAGAIN;
			goto fail;
		}

		pn544_dev->irq_enabled = true;
		enable_irq(pn544_dev->client->irq);
		ret = wait_event_interruptible(pn544_dev->read_wq,
				gpio_get_value(pn544_dev->irq_gpio));

		pn544_disable_irq(pn544_dev);

		if (ret)
			goto fail;
	}

	/* Read data */
	//msleep(100);  
	ret = i2c_master_recv(pn544_dev->client, tmp, count);
	mutex_unlock(&pn544_dev->read_mutex);

	if (ret < 0) 
	{
		pr_err("[pn547] %s: i2c_master_recv returned %d\n", __func__, ret);
		return ret;
	}
	if (ret > count) 
	{
		pr_err("[pn547] %s: received too many bytes from i2c (%d)\n", __func__, ret);
		return -EIO;
	}
	if (copy_to_user(buf, tmp, ret)) 
	{
		pr_warning("[pn547] %s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}
	
	pr_err("[pn547] IFD->PC:");
	for(i = 0; i < ret; i++)
	{
		pr_err("[pn547]  %02X", tmp[i]);
	}
	
	return ret;

fail:
	mutex_unlock(&pn544_dev->read_mutex);
	return ret;
}

static ssize_t pn544_dev_write(struct file *filp, const char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_dev  *pn544_dev;
	char tmp[MAX_BUFFER_SIZE];
	int ret,i;

	pn544_dev = filp->private_data;

	if (count > MAX_BUFFER_SIZE)
	{
		count = MAX_BUFFER_SIZE;
	}
	if (copy_from_user(tmp, buf, count)) 
	{
		pr_err("[pn547]%s : failed to copy from user space\n", __func__);
		return -EFAULT;
	}

	pr_err("[pn547]%s : writing %zu bytes.\n", __func__, count);
	/* Write data */
	//msleep(100);
	ret = i2c_master_send(pn544_dev->client, tmp, count);
	if (ret != count) 
	{
		pr_err("[pn547]%s : i2c_master_send returned %d\n", __func__, ret);
		ret = -EIO;
	}
	pr_err("[pn547] PC->IFD:");
	for(i = 0; i < count; i++)
	{
		pr_err("[pn547] %02X", tmp[i]);
	}	
	return ret;
}

static int pn544_dev_open(struct inode *inode, struct file *filp)
{

	struct pn544_dev *pn544_dev = container_of(filp->private_data, struct pn544_dev, pn544_device);
	
	filp->private_data = pn544_dev;

	pr_debug("[pn547]%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

	return 0;
}

static long pn544_dev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct pn544_dev *pn544_dev = filp->private_data;

	
	switch (cmd) 
	{
	case PN544_SET_PWR:
		if (arg == 2) 
		{
			/* 
			power on with firmware download (requires hw reset)
			 */
			pr_err("[pn547]%s power on with firmware pn544_dev->ven_gpio = %d \n", __func__, pn544_dev->ven_gpio);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			gpio_set_value(pn544_dev->firm_gpio, 1);
			msleep(20);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(60);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(20);
		} 
		else if (arg == 1) 
		{
			/* power on */
			pr_err("[pn547]%s power on pn544_dev->ven_gpio = %d \n", __func__, pn544_dev->ven_gpio);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 0);
			msleep(20);
		} 
		else  if (arg == 0) 
		{
			/* power off */
			pr_err("[pn547]%s power off pn544_dev->ven_gpio =%d \n", __func__, pn544_dev->ven_gpio);
			gpio_set_value(pn544_dev->firm_gpio, 0);
			gpio_set_value(pn544_dev->ven_gpio, 1);
			msleep(60);
			
		} else {
			pr_err("[pn547]%s bad arg %lu\n", __func__, arg);
			return -EINVAL;
		}
		break;
	default:
		pr_err("[pn547]%s bad ioctl %u\n", __func__, cmd);
		return -EINVAL;
	}

	return 0;
}

static const struct file_operations pn544_dev_fops = 
{
	.owner	= THIS_MODULE,
	.llseek	= no_llseek,
	.read	= pn544_dev_read,
	.write	= pn544_dev_write,
	.open	= pn544_dev_open,
	.unlocked_ioctl = pn544_dev_ioctl,
};
static int nfc_parse_dt(struct device *dev,
			struct pn544_i2c_platform_data *pdata)
{
	int rc = 0;
	struct device_node *np = dev->of_node;
	pdata->ven_gpio = of_get_named_gpio_flags(np, "qcom,dis-gpio",
				0, &pdata->ven_gpio);
	if (pdata->ven_gpio < 0)
		return pdata->ven_gpio;
	
	pdata->firm_gpio = of_get_named_gpio_flags(np, "qcom,firm_gpio",
				0, &pdata->firm_gpio);
	if (pdata->firm_gpio < 0)
		return pdata->firm_gpio;
	
	pdata->irq_gpio = of_get_named_gpio_flags(np, "qcom,irq-gpio",
				0, &pdata->irq_gpio);
	if (pdata->irq_gpio < 0)
		return pdata->irq_gpio;
	rc = of_property_read_string(np, "qcom,clk-src", &pdata->clk_src_name);
	pr_err("[pn547] qcom,clk-src = %s\n",pdata->clk_src_name);
	pdata->clkreq_gpio = of_get_named_gpio(np, "qcom,clk-gpio", 0);
	pr_err("[pn547] qcom,clk-gpio = %d\n",pdata->clkreq_gpio);
	if (rc)
		return -EINVAL;
	return rc;
}


static int pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret;//,irq;
	struct pn544_i2c_platform_data *platform_data;
	struct pn544_dev *pn544_dev;
	pr_err("[pn547]enter %s function\n",__func__);
	if (client->dev.of_node) {
 	pr_err("[pn547]  client->dev.of_node:::::: %s function\n",__func__);            
		platform_data = devm_kzalloc(&client->dev,
			sizeof(struct pn544_i2c_platform_data), GFP_KERNEL);
		if (!platform_data) {
			dev_err(&client->dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}
		ret = nfc_parse_dt(&client->dev, platform_data);
		if (ret) {
			dev_err(&client->dev, "DT parsing failed\n");
			return -ENODEV;
		}
	}
	else
	{
		platform_data = client->dev.platform_data;
	}
	if (platform_data == NULL) 
	{
		pr_err("[pn547]%s : nfc probe fail\n", __func__);
		return  -ENODEV;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
	{
		pr_err("[pn547]%s : need I2C_FUNC_I2C\n", __func__);
		return  -ENODEV;
	}
	//IRQ 
	pr_err("[pn547] start request platform_data->irq_gpio = %d\n",platform_data->irq_gpio);
	ret = gpio_request(platform_data->irq_gpio, "nfc_int");
	if (ret)
	{
		pr_err("[pn547]gpio_nfc_int request error\n");
		return  -ENODEV;
	}
	ret = gpio_direction_input(platform_data->irq_gpio);
	if (ret) {
		pr_err("[pn547]set_direction for irq gpio failed\n");
		goto err_exit;
	}
	
	//VEN
	pr_err("[pn547] start request platform_data->ven_gpio = %d\n",platform_data->ven_gpio);
	ret = gpio_request(platform_data->ven_gpio, "nfc_ven");
	if (ret)
	{
		pr_err("[pn547]gpio_nfc_ven request error\n");
		return  -ENODEV;
	}
	ret = gpio_direction_output(platform_data->ven_gpio, 0);
	if (ret) {
		pr_err("[pn547]set_direction for ven gpio failed\n");
		goto err_exit;
	}
	//FIRM
	pr_err("[pn547] start request platform_data->firm_gpio = %d\n",platform_data->firm_gpio);
	ret = gpio_request(platform_data->firm_gpio, "nfc_firm");
	if (ret)
	{
		pr_err("[pn547]gpio_nfc_firm request error\n");	
		return  -ENODEV;
	}
	ret = gpio_direction_output(platform_data->firm_gpio, 0);
	if (ret) {
		pr_err("[pn547]set_direction for firm gpio failed\n");
		goto err_exit;
	}
	//malloc dev
	pn544_dev = kzalloc(sizeof(*pn544_dev), GFP_KERNEL);
	if (pn544_dev == NULL) 
	{
		dev_err(&client->dev,
				"failed to allocate memory for module data\n");
		ret = -ENOMEM;
		goto err_exit;
	}
	
	//clk get
	pn544_dev->s_clk  =
			clk_get(&client->dev, "ref_clk");
	if (pn544_dev->s_clk == NULL){
		pr_err("[pn547] get clk fail\n");
		goto err_exit;
	}
	//enable clk
	ret= clk_prepare_enable(pn544_dev->s_clk);
	if (ret){
		pr_err("[pn547] enable clk fail\n");
		goto err_clk;
	}

	pr_err("[pn547] **qcom,clk-src = %s*********\n",platform_data->clk_src_name);

	if (gpio_is_valid(platform_data->clkreq_gpio)) {
		pr_err("[pn547] request clkreq_gpio = %d\n",platform_data->clkreq_gpio);
		ret = gpio_request(platform_data->clkreq_gpio,
			"nfc_clkreq_gpio");
		if (ret) {
			pr_err("[pn547clk]unable to request gpio [%d]\n",
					platform_data->clkreq_gpio);
			goto err_clkreq_gpio;
		}
		ret = gpio_direction_input(platform_data->clkreq_gpio);
		if (ret) {
			pr_err("[pn547clk]unable to set direction for gpio [%d]\n",
				platform_data->clkreq_gpio);
			goto err_clkreq_gpio;
		}
	} else {
			pr_err("[pn547clk]clkreq gpio not provided\n");
			goto err_exit;
	}
	pn544_dev->clkreq_gpio = platform_data->clkreq_gpio;
	
	pn544_dev->irq_gpio = platform_data->irq_gpio;
	pn544_dev->ven_gpio  = platform_data->ven_gpio;
	pn544_dev->firm_gpio  = platform_data->firm_gpio;
	pn544_dev->client   = client;

	/* init mutex and queues */
	init_waitqueue_head(&pn544_dev->read_wq);
	mutex_init(&pn544_dev->read_mutex);
	spin_lock_init(&pn544_dev->irq_enabled_lock);

	pn544_dev->pn544_device.minor = MISC_DYNAMIC_MINOR;
	pn544_dev->pn544_device.name = "pn544";
	pn544_dev->pn544_device.fops = &pn544_dev_fops;

	ret = misc_register(&pn544_dev->pn544_device);	
	if (ret) 
	{
		pr_err("[pn547]%s : misc_register failed\n", __FILE__);
		goto err_misc_register;
	}

	/* request irq.  the irq is set whenever the chip has data available
	 * for reading.  it is cleared when all data has been read.
	 */
	pr_err("[pn547]%s : requesting IRQ %d\n", __func__, platform_data->irq_gpio);
	pr_err("[pn547]%s : client->name %s\n", __func__, client->name);
	pn544_dev->irq_enabled = true;
	//irq = gpio_to_irq(platform_data->irq_gpio);
        ret = request_irq(client->irq, pn544_dev_irq_handler, IRQF_TRIGGER_HIGH, client->name, pn544_dev);
	if (ret)  
	{
		dev_err(&client->dev, "request_irq failed\n");
		goto err_request_irq_failed;
	}
	
	pn544_disable_irq(pn544_dev);
	i2c_set_clientdata(client, pn544_dev);

	return 0;

err_request_irq_failed:
	misc_deregister(&pn544_dev->pn544_device);
err_misc_register:
	mutex_destroy(&pn544_dev->read_mutex);
	kfree(pn544_dev);
err_clkreq_gpio:
	gpio_free(pn544_dev->clkreq_gpio);
	
err_clk:
	clk_disable_unprepare(pn544_dev->s_clk);
err_exit:
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);

	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_dev *pn544_dev;

	pn544_dev = i2c_get_clientdata(client);
	free_irq(client->irq, pn544_dev);
	misc_deregister(&pn544_dev->pn544_device);
	mutex_destroy(&pn544_dev->read_mutex);
	clk_disable_unprepare(pn544_dev->s_clk);
	gpio_free(pn544_dev->clkreq_gpio);
	gpio_free(pn544_dev->irq_gpio);
	gpio_free(pn544_dev->ven_gpio);
	gpio_free(pn544_dev->firm_gpio);
	kfree(pn544_dev);

	return 0;
}

static const struct i2c_device_id pn544_id[] = {
	{ "pn544", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pn544_id);

static const struct of_device_id pn544_of_match[] = {
	{ .compatible = "nfc,pn544", },
	{ },
};
 
static struct i2c_driver pn544_driver = {
	.id_table	= pn544_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.driver		= 
	{
		.owner	= THIS_MODULE,
		.name	= "pn544",
		.of_match_table = pn544_of_match,
	},
};

/*
 * module load/unload record keeping
 */

static int __init pn544_dev_init(void)
{
	pr_err("[pn547]Loading pn544 driver\n");
	return i2c_add_driver(&pn544_driver);
}
module_init(pn544_dev_init);

static void __exit pn544_dev_exit(void)
{
	pr_err("[pn547]Unloading pn544 driver\n");
	i2c_del_driver(&pn544_driver);
}
module_exit(pn544_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION("NFC PN544 driver");
MODULE_LICENSE("GPL");
