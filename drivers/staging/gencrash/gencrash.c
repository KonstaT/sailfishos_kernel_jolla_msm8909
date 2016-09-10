/*
 * Crash generator - generates different crashes in kernel space.
 *
 * Copyright (C) 2015 Jolla Ltd.
 * Contact: Artem Balynin <artem.balynin@symphonyteleca.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>

static void gencrash_div_by_zero(void)
{
	int a = 0, b = 0, c = 0;
	pr_err("Div by zero test\n");
	c = a / b;
	pr_err("Div by zero result = %d\n", c);
}

static void gencrash_null_ptr(void)
{
	int *ptr = NULL;
	pr_err("Null pointer test\n");
	pr_err("Null pointer value = %d\n", *ptr);
}

static void gencrash_stack_corrupt(void)
{
	u8 buf[10];
	memset(buf, 0, 20);
}

static void gencrash_mem_corrupt(void)
{
	int *buf = NULL;
	pr_err("Memory corruption test\n");
	buf = kmalloc(10, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memset(buf, 0, 20);
	pr_err("buf[10] = %d\n", buf[10]);
}

static void gencrash_handle(char key)
{
	int opt = 0;
	if ((key >= '0') && (key <= '9'))
		opt = key - '0';
	switch (opt) {
		case 1:
		gencrash_div_by_zero();
		break;
		case 2:
		gencrash_null_ptr();
		break;
		case 3:
		gencrash_stack_corrupt();
		break;
		case 4:
		gencrash_mem_corrupt();
		default:
		break;
	}

}

static ssize_t gencrash_write(struct file *file, const char __user *buf,
				   size_t count, loff_t *ppos)
{
	if (count) {
		char c;

		if (get_user(c, buf))
			return -EFAULT;
		gencrash_handle(c);
	}

	return count;
}

static const struct file_operations proc_gencrash_operations = {
	.write		= gencrash_write,
	.llseek		= noop_llseek,
};


static int __init gencrash_init(void)
{
	if (!proc_create("gen-crash", S_IWUSR, NULL,
			 &proc_gencrash_operations)) {
		pr_err("Failed to register proc interface\n");
		return -ENOMEM;
	}
	return 0;
}

static void __exit gencrash_exit(void)
{
	remove_proc_entry("gen-crash", NULL);
}

module_init(gencrash_init);
module_exit(gencrash_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Artem Balynin <Artem.Balynin@symphonyteleca.com>");
MODULE_DESCRIPTION("Crash generator in kernel");
