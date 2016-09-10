/*
 * Kmsg dumper - kmsg log dumper storing crashes to FS.
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
#include <linux/kmsg_dump.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/printk.h>
#include <linux/slab.h>

#define KMSG_DUMPER_RECORD_SIZE 32768
#define LOG_FILE_PATH "/var/log/lastlog_kmsg"

static struct kmsg_dumper_context {
	struct kmsg_dumper dump;
	void *oops_buf;
} oops_cxt;


static void kmsgdumper_store_logs(char *oops_buf, int size)
{
	ssize_t n = 0;
	loff_t offset = 0;
	mm_segment_t fs;
	struct file *f;

	// remove memory space limitation of FS operations in kernel
	fs = get_fs();
	set_fs(get_ds());

	f = filp_open(LOG_FILE_PATH, O_CREAT | O_RDWR | O_TRUNC, S_IRUSR | S_IWUSR);
	if ( IS_ERR(f) )
		return;

	if ( ( n = vfs_write(f, oops_buf, size, &offset)) != size ) {
		filp_close(f, NULL);
		// return back memory space limitation
		set_fs(fs);
		return;
	}

	vfs_fsync(f, 0);

	filp_close(f, NULL);

	// return back memory space limitation
	set_fs(fs);
}

static void kmsgdumper_do_dump(struct kmsg_dumper *dumper,
			    enum kmsg_dump_reason reason)
{
	struct kmsg_dumper_context *cxt = container_of(dumper,
			struct kmsg_dumper_context, dump);

	kmsg_dump_get_buffer(dumper, true, cxt->oops_buf,
			     KMSG_DUMPER_RECORD_SIZE, NULL);
	kmsgdumper_store_logs((char *)cxt->oops_buf, KMSG_DUMPER_RECORD_SIZE);

}

static int __init kmsgdumper_init(void)
{
	struct kmsg_dumper_context *cxt = &oops_cxt;
	int err;

	printk(KERN_INFO "kmsgdumper_init start\n");

	cxt->oops_buf = kzalloc(KMSG_DUMPER_RECORD_SIZE, GFP_KERNEL);
	if (!cxt->oops_buf) {
		printk(KERN_ERR "kmsgdumper: failed to allocate buffer workspace\n");
		return -ENOMEM;
	}

	cxt->dump.dump = kmsgdumper_do_dump;
	err = kmsg_dump_register(&cxt->dump);
	if (err)
		printk(KERN_ERR "kmsgdumper: registering kmsg dumper failed, error %d\n", err);

	return 0;
}

static void __exit kmsgdumper_exit(void)
{
	struct kmsg_dumper_context *cxt = &oops_cxt;
	printk(KERN_INFO "kmsgdumper_exit stop\n");
	if (kmsg_dump_unregister(&cxt->dump) < 0)
		printk(KERN_WARNING "kmsgdumper: could not unregister kmsg_dumper\n");

	kfree(cxt->oops_buf);
}

module_init(kmsgdumper_init);
module_exit(kmsgdumper_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Artem Balynin <Artem.Balynin@symphonyteleca.com>");
MODULE_DESCRIPTION("Kmsg Oops/Panic dumper");
