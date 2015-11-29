/*
 * kernel module helper for testing CMA
 *
 * Copyright (c) 2012 Cambridge Silicon Radio Limited, a CSR plc group company.
 *
 * Licensed under GPLv2 or later.
 */

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

struct cma_allocation {
	struct list_head list;
	size_t size;
	dma_addr_t dma;
	void *virt;
};

static struct device *cma_dev;
static LIST_HEAD(cma_allocations);
static DEFINE_SPINLOCK(cma_lock);

/*
 * any read request will free the 1st allocated coherent memory, eg.
 * cat /dev/cma_test
 */
static ssize_t
cma_test_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct cma_allocation *alloc = NULL;

	spin_lock(&cma_lock);
	if (!list_empty(&cma_allocations)) {
		alloc = list_first_entry(&cma_allocations,
			struct cma_allocation, list);
		list_del(&alloc->list);
	}
	spin_unlock(&cma_lock);

	if (!alloc)
		return -EIDRM;

	dma_free_coherent(cma_dev, alloc->size, alloc->virt,
		alloc->dma);

	_dev_info(cma_dev, "free: CM virt: %p dma: %p size:%zuK\n",
		alloc->virt, (void *)alloc->dma, alloc->size / SZ_1K);
	kfree(alloc);

	return 0;
}

/*
 * any write request will alloc a new coherent memory, eg.
 * echo 1024 > /dev/cma_test
 * will request 1024KiB by CMA
 */
static ssize_t
cma_test_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct cma_allocation *alloc;
	unsigned long size;
	int ret;

	ret = kstrtoul_from_user(buf, count, 0, &size);
	if (ret)
		return ret;

	if (!size)
		return -EINVAL;

	if (size > ~(size_t)0 / SZ_1K)
		return -EOVERFLOW;

	alloc = kmalloc(sizeof *alloc, GFP_KERNEL);
	if (!alloc)
		return -ENOMEM;

	alloc->size = size * SZ_1K;
	alloc->virt = dma_alloc_coherent(cma_dev, alloc->size,
		&alloc->dma, GFP_KERNEL);

	if (alloc->virt) {
		_dev_info(cma_dev, "alloc: virt: %p dma: %p size: %zuK\n",
			alloc->virt, (void *)alloc->dma, alloc->size / SZ_1K);

		spin_lock(&cma_lock);
		list_add_tail(&alloc->list, &cma_allocations);
		spin_unlock(&cma_lock);

		return count;
	} else {
		dev_err(cma_dev, "no mem in CMA area\n");
		kfree(alloc);
		return -ENOSPC;
	}
}

int scull_open(struct inode *inode, struct file *filp)
{
printk("TEST: opened");
return 0;
}

int scull_release(struct inode *inode, struct file *filp)
{
printk("TEST: released");
return 0;
}

static const struct file_operations cma_test_fops = {
	.owner =    THIS_MODULE,
	.read  =    cma_test_read,
	.write =    cma_test_write,
	.open  =    scull_open,
	.release =  scull_release,
};

static struct miscdevice cma_test_misc = {
	.name = "cma_test",
	.fops = &cma_test_fops,
};

static int __init cma_test_init(void)
{
	int ret = misc_register(&cma_test_misc);

	if (unlikely(ret)) {
		pr_err("failed to register cma test misc device!\n");
		return ret;
	}
	cma_dev = cma_test_misc.this_device;
	cma_dev->coherent_dma_mask = ~0;
	_dev_info(cma_dev, "registered.\n");

	return 0;
}
module_init(cma_test_init);

static void __exit cma_test_exit(void)
{
	misc_deregister(&cma_test_misc);
}
module_exit(cma_test_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Barry Song <Baohua.Song@csr.com>");
MODULE_DESCRIPTION("kernel module to help the test of CMA");
MODULE_ALIAS("cma_test");
