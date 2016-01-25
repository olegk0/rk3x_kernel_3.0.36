/*------------------------------------------------------------------------------
--                                                                            --
--       This software is confidential and proprietary and may be used        --
--        only as expressly authorized by a licensing agreement from          --
--                                                                            --
--                            Google Finland Oy.                              --
--                                                                            --
--                   (C) COPYRIGHT 2012 GOOGLE FINLAND OY                     --
--                            ALL RIGHTS RESERVED                             --
--                                                                            --
--                 The entire notice above must be reproduced                 --
--                  on all copies and should not be removed.                  --
--                                                                            --
--------------------------------------------------------------------------------
--
--  Abstract : x170 Decoder device driver (kernel module)
--
------------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/module.h>
/* needed for __init,__exit directives */
#include <linux/init.h>
/* needed for remap_pfn_range
	SetPageReserved
	ClearPageReserved
*/
#include <linux/mm.h>
/* obviously, for kmalloc */
#include <linux/slab.h>
/* for struct file_operations, register_chrdev() */
#include <linux/fs.h>
/* standard error codes */
#include <linux/errno.h>

#include <linux/moduleparam.h>
/* request_irq(), free_irq() */
#include <linux/interrupt.h>

/* needed for virt_to_phys() */
#include <asm/io.h>
#include <linux/pci.h>
#include <asm/uaccess.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/signal.h>
#include <linux/sched.h>

/* our own stuff */
#include "hx170dec.h"

/* module description */
//MODULE_LICENSE("Proprietary");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Google Finland Oy");
MODULE_DESCRIPTION("driver module for 8170/81990 Hantro decoder/pp");

/* Decoder interrupt register */
#define X170_INTERRUPT_REGISTER_DEC     (1*4)
//#define X170_INTERRUPT_REGISTER_DEC     (1)
#define X170_INTERRUPT_REGISTER_PP      (60*4)
//#define X170_INTERRUPT_REGISTER_PP      (60)

/* Logic module base address */
#define HXDEC_LOGIC_MODULE0_BASE   0xC0000000

#define VP_PB_INT_LT                    30

#define INT_EXPINT1                     10
#define INT_EXPINT2                     11
#define INT_EXPINT3                     12

/* these could be module params in the future */
	
#define RK30_VCODEC_DEC_OFFSET	    (0x200*2)
#define VPU_IO_BASE                 RK30_VCODEC_PHYS
#define VPU_IO_SIZE                 RK30_VCODEC_DEC_OFFSET+DEC_IO_SIZE

#define DEC_IO_OFFSET               RK30_VCODEC_DEC_OFFSET
#define DEC_IO_SIZE                 ((100+1) * 4)   /* bytes */
#define DEC_IRQ                     IRQ_VDPU

#define HX_DEC_INTERRUPT_BIT        0x100
#define HX_PP_INTERRUPT_BIT         0x100

static const int DecHwId[] = { 0x8190, 0x8170, 0x9170, 0x9190, 0x6731 };

static const char hx170dec_dev_name[] = "hx170";

struct hx170dec_private{
	dev_t dev;
	u32 hx_pp_instance;
	u32 hx_dec_instance;
//struct clk *video_dec_clk;
	struct clk *pd_video;
	struct clk *aclk_vepu;
	struct clk *hclk_vepu;
	struct cdev cdev;
	struct class *class;
	/* and this is our MAJOR; use 0 for dynamic allocation (recommended)*/
	int dev_major;
};

static struct hx170dec_private hx170dec_priv;

/* module_param(name, type, perm) */
//module_param(base_port, ulong, 0);
//module_param(irq, int, 0);

/* here's all the must remember stuff */
typedef struct
{
    char *buffer;
    unsigned long dec_io_offset;
    unsigned int dec_io_size;
    volatile u8 *dec_hwregs;
    volatile u8 *enc_hwregs;
    int irq;
#ifdef ENABLE_FASYNC
    struct fasync_struct *async_queue_dec;
    struct fasync_struct *async_queue_pp;
#else
    wait_queue_head_t	wait;
    int dec_irq;
    int pp_irq;
#endif
    atomic_t		irq_count_codec;
	atomic_t		irq_count_pp;
} hx170dec_t;

static hx170dec_t hx170dec_data;

#ifdef HW_PERFORMANCE
static struct timeval end_time;
#endif

static int ReserveIO(void);
static void ReleaseIO(void);

static void ResetAsic(hx170dec_t * dev);

#ifdef HX170DEC_DEBUG
static void dump_regs(unsigned long data);
#endif

static int irq_def = DEC_IRQ;
module_param(irq_def, int, S_IRUGO);

/* IRQ handler */
static irqreturn_t hx170dec_isr(int irq, void *dev_id);
static irqreturn_t hx170dec_thread(int irq, void *dev_id);

//***********************************From RockChip vpu_service***************************
#define MHZ					(1000*1000)

#define VPU_POWER_OFF_DELAY		4*HZ /* 4s */

typedef struct vpu_service_info {
//	struct delayed_work	power_off_work;
	struct mutex		lock;
//	struct list_head	waiting;		/* link to link_reg in struct vpu_reg */
//	struct list_head	running;		/* link to link_reg in struct vpu_reg */
//	struct list_head	done;			/* link to link_reg in struct vpu_reg */
//	struct list_head	session;		/* link to list_session in struct vpu_session */
	atomic_t		total_running;
	bool			enabled;
//	vpu_reg			*reg_codec;
//	vpu_reg			*reg_pproc;
//	vpu_reg			*reg_resev;
//	VPUHwDecConfig_t	dec_config;
//	VPUHwEncConfig_t	enc_config;
//	VPU_HW_INFO_E		*hw_info;
//	unsigned long		reg_size;
//	bool			auto_freq;
	atomic_t		freq_status;
} vpu_service_info;

static vpu_service_info service;

static void vpu_service_set_freq(VPU_FREQ freq)
{
    int fr;
	VPU_FREQ curr = atomic_read(&service.freq_status);
	if (curr == freq) {
		return ;
	}
	atomic_set(&service.freq_status, freq);    
	switch (freq) {
	case VPU_FREQ_200M : {
		clk_set_rate(hx170dec_priv.aclk_vepu, 200*MHZ);
        fr = 200;
		//printk("default: 200M\n");
	} break;
	case VPU_FREQ_266M : {
		clk_set_rate(hx170dec_priv.aclk_vepu, 266*MHZ);
        fr = 266;
		//printk("default: 266M\n");
	} break;
	case VPU_FREQ_300M : {
		clk_set_rate(hx170dec_priv.aclk_vepu, 300*MHZ);
        fr = 300;
		//printk("default: 300M\n");
	} break;
	case VPU_FREQ_400M : {
		clk_set_rate(hx170dec_priv.aclk_vepu, 400*MHZ);
        fr = 400;
		//printk("default: 400M\n");
	} break;
	default : {
		clk_set_rate(hx170dec_priv.aclk_vepu, 300*MHZ);
        fr = 300;
		//printk("default: 300M\n");
	} break;
	}
    printk("vpu_service_set_freq:%d MHz\n",fr);
}

static int vpu_get_clk(void)
{
	int ret =0;
	
	hx170dec_priv.pd_video	= clk_get(NULL, "pd_video");
	if (IS_ERR(hx170dec_priv.pd_video)) {
		ret = -1;
		pr_err("failed on clk_get pd_video\n");
	}
	hx170dec_priv.aclk_vepu 	= clk_get(NULL, "aclk_vepu");
	if (IS_ERR(hx170dec_priv.aclk_vepu)) {
		ret = -1;
		pr_err("failed on clk_get aclk_vepu\n");
	}
	hx170dec_priv.hclk_vepu 	= clk_get(NULL, "hclk_vepu");
	if (IS_ERR(hx170dec_priv.hclk_vepu)) {
		ret = -1;
		pr_err("failed on clk_get hclk_vepu\n");
	}

	return ret;
}

static void vpu_put_clk(void)
{
	clk_put(hx170dec_priv.pd_video);
	clk_put(hx170dec_priv.aclk_vepu);
	clk_put(hx170dec_priv.hclk_vepu);

}

static void vpu_service_power_off(void)
{
	int total_running;
	if (!service.enabled) {
		return;
	}

	service.enabled = false;
	total_running = atomic_read(&service.total_running);
	if (total_running) {
		pr_alert("alert: power off when %d task running!!\n", total_running);
		mdelay(50);
		pr_alert("alert: delay 50 ms for running task\n");
//		vpu_service_dump();
	}

	printk("vpu: power off...");
#ifdef CONFIG_ARCH_RK29
	pmu_set_power_domain(PD_VCODEC, false);
#else
	clk_disable(hx170dec_priv.pd_video);
#endif
	udelay(10);
//	clk_disable(aclk_ddr_vepu);
	clk_disable(hx170dec_priv.hclk_vepu);
	clk_disable(hx170dec_priv.aclk_vepu);
	printk("done\n");
}
/*
static inline void vpu_queue_power_off_work(void)
{
    PDEBUG("vpu_queue_power_off_work");
	queue_delayed_work(system_nrt_wq, &service.power_off_work, VPU_POWER_OFF_DELAY);
}

static void vpu_power_off_work(struct work_struct *work)
{
    PDEBUG("vpu_power_off_work");
	if (mutex_trylock(&service.lock)) {
		vpu_service_power_off();
		mutex_unlock(&service.lock);
	} else {
		// Come back later if the device is busy... 
		vpu_queue_power_off_work();
	}
}
*/
static int vpu_service_power_on(void)
{
/*	static ktime_t last;
	ktime_t now = ktime_get();
	if (ktime_to_ns(ktime_sub(now, last)) > NSEC_PER_SEC) {
//		cancel_delayed_work_sync(&service.power_off_work);
//		vpu_queue_power_off_work();
		last = now;
	}
	*/
	if (service.enabled)
		return 0;

	service.enabled = true;
	printk("vpu: power on\n");

	clk_enable(hx170dec_priv.aclk_vepu);
	clk_enable(hx170dec_priv.hclk_vepu);
	udelay(10);
#ifdef CONFIG_ARCH_RK29
	pmu_set_power_domain(PD_VCODEC, true);
#else
	clk_enable(hx170dec_priv.pd_video);
#endif
	udelay(10);
//	clk_enable(aclk_ddr_vepu);
	
	return 0;
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_ioctl
    Description     : communication method to/from the user space

    Return type     : long
------------------------------------------------------------------------------*/
static long hx170dec_ioctl(struct file *filp,
		unsigned int cmd, unsigned long arg)
{
    int err = 0, tmp;
    VPU_FREQ ufreq;
    
#ifdef HW_PERFORMANCE
    struct timeval *end_time_arg;
#endif

    PDEBUG("ioctl cmd 0x%08ux\n", cmd);
    /*
     * extract the type and number bitfields, and don't decode
     * wrong cmds: return ENOTTY (inappropriate ioctl) before access_ok()
     */
    if(_IOC_TYPE(cmd) != HX170DEC_IOC_MAGIC)
        return -ENOTTY;
    if(_IOC_NR(cmd) > HX170DEC_IOC_MAXNR)
        return -ENOTTY;

    /*
     * the direction is a bitmask, and VERIFY_WRITE catches R/W
     * transfers. `Type' is user-oriented, while
     * access_ok is kernel-oriented, so the concept of "read" and
     * "write" is reversed
     */
    if(_IOC_DIR(cmd) & _IOC_READ)
        err = !access_ok(VERIFY_WRITE, (void *) arg, _IOC_SIZE(cmd));
    else if(_IOC_DIR(cmd) & _IOC_WRITE)
        err = !access_ok(VERIFY_READ, (void *) arg, _IOC_SIZE(cmd));
    if(err)
        return -EFAULT;

    switch (cmd)
    {
    case HX170DEC_IOC_CLI:
        disable_irq(hx170dec_data.irq);
        break;

    case HX170DEC_IOC_STI:
        enable_irq(hx170dec_data.irq);
        break;
    case HX170DEC_IOCGHWOFFSET:
        err = put_user(hx170dec_data.dec_io_offset, (unsigned long *) arg);
        break;
    case HX170DEC_IOCGHWIOSIZE:
        err = put_user(hx170dec_data.dec_io_size, (unsigned long *) arg);
        break;
#ifdef ENABLE_FASYNC
    case HX170DEC_PP_INSTANCE:
        filp->private_data = &hx170dec_priv.hx_pp_instance;
        break;
#else
    case HX170DEC_IOC_WAIT_DEC:
        PDEBUG("HX170DEC_IOC_WAIT_DEC\n");
        err = get_user(tmp, (unsigned long *) arg);
        if(!err){
            if(tmp == 0){
                hx170dec_data.dec_irq = 0;
                PDEBUG("HX170DEC_IOC_WAIT_DEC -prepare\n");
            }
            else{
                err = wait_event_interruptible_timeout(hx170dec_data.wait, hx170dec_data.dec_irq, msecs_to_jiffies(tmp));
                PDEBUG("HX170DEC_IOC_WAIT_DEC++\n");
            }
        }
        break;
    case HX170DEC_IOC_WAIT_PP:
        PDEBUG("HX170DEC_IOC_WAIT_PP\n");
        err = get_user(tmp, (unsigned long *) arg);
        if(!err){
            if(tmp == 0){
                hx170dec_data.pp_irq = 0;
                PDEBUG("HX170DEC_IOC_WAIT_PP -prepare\n");
            }
            else{
                err = wait_event_interruptible_timeout(hx170dec_data.wait, hx170dec_data.pp_irq, msecs_to_jiffies(tmp));
                PDEBUG("HX170DEC_IOC_WAIT_PP++\n");
            }
        }
        break;
#endif
#ifdef HW_PERFORMANCE
    case HX170DEC_HW_PERFORMANCE:
        end_time_arg = (struct timeval *) arg;
        end_time_arg->tv_sec = end_time.tv_sec;
        end_time_arg->tv_usec = end_time.tv_usec;
        break;
#endif
    case HX170DEC_GET_VPU_FREQ:
        ufreq = atomic_read(&service.freq_status);
        err = put_user(ufreq, (unsigned long *) arg);
        break;
    case HX170DEC_SET_VPU_FREQ:        
        err = get_user(ufreq, (unsigned long *) arg);
        vpu_service_set_freq(ufreq);
        break;
    }
    return err;
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_open
    Description     : open method

    Return type     : int
------------------------------------------------------------------------------*/

static int hx170dec_open(struct inode *inode, struct file *filp)
{
    filp->private_data = &hx170dec_priv.hx_dec_instance;

	atomic_inc(&service.total_running);
	vpu_service_power_on();
    
    PDEBUG("dev opened, services:%d\n",atomic_read(&service.total_running));
    return 0;
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_fasync
    Description     : Method for signing up for a interrupt

    Return type     : int
------------------------------------------------------------------------------*/
#ifdef ENABLE_FASYNC
static int hx170dec_fasync(int fd, struct file *filp, int mode)
{

    hx170dec_t *dev = &hx170dec_data;
    struct fasync_struct **async_queue;

    /* select which interrupt this instance will sign up for */

    if(((u32 *) filp->private_data) == &hx170dec_priv.hx_dec_instance)
    {
        /* decoder */
        PDEBUG("decoder fasync called %d %x %d %x\n",
               fd, (u32) filp, mode, (u32) & dev->async_queue_dec);

        async_queue = &dev->async_queue_dec;
    }
    else
    {
        /* pp */
        PDEBUG("pp fasync called %d %x %d %x\n",
               fd, (u32) filp, mode, (u32) & dev->async_queue_pp);
        async_queue = &dev->async_queue_pp;
    }

    return fasync_helper(fd, filp, mode, async_queue);
}
#endif
/*------------------------------------------------------------------------------
    Function name   : hx170dec_release
    Description     : Release driver

    Return type     : int
------------------------------------------------------------------------------*/

static int hx170dec_release(struct inode *inode, struct file *filp)
{

    /* hx170dec_t *dev = &hx170dec_data; */
#ifdef ENABLE_FASYNC
    if(filp->f_flags & FASYNC)
    {
        /* remove this filp from the asynchronusly notified filp's */
        hx170dec_fasync(-1, filp, 0);
    }
#endif
    if(atomic_dec_and_test(&service.total_running))
        vpu_service_power_off();
//        vpu_queue_power_off_work();

    PDEBUG("dev closed, services:%d\n",atomic_read(&service.total_running));
    return 0;
}
//++++++++++++++++++++++++++++++++++++++++++++++++
/*
void hx170dec_vma_open(struct vm_area_struct *vma)
{
//	printk(KERN_NOTICE "hx170dec VMA open, virt %lx, phys %lx\n",
//			vma->vm_start, vma->vm_pgoff << PAGE_SHIFT);
    PDEBUG("hx170dec_vma_open\n");
}

void hx170dec_vma_close(struct vm_area_struct *vma)
{
//	printk(KERN_NOTICE "hx170dec VMA close.\n");
    PDEBUG("hx170dec_vma_close\n");
}

static struct vm_operations_struct hx170dec_mmap_ops = {
	.open =  hx170dec_vma_open,
	.close = hx170dec_vma_close,
};
*/
static int hx170dec_mmap(struct file *filp, struct vm_area_struct *vma)
{
	u32 size = vma->vm_end - vma->vm_start;

	if( size > PAGE_SIZE){
		printk("size:%d > PAGE_SIZE:%lu\n",size, PAGE_SIZE);
		return -EINVAL;
	}
    
    vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);    
	if (remap_pfn_range(vma, vma->vm_start, VPU_IO_BASE >> PAGE_SHIFT,
		size, vma->vm_page_prot))
		return -EAGAIN;

//	vma->vm_ops = &hx170dec_mmap_ops;
//	hx170dec_vma_open(vma);
    PDEBUG("hx170dec_mmap\n");
	return 0;
}

/* VFS methods */
static const struct file_operations hx170dec_fops = {
    .open		= hx170dec_open,
    .release	= hx170dec_release,
    .unlocked_ioctl	= hx170dec_ioctl,
#ifdef ENABLE_FASYNC
    .fasync		= hx170dec_fasync,
#endif
	.mmap = hx170dec_mmap,
};

int hx170dec_sysfs_register(struct hx170dec_private *device, dev_t dev,
		const char *hx170dec_dev_name)
{
    int err = 0;
    struct device *mdev;

    device->class = class_create(THIS_MODULE, hx170dec_dev_name);
    if (IS_ERR(device->class)) {
	err = PTR_ERR(device->class);
	goto init_class_err;
    }
    mdev = device_create(device->class, NULL, dev, NULL,
		hx170dec_dev_name);
    if (IS_ERR(mdev)) {
	err = PTR_ERR(mdev);
	goto init_mdev_err;
    }

    /* Success! */
    return 0;

init_mdev_err:
    class_destroy(device->class);
init_class_err:

    return err;
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_init
    Description     : Initialize the driver
    Return type     : int
------------------------------------------------------------------------------*/

int __init hx170dec_init(void)
{
    int result;

	hx170dec_priv.dev = 0;
	hx170dec_priv.hx_pp_instance = 0;
	hx170dec_priv.hx_dec_instance = 0;
	hx170dec_priv.dev_major = 0;

    hx170dec_data.dec_io_offset = DEC_IO_OFFSET;
    hx170dec_data.dec_io_size = DEC_IO_SIZE;
    hx170dec_data.irq = irq_def;

    PDEBUG("module init\n");

    printk(KERN_INFO "hx170dec: dec/pp kernel module. %s \n", "$Revision: 1.12 $");
    printk(KERN_INFO "hx170dec: supports 8170 and 8190 hardware \n");
    printk(KERN_INFO "hx170dec: base_port=0x%08x irq=%i\n", VPU_IO_BASE, hx170dec_data.irq);
#ifdef ENABLE_FASYNC
    hx170dec_data.async_queue_dec = NULL;
    hx170dec_data.async_queue_pp = NULL;
#else
    init_waitqueue_head(&hx170dec_data.wait);
#endif
//*********************	
	mutex_init(&service.lock);
//	service.reg_codec	= NULL;
//	service.reg_pproc	= NULL;
	atomic_set(&service.total_running, 0);
	service.enabled		= false;

	vpu_get_clk();

//	INIT_DELAYED_WORK(&service.power_off_work, vpu_power_off_work);
//***********************
	atomic_set(&hx170dec_data.irq_count_codec, 0);
	atomic_set(&hx170dec_data.irq_count_pp, 0);
    
	vpu_service_set_freq(VPU_FREQ_266M);

	result = vpu_service_power_on();
    if (result) {
		printk(KERN_ERR "Can't enable clock\n");
		goto put_clk;
    }

   if (0 == hx170dec_priv.dev_major) {
	/* auto select a major */
		result = alloc_chrdev_region(&hx170dec_priv.dev, 0, 1, hx170dec_dev_name);
		hx170dec_priv.dev_major = MAJOR(hx170dec_priv.dev);
    } else {
	/* use load time defined major number */
		hx170dec_priv.dev = MKDEV(hx170dec_priv.dev_major, 0);
		result = register_chrdev_region(hx170dec_priv.dev, 1, hx170dec_dev_name);
    }

    if (result)
		goto put_clk;


    memset(&hx170dec_priv.cdev, 0, sizeof(hx170dec_priv.cdev));

    /* initialize our char dev data */
    cdev_init(&hx170dec_priv.cdev, &hx170dec_fops);
    hx170dec_priv.cdev.owner = THIS_MODULE;
    hx170dec_priv.cdev.ops = &hx170dec_fops;

    /* register char dev with the kernel */
    result = cdev_add(&hx170dec_priv.cdev, hx170dec_priv.dev, 1/*count*/);
    if (result)
		goto init_cdev_err;

    result = hx170dec_sysfs_register(&hx170dec_priv, hx170dec_priv.dev, hx170dec_dev_name);
    if (result)
		goto init_sysfs_err;

    result = ReserveIO();
    if(result < 0)
    {
        goto err;
    }

    ResetAsic(&hx170dec_data);  /* reset hardware */
    /* get the IRQ line */
    if(hx170dec_data.irq > 0)
    {
        result = request_threaded_irq(hx170dec_data.irq, hx170dec_isr, hx170dec_thread, 0, "vdpu", (void *)&hx170dec_data);
        if(result != 0)
        {
            if(result == -EINVAL)
            {
                printk(KERN_ERR "hx170dec: Bad irq number or handler\n");
            }
            else if(result == -EBUSY)
            {
                printk(KERN_ERR "hx170dec: IRQ <%d> busy, change your config\n",
                       hx170dec_data.irq);
            }

            ReleaseIO();
            goto err;
        }
    }
    else
    {
        printk(KERN_INFO "hx170dec: IRQ not in use!\n");
    }

    printk(KERN_INFO "hx170dec: module inserted. Major = %d\n", hx170dec_priv.dev_major);
    
    vpu_service_power_off();
//    vpu_queue_power_off_work();
    return 0;

err:
	device_destroy(hx170dec_priv.class, hx170dec_priv.dev);
    class_destroy(hx170dec_priv.class);
init_sysfs_err:
    cdev_del(&hx170dec_priv.cdev);
init_cdev_err:
    unregister_chrdev_region(hx170dec_priv.dev, 1/*count*/);
put_clk:
	vpu_service_power_off();
	vpu_put_clk();
    printk(KERN_INFO "hx170dec: module not inserted\n");
    return result;
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_cleanup
    Description     : clean up
    Return type     : int
------------------------------------------------------------------------------*/

void __exit hx170dec_cleanup(void)
{
//    hx170dec_t *dev = (hx170dec_t *) & hx170dec_data;

    /* clear dec IRQ */
    writel(0, hx170dec_data.dec_hwregs + X170_INTERRUPT_REGISTER_DEC);
    /* clear pp IRQ */
    writel(0, hx170dec_data.dec_hwregs + X170_INTERRUPT_REGISTER_PP);

#ifdef HX170DEC_DEBUG
    dump_regs((unsigned long) &hx170dec_data); /* dump the regs */
#endif

    /* free the IRQ */
    if(hx170dec_data.irq != -1)
    {
        free_irq(hx170dec_data.irq, (void *) &hx170dec_data);
    }
    hx170dec_data.pp_irq = 1;
    hx170dec_data.dec_irq = 1;
    wake_up_interruptible_sync(&hx170dec_data.wait);
    
    ReleaseIO();
	vpu_service_power_off();
	device_destroy(hx170dec_priv.class, hx170dec_priv.dev);
    class_destroy(hx170dec_priv.class);
	cdev_del(&hx170dec_priv.cdev);
    unregister_chrdev_region(hx170dec_priv.dev, 1/*count*/);
	
	vpu_put_clk();
    printk(KERN_INFO "hx170dec: module removed\n");
    return;
}

module_init(hx170dec_init);
module_exit(hx170dec_cleanup);

static int CheckHwId(hx170dec_t * dev)
{
    long int hwid;

    size_t numHw = sizeof(DecHwId) / sizeof(*DecHwId);

    hwid = readl(dev->dec_hwregs);
    printk(KERN_INFO "hx170dec: HW ID=0x%08lx\n", hwid);

    hwid = (hwid >> 16) & 0xFFFF;   /* product version only */

    while(numHw--)
    {
        if(hwid == DecHwId[numHw])
        {
            printk(KERN_INFO "hx170dec: Compatible HW found at 0x%08lx\n", dev->dec_hwregs);
            return 1;
        }
    }

    printk(KERN_INFO "hx170dec: No Compatible HW found at offset 0x%08lx\n",
           dev->dec_io_offset);
    return 0;
}

/*------------------------------------------------------------------------------
    Function name   : ReserveIO
    Description     : IO reserve
    Return type     : int
------------------------------------------------------------------------------*/
static int ReserveIO(void)
{
    if(!request_mem_region(VPU_IO_BASE, VPU_IO_SIZE, "hx170dec"))
    {
        printk(KERN_INFO "hx170dec: failed to reserve HW regs\n");
        return -EBUSY;
    }

    hx170dec_data.enc_hwregs =
        (volatile u8 *) ioremap_nocache(VPU_IO_BASE, VPU_IO_SIZE);

    if(hx170dec_data.enc_hwregs == NULL)
    {
        printk(KERN_INFO "hx170dec: failed to ioremap HW regs\n");
        ReleaseIO();
        return -EBUSY;
    }
    hx170dec_data.dec_hwregs = hx170dec_data.enc_hwregs + hx170dec_data.dec_io_offset;
    /* check for correct HW */
    if(!CheckHwId(&hx170dec_data))
    {
        ReleaseIO();
        return -EBUSY;
    }

    return 0;
}

/*------------------------------------------------------------------------------
    Function name   : releaseIO
    Description     : release
    Return type     : void
------------------------------------------------------------------------------*/

static void ReleaseIO(void)
{
    if(hx170dec_data.dec_hwregs)
        iounmap((void *) hx170dec_data.dec_hwregs);
    release_mem_region(VPU_IO_BASE, VPU_IO_SIZE);
}

/*------------------------------------------------------------------------------
    Function name   : hx170dec_isr
    Description     : interrupt handler
    Return type     : irqreturn_t
------------------------------------------------------------------------------*/
static irqreturn_t hx170dec_isr(int irq, void *dev_id)
{
	hx170dec_t *dev = (hx170dec_t *) dev_id;
	u32 irq_status = readl(dev->dec_hwregs + X170_INTERRUPT_REGISTER_DEC);

	PDEBUG("vdpu_irq\n");

	if (irq_status & HX_DEC_INTERRUPT_BIT) {
		PDEBUG("vdpu_isr dec %x\n", irq_status);
		if ((irq_status & 0x40001) == 0x40001)
		{
			do {
				irq_status = readl(dev->dec_hwregs + X170_INTERRUPT_REGISTER_DEC);
			} while ((irq_status & 0x40001) == 0x40001);
		}
		/* clear dec IRQ */
		writel(irq_status & (~HX_DEC_INTERRUPT_BIT), dev->dec_hwregs + X170_INTERRUPT_REGISTER_DEC);
		atomic_add(1, &dev->irq_count_codec);
	}

	irq_status  = readl(dev->dec_hwregs + X170_INTERRUPT_REGISTER_PP);
	if (irq_status & HX_PP_INTERRUPT_BIT) {
		PDEBUG("vdpu_isr pp  %x\n", irq_status);
		/* clear pp IRQ */
		writel(irq_status & (~HX_PP_INTERRUPT_BIT), dev->dec_hwregs + X170_INTERRUPT_REGISTER_PP);
		atomic_add(1, &dev->irq_count_pp);
	}

	return IRQ_WAKE_THREAD;
}

irqreturn_t hx170dec_thread(int irq, void *dev_id)
{
    hx170dec_t *dev = (hx170dec_t *) dev_id;

//mutex_lock(&service.lock);
    if(atomic_read(&dev->irq_count_codec))
    {
#ifdef HW_PERFORMANCE
            do_gettimeofday(&end_time);
#endif
		atomic_sub(1, &dev->irq_count_codec);
		hx170dec_data.dec_irq = 1;
    }

    if(atomic_read(&dev->irq_count_pp))
    {
#ifdef HW_PERFORMANCE
            do_gettimeofday(&end_time);
#endif

		atomic_sub(1, &dev->irq_count_pp);
		hx170dec_data.pp_irq = 1;

    }
//        wake_up_interruptible_sync(&hx170dec_data.wait);
    if(hx170dec_data.pp_irq || hx170dec_data.dec_irq)
        wake_up_interruptible_all(&hx170dec_data.wait);

    return IRQ_HANDLED;
}

/*------------------------------------------------------------------------------
    Function name   : ResetAsic
    Description     : reset asic

    Return type     :
------------------------------------------------------------------------------*/

void ResetAsic(hx170dec_t * dev)
{
    int i;

    writel(0, dev->dec_hwregs + 0x04);

    for(i = 4; i < dev->dec_io_size; i += 4)
    {
        writel(0, dev->dec_hwregs + i);
    }
}

/*------------------------------------------------------------------------------
    Function name   : dump_regs
    Description     : Dump registers

    Return type     :
------------------------------------------------------------------------------*/
#ifdef HX170DEC_DEBUG
void dump_regs(unsigned long data)
{
    hx170dec_t *dev = (hx170dec_t *) data;
    int i;

    PDEBUG("Reg Dump Start\n");
    for(i = 0; i < dev->iosize; i += 4)
    {
        PDEBUG("\toffset %02X = %08X\n", i, readl(dev->dec_hwregs + i));
    }
    PDEBUG("Reg Dump End\n");
}
#endif
