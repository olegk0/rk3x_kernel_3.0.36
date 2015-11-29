/*
 * simple memory allocator based on CMA and integrated with UMP
 * Copyright (C) 2015 olegvedi@gmail.com
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include "ump_kernel_interface_ref_drv.h"
#include <linux/rk_fb.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/list.h>
#include <asm/atomic.h>
#include <linux/dma-mapping.h>
#include "rk_ump.h"

#if 1
#define DDEBUG(fmt, args...)	{printk("%s - " fmt "\n",__func__, ##args);}
#else
#define DDEBUG(...)
#endif

static DEFINE_MUTEX(usi_lock);
static LIST_HEAD(usi_list);

struct usi_mbs {
	struct  list_head node;
	struct usi_ump_mbs uum;
	void    *umh;
	void    *virt;
	int     ref_id;
};

struct usi_private{
	struct device *usi_dev;
	u32			full_size;
	atomic_t	used;
	atomic_t	ref_cnt;
};

static struct usi_private usi_priv;

static int usi_ump_free(int ref_id, ump_secure_id secure_id)
{
    int i, ret=0;
    struct usi_mbs *umbs, *tumbs;
    u32 size=0;

	if(list_empty(&usi_list))
		return -EAGAIN;

    mutex_lock(&usi_lock);
    list_for_each_entry_safe(umbs, tumbs, &usi_list, node) {
		if(umbs){
            if((secure_id && umbs->uum.secure_id == secure_id ) ||//del by secure id
                ( ref_id && umbs->ref_id == ref_id ) ||//del by ref id
                (!secure_id && !ref_id)){//del all
                size += umbs->uum.size;
                if(umbs->umh){
                    i = 100;//reassurance
                    while(ump_dd_reference_release(umbs->umh) && i--); //WARNING  it requires change ump kernel driver (ump_dd_reference_release func)
                }
                list_del(&umbs->node);
                dma_free_coherent(usi_priv.usi_dev, umbs->uum.size, umbs->virt, umbs->uum.addr);
                kfree(umbs);
                umbs = NULL;
                if(secure_id)
                    break;
            }
        }
    }
    mutex_unlock(&usi_lock);
    
    if(size){
        atomic_sub(size, &usi_priv.used);
        DDEBUG("Release mem block");
    }else{
        DDEBUG("Mem block dont found");
        ret = -EINVAL;
    }    
    DDEBUG("ref_id:%d secure_id:%d used:%d", ref_id, secure_id, atomic_read(&usi_priv.used));    
    
    return ret;
}

static struct usi_ump_mbs *usi_ump_alloc_mb(struct usi_ump_mbs *puum, int ref_id)
{
	struct usi_mbs *new_umbs=NULL;
	ump_dd_physical_block upb;

	DDEBUG("Request on %d bytes", puum->size);
	puum->size = UMP_SIZE_ALIGN(puum->size);
	
	new_umbs = kzalloc(sizeof(*new_umbs), GFP_KERNEL);
	if(!new_umbs){
		dev_err(usi_priv.usi_dev, "Don`t allocate mem for list entry");
		return NULL;
	}

    new_umbs->uum.size = puum->size;
    new_umbs->virt = dma_alloc_coherent(usi_priv.usi_dev, new_umbs->uum.size, &(new_umbs->uum.addr), GFP_KERNEL);
    if (!new_umbs->virt) {
        dev_err(usi_priv.usi_dev, "No free CMA memory\n");
        kfree(new_umbs);
		return NULL;
	}
    
	upb.addr = new_umbs->uum.addr;
	upb.size = new_umbs->uum.size;

	new_umbs->umh = ump_dd_handle_create_from_phys_blocks(&upb, 1);
	if(new_umbs->umh == UMP_DD_HANDLE_INVALID){
	    dev_err(usi_priv.usi_dev, "Error get ump handle");
	    goto err_free;
	}
    
	new_umbs->uum.secure_id = ump_dd_secure_id_get(new_umbs->umh);//UMP_INVALID_SECURE_ID
    new_umbs->ref_id = ref_id;

	mutex_lock(&usi_lock);
	list_add_tail(&new_umbs->node, &usi_list);
	mutex_unlock(&usi_lock);
    
    atomic_add(new_umbs->uum.size, &usi_priv.used);
	
    DDEBUG("Secure id:%d used:%d paddr:%X",new_umbs->uum.secure_id, atomic_read(&usi_priv.used), new_umbs->uum.addr);
    return &new_umbs->uum;

err_free:
    dma_free_coherent(usi_priv.usi_dev, new_umbs->uum.size, new_umbs->virt, new_umbs->uum.addr);
	kfree(new_umbs);
	return NULL;
}

static long usi_ump_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    unsigned long __user *puser = (unsigned long __user *) arg;
    u32 param[2];
	struct usi_ump_mbs	uum, *puum;
	struct usi_ump_mbs_info	uumi;
    int cnt = (int)file->private_data;

    switch (cmd) {
    case USI_ALLOC_MEM_BLK:
		if (copy_from_user(&uum, puser, sizeof(uum)))
		    return -EFAULT;
		puum = usi_ump_alloc_mb(&uum, cnt);
		if(!puum)
			return -EFAULT;
		if(copy_to_user(puser, puum, sizeof(*puum)))
			return -EFAULT;
		return 0;
    case USI_FREE_MEM_BLK:
		if (copy_from_user(param, puser, 4))
		    return -EFAULT;
		return usi_ump_free( 0, param[0]);
    case USI_GET_INFO:
		uumi.size_full = usi_priv.full_size;
		uumi.size_used = atomic_read(&usi_priv.used);
		if(copy_to_user(puser, &uumi, sizeof(uumi)))
			return -EFAULT;
		return 0;
	case USI_FREE_ALL_BLKS:
		return usi_ump_free( cnt, 0);
    }
    return -EFAULT;
}

int usi_ump_release(struct inode *inode, struct file *file)
{
    int cnt = (int)file->private_data;
    
    usi_ump_free(cnt, 0);
//    atomic_dec(&usi_priv.ref_cnt); TODO 
    return 0;
}

int usi_ump_open(struct inode *inode, struct file *file)
{
    int cnt;

    cnt = atomic_inc_return(&usi_priv.ref_cnt);
    file->private_data = (void *)cnt;
    return 0;
}

static const struct file_operations usi_fops = {
    .owner  = THIS_MODULE,
    .unlocked_ioctl = usi_ump_ioctl,
    .open = usi_ump_open,
    .release = usi_ump_release,
};

static struct miscdevice usi_misc_dev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = USI_UMP_DRV_NAME,
    .fops = &usi_fops,
    .parent = NULL,
};

static int __init usi_ump_init(void)
{
	int ret = 0;

	usi_priv.full_size = 0;//not implemented yet
	atomic_set(&usi_priv.used, 0);
	atomic_set(&usi_priv.ref_cnt, 0);

	ret = misc_register(&usi_misc_dev);
	if(ret){
		dev_err(usi_priv.usi_dev, "Unable to register usi_ump misk dev\n" );
		return ret;
	}
	usi_priv.usi_dev = usi_misc_dev.this_device;
	usi_priv.usi_dev->coherent_dma_mask = ~0;
    mutex_init(&usi_lock);
	_dev_info(usi_priv.usi_dev, "USI allocator initialized.\n");
//	DDEBUG("Mem allocate for buffer %d Mb", usi_priv.full_size / (1024 * 1024));
//    platform_set_drvdata(pdev, ud);

	return 0;
}

static void __exit usi_ump_exit(void)
{
    misc_deregister(&usi_misc_dev);
    usi_ump_free( 0, 0);
    return;
}

module_init(usi_ump_init);
module_exit(usi_ump_exit);
MODULE_AUTHOR("olegvedi@gmail.com");
MODULE_LICENSE("GPL");
