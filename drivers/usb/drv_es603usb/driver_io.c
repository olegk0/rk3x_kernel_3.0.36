/*
 * SS801U Finger Print Sensor Linux Driver
 * Copyright (C) 2008 by Egistec 
 */



#include <linux/egistec/linux_driver.h>
//#include "driver_main.h"
#include "driver_usb.h"

#include "ioctrl_handler.h"

#include <linux/proc_fs.h>
//Wayne for SPI to USB
#include <linux/egistec/Define_JadeOpc.h>
#include <linux/spi/spidev.h>
//#include <linux/spi/egisspi.h>

#include <linux/egistec/utility.h>


extern struct usb_driver ss801u_driver;
extern void ss801u_delete(struct kref *kref);

/*
void ss801u_draw_down(usb_ss801u *dev)
{
    int time;

    //----- wait for submitting URB
    time = usb_wait_anchor_empty_timeout(&dev->submitted, 1000);
    if (!time)
        usb_kill_anchored_urbs(&dev->submitted);
}
*/


//==============================================================================//
//==============================================================================//

static int ss801u_open(struct inode *inode, struct file *file)
{
    usb_ss801u *dev;
    struct usb_interface *interface;
    int subminor;
    int retval = 0;
	pr_debug("%s:%s:%d\n", __FILE__, __FUNCTION__, __LINE__);
    subminor = iminor(inode);
	
    //----- use minor code to find the interface which we store data in.
    interface = usb_find_interface(&ss801u_driver, subminor);
    if (!interface) {
        err ("%s - error, can't find device for minor %d",
                __FUNCTION__, subminor);
        retval = -ENODEV;
        goto exit;
    }

    //----- get data we saved in the interface
    dev = usb_get_intfdata(interface);
    if (!dev) {
        retval = -ENODEV;
        goto exit;
    }


    /* increment our usage count for the device */
    kref_get(&dev->kref);

    mutex_lock(&dev->io_mutex);

    /*NOTE: open_count initializes in probe. */
    //----- only happen when open_count is zero 
    // ==> means if someone is opening, another can't enter this if-case at same time
    if (!dev->open_count++) {
        retval = 0;//usb_autopm_get_interface(interface); //usb_autopm_enable(interface);
        if (retval) {
            dev->open_count--;
            mutex_unlock(&dev->io_mutex);
            kref_put(&dev->kref, ss801u_delete);
            goto exit;
        }
    }
/*    else
    {
         retval = -EBUSY;
         dev->open_count--;
         mutex_unlock(&dev->io_mutex);
         kref_put(&dev->kref, ss801u_delete);
         goto exit;
    }*/
    file->private_data = dev;
    mutex_unlock(&dev->io_mutex);

exit:
    return retval;
}


static int ss801u_fasync(int fd, struct file* filp, int mode)
{
    usb_ss801u* dev = (usb_ss801u *)filp->private_data;
    int iret=0;

	//sprintf(g_dbgbuf, "=fasync=\r\n");
	//----- release async signal pipe
	if(-1==fd && 0==mode){
		EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "\r\n=FASYNC= relase \r\n");
		iret = fasync_helper(fd, filp, mode, &dev->async_suspend);
		iret = fasync_helper(fd, filp, mode, &dev->async_resume);
		iret = fasync_helper(fd, filp, mode, &dev->async_disconnect);
		dev->async_suspend = dev->async_resume = dev->async_disconnect = NULL;
	}
	else if(0!=mode){ //----- create async signal pipe
		EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "\r\n=FASYNC= type ? %d \r\n", dev->sig_type);
		switch(dev->sig_type)
		{
			case SIG_SUSPEND:
				if(!dev->async_suspend){
					iret = fasync_helper(fd, filp, mode, &dev->async_suspend);
					dev->sig_type = SIG_UNKNOW;
				}
				break;
			case SIG_RESUME:
				if(!dev->async_resume){
					iret = fasync_helper(fd, filp, mode, &dev->async_resume);
					dev->sig_type = SIG_UNKNOW;
				}
				break;
			case SIG_DISCONNECT:
				if(!dev->async_disconnect){
					iret = fasync_helper(fd, filp, mode, &dev->async_disconnect);
					dev->sig_type = SIG_UNKNOW;
				}
				break;
			case SIG_UNKNOW:
			default:
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=FASYNC= type unknow \r\n");
				break;
		}
	}
    return iret;
}


static int ss801u_release(struct inode *inode, struct file *file)
{
    usb_ss801u *dev;

    dev = (usb_ss801u *)file->private_data;
    if (dev == NULL)
        return -ENODEV;

    /* allow the device to be autosuspended */
    mutex_lock(&dev->io_mutex);
    if (!--dev->open_count && dev->interface){
		//usb_autopm_disable(dev->interface);
        //usb_autopm_put_interface(dev->interface);
	    ss801u_fasync(-1, file, 0);
    }
    mutex_unlock(&dev->io_mutex);


    /* decrement the count on our device */
    kref_put(&dev->kref, ss801u_delete);

    EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=RELEEASE=\r\n");    
    return 0;
}

static int ss801u_flush(struct file *file, fl_owner_t id)
{
    usb_ss801u *dev;
    int res;

    dev = (usb_ss801u *)file->private_data;
    if (dev == NULL)
        return -ENODEV;

    /* wait for io to stop */
    mutex_lock(&dev->io_mutex);
    //ss801u_draw_down(dev);

    /* read out errors, leave subsequent opens a clean slate */
    spin_lock_irq(&dev->err_lock);
    res = dev->errors ? (dev->errors == -EPIPE ? -EPIPE : -EIO) : 0;
    dev->errors = 0;
    spin_unlock_irq(&dev->err_lock);

    mutex_unlock(&dev->io_mutex);

    EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=flush= res %d", res);
    return res;
}

static int ss801u_ioctl(struct file* file, unsigned int cmd, unsigned long arg)
{
    usb_ss801u *dev;
    int argerr = 0;
    int retval = 0;
	//PLOG_PACKAGE pLog;
	char *szLog;

	u32			tmp;
	unsigned		n_ioc = 0;
	struct spi_ioc_transfer	*ioc = NULL;

    dev = (usb_ss801u *)file->private_data;

	pr_debug("ioctl Start!!\n");
    //================================ Check input cmd and arg ========================================//
    //=================================================================================================//
    //---- Check control code ----//
	if(EGIS_IOCTL_MAXNR <= _IOC_NR(cmd))   return -ENOTTY;

	//================================== Switch control code ==========================================//
	//=================================================================================================//
	mutex_lock(&dev->io_mutex);

	if (!dev->interface) {		/* disconnect() was called */
		retval = -ENODEV;		
		goto exit;        		
	}

	//----- Avoid suspend -----//
	usb_autopm_get_interface(dev->interface);
	
	//---- Check argument buffer ----//
	if(_IOC_READ & _IOC_DIR(cmd)){
		// access_ok : 1 means success 
		// VERIFY_WRITE : A buffer that SDK reads means drivers should write it.
		argerr = !access_ok(VERIFY_WRITE, (void __user*)arg, _IOC_SIZE(cmd));
	}
	if(_IOC_WRITE & _IOC_DIR(cmd)){
		argerr = !access_ok(VERIFY_READ, (void __user*)arg, _IOC_SIZE(cmd));
	}
	if(argerr) return -EINVAL;

	if(SPI_IOC_MAGIC == _IOC_TYPE(cmd))
	{
		tmp = _IOC_SIZE(cmd);
		if ((tmp % sizeof(struct spi_ioc_transfer)) != 0) {
			retval = -EINVAL;
			goto done;
		}
		n_ioc = tmp / sizeof(struct spi_ioc_transfer);
		pr_debug("%s->%s: Got %d spi_ioc_transfer Packages\n", __FILE__, __FUNCTION__, n_ioc);
		if (n_ioc == 0)
			goto done;

		/* copy into scratch area */
		ioc = kmalloc(tmp, GFP_KERNEL);
		if (!ioc) {
			retval = -ENOMEM;
			goto done;
		}
		if (__copy_from_user(ioc, (void __user *)arg, tmp)) {
			kfree(ioc);
			retval = -EFAULT;
			goto done;
		}
////Wayne ToDo
//        if (u_tmp->rx_buf)
//        {
//            k_tmp->rx_buf = buf;
//            if ((u_tmp->opcode == JADE_GET_ONE_IMG) && (i == (l -1)))
//            {
//                buf += tx_buf[1] * 2;
//                k_tmp->len = (tx_buf[1] * 2 + 1);
//                //Wayne TODO: One time is enough
//                if (!access_ok(VERIFY_WRITE, (u8 __user *)
//                            (uintptr_t) u_tmp->rx_buf,
//                            tx_buf[1] * 2))
//                    goto erracc;
//            }
//            else
//            {
//                buf += k_tmp->len;
//                //Wayne TODO: One time is enough
//                if (!access_ok(VERIFY_WRITE, (u8 __user *)
//                            (uintptr_t) u_tmp->rx_buf,
//                            u_tmp->len))
//                    goto erracc;
//            }
//        }

		switch (ioc->opcode)
		{
		case JADE_REGISTER_MASSREAD:
				if (ioc->rx_buf) 
				{
					if (!access_ok(VERIFY_WRITE, (u8 __user *)
								(uintptr_t) ioc->rx_buf,
								ioc->len))
					{
						pr_debug("!access_ok on JADE_REGISTER_MASSREAD!!");
						retval =  -EACCES;
						goto done;
					}
				}
				retval = ES603_IO_Bulk_Read(dev, ioc, n_ioc);
				break;
			case JADE_REGISTER_MASSWRITE:
				retval = ES603_IO_Bulk_Write(dev, ioc, n_ioc);
				break;
			case JADE_GET_ONE_IMG:
				retval = es603_io_bulk_get_image(dev, ioc, n_ioc);
				break;
			case JADE_GET_FULL_IMAGE2:
				retval = es603_io_bulk_get_full_image(dev, ioc, n_ioc);
				break;
			default:
				retval = -EFAULT;
		}
//erracc:
//        retval =  -EACCES
done:
		if (ioc)
		{
				kfree(ioc);
		}
	}
 	else if(EGIS_IOCTL_MAGIC == _IOC_TYPE(cmd))
	{
		switch(cmd)
		{
			//-------------------- ss801u SCSI CMD XFER ----------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_SCSI_READ:
			{
				retval = Egis_IO_SCSI_Read(dev, arg);
				break;
			}
	
			case EGIS_IOCTL_SCSI_WRITE:
			{
				retval = Egis_IO_SCSI_Write(dev, arg);
				break;
			}
	
			case EGIS_IOCTL_SET_NORMALMODE_REG:
			{
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl=EGIS_IOCTL_SET_NORMALMODE_REG ");
				if(copy_from_user((void*)&dev->normalCBW, (void*)arg, CBW_SIZE)){
					EgisMsg(dev->bPrintDbgMsg, KERN_ERR, "=ioctl=EGIS_IOCTL_SET_NORMALMODE_REG copy from user fail\r\n");
					retval = -EFAULT; break;
				}	
				break;
			}
	
			case EGIS_IOCTL_SET_CONTACTMODE_REG:
			{
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl=EGIS_IOCTL_SET_CONTACTMODE_REG ");
	
				if(copy_from_user(&dev->contactCBW, (void*)arg, CBW_SIZE)){
					EgisMsg(dev->bPrintDbgMsg, KERN_ERR, "=ioctl=EGIS_IOCTL_SET_CONTACTMODE_REG copy from user fail\r\n");
					retval = -EFAULT; break;
				}	
				break;
			}
	
			//-------------------- JADE USB BASIC XFER -----------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_BULK_READ:	
			{	
				retval = Egis_IO_Bulk_Read(dev, arg);
				break;
			}
			case EGIS_IOCTL_BULK_WRITE:	
			{	
				retval = Egis_IO_Bulk_Write(dev, arg);
				break;
			}
	
			case EGIS_IOCTL_CTRLXFER_READ:	
			{		
				retval = Egis_IO_CtrlXfer(dev, arg, Ctrl_IN);
				break;
			}
			case EGIS_IOCTL_CTRLXFER_WRITE:	
			{	
				retval = Egis_IO_CtrlXfer(dev, arg, Ctrl_OUT);
				break;
			}
	
	
			//---------------------- Helper function -------------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_ENTER_SUSPEND :
				///dev->udev->autosuspend_delay = 0;
				break;
	
			case EGIS_IOCTL_RESET_DEVICE:
				retval = Egis_IO_Reset_Device(dev);
				break;
	
			case EGIS_IOCTL_CREATE_SIGNAL:
				dev->sig_type = arg;
				break;
	
			//-------------------- Alternative Setting -----------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_SET_AUTOSUSPEND_TIME:
				dev->idle_delay_time = arg;
				///dev->udev->autosuspend_delay = dev->idle_delay_time*HZ;
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= idle-delay time %d \r\n", dev->idle_delay_time);	
				break;
	
			//------------------------- Information --------------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_GET_VERSION:
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= EGIS_IOCTL_GET_VERSION %s\r\n", dev->version.vstring);
	
				if(copy_to_user((void*)arg, &dev->version, sizeof(FPDriver_Version))){
					EgisMsg(dev->bPrintDbgMsg, KERN_ERR, "=ioctl= EGIS_IOCTL_GET_VERSION copy to user fail\r\n");
					retval = -EFAULT; break;
				}	
				break;
			
			//------------------------ Debug usage ---------------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_ENABLE_DBG_MSG:
				dev->bPrintDbgMsg = arg;
				break;
	
			//------------------------ FOR TEST ------------------------------//
			//----------------------------------------------------------------//
			case EGIS_IOCTL_TS_SIGNAL:
				if(dev->async_resume){
					EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= kill_fasync resume!!\r\n");
					kill_fasync(&dev->async_resume, SIGIO, POLL_IN);
				}
				if(dev->async_suspend){
					EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= kill_fasync suspend !!\r\n");
					kill_fasync(&dev->async_suspend, SIGIO, POLL_IN);
				}
				break;
	
			case EGIS_IOCTL_TS_SWITCH_AUTOSUSPEND:
				///dev->udev->autosuspend_disabled = !arg;
				///EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= autosuspend_disabled %d \r\n", 
				///		dev->udev->autosuspend_disabled);
				break;
	
			case EGIS_IOCTL_TS_SWITCH_RMWAKEUP:
				dev->udev->do_remote_wakeup = arg;
				EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "=ioctl= do_remote_wakeup %d \r\n", 
						dev->udev->do_remote_wakeup);
				break;
	//Wayne for test
	//  	case EGIS_IOCTL_WRITELOG:
	//  		//pLog = (PLOG_PACKAGE)arg;
	//  		szLog = kzalloc(pLog->ulDataSize,GFP_KERNEL);
	//  		copy_from_user(szLog, (PLOG_PACKAGE)arg->szLog, (PLOG_PACKAGE)arg->ulDataSize);
	//  		printk(szLog);
	//  		break;
			//----------------------------------------------------------------//
	
			default :
				retval = -ENOTTY;
				break;
		}
	}
	else
	{
		retval =  -ENOTTY;
	}
exit:
	//----- Auto suspend -----//
	usb_autopm_put_interface(dev->interface);
	mutex_unlock(&dev->io_mutex);

	return retval;
}


static ssize_t ss801u_read(struct file *file, char *buffer, size_t count, loff_t *ppos)
{
    usb_ss801u *dev;
    int retval=0;
    int bytes_read;
	char* kbuf;

    dev = (usb_ss801u *)file->private_data;

    mutex_lock(&dev->io_mutex);
	
	//----- To NOT go to suspend -----//
    usb_autopm_get_interface(dev->interface);
	
    if (!dev->interface) {		/* disconnect() was called */
        retval = -ENODEV;
        goto exit;
    }


	//*/
	//--------------- CASE1 : only return driver version -----------------------//	
	//--------------------------------------------------------------------------//
	if(dev->bReadTwice){
		dev->bReadTwice=false;
		goto exit;
	}
	kbuf = kzalloc(count, GFP_KERNEL);
	if(!kbuf) goto exit;

	sprintf(kbuf, "%s v%s\r\n", NAME_EGISDRIVER, dev->version.vstring);
	bytes_read = strlen(kbuf);
	if(copy_to_user(buffer, kbuf, bytes_read)){
		retval = 0;
	}
	else{
		retval = count;
		ppos += bytes_read;
		dev->bReadTwice = true;
	}
	if(kbuf) kfree(kbuf);
	/*/
	//----------------------- CASE2 : bulk read --------------------------------//	
	//----------------------- ---------------------------------------------------//		
    //----- do a blocking bulk read to get data from the device -----/	
    retval = usb_bulk_msg(
				dev->udev,
	            usb_rcvbulkpipe(dev->udev, dev->bulk_in_endpointAddr),
	            dev->bulk_in_buffer,
	            min(dev->bulk_in_size, count),
	            &bytes_read, 10000
				);


    //----- if the read was successful, copy the data to userspace -----//
    if (!retval) {
        if (copy_to_user(buffer, dev->bulk_in_buffer , bytes_read))
            retval = -EFAULT;
        else
            retval = bytes_read;
    }
    else{
		Egis_Reset_device(dev);
		EgisMsg(dev->bPrintDbgMsg, KERN_ERR, "=READ= fail %d\r\n", retval);
    }
	//*/

	
exit:

	//----- Auto suspend -----//
	usb_autopm_put_interface(dev->interface);

    mutex_unlock(&dev->io_mutex);
	EgisMsg(dev->bPrintDbgMsg, KERN_INFO, "READ bytes %d\r\n" ,retval);
    return retval;
}
/*
static void ss801u_write_bulk_callback(struct urb *urb)
{
    usb_ss801u *dev;

    dev = (usb_ss801u *)urb->context;

    // sync/async unlink faults aren't errors //
    if (urb->status) {
        if(!(urb->status == -ENOENT ||
                    urb->status == -ECONNRESET ||
                    urb->status == -ESHUTDOWN))
            err("%s - nonzero write bulk status received: %d",
                    __FUNCTION__, urb->status);

		Egis_Reset_device(dev);

        spin_lock(&dev->err_lock);
        dev->errors = urb->status;
        spin_unlock(&dev->err_lock);
    }

    // free up our allocated buffer //
    usb_buffer_free(urb->dev, urb->transfer_buffer_length,
            urb->transfer_buffer, urb->transfer_dma);
    up(&dev->limit_sem);
}
*/
static ssize_t ss801u_write(struct file *file, const char *user_buffer, size_t count, loff_t *ppos)
{
	//*/
	//--------------- CASE1 : return only -----------------------//	
	//-----------------------------------------------------------//
	return -ENOSYS; // function not implement

	/*/
	//---------------- CASE2 : bulk write -----------------------//	
	//-----------------------------------------------------------//	
    usb_ss801u *dev;
    int retval = 0;
    struct urb *urb = NULL;
    char *buf = NULL;
    size_t writesize = min(count, (size_t)MAX_TRANSFER);

    dev = (usb_ss801u *)file->private_data;

    if (count == 0)
        goto exit;

    if (down_interruptible(&dev->limit_sem))
    {
        retval = -ERESTARTSYS;
        goto exit;
    }

    spin_lock_irq(&dev->err_lock);
    if ((retval = dev->errors) < 0) {
        dev->errors = 0;
        retval = (retval == -EPIPE) ? retval : -EIO;
    }
    spin_unlock_irq(&dev->err_lock);
    if (retval < 0)
        goto error;

    //----- create a urb, and a buffer for it, and copy the data to the urb -----//
    urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!urb) {
        retval = -ENOMEM;
        goto error;
    }

    buf = usb_buffer_alloc(dev->udev, writesize, GFP_KERNEL, &urb->transfer_dma);
    if (!buf) {
        retval = -ENOMEM;
        goto error;
    }

    if (copy_from_user(buf, user_buffer, writesize)) {
        retval = -EFAULT;
        goto error;
    }

    //----- this lock makes sure we don't submit URBs to gone devices -----//
	//---------------------------------------------------------------------//
    mutex_lock(&dev->io_mutex);
    if (!dev->interface) {		// disconnect() was called 
        mutex_unlock(&dev->io_mutex);
        retval = -ENODEV;
        goto error;
    }

	//----- To NOT go to suspend -----//
    usb_autopm_get_interface(dev->interface);

    //----- initialize the urb properly -----//
    usb_fill_bulk_urb(urb, dev->udev,
            usb_sndbulkpipe(dev->udev, dev->bulk_out_endpointAddr),
            buf, writesize, ss801u_write_bulk_callback, dev);
    urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;
    //usb_anchor_urb(urb, &dev->submitted);

    //----- send the data out the bulk port -----//
    retval = usb_submit_urb(urb, GFP_KERNEL);

	//----- Auto suspend -----//
    usb_autopm_put_interface(dev->interface);

    mutex_unlock(&dev->io_mutex);
	//-----------------------------------------------------------//
	//-----------------------------------------------------------//    
    
    if (retval) {
		Egis_Reset_device(dev);
        err("%s - failed submitting write urb, error %d", __FUNCTION__, retval);
        goto error_unanchor;
    }

    //----- release our reference to this urb, the USB core will eventually free it entirely -----/
    usb_free_urb(urb);


    return writesize;

error_unanchor:
    usb_unanchor_urb(urb);
error:
    if (urb) {
        usb_buffer_free(dev->udev, writesize, buf, urb->transfer_dma);
        usb_free_urb(urb);
    }
    up(&dev->limit_sem);

exit:
    return retval;
//*/
}

//------------------------- file operation ---------------------------------//
//--------------------------------------------------------------------------//
const struct file_operations ss801u_fops = {
    .owner 	=	THIS_MODULE,

    .read 	=	ss801u_read,
    .write 	=	ss801u_write,
    .unlocked_ioctl 	= 	ss801u_ioctl,

    .open 	=	ss801u_open,
    .release=	ss801u_release,
    .flush 	=	ss801u_flush,

    .fasync =   ss801u_fasync,
};

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

