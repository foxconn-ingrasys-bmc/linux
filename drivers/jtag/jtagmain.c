/****************************************************************
 **                                                            **
 **    (C)Copyright 2006-2009, American Megatrends Inc.        **
 **                                                            **
 **            All Rights Reserved.                            **
 **                                                            **
 **        5555 Oakbrook Pkwy Suite 200, Norcross              **
 **                                                            **
 **        Georgia - 30093, USA. Phone-(770)-246-8600.         **
 **                                                            **
****************************************************************/

 /*
 * File name: jtagmain.c
 * This driver provides common layer, independent of the hardware, for the JTAG driver.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/io.h>
//#include "helper.h"
#include "../driver_hal/driver_hal.h"
//#include "dbgout.h"
#include "jtag.h"
#include "jtag_ioctl.h"

#ifdef HAVE_UNLOCKED_IOCTL
  #if HAVE_UNLOCKED_IOCTL
  #define USE_UNLOCKED_IOCTL
  #endif
#endif

#define JTAG_MAJOR           175
#define JTAG_MINOR	    	   0
#define JTAG_MAX_DEVICES     255
//willen
#define JTAG_DEV_NAME        "jtag"
//#define JTAG_DEV_NAME		"/dev/ast-jtag"

#define JTAG_DRIVER_NAME        "jtag"
#define AST_JTAG_BUFFER_SIZE 0x10000
#define AST_FW_BUFFER_SIZE  0x80000  //512KB

//static struct cdev *jtag_cdev;
//static dev_t jtag_devno = MKDEV(JTAG_MAJOR, JTAG_MINOR);
static jtag_hw_device_operations_t *pjhwd_ops = NULL;

static unsigned int chrdev_jtag_major = 0;
static struct cdev chrdev_jtag_cdev;
static struct class *chrdev_jtag_class = NULL;
static unsigned int num_of_dev = 1;


unsigned int *JTAG_read_buffer = NULL;
unsigned int *JTAG_write_buffer= NULL;
unsigned long *JTAG_other_buffer= NULL;

JTAG_DEVICE_INFO	JTAG_device_information;

struct class *jtag_class;

int register_jtag_hw_device_ops (jtag_hw_device_operations_t *pjhwd)
{
	pjhwd_ops = pjhwd;

	return 0;
}


int unregister_jtag_hw_device_ops (void)
{
	if(pjhwd_ops != NULL){
			pjhwd_ops = NULL;
	}

	return 0;
}


int register_jtag_hal_module (unsigned char num_instances, void *phal_ops, void **phw_data)
{
	struct jtag_hal *pjtag_hal;

	pjtag_hal = (struct jtag_hal*) kmalloc (sizeof(struct jtag_hal), GFP_KERNEL);
	if (!pjtag_hal)
	{
		return -ENOMEM;
	}

	pjtag_hal->pjtag_hal_ops = ( jtag_hal_operations_t *) phal_ops;
	*phw_data = (void *) pjtag_hal;	

	return 0;	
}


int unregister_jtag_hal_module (void *phw_data)
{
	struct jtag_hal *pjtag_hal = (struct jtag_hal*) phw_data;

	kfree (pjtag_hal);

	return 0;
}

#if 0
/*
 * get_write_buffer
 */
unsigned int* get_jtag_write_buffer(void)
{
	return JTAG_write_buffer;
}


/*
 * get_read_buffer
 */
unsigned int* get_jtag_read_buffer(void)
{
	return JTAG_read_buffer;
}
#endif

static int jtag_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct jtag_hal *pjtag_hal;
	struct jtag_dev *pdev;
	hw_info_t jtag_hw_info;
	unsigned char open_count;
	int ret;

	printk("willen jtag_open\n");

	ret = hw_open (EDEV_TYPE_JTAG, minor,&open_count, &jtag_hw_info);
	if (ret)
	{
		printk("willen hw_open failed\n");
		return -ENXIO;
	}
	pjtag_hal = jtag_hw_info.pdrv_data;

	pdev = (struct jtag_dev*)kmalloc(sizeof(struct jtag_dev), GFP_KERNEL);
	
	if (!pdev)
	{
		hw_close (EDEV_TYPE_JTAG, minor, &open_count);
		printk ("willen %s: failed to allocate jtag private dev structure for jtag iminor: %d\n", JTAG_DEV_NAME, minor);
		return -ENOMEM;
	}

	pdev->pjtag_hal = pjtag_hal;
	file->private_data = pdev;

	return 0;
}


static int jtag_release(struct inode *inode, struct file *file)
{
	int ret;
	unsigned char open_count;
  	struct jtag_dev *pdev = (struct jtag_dev*)file->private_data;
	printk("willen jtag_release\n");
	//pdev->pjtag_hal = NULL;
  	file->private_data = NULL;
  	ret = hw_close (EDEV_TYPE_JTAG, iminor(inode), &open_count);
  	if(ret) { return -1; }
		kfree (pdev);

	return 0;
}

#ifdef USE_UNLOCKED_IOCTL
static long jtag_ioctl(struct file *file,unsigned int cmd, unsigned long arg)
#else
static int jtag_ioctl(struct inode *inode, struct file *file,unsigned int cmd, unsigned long arg)
#endif
{
	int ret = 0;
#if 0
	struct jtag_dev *pdev = (struct jtag_dev*) file->private_data;
	unsigned long	idcode;
	unsigned long	usercode; //wn023
	IO_ACCESS_DATA Kernal_IO_Data;

#ifdef INTEL_JTAG_ADDITIONS
	/* Handle the following IOCTLs first because of the straight user data passed in */
	switch (cmd)
	{
        case AST_JTAG_SIOCFREQ:
        case AST_JTAG_GIOCFREQ:
            printk("JTAG frequency changes not supported.\n");
            return -ENOTTY;
            break;
        case AST_JTAG_SLAVECONTLR:
            printk("JTAG slave/master control not supported.\n");
            return 0;
            break;
        case AST_JTAG_BITBANG:
        case AST_JTAG_SET_TAPSTATE:
        case AST_JTAG_READWRITESCAN:
        case AST_JTAG_ASD_INIT:
        case AST_JTAG_ASD_DEINIT:
            ret = pdev->pjtag_hal->pjtag_hal_ops->intel_jtag_ioctl(cmd, arg);
			return ret;
			break;
	}

	/* the rest of IOCTLs handle the pointer of the user data */
#endif

	memset (&Kernal_IO_Data, 0x00, sizeof(IO_ACCESS_DATA));
	Kernal_IO_Data = *(IO_ACCESS_DATA *)arg;	


	switch (cmd)
	{
		case IOCTL_IO_READ:
			Kernal_IO_Data.Data  = *(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address));
			*(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_IO_WRITE:
			*(u32 *)(IO_ADDRESS(Kernal_IO_Data.Address)) = Kernal_IO_Data.Data;
			break;
			
		case IOCTL_JTAG_RESET:
			pdev->pjtag_hal->pjtag_hal_ops->reset_jtag();
			*(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_JTAG_IDCODE_READ:
			pjhwd_ops->get_hw_device_idcode(&idcode);
			Kernal_IO_Data.Data = idcode;
			*(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_JTAG_ERASE_DEVICE:
			Kernal_IO_Data.Data = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_ERASE_DEVICE, 0, 0);
			*(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_JTAG_PROGRAM_DEVICE:
			if(Kernal_IO_Data.Data * 8 != JTAG_device_information.Device_All_Bits_Length){
				Kernal_IO_Data.Data= 1;
				printk("%s: Oops~ program bits should be %d!\n", JTAG_DEV_NAME, (int)JTAG_device_information.Device_All_Bits_Length);
			}
			else{
    		ret = copy_from_user ((u32 *)JTAG_write_buffer, (u8 *)Kernal_IO_Data.Input_Buffer_Base, Kernal_IO_Data.Data);
    		Kernal_IO_Data.Data  = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_PROGRAM_DEVICE,(void*)JTAG_write_buffer, Kernal_IO_Data.Data * 8);
      }
      *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_JTAG_VERIFY_DEVICE:
    	if(Kernal_IO_Data.Data * 8 != JTAG_device_information.Device_All_Bits_Length){
    		printk("%s: Oops~ verify bits should be %d!,\nYou send %d bits.\n", JTAG_DEV_NAME, (int)JTAG_device_information.Device_All_Bits_Length,(int)Kernal_IO_Data.Data * 8);
    		Kernal_IO_Data.Data= 1;
			}
			else{
				ret = copy_from_user ((u32 *)JTAG_write_buffer, (u8 *)Kernal_IO_Data.Input_Buffer_Base, Kernal_IO_Data.Data);
        Kernal_IO_Data.Data = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_VERIFY_DEVICE,(void*)JTAG_write_buffer, Kernal_IO_Data.Data * 8);
			}
      *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
				
		case IOCTL_JTAG_DEVICE_TFR:
      Kernal_IO_Data.Data = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_DEVICE_TFR, 0, 0);
      *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
		case IOCTL_JTAG_DEVICE_CHECKSUM:
			pjhwd_ops->get_hw_device_idcode(&idcode);
			ret = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_DEVICE_CHECKSUM,(void*)&Kernal_IO_Data.Data, 0); 
      *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
			
    case IOCTL_JTAG_UPDATE_DEVICE:
      printk("Enter Jtag kernel driver limit %d ,real %d\n",(unsigned int)AST_FW_BUFFER_SIZE,(unsigned int)Kernal_IO_Data.Data);
    	pjhwd_ops->get_hw_device_idcode(&idcode);
      if(Kernal_IO_Data.Data < AST_FW_BUFFER_SIZE){
        printk("JED Size OK\n");
        ret = copy_from_user ((u32 *)JTAG_other_buffer, (u8 *)Kernal_IO_Data.Input_Buffer_Base, Kernal_IO_Data.Data);
        Kernal_IO_Data.Data = pjhwd_ops->set_hw_device_ctl(IOCTL_JTAG_UPDATE_DEVICE,(void*)JTAG_other_buffer,Kernal_IO_Data.Data);
        *(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
      }
      else {
        printk("JED Size NG\n");
        ret = -1;
      }
      break;

		//wn023
		case IOCTL_JTAG_DEVICE_USERCODE:
			pjhwd_ops->get_hw_device_usercode(&usercode);
			Kernal_IO_Data.Data = usercode;
			*(IO_ACCESS_DATA *)arg = Kernal_IO_Data;
			break;
   		
		default:
			printk ( "Invalid JTAG Function\n");
			return -EINVAL;
	}
#endif
  return ret;
}


/* ----- Driver registration ---------------------------------------------- */
static struct file_operations jtag_ops = {
	.owner = THIS_MODULE,
 	.read = NULL,
	.write = NULL,
#ifdef USE_UNLOCKED_IOCTL
	.unlocked_ioctl = jtag_ioctl,
#else
	.ioctl = jtag_ioctl,
#endif
	.open = jtag_open,
	.release = jtag_release,
};

#if 0
static jtag_core_funcs_t jtag_core_funcs = {
	.get_jtag_core_data = NULL,
};
#endif
//static core_hal_t jtag_core_hal = {
//	.owner		             = THIS_MODULE,
//	.name		               = "JTAG CORE",
//	.dev_type              = EDEV_TYPE_JTAG,
//	.register_hal_module   = register_jtag_hal_module,
//	.unregister_hal_module = unregister_jtag_hal_module,
//	.pcore_funcs           = (void *)&jtag_core_funcs
//};

/*
 * JTGA driver init function
 */
int __init jtag_init(void)
{
	dev_t dev = MKDEV(chrdev_jtag_major, 0);

	int alloc_ret = 0;
	int cdev_ret = 0;
	int ret = -1;

	printk("willen jtag_init\n");

	alloc_ret = alloc_chrdev_region(&dev, 0, num_of_dev, JTAG_DRIVER_NAME);
	if (alloc_ret)
	{
		printk("willen alloc_chrdev_region failed\n");
		goto error;
	}
	chrdev_jtag_major = MAJOR(dev);
	cdev_init(&chrdev_jtag_cdev, &jtag_ops);
 	cdev_ret = cdev_add(&chrdev_jtag_cdev, dev, num_of_dev);
	if (cdev_ret)
	{
		printk("willen cdev_add failed\n");
		goto error;
	}

	chrdev_jtag_class = class_create(THIS_MODULE, JTAG_DRIVER_NAME);
 	if (IS_ERR(chrdev_jtag_class))
	{
  		printk("willen chrdev_jtag_class create failed\n");
		goto error;
	}

	device_create(chrdev_jtag_class,NULL,MKDEV(chrdev_jtag_major, 0),NULL,"ast-jtag");

	printk("willen %s driver(major number %d) installed.\n", JTAG_DRIVER_NAME, chrdev_jtag_major);
 	return 0;
error:
	if (cdev_ret == 0)
  		cdev_del(&chrdev_jtag_cdev);
 	if (alloc_ret == 0)
  		unregister_chrdev_region(dev, num_of_dev);

	return ret;
#if 0
  memset (&JTAG_device_information, 0, sizeof(JTAG_DEVICE_INFO));
  
  JTAG_read_buffer = kmalloc (AST_JTAG_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
  if (JTAG_read_buffer == NULL) {
  	printk (KERN_WARNING "%s: Can't allocate read_buffer\n", JTAG_DEV_NAME);
    ret = -ENOMEM;
    goto out_no_mem;
  }

  JTAG_write_buffer = kmalloc (AST_JTAG_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
  if (JTAG_write_buffer == NULL) {
    printk (KERN_WARNING "%s: Can't allocate write_buffer\n", JTAG_DEV_NAME);
    ret = -ENOMEM;
    goto out_no_mem;
  }

  JTAG_other_buffer = kmalloc (AST_FW_BUFFER_SIZE, GFP_DMA|GFP_KERNEL);
  if (JTAG_other_buffer == NULL) {
    printk (KERN_WARNING "%s: Can't allocate other_buffer\n", JTAG_DEV_NAME);
    ret = -ENOMEM;
    goto out_no_mem;
  }
  memset(JTAG_other_buffer, 0xff, AST_FW_BUFFER_SIZE);

	printk("The JTAG Driver is loaded successfully.\n" );
  return 0;
  
out_no_mem:
	cdev_del (jtag_cdev);
	unregister_chrdev_region (jtag_devno, JTAG_MAX_DEVICES);	
	
  if (JTAG_read_buffer != NULL)
  	kfree(JTAG_read_buffer);
  if (JTAG_write_buffer != NULL)
  	kfree(JTAG_write_buffer);
	if (JTAG_other_buffer != NULL)
		kfree(JTAG_other_buffer);  
  return ret;
#endif
}

/*!
 * JTGA driver exit function
 */
void __exit jtag_exit(void)
{
	dev_t dev = MKDEV(chrdev_jtag_major, 0);
	
	printk("willen jtag_exit\n");
 
 	device_destroy(chrdev_jtag_class, dev);
        class_destroy(chrdev_jtag_class);
	cdev_del(&chrdev_jtag_cdev);
	unregister_chrdev_region(dev, num_of_dev);
	
//	unregister_core_hal_module (EDEV_TYPE_JTAG);

 	printk("willen %s driver removed.\n", JTAG_DRIVER_NAME);
	
//	kfree(JTAG_read_buffer);
//	kfree(JTAG_write_buffer);
//	kfree(JTAG_other_buffer);

//	printk("willen Unregistered the JTAG Driver Sucessfully\n");
	return;	
}
#if 0
EXPORT_SYMBOL(JTAG_device_information);
EXPORT_SYMBOL(register_jtag_hw_device_ops);
EXPORT_SYMBOL(unregister_jtag_hw_device_ops);
EXPORT_SYMBOL(get_jtag_write_buffer);
EXPORT_SYMBOL(get_jtag_read_buffer);
#endif
module_init(jtag_init);
module_exit(jtag_exit);

MODULE_AUTHOR("American Megatrends Inc.");
MODULE_DESCRIPTION("JTAG Common Driver");
MODULE_LICENSE ("GPL");

int jtag_core_loaded =1;
EXPORT_SYMBOL(jtag_core_loaded);

