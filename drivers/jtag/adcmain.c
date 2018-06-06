#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <asm/uaccess.h>
#include <linux/wm97xx.h>
#include <linux/kdev_t.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
 
typedef struct
{
    unsigned int channelNumber;
    unsigned int adcValue;
}adcData;
 
#define SET_ADC_CHANNEL        _IOW('q', 1, adcData *)
#define GET_ADC_DATA           _IOR('q', 2, adcData *)
 
static dev_t first;         // Global variable for the first device number
static struct cdev c_dev;   // Global variable for the character device structure
static struct class *cl;    // Global variable for the device class
static struct wm97xx *wm;
static int init_result;
static int adcChannel;
static int adcValue;
 
static ssize_t adc_read(struct file* F, char *buf, size_t count, loff_t *f_pos)
{
return -EPERM;
}
 
static ssize_t adc_write(struct file* F, const char *buf, size_t count, loff_t *f_pos)
{
    return -EPERM;
}
 
static int adc_open(struct inode *inode, struct file *file)
{
    return 0;
}
 
static int adc_close(struct inode *inode, struct file *file)
{
    return 0;
}
 
static long adc_device_ioctl(struct file *f, unsigned int adc_channel, unsigned long arg)
{
    adcData adc;
 
switch(adc_channel)
{
    case SET_ADC_CHANNEL:
    if (copy_from_user(&adc, (adcData*)arg, sizeof(adcData)))
    {
        return -EFAULT;
    }
    adcChannel = adc.channelNumber;
    break;
 
    case GET_ADC_DATA:
        switch (adcChannel)
        {
            case 1:
                adcValue = wm97xx_read_aux_adc(wm, WM97XX_AUX_ID1);
                break;
 
            case 2:
                adcValue = wm97xx_read_aux_adc(wm, WM97XX_AUX_ID2);
                break;
 
            case 3:
                adcValue = wm97xx_read_aux_adc(wm, WM97XX_AUX_ID3);
                break;
 
            case 4:
                adcValue = wm97xx_read_aux_adc(wm, WM97XX_AUX_ID4);
                break;
 
            default:
                return -EINVAL;
                break;
        }
 
        adc.channelNumber = adcChannel;
        adc.adcValue = adcValue;
        if (copy_to_user((adcData*)arg, &adc, sizeof(adcData)))
        {
            return -EFAULT;
        }
        printk(KERN_ALERT "AUX ADC%d reading: %d\n", adcChannel, adcValue);
        break;
 
        default:
        break;
    }
 
    return 0;
}
 
static int sample_wm97xx_probe(struct platform_device *pdev)
{
    wm = platform_get_drvdata(pdev);
 
    if (wm == NULL)
    {
        printk(KERN_ALERT "Platform get drvdata returned NULL\n");
        return -1;
    }
 
    return 0;
}
 
static int sample_wm97xx_remove(struct platform_device *pdev)
{
    /* http://opensource.wolfsonmicro.com/content/using-auxadc-wm97xx-touchscreen-drivers */
 
    return 0;
}
 
static struct platform_driver sample_wm97xx_driver = {
    .probe  = sample_wm97xx_probe,
    .remove = sample_wm97xx_remove,
    .driver = {
        .name = "colibri_adc",
        .owner = THIS_MODULE,
    },
};
 
static struct file_operations FileOps =
{
    .owner                = THIS_MODULE,
    .open                 = adc_open,
    .read                 = adc_read,
    .write                = adc_write,
    .release              = adc_close,
    .unlocked_ioctl        = adc_device_ioctl,
};
 
static int sample_wm97xx_init(void)
{
   init_result = platform_driver_probe(&sample_wm97xx_driver, &sample_wm97xx_probe);
 
   if (init_result < 0)
   {
       printk(KERN_ALERT "ADC Platform Driver probe failed with :%d\n", init_result);
       return -1;
   }
   else
   {
       init_result = alloc_chrdev_region( &first, 0, 1, "adc_drv" );
       if( 0 > init_result )
       {
           platform_driver_unregister(&sample_wm97xx_driver);
           printk( KERN_ALERT "ADC Device Registration failed\n" );
           return -1;
        }
       if ( (cl = class_create( THIS_MODULE, "chardev" ) ) == NULL )
       {
           platform_driver_unregister(&sample_wm97xx_driver);
           printk( KERN_ALERT "ADC Class creation failed\n" );
           unregister_chrdev_region( first, 1 );
           return -1;
    }
 
    if( device_create( cl, NULL, first, NULL, "adc_drv" ) == NULL )
    {
        platform_driver_unregister(&sample_wm97xx_driver);
        printk( KERN_ALERT "ADC Device creation failed\n" );
        class_destroy(cl);
        unregister_chrdev_region( first, 1 );
        return -1;
    }
 
    cdev_init( &c_dev, &FileOps );
 
    if( cdev_add( &c_dev, first, 1 ) == -1)
    {
        platform_driver_unregister(&sample_wm97xx_driver);
        printk( KERN_ALERT "ADC Device addition failed\n" );
        device_destroy( cl, first );
        class_destroy( cl );
        unregister_chrdev_region( first, 1 );
        return -1;
    }
    return 0;
}
 
static void sample_wm97xx_exit(void)
{
    platform_driver_unregister(&sample_wm97xx_driver);
    cdev_del( &c_dev );
    device_destroy( cl, first );
    class_destroy( cl );
    unregister_chrdev_region( first, 1 );
 
    printk(KERN_ALERT "ADC Driver unregistered\n");
}
 
module_init(sample_wm97xx_init);
module_exit(sample_wm97xx_exit);
 
MODULE_AUTHOR("Sanchayan Maity");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Colibri T20 ADC Driver");