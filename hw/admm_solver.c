/* Device driver for the admm_solver FPGA module
 *
 * A Platform device using the misc subsystem.
 *
 * Author: Godwill Agbehonou
 * Based on: VGA ball driver by Stephen A. Edwards
 */

 #include <linux/module.h>
 #include <linux/init.h>
 #include <linux/errno.h>
 #include <linux/version.h>
 #include <linux/kernel.h>
 #include <linux/platform_device.h>
 #include <linux/miscdevice.h>
 #include <linux/io.h>
 #include <linux/of.h>
 #include <linux/of_address.h>
 #include <linux/fs.h>
 #include <linux/uaccess.h>
 #include "admm_solver.h"
 
 #define DRIVER_NAME "admm_solver"
 
 /* Register offsets */
 #define REG_START        0x0000
 #define REG_DONE         0x0000
 #define REG_ITER_COUNT   0x0004
 #define REG_HORIZON      0x0008
 
 struct admm_solver_dev {
     struct resource res;
     void __iomem *virtbase;
     admm_solver_arg_t last_status;
 } dev;
 
 /* Trigger ADMM solver by writing config and polling done bit */
 static void run_admm_solver(const admm_solver_arg_t *arg) {
     iowrite32(arg->horizon, dev.virtbase + REG_HORIZON);
     iowrite32(arg->start ? 1 : 0, dev.virtbase + REG_START);
 
     while (!(ioread32(dev.virtbase + REG_DONE) & 1))
         cpu_relax();
 
     dev.last_status.iter_count = ioread32(dev.virtbase + REG_ITER_COUNT);
     dev.last_status.done = 1;
 }
 
 /* ioctl handler */
 static long admm_solver_ioctl(struct file *f, unsigned int cmd, unsigned long arg) {
     admm_solver_arg_t user_arg;
 
     switch (cmd) {
     case ADMM_SOLVER_WRITE:
         if (copy_from_user(&user_arg, (void __user *)arg, sizeof(user_arg)))
             return -EFAULT;
         run_admm_solver(&user_arg);
         break;
 
     case ADMM_SOLVER_READ:
         if (copy_to_user((void __user *)arg, &dev.last_status, sizeof(dev.last_status)))
             return -EFAULT;
         break;
 
     default:
         return -EINVAL;
     }
 
     return 0;
 }
 
 static const struct file_operations admm_solver_fops = {
     .owner = THIS_MODULE,
     .unlocked_ioctl = admm_solver_ioctl,
 };
 
 static struct miscdevice admm_solver_misc_device = {
     .minor = MISC_DYNAMIC_MINOR,
     .name = DRIVER_NAME,
     .fops = &admm_solver_fops,
 };
 
 static int __init admm_solver_probe(struct platform_device *pdev) {
     int ret;
 
     ret = misc_register(&admm_solver_misc_device);
     if (ret)
         return ret;
 
     ret = of_address_to_resource(pdev->dev.of_node, 0, &dev.res);
     if (ret)
         goto fail_misc_deregister;
 
     if (!request_mem_region(dev.res.start, resource_size(&dev.res), DRIVER_NAME)) {
         ret = -EBUSY;
         goto fail_misc_deregister;
     }
 
     dev.virtbase = of_iomap(pdev->dev.of_node, 0);
     if (!dev.virtbase) {
         ret = -ENOMEM;
         goto fail_release_mem;
     }
 
     printk(KERN_INFO DRIVER_NAME ": Probed and ready\n");
     return 0;
 
 fail_release_mem:
     release_mem_region(dev.res.start, resource_size(&dev.res));
 fail_misc_deregister:
     misc_deregister(&admm_solver_misc_device);
     return ret;
 }
 
 static int admm_solver_remove(struct platform_device *pdev) {
     iounmap(dev.virtbase);
     release_mem_region(dev.res.start, resource_size(&dev.res));
     misc_deregister(&admm_solver_misc_device);
     return 0;
 }
 
 #ifdef CONFIG_OF
 static const struct of_device_id admm_solver_of_match[] = {
     { .compatible = "csee4840,admm-solver-1.0" },
     {},
 };
 MODULE_DEVICE_TABLE(of, admm_solver_of_match);
 #endif
 
 static struct platform_driver admm_solver_driver = {
     .driver = {
         .name = DRIVER_NAME,
         .owner = THIS_MODULE,
         .of_match_table = of_match_ptr(admm_solver_of_match),
     },
     .remove = __exit_p(admm_solver_remove),
 };
 
 module_platform_driver_probe(admm_solver_driver, admm_solver_probe);
 
 MODULE_LICENSE("GPL");
 MODULE_AUTHOR("Godwill Agbehonou");
 MODULE_DESCRIPTION("admm_solver FPGA hardware interface driver");
 