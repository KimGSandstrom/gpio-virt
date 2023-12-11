// SPDX-License-Identifier: GPL-2.0-only
/**
 * NVIDIA GPIO Guest Proxy Kernel Module
 * (c) 2023 Unikie, Oy
 * (c) 2023 Kim Sandstrom kim.sandstrom@unikie.com
 *
 **/
#include <linux/module.h>	  // Core header for modules.
#include <linux/device.h>	  // Supports driver model.
#include <linux/kernel.h>	  // Kernel header for convenient functions.
#include <linux/fs.h>		  // File-system support.
#include <linux/uaccess.h>	  // User access copy function support.
#include <linux/slab.h>
//#include <soc/tegra/gpio.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>
//#include "gpio-host-proxy.h"
#include "../gpio-host-proxy/gpio-host-proxy.h"

#define DEVICE_NAME "gpio-host"   // Device name.
#define CLASS_NAME  "chardrv"	  // < The device class -- this is a character device driver

MODULE_LICENSE("GPL");						///< The license type -- this affects available functionality
MODULE_AUTHOR("Kim Sandström");					///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("NVidia GPIO Host Proxy Kernel Module");	///< The description -- see modinfo
MODULE_VERSION("0.0");						///< A version number to inform users

#define GPIO_VERBOSE
#define GPIO_HOST_VERBOSE    0

#if GPIO_HOST_VERBOSE
#define deb_info(...)     printk(KERN_INFO DEVICE_NAME ": "__VA_ARGS__)
#else
#define deb_info(...)
#endif

#define deb_error(...)    printk(KERN_ALERT DEVICE_NAME ": "__VA_ARGS__)

extern struct tegra_pmx *tegra_pmx_host;
extern u32 pmx_readl(struct tegra_pmx *, u32, u32);
extern void pmx_writel(struct tegra_pmx *, u32, u32, u32);

/**
 * Important variables that store data and keep track of relevant information.
 */
static int major_number;

static struct class *gpio_host_proxy_class = NULL;	///< The device-driver class struct pointer
static struct device *gpio_host_proxy_device = NULL; ///< The device-driver device struct pointer

/**
 * Prototype functions for file operations.
 */
static int open(struct inode *, struct file *);
static int close(struct inode *, struct file *);
static ssize_t read(struct file *, char *, size_t, loff_t *);
static ssize_t write(struct file *, const char *, size_t, loff_t *);

/**
 * File operations structure and the functions it points to.
 */
static struct file_operations fops =
	{
		.owner = THIS_MODULE,
		.open = open,
		.release = close,
		.read = read,
		.write = write,
};

// GPIO allowed resources structure
// static struct gpio_allowed_res gpio_ares; 

#if GPIO_HOST_VERBOSE
// Usage:
//     hexDump(desc, addr, len, perLine);
//         desc:    if non-NULL, printed as a description before hex dump.
//         addr:    the address to start dumping from.
//         len:     the number of bytes to dump.
//         perLine: number of bytes on each output line.
void static hexDump (
    const char * desc,
    const void * addr,
    const int len
) {
    // Silently ignore silly per-line values.

    int i;
    unsigned char buff[17];
    unsigned char out_buff[4000];
    unsigned char *p_out_buff = out_buff;
    const unsigned char * pc = (const unsigned char *)addr;



    // Output description if given.

    if (desc != NULL) printk ("%s:\n", desc);

    // Length checks.

    if (len == 0) {
        printk(DEVICE_NAME ":   ZERO LENGTH\n");
        return;
    }
    if (len < 0) {
        printk(DEVICE_NAME ":   NEGATIVE LENGTH: %d\n", len);
        return;
    }

	if(len > 400){
        printk(DEVICE_NAME ":   VERY LONG: %d\n", len);
        return;
    }

    // Process every byte in the data.

    for (i = 0; i < len; i++) {
        // Multiple of perLine means new or first line (with line offset).

        if ((i % 16) == 0) {
            // Only print previous-line ASCII buffer for lines beyond first.

            if (i != 0) {
				p_out_buff += sprintf (p_out_buff, "  %s\n", buff);
			}
            // Output the offset of current line.

            p_out_buff += sprintf (p_out_buff,"  %04x ", i);
        }

        // Now the hex code for the specific character.

        p_out_buff += sprintf (p_out_buff, " %02x", pc[i]);

        // And buffer a printable ASCII character for later.

        if ((pc[i] < 0x20) || (pc[i] > 0x7e)) // isprint() may be better.
            buff[i % 16] = '.';
        else
            buff[i % 16] = pc[i];
        buff[(i % 16) + 1] = '\0';
    }

    // Pad out last line if not exactly perLine characters.

    while ((i % 16) != 0) {
        p_out_buff += sprintf (p_out_buff, "   ");
        i++;
    }

    // And print the final ASCII buffer.

    p_out_buff += sprintf (p_out_buff, "  %s\n", buff);

	printk(DEVICE_NAME ": %s", out_buff);
}
#else
	#define hexDump(...)
#endif

/**
 * Initializes module at installation
 */
static int gpio_host_proxy_probe(struct platform_device *pdev)
{
//	int i;
	
	deb_info("%s, installing module.", __func__);

// *********************
// start of TODO clocks and resets

//	// Read allowed clocks and reset from the device tree
//	// if clocks or resets are not defined, not initialize the module
//	gpio_ares.clocks_size = of_property_read_variable_u32_array(pdev->dev.of_node, 
//		"allowed-clocks", gpio_ares.clock, 0, GPIO_HOST_MAX_CLOCKS_SIZE);
//
//	if(gpio_ares.clocks_size <= 0){
//		deb_error("No allowed clocks defined");
//		return EINVAL;
//	}
//
//	deb_info("gpio_ares.clocks_size: %d", gpio_ares.clocks_size);
//	for (i = 0; i < gpio_ares.clocks_size; i++)	{
//		deb_info("gpio_ares.clock %d", gpio_ares.clock[i]);
//	}
//
//	gpio_ares.resets_size = of_property_read_variable_u32_array(pdev->dev.of_node, 
//		"allowed-resets", gpio_ares.reset, 0, GPIO_HOST_MAX_RESETS_SIZE);
//
//	if(gpio_ares.resets_size <= 0){
//		deb_error("No allowed resets defined");
//		return EINVAL;
//	}
//
//	deb_info("gpio_ares.resets_size: %d", gpio_ares.resets_size);
//	for (i = 0; i < gpio_ares.resets_size; i++)	{
//		deb_info("gpio_ares.reset %d", gpio_ares.reset[i]);
//	}
// end of TODO clocks and resets
// *********************

	// Allocate a major number for the device.
	major_number = register_chrdev(0, DEVICE_NAME, &fops);
	if (major_number < 0)
	{
		deb_error("could not register number.\n");
		return major_number;
	}
	deb_info("registered correctly with major number %d\n", major_number);

	// Register the device class
	gpio_host_proxy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gpio_host_proxy_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to register device class\n");
		return PTR_ERR(gpio_host_proxy_class); // Correct way to return an error on a pointer
	}
	deb_info("device class registered correctly\n");

	// Register the device driver
	gpio_host_proxy_device = device_create(gpio_host_proxy_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gpio_host_proxy_device))
	{								 // Clean up if there is an error
		class_destroy(gpio_host_proxy_class); 
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to create the device\n");
		return PTR_ERR(gpio_host_proxy_device);
	}

	deb_info("device class created correctly\n"); // Made it! device was initialized

	return 0;
}



/*
 * Removes module, sends appropriate message to kernel
 */
static int gpio_host_proxy_remove(struct platform_device *pdev)
{
	deb_info("removing module.\n");
	device_destroy(gpio_host_proxy_class, MKDEV(major_number, 0)); // remove the device
	class_unregister(gpio_host_proxy_class);						  // unregister the device class
	class_destroy(gpio_host_proxy_class);						  // remove the device class
	unregister_chrdev(major_number, DEVICE_NAME);		  // unregister the major number
	deb_info("Goodbye from the LKM!\n");
	unregister_chrdev(major_number, DEVICE_NAME);
	return 0;
}

/*
 * Opens device module, sends appropriate message to kernel
 */
static int open(struct inode *inodep, struct file *filep)
{
	deb_info("device opened.\n");
	return 0;
}

/*
 * Closes device module, sends appropriate message to kernel
 */
static int close(struct inode *inodep, struct file *filep)
{
	deb_info("device closed.\n");
	return 0;
}

/*
 * Reads from device, displays in userspace, and deletes the read data
 */
static ssize_t read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	deb_info("read stub");
	return 0;
}

// TODO
/*
 * Checks if the value to transmit through the
 * gpio-host is allowed by the device tree configuration
 */
/*
static bool check_if_allowed(int val)
{
	return false;
}
*/


/*
 * Writes to the device
 */

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	int ret = len;
	struct tegra_gpio_op *kbuf = NULL;

	if (len > 65535) {	/* paranoia */
		deb_error("count %zu exceeds max # of bytes allowed, "
			"aborting write\n", len);
		goto out_nomem;
	}

	deb_info("wants to write %zu bytes\n", len);

	ret = -ENOMEM;
	kbuf = kmalloc(len, GFP_KERNEL);

	if (!kbuf)
		goto out_nomem;

	memset(kbuf, 0, len);

	if(len != sizeof(struct tegra_gpio_op));
		deb_error("Illegal data length %s\n", __func__);

	ret = -EFAULT;
// TODO
// what format do we want to use?
// guest does not include bank and reg	
	// Copy header (header is / tegra_gpio_op / bank / reg / )
	if (copy_from_user(kbuf, buffer, sizeof(struct tegra_gpio_op))) {
		deb_error("copy_from_user(1) failed\n");
		goto out_cfu;
	}

	if(!tegra_pmx_host){
		deb_error("host device not initialised, can't do transfer!");
		return -EFAULT;
	}

	// kbuf->io_address = (void __iomem *)tegra_pmx_host->regs[kbuf->bank] + kbuf->reg;

// TODO: this will be very simple
//	if(!check_if_allowed(kbuf)){
//		goto out_cfu;
//	}

	// calls pmx_readl or pmx_writel depending on which ...
	switch (kbuf->signal) {
		case 'r':
			kbuf->value = pmx_readl(tegra_pmx_host, kbuf->bank, kbuf->reg);
		break;
		case 'w':
			pmx_writel(tegra_pmx_host, kbuf->value, kbuf->bank, kbuf->reg);
		break;
		default: deb_error("Illegal proxy readl/writel signal type in %s\n", __func__);
		break;
	};

	if (copy_to_user((void *)buffer, kbuf, len)) {
		deb_error("copy_to_user(1) failed\n");
		goto out_notok;
	}


	kfree(kbuf);
	return len;
out_notok:
out_nomem:
	deb_error("memory allocation failed");
out_cfu:
	kfree(kbuf);
	return -EINVAL;

}

static const struct of_device_id gpio_host_proxy_ids[] = {
	{ .compatible = "nvidia,gpio-host-proxy" },
	{ }
};

static struct platform_driver gpio_host_proxy_driver = {
	.driver = {
		.name = "gpio_host_proxy",
		.of_match_table = gpio_host_proxy_ids,
	},
	.probe = gpio_host_proxy_probe,
	.remove = gpio_host_proxy_remove,
};
builtin_platform_driver(gpio_host_proxy_driver);
