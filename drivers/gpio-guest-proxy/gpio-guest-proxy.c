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
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/memory_hotplug.h>
#include <linux/io.h>
#include "../gpio-host-proxy/gpio-host-proxy.h"
#include <linux/gpio/driver.h>

#define DEVICE_NAME "gpio-guest" // Device name.
#define CLASS_NAME "char"	  

MODULE_LICENSE("GPL");						 
MODULE_AUTHOR("Kim Sandstrom");					 
MODULE_DESCRIPTION("NVidia GPIO Guest Proxy Kernel Module"); 
MODULE_VERSION("0.0");				 



#define GPIO_GUEST_VERBOSE	1

#if GPIO_GUEST_VERBOSE
#define deb_info(...)     printk(KERN_INFO DEVICE_NAME ": "__VA_ARGS__)
#define deb_debug(...)    printk(KERN_DEBUG DEVICE_NAME ": "__VA_ARGS__)

#else
#define deb_info(...)
#define deb_debug(...)
#endif

#define GPIO_VERBOSE

static volatile void __iomem  *mem_iova = NULL;

// extern struct tegra_pmx *tegra_pmx_host;
#define MAX_CHIPS 2		// note this definition must match extern definintion (on NVIDIA Jetson AGX Orin it is 2)
extern struct gpio_chip *tegra_gpio_hosts[MAX_CHIPS];			// gpio_chip declaration is in driver.h

// extern u32 pmx_readl(struct tegra_pmx *, u32, u32);
// extern void pmx_writel(struct tegra_pmx *, u32, u32, u32);
// extern u32 (*pmx_readl_redirect)(void __iomem *);
// extern void (*pmx_writel_redirect)(u32, void __iomem *);

// static inline u32 my_pmx_readl(void __iomem *);
// static inline void my_pmx_writel(u32, void __iomem *);
extern void (*tegra186_gpio_set_redirect)(const char *, unsigned int, int);
void my_tegra186_gpio_set(const char *, unsigned int, int);
extern void tegra186_gpio_set(struct gpio_chip *, unsigned int, int);

extern int gpio_outloud;
extern uint64_t gpio_vpa;


/**
 * Important variables that store data and keep track of relevant information.
 */
static int major_number;

static struct class *gpio_guest_proxy_class = NULL;	///< The device-driver class struct pointer
static struct device *gpio_guest_proxy_device = NULL; ///< The device-driver device struct pointer

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
		.write = write
	};


/**
 * Initializes module at installation
 */
int tegra_gpio_guest_init(void)
{

	
	deb_info("%s, installing module.", __func__);

	deb_info("gpio_vpa: 0x%llX", gpio_vpa);

	if(!gpio_vpa){
		pr_err("Failed, gpio_vpa not defined\n");
	}

	// Allocate a major number for the device.
	major_number = register_chrdev(0, DEVICE_NAME, &fops);
	if (major_number < 0)
	{
		pr_err("could not register number.\n");
		return major_number;
	}
	deb_info("registered correctly with major number %d\n", major_number);

	// Register the device class
	gpio_guest_proxy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gpio_guest_proxy_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(major_number, DEVICE_NAME);
		pr_err("Failed to register device class\n");
		return PTR_ERR(gpio_guest_proxy_class); // Correct way to return an error on a pointer
	}
	deb_info("device class registered correctly\n");

	// Register the char device driver
	gpio_guest_proxy_device = device_create(gpio_guest_proxy_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gpio_guest_proxy_device))
	{								 // Clean up if there is an error
		class_destroy(gpio_guest_proxy_class); 
		unregister_chrdev(major_number, DEVICE_NAME);
		pr_err("Failed to create the device\n");
		return PTR_ERR(gpio_guest_proxy_device);
	}
	deb_info("device class created correctly\n"); // Made it! device was initialized

	// map iomem used/provided by Qemu
	// gpio_vpa is fake-physical memory
	// value of gpio_vpa defines address in io-space, 
	// second parameter gives the size of the memory range
	mem_iova = ioremap(gpio_vpa, sizeof(struct tegra_gpio_pt));

	if (!mem_iova) {
		pr_err("ioremap failed\n");
		return -ENOMEM;
	}

	deb_info("gpio_vpa: 0x%llX, mem_iova: %p\n", gpio_vpa, mem_iova);

	// pmx_readl_redirect = my_pmx_readl;
	// pmx_writel_redirect = my_pmx_writel;
	tegra186_gpio_set_redirect = my_tegra186_gpio_set; 

	return 0;
}
EXPORT_SYMBOL(tegra_gpio_guest_init);


/*
 * Removes module, sends appropriate message to kernel
 */
void tegra_gpio_guest_cleanup(void)
{
	deb_info("removing module.\n");

	// unmap iomem
	iounmap((void __iomem*)gpio_vpa);

	// pmx_writel_redirect = NULL;	// unhook function
	// pmx_readl_redirect = NULL;	// unhook function
	tegra186_gpio_set_redirect = NULL;	// unhook function
	device_destroy(gpio_guest_proxy_class, MKDEV(major_number, 0)); // remove the device
	class_unregister(gpio_guest_proxy_class);						  // unregister the device class
	class_destroy(gpio_guest_proxy_class);						  // remove the device class
	unregister_chrdev(major_number, DEVICE_NAME);		  // unregister the major number
	deb_info("Goodbye from the LKM!\n");
	unregister_chrdev(major_number, DEVICE_NAME);
	return;
}

/*
 * Opens device module, sends appropriate message to kernel
 */
static int open(struct inode *inodep, struct file *filep)
{
	deb_info("device opened.\n");
	gpio_outloud = 1;
	return 0;
}

/*
 * Closes device module, sends appropriate message to kernel
 */
static int close(struct inode *inodep, struct file *filep)
{
	deb_info("device closed.\n");
	gpio_outloud = 0;
	return 0;
}

/*
 * Reads from device
 */
static ssize_t read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	deb_info("read stub");
	return 0;
}

void my_tegra186_gpio_set(const char *gpio_label, unsigned int offset, int level)
{
	struct tegra_gpio_pt io_data = {
	.signal = 's',       // 's' stands for "set"
	// .label = memcpy...gpio_label,
	.offset = offset,
	.level = level
	};
	
	deb_info("%s\n", __func__);
	
	// copy the label of the gpiochip 
	// At this moment we might not pass through both chips -- thus this is maybe redundant
	memcpy(io_data.label, gpio_label, strlen(gpio_label)+1);
	
	// Execute the request by copying to qemu-io-memory for chardev passthrough
	memcpy_toio(mem_iova, &io_data, sizeof(io_data));
	
	// read response (no response)
	// memcpy_fromio(io_data.label, mem_iova, sizeof(struct tegra186_gpio_pt));
}

/*
 * Writes to the device
 */

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	int i = 0;
	struct tegra_gpio_pt *kbuf = NULL;

	deb_info("wants to write %zu bytes\n", len);
	
	if (len > 65535) {	
		pr_err("count %zu exceeds max # of bytes allowed, "
			"aborting write\n", len);
		return -EINVAL;
	}

	if(len != sizeof(struct tegra_gpio_pt)) {
		pr_err("Illegal data length %s\n", __func__);
		return -EFAULT;
	}

	kbuf = kmalloc(len, GFP_KERNEL);
	if ( !kbuf ) {
	  pr_err("memory allocation failed");
	  return -ENOMEM;
	}

	memset(kbuf, 0, len);

	// Copy header
	if (copy_from_user(kbuf, buffer, sizeof(struct tegra_gpio_pt))) {
		pr_err("copy_from_user failed\n");
	  kfree(kbuf);
	  return -EINVAL;
	}

	// copied user parameters
	deb_debug("GPIO chardev parameters: Chip %s, Offset %d, Level %d", kbuf->label, kbuf->offset, kbuf->level);
	
	if ( strlen(kbuf->label) >= GPIOCHIP_PTLABEL ) {
	  pr_err("GPIO chardev label length is too big");
	  kfree(kbuf);
	  return -EINVAL;
	}

	// check if host has initialised gpio chip
	while (i <= MAX_CHIPS) {
		if ( strcmp(kbuf->label, tegra_gpio_hosts[i]->label) ) {
		i++; // chip not found
		}
		else { break; }
	}

	if ( i >= MAX_CHIPS) 
	{
		pr_err("host device not initialised, can't do transfer!");
		kfree(kbuf);
		return -EFAULT;
	}

	deb_info("Using GPIO chip %s", tegra_gpio_hosts[i]->label);
	// hexDump ("Chardev struct:",kbuf, len);  // hexDump is defined in gpio-host-proxy.c

	// make call to manipulate pins
	switch (kbuf->signal) {
		// only 's' for "set" is implemented at the moment
		case 's':		
			tegra186_gpio_set(tegra_gpio_hosts[i], kbuf->offset, kbuf->level);  		// only funtion implemented at the moment
		break;
		default: pr_err("Illegal proxy readl/writel signal type in %s\n", __func__);
		break;
	};

	// no need to copy a response because there is none.
	// if (copy_to_user((void *)buffer, kbuf, len)) {
	//	pr_err("copy_to_user(1) failed\n");
	//	goto out_notok;
	// }


	kfree(kbuf);
	return len;
}
