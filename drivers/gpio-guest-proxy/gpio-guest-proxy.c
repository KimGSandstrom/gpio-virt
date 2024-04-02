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
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/namei.h>
#include <linux/delay.h>

// we need a workaround for this -- relevant data copied to gpio-host-proxy.h
// #include "$(abs_srctree)/drivers/gpio/gpiolib.h"



#define DEVICE_NAME "gpio-guest" // Device name.
#define CLASS_NAME "char"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kim Sandstrom");
MODULE_DESCRIPTION("NVidia GPIO Guest Proxy Kernel Module");
MODULE_VERSION("0.0");

#define GPIO_DEBUG
#define GPIO_DEBUG_VERBOSE       // also activates deb_verbose commands

#ifdef GPIO_DEBUG
  #define deb_info(fmt, ...)     printk(KERN_INFO "GPIO func \'%s\' in file \'%s\'" fmt, __func__, __FILE__, ##__VA_ARGS__)
  #define deb_debug(fmt, ...)    printk(KERN_DEBUG "GPIO func \'%s\' in file \'%s\'" fmt, __func__ , __FILE__, ##__VA_ARGS__)
  #define deb_error(fmt, ...)    printk(KERN_ERR "GPIO func \'%s\' in file \'%s\'" fmt, __func__ , __FILE__, ##__VA_ARGS__)
#else
  #define deb_info(fmt, ...)
  #define deb_debug(fmt, ...)
  #define deb_error(fmt, ...)
#endif

#ifdef GPIO_DEBUG_VERBOSE
  #define deb_verbose           deb_debug
  extern void hexDump (
    const char * desc,
    const void * addr,
    const int len
  );
#else
  #define deb_verbose(fmt, ...)
#endif

#define MEM_SIZE       0x0600   // size of passthrough memory (mem_iova) -- larger than needed
static volatile void __iomem  *mem_iova = NULL;

extern struct gpio_chip *find_chip_by_name(const char *);
extern struct gpio_chip *find_chip_by_id(int);
extern const char **tegra_chiplabel;

extern int gpio_outloud;
extern uint64_t gpio_vpa;

/* functions redirected to guest-proxy from gpio-tegra186.c
 * from setup of gpio_chip in tegra186_gpio_probe
 */

/* guest_chardev_transfer
 * a helper function to transfer between guest and host using /dev/gpio-host
 * mgs: is a pointer to data -- in practice tegra_gpio_pt and tegra_gpio_pt_extended structs
 * generic return: poiter to return data, may be NULL if we do not expect a return value
 */ 
void guest_chardev_transfer(void *msg, int msg_len, int *generic_return)
{
	unsigned char *io_buffer;

	deb_debug("\n");

	// Copy msg, to io_buffer
	io_buffer = kmalloc(msg_len, GFP_KERNEL);
	memset(io_buffer, 0, msg_len);
  memcpy(io_buffer, msg, msg_len);

  #ifdef GPIO_DEBUG_VERBOSE
    hexDump("msg", &msg, msg_len);
    deb_verbose("msg signal is: %c\n", *(char *)msg);
  #endif

	// Execute the request by copying the io_buffer
	memcpy_toio(mem_iova, io_buffer, msg_len);

	// Read response to io_buffer
	memcpy_fromio(io_buffer, mem_iova, sizeof(*generic_return));

  // check if we expect a return value
  if(generic_return) {
	// Copy reply to io_buffer
	memcpy(generic_return, io_buffer, sizeof(*generic_return));
	deb_verbose("return value is copied %d\n", *generic_return);
  }

  kfree(io_buffer);
}

int gpiochip_generic_request_redirect(struct gpio_chip *chip, unsigned offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_REQ;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

void gpiochip_generic_free_redirect(struct gpio_chip *chip, unsigned offset) {
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_FREE;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL);
}

int tegra186_gpio_get_direction_redirect(struct gpio_chip *chip,
				       unsigned int offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_GET_DIR;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra186_gpio_direction_input_redirect(struct gpio_chip *chip,
					 unsigned int offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_SET_IN;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra186_gpio_direction_output_redirect(struct gpio_chip *chip,
					  unsigned int offset, int level) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_SET_OUT;
  msg.chipnum = chip->gpiodev->id;
  msg.level = level;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra186_gpio_get_redirect(struct gpio_chip *chip, unsigned int offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_GET;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

void tegra186_gpio_set_redirect(struct gpio_chip *chip, unsigned int offset,
			      int level) {
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_SET;
  msg.chipnum = chip->gpiodev->id;
  msg.level = level;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL);
}

void tegra186_gpio_set_by_name_redirect(const char *name, unsigned int offset,
			      int level) {
  struct tegra_gpio_pt msg;
  struct gpio_chip *chip = find_chip_by_name(name);
  msg.signal = GPIO_SET_BY_NAME;
  msg.chipnum = chip->gpiodev->id;
  msg.level = level;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL);
}

int tegra186_gpio_set_config_redirect(struct gpio_chip *chip,
				    unsigned int offset,
				    unsigned long config) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_CONFIG;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra_gpio_timestamp_control_redirect(struct gpio_chip *chip, unsigned offset,
					int enable) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_TIMESTAMP_CTRL;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra_gpio_timestamp_read_redirect(struct gpio_chip *chip, unsigned offset,
				     u64 *ts) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_TIMESTAMP_READ;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

int tegra_gpio_suspend_configure_redirect(struct gpio_chip *chip, unsigned offset,
					enum gpiod_flags dflags) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_SUSPEND_CONF;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

// TODO,  this probably does not work because action parameters should be in gpio_chip struct, but guest did not copy the over the passthrough interface
int tegra186_gpio_add_pin_ranges_redirect(struct gpio_chip *chip) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_ADD_PINRANGES;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = 0;

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return ret;
}

/*
// export of redirection functions
EXPORT_SYMBOL_GPL(gpiochip_generic_request_redirect);
EXPORT_SYMBOL_GPL(gpiochip_generic_free_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_get_direction_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_direction_input_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_direction_output_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_get_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_set_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_set_by_name_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_set_config_redirect);
EXPORT_SYMBOL_GPL(tegra_gpio_timestamp_control_redirect);
EXPORT_SYMBOL_GPL(tegra_gpio_timestamp_read_redirect);
EXPORT_SYMBOL_GPL(tegra_gpio_suspend_configure_redirect);
EXPORT_SYMBOL_GPL(tegra186_gpio_add_pin_ranges_redirect);
*/

// unpreserve_all_tegrachips also does unhooking ?
extern void unpreserve_all_tegrachips(void);
struct gpio_chip * find_chip_by_id(int id);

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
int tegra_gpio_guest_init(struct gpio_chip *gpio)
{
	deb_info("installing module.");

	deb_info("gpio_vpa: 0x%llx", gpio_vpa);

	if(!gpio_vpa){
		pr_err("Failed, gpio_vpa not defined");
	}

	// Allocate a major number for the device.
	major_number = register_chrdev(0, DEVICE_NAME, &fops);
	if (major_number < 0)
	{
		pr_err("could not register number.");
		return major_number;
	}
	deb_info("registered correctly with major number %d", major_number);

	// Register the device class
	gpio_guest_proxy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gpio_guest_proxy_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to register device class\n");
		return PTR_ERR(gpio_guest_proxy_class); // Correct way to return an error on a pointer
	}
	deb_info("device class registered correctly\n");

	// Register the device driver
	gpio_guest_proxy_device = device_create(gpio_guest_proxy_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gpio_guest_proxy_device))
	{								 // Clean up if there is an error
		class_destroy(gpio_guest_proxy_class); 
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to create the device\n");
		return PTR_ERR(gpio_guest_proxy_device);
	}
	deb_info("device class created correctly\n"); // Made it! device was initialized

	// map iomem
	mem_iova = ioremap(gpio_vpa, MEM_SIZE);

	if (!mem_iova) {
        deb_error("ioremap failed\n");
        return -ENOMEM;
  }

	deb_info("gpio_vpa: 0x%llX, mem_iova: %p\n", gpio_vpa, mem_iova);

  // gpio_hook is called by preserve_tegrachip() in gpio_tegra186.c -- don't call it here
  // gpio_hook()

	return 0;
}

EXPORT_SYMBOL_GPL(tegra_gpio_guest_init);

/*
 * Removes module, sends appropriate message to kernel
 */
void tegra_gpio_guest_cleanup(void)
{
	deb_info("removing module.\n");

	// unmap iomem
	iounmap((void __iomem*)gpio_vpa);

  // gpio_unhook is called by unpreserve_all_tegrachips()
  // gpio_unhook()
  // clean up shared memory with stock driver and unhook all functions
  unpreserve_all_tegrachips();

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
 * Reads from device, displays in userspace, and deletes the read data
 */
static ssize_t read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	deb_info("read stub");
	return 0;
}

/*
 * Writes to the device
 */

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	// tint i = 0;
  unsigned int ret;
	unsigned long int ret_l;
	// struct tegra_gpio_pt *kbuf = NULL;
	// static struct file *file;
	// static struct inode *inode = NULL;
  // 	struct gpio_chip *chip;
	// unsigned int gpio_number;
  char *return_buffer = (char *)buffer;
  char *read_buffer = (char *)buffer;

	deb_info("wants to write %zu bytes", len);

	if (len > 65535) {
		pr_err("count %zu exceeds max # of bytes allowed, aborting write", len);
		return -EINVAL;
	}

	if(len != sizeof(struct tegra_gpio_pt) || len != sizeof(tegra_gpio_pt_extended) ) {
		pr_err("Illegal data length %ld", len);
		return -EFAULT;
	}

  /*
	kbuf = kmalloc(len, GFP_KERNEL);
	if ( !kbuf ) {
	  pr_err("memory allocation failed");
	  return -ENOMEM;
	}

	memset(kbuf, 0, len);

	// Copy header
	if (copy_from_user(kbuf, read_buffer, sizeof(struct tegra_gpio_pt))) {
		pr_err("copy_from_user failed");
	  kfree(kbuf);
	  return -ENOMEM;
	}

	// copied user parameters
	deb_debug("GPIO chardev parameters: Chip %d, Offset %d, Level %d", kbuf->chipnum, kbuf->offset, kbuf->level);
	*/

  // send data "as is" to host
	memcpy_toio(mem_iova, read_buffer, len);
  // TODO handle return messages from host

	goto end;
  goto retlong;
  goto retval;

	retlong:
	if ( copy_to_user(return_buffer, &ret_l, sizeof(ret_l)) ) {
		pr_err("GPIO, copying int user return value failed");
		//kfree(kbuf);
		return -EFAULT;
	}
	goto end;

	retval:
	if ( copy_to_user(return_buffer, &ret, sizeof(ret)) ) {
		pr_err("GPIO, copying long int user return value failed");
		//kfree(kbuf);
		return -EFAULT;
	}
	goto end;

	end:
	//kfree(kbuf);
	return len;
}
