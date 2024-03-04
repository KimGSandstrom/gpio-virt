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
#include "../gpio-host-proxy/gpio-host-proxy.h"
#include <linux/gpio/driver.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/namei.h>

#define DEVICE_NAME "gpio-host"   // Device name.
#define CLASS_NAME  "chardrv"	  // < The device class -- this is a character device driver

MODULE_LICENSE("GPL");						///< The license type -- this affects available functionality
MODULE_AUTHOR("Kim Sandström");					///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("NVidia GPIO Host Proxy Kernel Module");	///< The description -- see modinfo
MODULE_VERSION("0.0");						///< A version number to inform users

#define GPIO_VERBOSE
#define GPIO_HOST_VERBOSE

#ifdef GPIO_HOST_VERBOSE
#define deb_info(fmt, args...)     printk(KERN_INFO DEVICE_NAME ": " fmt, ##args)
#define deb_debug(fmt, args...)    printk(KERN_DEBUG DEVICE_NAME ": " fmt, ##args)
#else
#define deb_info(fmt, args...)
#define deb_debug(fmt, args...)
#endif

// extern struct tegra_pmx *tegra_pmx_host;
// extern u32 pmx_readl(struct tegra_pmx *, u32, u32);
// extern void pmx_writel(struct tegra_pmx *, u32, u32, u32);
// #define MAX_CHIPS 2		// note this definition must match extern definintion (on NVIDIA Jetson AGX Orin it is 2)
// extern struct gpio_chip *tegra_gpio_hosts[MAX_CHIPS];			// gpio_chip declaration is in driver.h
// extern struct gpio_device *proxy_host_gpio_dev[MAX_CHIPS];

// these functions are actually not used
/*
extern int gpiod_request(struct gpio_chip *);
extern int gpiochip_generic_request(struct gpio_chip *, unsigned);
extern int tegra186_gpio_get_direction(struct gpio_chip *, unsigned int);
extern int tegra186_gpio_direction_input(struct gpio_chip *, unsigned int);
extern int tegra186_gpio_direction_output(struct gpio_chip *, unsigned int);
extern void tegra186_gpio_set(struct gpio_chip *, unsigned int, int);
extern void gpiod_free(struct gpio_chip *);
*/

extern struct gpio_chip *find_chip_by_name(const char *);

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
		.write = write
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

	// Process every byte of hexDump the data.

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

	printk(DEVICE_NAME ": \n%s", out_buff);
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
//		pr_err("No allowed clocks defined");
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
//		pr_err("No allowed resets defined");
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
		pr_err("could not register number.\n");
		return major_number;
	}
	deb_info("registered correctly with major number %d\n", major_number);

	// Register the device class
	gpio_host_proxy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gpio_host_proxy_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(major_number, DEVICE_NAME);
		pr_err("Failed to register device class\n");
		return PTR_ERR(gpio_host_proxy_class); // Correct way to return an error on a pointer
	}
	deb_info("device class registered correctly\n");

	// Register the device driver
	gpio_host_proxy_device = device_create(gpio_host_proxy_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gpio_host_proxy_device))
	{								 // Clean up if there is an error
		class_destroy(gpio_host_proxy_class);
		unregister_chrdev(major_number, DEVICE_NAME);
		pr_err("Failed to create the device\n");
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
	unsigned int ret;
	unsigned long int ret_l;
	struct tegra_gpio_pt *kbuf = NULL;
	static struct file *file;
	static struct inode *inode = NULL;
	struct gpio_chip *chip;
	// unsigned int gpio_number;
	char *return_buffer = (char *)buffer;

	deb_info("wants to write %zu bytes in chardev %s\n", len, __func__);

	if (len > 65535) {
		pr_err("count %zu exceeds max # of bytes allowed, "
			"aborting write\n", len);
		return -EINVAL;
	}

	if(len != sizeof(struct tegra_gpio_pt)) {
		pr_err("Illegal %s chardev data length. Expected %ld, got %ld\n", __func__, sizeof(struct tegra_gpio_pt), len);
		return -ENOEXEC;
	}

	if (offset) {
		pr_err("In %s, offset was expected to be zero but %p was found\n", __func__, offset);
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
		return -EFAULT;
	}

	if ( strlen(kbuf->label) >= GPIOCHIP_LABEL ) {
		printk(KERN_ERR "GPIO chardev label length is too big");
		kfree(kbuf);
		return -EINVAL;
	}
	
	// copied user parameters
	deb_debug("parameters, Chip %s, Offset %d, Level %d", kbuf->label, kbuf->p1.offset, kbuf->p2.level);

	/* removed because we cant easily use the struct tegra_gpio_hosts points to
	// check if host has initialised gpio chip
	while (i <= MAX_CHIPS) {
		if ( tegra_gpio_hosts[i] == NULL) { i = MAX_CHIPS; break; } // we do not have initalised tegra_gpio_hosts
		// if (is_valid_user_ptr(tegra_gpio_hosts[i]->label, size))
		if ( strcmp(kbuf->label, tegra_gpio_hosts[i]->label) ) { i++; } // chip not found 
		else { break; }
	}

	if ( i >= MAX_CHIPS)
	{
		pr_err("host device not initialised, can't do transfer!");
		kfree(kbuf);
		return -ENODEV;
	}
*/

	//	deb_info("Using GPIO chip %s", tegra_gpio_hosts[i]->label);
	hexDump ("Chardev struct", kbuf, len);

	// make call to gpio
	switch (kbuf->signal) {
		/*
		// functions in include/linux/gpio.h
		 * based on gpio number
		static inline int gpio_get_value(unsigned int gpio)
		static inline void gpio_set_value(unsigned int gpio, int value)
		static inline int gpio_direction_output(unsigned gpio, int value)
		static inline int gpio_get_value(unsigned gpio)
		static inline void gpio_set_value(unsigned gpio, int value)
		static inline int gpio_get_value_cansleep(unsigned gpio)
		static inline void gpio_set_value_cansleep(unsigned gpio, int value)

		// functions in  drivers/gpio/gpiolib-legacy.c
		 * based on gpio number
		void gpio_free(unsigned gpio)
		int gpio_request_one(unsigned gpio, unsigned long flags, const char *label)
		int gpio_request(unsigned gpio, const char *label)
		int gpio_request_array(const struct gpio *array, size_t num)
		void gpio_free_array(const struct gpio *array, size_t num)

		*/

		/* functions in drivers/gpio/gpio-tegra.c
		 * based on gpio_chip
		static void tegra_gpio_free(struct gpio_chip *chip, unsigned int offset)
		static void tegra_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
		static int tegra_gpio_get(struct gpio_chip *chip, unsigned int offset)
		static int tegra_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
		static int tegra_gpio_direction_output(struct gpio_chip *chip, unsigned int offset, int value)
		static int tegra_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
		*/
		/*
		static const struct file_operations gpio_fileops = {
		.release = gpio_chrdev_release,
		.open = gpio_chrdev_open,
		.poll = lineinfo_watch_poll,
		.read = lineinfo_watch_read,
		.owner = THIS_MODULE,
		.llseek = no_llseek,
		.unlocked_ioctl = gpio_ioctl,
		#ifdef CONFIG_COMPAT
			.compat_ioctl = gpio_ioctl_compat,
		#endif
		};
		*/
		// We could want to use the gpio chardev (/dev/gpiochip0 and /dev/gpiochip1) /bc userspace functions use it
		case GPIO_CHARDEV_OPEN:	// .open = gpio_chrdev_open
			file = filp_open(kbuf->label, O_RDWR, 0);
		    if (IS_ERR(file)) {
				pr_err("GPIO %s, failed to open chardev %s: %ld\n", __func__, kbuf->label, PTR_ERR(file));
				kfree(kbuf);
				return -ENOENT;
			}

			inode = file->f_path.dentry->d_inode;
			// defined as: static int gpio_chrdev_open(struct inode *inode, struct file *file)
			ret = file->f_op->open(inode, file);
			goto retval;
		break;
		case GPIO_CHARDEV_IOCTL:	// .unlocked_ioctl = gpio_ioctl
			// user space triggers gpio_ioctl -- it is .unlocked_ioctl on the chardev
			if( !file ) {
				pr_err("GPIO %s, chardev file was expected to be open\n", __func__);
				kfree(kbuf);
				return -ENOENT;
			}	
			// defined as: static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
			ret_l = file->f_op->unlocked_ioctl(file, kbuf->p1.cmd, kbuf->p2.arg);	// offset is used for cmd, arg is pointer data which should have been copied
			goto retlong;
		break;
		case GPIO_CHARDEV_RELEASE: // .release = gpio_chrdev_release
			if( !file ) {
				pr_err("GPIO %s, chardev file was expected to be open\n", __func__);
				kfree(kbuf);
				return -ENOENT;
			}				
			// defined as: static int gpio_chrdev_release(struct inode *inode, struct file *file)
			ret = file->f_op->release(inode, file);		
			goto retval;
		break;
		case GPIO_CHARDEV_POLL: // .poll = lineinfo_watch_poll
			if( !file ) {
				pr_err("GPIO %s, chardev file was expected to be open\n", __func__);
				kfree(kbuf);
				return -ENOENT;
			}				
			// defined as: static __poll_t lineinfo_watch_poll(struct file *file, struct poll_table_struct *pollt)
			ret = file->f_op->poll(file, kbuf->p2.poll);	// TODO arg is pointer data which should have been copied
			goto retval;	// __poll_t is of size unsigned int 
		break;
		case GPIO_CHARDEV_READ: // .read = lineinfo_watch_read
			if( !file ) {
				pr_err("GPIO %s, chardev file was expected to be open\n", __func__);
				kfree(kbuf);
				return -ENOENT;
			}				
			// defined as: static ssize_t lineinfo_watch_read(struct file *file, char __user *buf, size_t count, loff_t *off)
			ret = file->f_op->read(file, return_buffer, kbuf->p2.count, NULL);		// 
			if (ret) {
				pr_err("Reading lineinfo returned zero\n");
				kfree(kbuf);
				return -EFAULT;
			}
		return -ENXIO;
		case GPIO_CHARDEV_OWNER: // .owner = THIS_MODULE
			if (copy_to_user(return_buffer, file->f_op->owner->name, strlen(file->f_op->owner->name)+1)) {
				pr_err("GPIO %s, copying user return value failed\n", __func__);
				kfree(kbuf);
				return -EFAULT;
			}
			// ret_sz = strlen(file->f_op->owner->name) + 1;
			// goto generic_ret
		break;
		/* tegra gpio from gpio-tegra186.c
		 * from setup of gpio_chip in tegra186_gpio_probe
		gpio->gpio.request = gpiochip_generic_request;
		gpio->gpio.free = gpiochip_generic_free;						
		gpio->gpio.get_direction = tegra186_gpio_get_direction;
		gpio->gpio.direction_input = tegra186_gpio_direction_input;
		gpio->gpio.direction_output = tegra186_gpio_direction_output;
		gpio->gpio.get = tegra186_gpio_get,
		gpio->gpio.set = tegra186_gpio_set;
		gpio->gpio.set_config = tegra186_gpio_set_config;
		gpio->gpio.timestamp_control = tegra_gpio_timestamp_control;
		gpio->gpio.timestamp_read = tegra_gpio_timestamp_read;
		gpio->gpio.suspend_configure = tegra_gpio_suspend_configure;
		gpio->gpio.add_pin_ranges = tegra186_gpio_add_pin_ranges;
		gpio->gpio.base = -1;
		* struct gpio_chip in gpio-tegra.c seems not likely
		*/
		// version using gpio_chip from tegra186_gpio_probe
		case GPIO_REQ:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->request(chip, kbuf->p1.offset);
			goto retval;
		break;
		case GPIO_FREE:
			chip = find_chip_by_name(kbuf->label);
			chip->free(chip, kbuf->p1.offset);
		break;
		case GPIO_GET_DIR:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->get_direction(chip, kbuf->p1.offset);
			goto retval;
		break;
		case GPIO_SET_IN:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->direction_input(chip, kbuf->p1.offset);
			goto retval;
		break;
		case GPIO_SET_OUT:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->direction_output(chip, kbuf->p1.offset, kbuf->p2.level);
			goto retval;
		break;
		case GPIO_GET_VALUE:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->get(chip, kbuf->p1.offset);
			goto retval;
		break;
		case GPIO_SET_VALUE:
			chip = find_chip_by_name(kbuf->label);
			chip->set(chip, kbuf->p1.offset, kbuf->p2.level);
		break;
		case GPIO_CONFIG:
			chip = find_chip_by_name(kbuf->label);
			// static int tegra186_gpio_set_config(struct gpio_chip *chip, unsigned int offset, unsigned long config)
			chip->set_config(chip, kbuf->p1.offset, kbuf->p2.config); // arg mapped to unsigned long config
		break;
		case GPIO_TIMESTAMP_CTRL:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->timestamp_control(chip, kbuf->p1.offset, kbuf->p2.level);	// mapping level onto enable
			goto retval;
		break;
		case GPIO_TIMESTAMP_READ:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->timestamp_read(chip, kbuf->p1.offset, (u64 *)return_buffer);	// timestamp is u64, return value as pointer
			// timestamp_read returns value in return_buffer
		break;
		case GPIO_SUSPEND_CONF:
			chip = find_chip_by_name(kbuf->label);
			ret = chip->suspend_configure(chip, kbuf->p1.offset, kbuf->p2.dflags);	// level is int same sizeof as enum gpiod_flags dflags
			goto retval;
		break;		
		/* 
		 * linehandle_create	-- when userspace requests output (called by gpio_ioctl) -- bypasses the chardev
		 * linehandle_ioctl		-- linehandle_ioctl when userspace does actual io (toggles pin)
		 * 						cmd:
		 * 							GPIOHANDLE_GET_LINE_VALUES_IOCTL,
		 * 							GPIOHANDLE_SET_LINE_VALUES_IOCTL,
		 * 							GPIOHANDLE_SET_CONFIG_IOCTL
		 * 						arg: user input or output
		 */
		default:
			pr_err("GPIO %s, Illegal proxy signal type\n", __func__);
			kfree(kbuf);
			return -EPERM;
		break;
	};

	goto end;

	retlong:
	if ( copy_to_user(return_buffer, &ret_l, sizeof(ret_l)) ) {
		pr_err("GPIO %s, copying int user return value failed\n", __func__);
		kfree(kbuf);
		return -EFAULT;
	}
	goto end;
	
	retval:
	if ( copy_to_user(return_buffer, &ret, sizeof(ret)) ) {
		pr_err("GPIO %s, copying long int user return value failed\n", __func__);
		kfree(kbuf);
		return -EFAULT;
	}
	goto end;
	
	end:
	kfree(kbuf);
	return len;
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
