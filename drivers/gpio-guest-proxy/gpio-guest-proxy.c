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

#define GPIO_VERBOSE
#define GPIO_GUEST_VERBOSE

#ifdef GPIO_GUEST_VERBOSE
#define deb_info(fmt, ...)     printk(KERN_INFO DEVICE_NAME ": " fmt, ##__VA_ARGS__)
#define deb_debug(fmt, ...)    printk(KERN_DEBUG DEVICE_NAME ": " fmt, ##__VA_ARGS__)

#else
#define deb_info(fmt, ...)
#define deb_debug(fmt, ...)
#endif


static volatile void __iomem  *mem_iova = NULL;

// extern struct tegra_pmx *tegra_pmx_host;
// #define MAX_CHIPS 2		// note this definition must match extern definintion (on NVIDIA Jetson AGX Orin it is 2)
// extern struct gpio_chip *tegra_gpio_hosts[MAX_CHIPS];			// gpio_chip declaration is in driver.h

// extern u32 pmx_readl(struct tegra_pmx *, u32, u32);
// extern void pmx_writel(struct tegra_pmx *, u32, u32, u32);
// extern u32 (*pmx_readl_redirect)(void __iomem *);
// extern void (*pmx_writel_redirect)(u32, void __iomem *);

// static inline u32 my_pmx_readl(void __iomem *);
// static inline void my_pmx_writel(u32, void __iomem *);

extern struct gpio_chip *find_chip_by_name(const char *);	// this is irrelevant for guest proxy?

// void my_tegra186_gpio_set(const char *, unsigned int, int);
// extern void tegra186_gpio_set(struct gpio_chip *, unsigned int, int);

extern int gpio_outloud;
extern uint64_t gpio_vpa;


/* functions redirected to guest-proxy from gpio-tegra186.c
 * from setup of gpio_chip in tegra186_gpio_probe
 */ 

// TODO write proper bodies

int gpiochip_generic_request_redirect(struct gpio_chip *gc, unsigned offset) {
  int ret = 0;
  return ret;
}

void gpiochip_generic_free_redirect(struct gpio_chip *gc, unsigned offset) {}

int tegra186_gpio_get_direction_redirect(struct gpio_chip *chip,
				       unsigned int offset) {
  int ret = 0;
  return ret;
}

int tegra186_gpio_direction_input_redirect(struct gpio_chip *chip,
					 unsigned int offset) {
  int ret = 0;
  return ret;
}

int tegra186_gpio_direction_output_redirect(struct gpio_chip *chip,
					  unsigned int offset, int level) {
  int ret = 0;
  return ret;
}

int tegra186_gpio_get_redirect(struct gpio_chip *chip, unsigned int offset) {
  int ret = 0;
  return ret;
}

void tegra186_gpio_set_redirect(struct gpio_chip *chip, unsigned int offset,
			      int level) {}

void tegra186_gpio_set_by_name_redirect(const char *name, unsigned int offset,
			      int level) {}

int tegra186_gpio_set_config_redirect(struct gpio_chip *chip,
				    unsigned int offset,
				    unsigned long config) {
  int ret = 0;
  return ret;
}

int tegra_gpio_timestamp_control_redirect(struct gpio_chip *chip, unsigned offset,
					int enable) {
  int ret = 0;
  return ret;
}

int tegra_gpio_timestamp_read_redirect(struct gpio_chip *chip, unsigned offset,
				     u64 *ts) {
  int ret = 0;
  return ret;
}

int tegra_gpio_suspend_configure_redirect(struct gpio_chip *chip, unsigned offset,
					enum gpiod_flags dflags) {
  int ret = 0;
  return ret;
}

int tegra186_gpio_add_pin_ranges_redirect(struct gpio_chip *chip) {
  int ret = 0;
  return ret;
}


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


/* copied here as an implemetation template
int my_tegra_bpmp_transfer(struct tegra_bpmp *bpmp, struct tegra_bpmp_message *msg)
{   

	unsigned char io_buffer[MEM_SIZE];
	deb_info("%s\n", __func__);

	memset(io_buffer, 0, sizeof(io_buffer));
	
    if (msg->tx.size >= MESSAGE_SIZE)
        return -EINVAL;

	// Copy msg, tx data and rx data to a single io_buffer
    memcpy(&io_buffer[TX_BUF], msg->tx.data, msg->tx.size);
	memcpy(&io_buffer[TX_SIZ], &msg->tx.size, sizeof(msg->tx.size));
	
	memcpy(&io_buffer[RX_BUF], msg->rx.data, msg->rx.size);
	memcpy(&io_buffer[RX_SIZ], &msg->rx.size, sizeof(msg->rx.size));

	memcpy(&io_buffer[MRQ], &msg->mrq, sizeof(msg->mrq));
	

    hexDump("msg", &msg, sizeof(struct tegra_bpmp_message));
    deb_info("msg.tx.data: %p\n", msg->tx.data);
    hexDump("msg.tx.data", msg->tx.data, msg->tx.size);
	deb_info("msg->rx.size: %ld\n", msg->rx.size);
	
	// Execute the request by coping the io_buffer
	memcpy_toio(mem_iova, io_buffer, MEM_SIZE);

	// Read response to io_buffer
	memcpy_fromio(io_buffer, mem_iova, MEM_SIZE);

	// Copy from io_buffer to msg, tx data and rx data
	memcpy(&msg->tx.size, &io_buffer[TX_SIZ], sizeof(msg->tx.size));
    memcpy((void *)msg->tx.data, &io_buffer[TX_BUF], msg->tx.size);

	memcpy(&msg->rx.size, &io_buffer[RX_SIZ], sizeof(msg->rx.size));
	memcpy(msg->rx.data, &io_buffer[RX_BUF], msg->rx.size);
	
	memcpy(&msg->rx.ret, &io_buffer[RET_COD], sizeof(msg->rx.ret));

	deb_info("%s, END ret: %d\n", __func__, msg->rx.ret);

    return msg->rx.ret;
}
*/

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

	gpio->request = gpiochip_generic_request_redirect;
	gpio->free = gpiochip_generic_free_redirect;
	gpio->get_direction = tegra186_gpio_get_direction_redirect;
	gpio->direction_input = tegra186_gpio_direction_input_redirect;
	gpio->direction_output = tegra186_gpio_direction_output_redirect;
	gpio->get = tegra186_gpio_get_redirect,
	gpio->set = tegra186_gpio_set_redirect;
	gpio->set_config = tegra186_gpio_set_config_redirect;
	gpio->timestamp_control = tegra_gpio_timestamp_control_redirect;
	gpio->timestamp_read = tegra_gpio_timestamp_read_redirect;
	gpio->suspend_configure = tegra_gpio_suspend_configure_redirect;
	gpio->add_pin_ranges = tegra186_gpio_add_pin_ranges_redirect;

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
	// tegra186_gpio_set_redirect = NULL;	// unhook function
 
  // TODO unhook all used functions


	device_destroy(gpio_guest_proxy_class, MKDEV(major_number, 0)); // remove the device
	class_unregister(gpio_guest_proxy_class);						  // unregister the device class
	class_destroy(gpio_guest_proxy_class);						  // remove the device class
	unregister_chrdev(major_number, DEVICE_NAME);		  // unregister the major number
	deb_info("Goodbye from the LKM!\n");
	unregister_chrdev(major_number, DEVICE_NAME);
	return;
}

/* control functons of the drivers char device 
 */

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
	// memcpy(io_data.label, gpio_label, strlen(gpio_label)+1);
  if (strcmp(gpio_label, TEGRA_GPIO_LABEL) == 0) {
    io_data.chipnum = 0;
  }
  else if (strcmp(gpio_label, TEGRA_GPIO_AON_LABEL) == 0) {
    io_data.chipnum = 1;
  } 
  else {
		pr_err("setting label failed\n");
  }
	
	// Execute the request by copying to qemu-io-memory for chardev passthrough
  // we will truncate tegra186_gpio_pt because setended parameters are not used
  // TODO tidy up the implemetation of extended parameters or completely remove them
	memcpy_toio(mem_iova, &io_data, sizeof(io_data)/2);
	
	// read response (no response)
	// memcpy_fromio(io_data.label, mem_iova, sizeof(struct tegra186_gpio_pt));
}

/*
 * Writes to the device
 */

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	// tint i = 0;
  // 	unsigned int ret;
	// unsigned long int ret_l;
	struct tegra_gpio_pt *kbuf = NULL;
	// static struct file *file;
	// static struct inode *inode = NULL;
  // 	struct gpio_chip *chip;
	// unsigned int gpio_number;
  //	char *return_buffer = (char *)buffer;

  // const char *chiplabel[2] = {TEGRA_GPIO_LABEL,TEGRA_GPIO_AON_LABEL};


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
	deb_debug("GPIO chardev parameters: Chip %d, Offset %d, Level %d", kbuf->chipnum, kbuf->offset, kbuf->level);
	
	goto end;

	/*
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
	*/
	
	end:
	kfree(kbuf);
	return len;
}
