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
#include <linux/gpio/driver.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/namei.h>
#include <linux/delay.h>

#include "../gpio-host-proxy/gpio-host-proxy.h"

#define DEVICE_NAME "gpio-guest" // Device name.
#define CLASS_NAME "char"

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Kim Sandstrom");
MODULE_DESCRIPTION("NVidia GPIO Guest Proxy Kernel Module");
MODULE_VERSION("0.0");

#define GPIO_DEBUG
#define GPIO_DEBUG_VERBOSE       // also activates deb_verbose commands

#ifdef GPIO_DEBUG
  #define deb_info(fmt, ...)     printk(KERN_INFO "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__, kbasename(__FILE__), ##__VA_ARGS__)
  #define deb_debug(fmt, ...)    printk(KERN_DEBUG "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__ , kbasename(__FILE__), ##__VA_ARGS__)
  #define deb_error(fmt, ...)    printk(KERN_ERR "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__ , kbasename(__FILE__), ##__VA_ARGS__)
#else
  #define deb_info(fmt, ...)
  #define deb_debug(fmt, ...)
  #define deb_error(fmt, ...)
#endif

#ifdef GPIO_DEBUG_VERBOSE
  #define deb_verbose           deb_debug
  extern void hexDump (
    const char * deviceName,
    const char * desc,
    const void * addr,
    const int len
  );
#else
  #define deb_verbose(fmt, ...)
  #define hexDump(...)
#endif

// MEM_SIZE defined in gpio-host-proxy.h
static volatile void __iomem  *mem_iova = NULL;

extern struct gpio_chip *find_chip_by_name(const char *);
extern struct gpio_chip *find_chip_by_id(int);
extern const char **tegra_chiplabel;

extern int gpio_outloud;
extern uint64_t gpio_vpa;

#define RET_SIZE 8  // should be sizeof(uint64_t)
// static char return_buffer[MEM_SIZE];   // using the same size as the input buffer
static char return_buffer[RET_SIZE] = {0, 0, 0, 0, 0, 0, 0, 0};
static unsigned int return_size = 0;

_Static_assert(sizeof(uint64_t) == RET_SIZE, "return size assertion for RET_SIZE failed");
_Static_assert(sizeof(u32) == sizeof(int), "return size assertion for int failed");
_Static_assert(sizeof(volatile void __iomem *) == sizeof(uint64_t), "return size assertion for iomem pointer failed");

/* functions redirected to guest-proxy from gpio-tegra186.c
 * from setup of gpio_chip in tegra186_gpio_probe
 */

/* guest_chardev_transfer
 * a helper function to transfer between guest and host using /dev/gpio-host
 * mgs: is a pointer to data -- in practice tegra_gpio_pt and tegra_gpio_pt_extended structs
 * generic return: poiter to return data, may be NULL if we do not expect a return value
 */ 
inline void guest_chardev_transfer(void *msg, char msg_len, void *generic_return, int ret_len)
{
  // encode message length into first byte (LSB bit is preserved for other use)
  unsigned char *length = (unsigned char *)msg;
  *length = *length | (unsigned char)msg_len << 1;

  if(msg_len & 0x80) {
    deb_error("Illegal message length\n"); // msb will be deleted and must be zero
  }

  // deb_verbose("PT transfer signal is: %c", *((char *)msg + 1));
  hexDump(DEVICE_NAME, "PT transfer (message to host)", msg, msg_len);

	// Execute the request by copying to io memory
	memcpy_toio(mem_iova, msg, msg_len);
  // deb_verbose("PT request value is copied, length = %d", msg_len);

  // deb_verbose("PT generic_return pointer: 0x%llX\n", (long long int)generic_return);
  // check if we expect a return value
  if(generic_return) {
    // Read response from io_buffer
    hexDump(DEVICE_NAME, "GPIO: PT dump mem_iova", (char *)mem_iova, MEM_SIZE);
    deb_verbose("ret_len = %d\n", ret_len);
    memcpy_fromio(generic_return, (void *)((uint64_t *)mem_iova + RETURN_OFF), ret_len); // 32 bits except for tegra186_gpio_get_base_redirect 64 bits
    hexDump(DEVICE_NAME, "GPIO: PT transfer (retrieved retval from host)", generic_return, ret_len );  }
}

void __iomem *tegra186_gpio_get_base_redirect(unsigned char id, unsigned int pin) {
  void __iomem *ret_ptr = (void __iomem *)0xDEADBEEFDEADBEEF;  
  struct tegra_getbase_pt msg;

  msg.signal = TEGRA_186_GETBASE;
  msg.chipnum = id;
  msg.pin = pin;
  
  guest_chardev_transfer(&msg, sizeof(msg), &ret_ptr, sizeof(ret_ptr));
  return ret_ptr;
}

// redirect static inline u32 readl_x(const volatile void __iomem *addr)
inline u32 readl_redirect( void * addr, const unsigned char rwltype) {
  u32 ret = 0;
  struct tegra_readl_writel rwlmsg;

  rwlmsg.signal = GPIO_READL;
  rwlmsg.length = 0; // will be updated later
  rwlmsg.rwltype = rwltype;
  rwlmsg.address = addr;
  rwlmsg.value = 0;  // value field is not used for readl

  guest_chardev_transfer(&rwlmsg, sizeof(rwlmsg), &ret, sizeof(ret));
  deb_verbose(" 0x%X",  ret);
  return ret;
}
EXPORT_SYMBOL_GPL(readl_redirect);

// redirect: static inline void writel_x(u32 value, volatile void __iomem *addr)
inline void writel_redirect( u32 value, void * addr, const unsigned char rwltype) {
  struct tegra_readl_writel rwlmsg;

  rwlmsg.signal = GPIO_WRITEL;
  rwlmsg.length = 0; // will be updated later
  rwlmsg.rwltype = rwltype;
  rwlmsg.address = addr;
  rwlmsg.value = value;

  guest_chardev_transfer(&rwlmsg, sizeof(rwlmsg), NULL, 0);
}
EXPORT_SYMBOL_GPL(writel_redirect);

int gpiochip_generic_request_redirect(struct gpio_chip *chip, unsigned offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_REQ;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

void gpiochip_generic_free_redirect(struct gpio_chip *chip, unsigned offset) {
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_FREE;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL, 0);
}

int tegra186_gpio_get_direction_redirect(struct gpio_chip *chip,
				       unsigned int offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_GET_DIR;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

int tegra186_gpio_get_redirect(struct gpio_chip *chip, unsigned int offset) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_GET;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

void tegra186_gpio_set_redirect(struct gpio_chip *chip, unsigned int offset,
			      int level) {
  struct tegra_gpio_pt msg;

  msg.signal = GPIO_SET;
  msg.chipnum = chip->gpiodev->id;
  msg.level = level;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL, 0);
}

void tegra186_gpio_set_by_name_redirect(const char *name, unsigned int offset,
			      int level) {
  struct tegra_gpio_pt msg;
  struct gpio_chip *chip = find_chip_by_name(name);
  msg.signal = GPIO_SET_BY_NAME;
  msg.chipnum = chip->gpiodev->id;
  msg.level = level;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), NULL, 0);
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
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

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

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

static bool is_set_up = false;

/**
 * Initializes module at installation
 */
int tegra_gpio_guest_init(void)
{
  // Note: gpio is not referenced, the init is agnostic to which chip triggered this function
  // In an earlier versio of the code we stored the gpio struct pointer in a static table
  // int this_chip_id = gpio.gpio->gpiodev.id;
  // char this_de:avice[12];
  if(is_set_up) { 
    deb_error("Attempting to set up guest driver twice\n");
    return -EPERM;
  }

	deb_info("installing module.");

	if(!gpio_vpa){
		pr_err("Failed, gpio_vpa not defined");
    return -1;
	}
	deb_info("gpio_vpa: 0x%llx", gpio_vpa);

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

	deb_info("mem_iova: 0x%llx\n", (long long unsigned int)mem_iova);

  is_set_up = true;
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

  is_set_up = false;
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
static ssize_t read(struct file *filp, char *buf, size_t len, loff_t *offset) {
	int remaining_length = return_size - *offset;

	deb_info("guest: read gpio chardev\n");
	deb_verbose("guest: read op: len = %ld, offset = %lld, *return_buffer = 0x%016llX\n", len, *offset, *(uint64_t *)return_buffer);
	hexDump (DEVICE_NAME, "Chardev (guest read) dump buffer", return_buffer, len);
	//deb_verbose("guest: read op: len = %ld, offset = %lld, *return_value = 0x%016llX\n", len, *offset, *return_value);
	// hexDump (DEVICE_NAME, "Chardev (guest read) dump buffer", (char *)return_value, return_size);

	if ( remaining_length < 0 ) {
		deb_info("guest: unrecoverable length *error*, remaining_length = %d\n", remaining_length);
		return -EINVAL;
	}

	if ( len > remaining_length ) {
		deb_info("guest: recoverable length *error*, len = %ld, remaining_length = %d, return_size = %d\n", len, remaining_length, return_size);
		len = remaining_length - *offset;
	}

	if (copy_to_user(buf + *offset, (char *)return_buffer + *offset, len)) {
		deb_info("guest: failed to copy to user\n");
		return -EFAULT;
	}

	*offset += len;

	// Check if all data was copied
	if (remaining_length > len) {
		deb_info("guest: not all bytes were copied\n");
		// If not, set the error status and return the number of bytes actually copied
		// return -EINVAL;
	}
	else if (remaining_length == len) {
		deb_verbose("reset return_size\n");
		return_size = 0;
		memset(return_buffer, 0, RET_SIZE);
	}

	// Indicate success by returning the number of bytes read
	return len;
}

/*
 * Writes to the device
 */

// TODO this is shared code with host proxy include function from there
// look at line 335 of this file
// extern static ssize_t chardev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	int ret;
	// unsigned long int ret_l;
	struct tegra_gpio_pt *kbuf = NULL;
	tegra_gpio_pt_extended *kbuf_ext = NULL;
	struct tegra_getbase_pt *kbuf_getbase = NULL;
  // unsigned char *mask;
  void __iomem *ret_ptr = NULL;

  /*
	static struct file *file;
	static struct inode *inode = NULL;
  */
	struct gpio_chip *chip;
  #ifdef GPIO_DEBUG_VERBOSE
	  // struct gpio_chip *chip_alt;
  #endif

	char *buffer_pos = (char *)buffer;

	deb_info("## writeing %zu bytes to chardev ##", len);

  // We allow tegra_gpio_pt alone or with tegra_gpio_pt_extended
	if( len != sizeof(struct tegra_gpio_pt) && len != sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended) )  {
		pr_err("Illegal chardev data length. Expected %ld or %ld, got %ld", sizeof(struct tegra_gpio_pt), sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended), len);
    hexDump (DEVICE_NAME, "Chardev (guest) input error", buffer, len);
		return -ENOEXEC;
    return 0;
	}
	if( len != sizeof(struct tegra_gpio_pt) && 
			len != sizeof(struct tegra_getbase_pt) && 
			len != sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended) &&
      len != sizeof(struct tegra_readl_writel) )  {
		pr_err("Illegal chardev data length. Expected %ld, %ld, %ld or %ld, but got %ld\n", 
				sizeof(struct tegra_gpio_pt), 
				sizeof(struct tegra_getbase_pt),
				sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended),
				sizeof(struct tegra_readl_writel), 
				len);

    hexDump (DEVICE_NAME, "Chardev (guest) input error", buffer, len);
		return -ENOEXEC;
	}

  if(!offset) {
    pr_err("offset pointer is null, ignoring offset\n");
  }
  else {
	  buffer_pos += (*offset);
  }

	kbuf = kmalloc(len, GFP_KERNEL);
	if ( !kbuf ) {
	  pr_err("kbuf memory allocation failed\n");
	  return -ENOMEM;
	}
	memset(kbuf, 0, len);

	// Copy header
  if (copy_from_user(kbuf, buffer_pos, len)) {
    pr_err("copy_from_user failed\n");
    kfree(kbuf);
    return -ENOMEM;
  }

  // we are not checking if tegra_gpio_pt_extended is used, we only check for memory allocation
  if( len == (sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended) ) ) {
    kbuf_ext = (tegra_gpio_pt_extended *)(kbuf + 1);
      deb_verbose("kbuf_ext is set up at kbuf_ext=%p", kbuf_ext);
  }

  // print copied user parameters
  hexDump (DEVICE_NAME, "Chardev input", kbuf, len);

  // make chardev type call to gpio
	deb_verbose("Passthrough from guest with signal: %c, Chip %d, Offset %d, Level %d", kbuf->signal, kbuf->chipnum, kbuf->offset, kbuf->level);

  chip = find_chip_by_id(kbuf->chipnum);
  /*
	#ifdef GPIO_DEBUG_VERBOSE
		chip_alt = find_chip_by_name(tegra_chiplabel[kbuf->chipnum]);
		if(chip != chip_alt) {
			deb_debug("conflicting chip pointers -- primary %p, alternative %p", chip, chip_alt);
			chip = chip_alt; // we assume find_chip_by_name is more reliable
		}
	#endif
	*/
	if(!chip) {
		pr_err("chip pointer's pvalue is unexpectedly NULL for chip %s\n", tegra_chiplabel[kbuf->chipnum]);
		kfree(kbuf);
		return len;
	}
	
  switch (kbuf->signal) {
    case GPIO_REQ:
      deb_verbose("GPIO_REQ, using GPIO chip %s, for device %d\n", chip->label, kbuf->chipnum);
      ret = chip->request(chip, kbuf->offset);
	    goto end;
    break;
    case GPIO_FREE:
      deb_verbose("GPIO_FREE\n");
      chip->free(chip, kbuf->offset);
      // chip_alt = NULL;
      goto end;
    break;
    case GPIO_GET_DIR:
      deb_verbose("GPIO_GET_DIR\n");
      ret = chip->get_direction(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_IN:
      deb_verbose("GPIO_SET_IN\n");
      ret = chip->direction_input(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_OUT:
      deb_verbose("GPIO_SET_OUT\n");
      ret = chip->direction_output(chip, kbuf->offset, kbuf->level);
      goto retval;
    break;
    case GPIO_GET:
      deb_verbose("GPIO_GET\n");
      ret = chip->get(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET:
      deb_verbose("GPIO_SET, set %d at offset 0x%x in gpiochip %s\n", kbuf->level, kbuf->offset, chip->label);
      chip->set(chip, kbuf->offset, kbuf->level);
	    goto end;
    break;
    case GPIO_CONFIG:
      deb_verbose("GPIO_CONFIG\n");
      chip->set_config(chip, kbuf->offset, kbuf_ext->config); // arg mapped to unsigned long config
	    goto end;
    break;
    case GPIO_TIMESTAMP_CTRL:
      deb_verbose("GPIO_TIMESTAMP_CTRL\n");
      ret = chip->timestamp_control(chip, kbuf->offset, kbuf->level);	// mapping level onto enable
      goto retval;
    break;
    case GPIO_TIMESTAMP_READ:
      deb_verbose("GPIO_TIMESTAMP_READ\n");
      ret = chip->timestamp_read(chip, kbuf->offset, (u64 *)buffer_pos);	// timestamp is u64, return value as pointer
      if(ret) {
        pr_err("GPIO_TIMESTAMP_READ error\n");
        goto end;
      }
      // timestamp_read returns value directly to buffer_pos
      goto end;
    break;
    case GPIO_SUSPEND_CONF:
      deb_verbose("GPIO_SUSPEND_CONF\n");
      if(!kbuf_ext) {
        pr_err("Parameter error in GPIO_SUSPEND_CONF\n");
        return -EINVAL;
      }
      ret = chip->suspend_configure(chip, kbuf->offset, kbuf_ext->dflags);
      goto retval;
    break;
    case GPIO_ADD_PINRANGES:
      deb_verbose("GPIO_ADD_PINRANGES\n");
      ret = chip->add_pin_ranges(chip);
      goto retval;
    break;
    case TEGRA_186_GETBASE:
      deb_verbose("TEGRA_186_GETBASE\n");
      kbuf_getbase = (void *)kbuf;
      ret_ptr = tegra186_gpio_get_base_redirect(kbuf_getbase->chipnum, kbuf_getbase->pin);
      goto retptr; // 64 bit?
    break;
  }

  /* ioctl signals use flieops because it relises on the standard gpio chardevs
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

  /* ioctl is exluded from this version
  if(kbuf_ext) {
    switch (kbuf->signal) {
  */
      /* commands to ioctl below (the std gpio chardev)
      * not fully implemented
        file = filp_open(tegra_chiplabel[kbuf->chipnum], O_RDWR, 0);
          if (IS_ERR(file)) {
            pr_err("GPIO, failed to open chardev for chip %s: %ld", tegra_chiplabel[kbuf->chipnum], PTR_ERR(file));
            kfree(kbuf);
            return -ENOENT;
        file = filp_open(tegra_chiplabel[kbuf->chipnum], O_RDWR, 0);
          if (IS_ERR(file)) {
            pr_err("GPIO, failed to open chardev for chip %s: %ld", tegra_chiplabel[kbuf->chipnum], PTR_ERR(file));
            kfree(kbuf);
            return -ENOENT;
          }
        // note: inode and file are static variables
        inode = file->f_path.dentry->d_inode;
        // defined as: static int gpio_chrdev_open(struct inode *inode, struct file *file)
        ret = file->f_op->open(inode, file);
        goto retval;
      break;
      case GPIO_CHARDEV_IOCTL:	// .unlocked_ioctl = gpio_ioctl
        // user space triggers gpio_ioctl -- it is .unlocked_ioctl on the chardev
        if( !file ) {
          pr_err("GPIO, chardev file was expected to be open\n");
          kfree(kbuf);
          return -ENOENT;
        }
        // defined as: static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
        ret_l = file->f_op->unlocked_ioctl(file, kbuf->cmd, kbuf_ext->arg);	// arg is pointer data which should have been copied from userspace
        goto retlong;
      break;
      case GPIO_CHARDEV_RELEASE: // .release = gpio_chrdev_release
        if( !file ) {
          pr_err("GPIO, chardev file was expected to be open\n");
          kfree(kbuf);
          return -ENOENT;
        }
        // defined as: static int gpio_chrdev_release(struct inode *inode, struct file *file)
        ret = file->f_op->release(inode, file);
        goto retval;
      break;
      case GPIO_CHARDEV_POLL: // .poll = lineinfo_watch_poll
        if( !file ) {
          pr_err("GPIO, chardev file was expected to be open\n");
          kfree(kbuf);
          return -ENOENT;
        }
        // defined as: static __poll_t lineinfo_watch_poll(struct file *file, struct poll_table_struct *pollt)
        ret = file->f_op->poll(file, kbuf_ext->poll);	// TODO arg is pointer data which should have been copied
        goto retval;	// __poll_t is of size unsigned int
      break;
      case GPIO_CHARDEV_READ: // .read = lineinfo_watch_read
        if( !file ) {
          pr_err("GPIO, chardev file was expected to be open\n");
          kfree(kbuf);
          return -ENOENT;
        }
        // defined as: static ssize_t lineinfo_watch_read(struct file *file, char __user *buf, size_t count, loff_t *off)
        ret = file->f_op->read(file, buffer_pos, kbuf_ext->count, NULL);		//
        if (ret) {
          pr_err("Reading lineinfo returned zero\n");
          kfree(kbuf);
          return -EFAULT;
        }
      return -ENXIO;
      case GPIO_CHARDEV_OWNER: // .owner = THIS_MODULE
        if (copy_to_user(buffer_pos, file->f_op->owner->name, strlen(file->f_op->owner->name)+1)) {
          pr_err("GPIO, copying user return value failed\n");
          kfree(kbuf);
          return -EFAULT;
        }
      break;
      default:
        pr_err("GPIO, Illegal proxy signal type\n");
        kfree(kbuf);
        return -EPERM;
      break;
    };
  };
  */

	goto end;

	retptr:
	return_size = sizeof(ret_ptr);
	memcpy(&return_buffer, &ret_ptr, return_size);
		deb_verbose("retval pointer (host): 0x%p, 0x%016llX", ret_ptr, (uint64_t)ret_ptr);
	/*
	return_size = sizeof(ret_ptr);
	memset(return_buffer, 0, MEM_SIZE);
	memcpy(return_buffer, &ret_ptr, return_size);
	*/
	if ( copy_to_user((uint64_t *)kbuf + RETURN_OFF, &ret_ptr, sizeof(ret_ptr)) ) {
		pr_err("GPIO, copying pointer pvalue, user return value failed\n");
		kfree(kbuf);
		return -EFAULT;
	};
	// deb_verbose("retval copied to buffer (host): 0x%016llX", *(uint64_t	*)return_buffer);

	goto end;

	retval:

	return_size = sizeof(ret);
	memcpy(return_buffer, &ret, return_size);
	deb_verbose("retval int (host): 0x%X", ret);
	/*
	return_size = sizeof(ret);
	memset(return_buffer, 0, MEM_SIZE);
	memcpy(return_buffer, &ret, return_size);
	*/

	if ( copy_to_user((void *)kbuf + RETURN_OFF, &ret, sizeof(ret)) ) {
		pr_err("GPIO, copying unsigned int user return value failed\n");
		kfree(kbuf);
		return -EFAULT;
	};
  // deb_verbose("retval copied to buffer (host): 0x%016llX", *(uint64_t	*)return_buffer);

	end:
	kfree(kbuf);
	return len;
}
