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

#include <gpio-proxy.h>
#include "../gpio-host-proxy/gpio-host-proxy.h"

#define DEVICE_NAME "gpio-guest"   // Device name.
#define CLASS_NAME  "chardrv"	  // < The device class -- this is a character device driver

MODULE_LICENSE("GPL");						///< The license type -- this affects available functionality
MODULE_AUTHOR("Kim Sandström");					///< The author -- visible when you use modinfo
MODULE_DESCRIPTION("NVidia GPIO Guest Proxy Kernel Module");	///< The description -- see modinfo
MODULE_VERSION("0.0");						///< A version number to inform users

#define GPIO_DEBUG
#define GPIO_DEBUG_VERBOSE       // also activates deb_verbose commands

#ifdef GPIO_DEBUG
  /*
  #define deb_info(fmt, ...)     printk(KERN_INFO "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__, kbasename(__FILE__), ##__VA_ARGS__)
  #define deb_debug(fmt, ...)    printk(KERN_DEBUG "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__, kbasename(__FILE__), ##__VA_ARGS__)
  #define deb_error(fmt, ...)    printk(KERN_ERR "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__ , kbasename(__FILE__), ##__VA_ARGS__)
  */
  #define deb_info(fmt, ...)     printk(KERN_INFO "GPIO func \'%s\' -- " fmt, __func__, ##__VA_ARGS__)
  #define deb_debug(fmt, ...)    printk(KERN_DEBUG "GPIO func \'%s\' -- " fmt, __func__, ##__VA_ARGS__)
  #define deb_error(fmt, ...)    printk(KERN_ERR "GPIO func \'%s\' -- " fmt, __func__ , ##__VA_ARGS__)
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
static uint64_t return_value = 0;
static char *return_buffer = (char *)&return_value;
static unsigned int return_size = 0;

_Static_assert(sizeof(uint64_t) == RET_SIZE, "return size assertion for RET_SIZE failed");
_Static_assert(sizeof(u32) == sizeof(int), "return size assertion for int failed");
_Static_assert(sizeof(volatile void __iomem *) == sizeof(uint64_t), "return size assertion for iomem pointer failed");

/* functions redirected to guest-proxy from gpio-tegra186.c
 * from setup of gpio_chip in tegra186_gpio_probe
 */

/* guest_chardev_transfer
 * a helper function to transfer between guest and host using /dev/gpio-host
 * mgs: is a pointer to data -- in practice tegra_gpio_pt and tegra_gpio_pt_ext structs
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

  deb_verbose("PT transfer signal is: %c", *((char *)msg + 1));
  hexDump(DEVICE_NAME, "GPIO: PT transfer (message from guest)", msg, msg_len);

	// Execute the request by copying to io memory
	memcpy_toio(mem_iova, msg, msg_len);
  if(generic_return) {
    // Read response from io_buffer
    memcpy_fromio(generic_return, (void *)((char *)mem_iova + RETURN_OFF), ret_len); // 32 bits except for tegra186_gpio_get_base_redirect 64 bits
    hexDump(DEVICE_NAME, "GPIO: PT transfer (retrieved retval to guest)", generic_return, ret_len );  }
}

void *gpio_get_host_values(unsigned char id, unsigned int h_val) {
  void *ret_ptr = (void *)0x01234567ABADFACE; // simulating a pointer with a uint64_t
  struct tegra_gpio_pt msg;

  deb_verbose("\n");
  msg.signal = GPIO_GET_HOST_VALUES;
  msg.chipnum = id;
  msg.level = 0;
  msg.offset = h_val;
 
  guest_chardev_transfer(&msg, sizeof(msg), &ret_ptr, sizeof(ret_ptr));
  return ret_ptr;
}

void *tegra186_gpio_get_base_redirect(unsigned char id, unsigned int pin) {
  void *ret_ptr = (void *)0x01234567ABADFACE; // simulating a pointer with a uint64_t
  struct tegra_gpio_pt_ext msg;

  deb_verbose("\n");
  msg.base.signal = TEGRA_186_GETBASE;
  msg.base.chipnum = id;
  msg.ext.pin = pin;
 
  guest_chardev_transfer(&msg, sizeof(msg), &ret_ptr, sizeof(ret_ptr));
  return ret_ptr;
}

// redirect static inline u32 readl_x(const volatile void __iomem *addr)
inline u32 readl_redirect( void * addr, const unsigned char rwltype) {
  u32 ret = 0;
  struct tegra_readl_writel rwlmsg;

  #ifdef GPIO_DEBUG_VERBOSE
  deb_verbose("\n");
  dump_stack();
  #endif

  rwlmsg.signal = GPIO_READL;
  rwlmsg.length = 0; // will be updated later
  rwlmsg.rwltype = rwltype;
  rwlmsg.address = addr;
  rwlmsg.value = 0;  // value field is not used for readl

  guest_chardev_transfer(&rwlmsg, sizeof(rwlmsg), &ret, sizeof(ret));
  return ret;
}
EXPORT_SYMBOL_GPL(readl_redirect);

// redirect: static inline void writel_x(u32 value, volatile void __iomem *addr)
inline void writel_redirect( u32 value, void * addr, const unsigned char rwltype) {
  struct tegra_readl_writel rwlmsg;

  #ifdef GPIO_DEBUG_VERBOSE
  deb_verbose("\n");
  dump_stack();
  #endif

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

  deb_verbose("\n");
  msg.signal = GPIO_REQ;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

void gpiochip_generic_free_redirect(struct gpio_chip *chip, unsigned offset) {
  struct tegra_gpio_pt msg;

  deb_verbose("\n");
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

  deb_verbose("\n");
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

  deb_verbose("\n");
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

  deb_verbose("\n");
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

  deb_verbose("\n");
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

  deb_verbose("\n");
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

  deb_verbose("\n");
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
	struct tegra_gpio_pt_ext msg;

  deb_verbose("\n");
  msg.base.signal = GPIO_CONFIG;
  msg.base.chipnum = chip->gpiodev->id;
  msg.base.level = 0;
  msg.base.offset = offset;
  msg.ext.config = config;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

int tegra_gpio_timestamp_control_redirect(struct gpio_chip *chip, unsigned offset,
					int enable) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  deb_verbose("\n");
  msg.signal = GPIO_TIMESTAMP_CTRL;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

int tegra_gpio_timestamp_read_redirect(struct gpio_chip *chip, unsigned offset,
				     u64 *ts) {
	/* TODO include 'ret' in the return values (requires change in qemu)
	struct ret_ts {
		uint64_t timestamp;
		int ret = 0;
	}	*/

	int ret = 0;				
  struct tegra_gpio_pt msg;

  deb_verbose("\n");
  msg.signal = GPIO_TIMESTAMP_READ;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = offset;

  guest_chardev_transfer(&msg, sizeof(msg), ts, sizeof(*ts));
  // *ts = ret_ts.timestamp
  // return ret_ts.ret;
  if(*ts == -EOPNOTSUPP) ret = (int)*ts;		// possible hack to use *ts for error reporting

	// a hack to read extra return att offset -1
  // memcpy_fromio(&ret, (void *)((char *)mem_iova + RETURN_OFF - 1), sizeof(ret)); // 32 bits except for tegra186_gpio_get_base_redirect 64 bits

  return ret;
}

int tegra_gpio_suspend_configure_redirect(struct gpio_chip *chip, unsigned offset,
					enum gpiod_flags dflags) {
  int ret = 0;
  struct tegra_gpio_pt_ext msg;

  deb_verbose("\n");
  msg.base.signal = GPIO_SUSPEND_CONF;
  msg.base.chipnum = chip->gpiodev->id;
  msg.base.level = 0;
  msg.base.offset = offset;
  msg.ext.dflags = dflags;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

// TODO,  this probably does not work because action parameters should be in gpio_chip struct, but guest did not copy the over the passthrough interface
int tegra186_gpio_add_pin_ranges_redirect(struct gpio_chip *chip) {
  int ret = 0;
  struct tegra_gpio_pt msg;

  deb_verbose("\n");
  msg.signal = GPIO_ADD_PINRANGES;
  msg.chipnum = chip->gpiodev->id;
  msg.level = 0;
  msg.offset = 0;

  guest_chardev_transfer(&msg, sizeof(msg), &ret, sizeof(ret));
  return ret;
}

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
		deb_error("Failed, gpio_vpa not defined");
    return -1;
	}
	deb_debug("gpio_vpa: 0x%llx", gpio_vpa);

	// Allocate a major number for the device.
	major_number = register_chrdev(0, DEVICE_NAME, &fops);
	if (major_number < 0)
	{
		deb_error("could not register number.");
		return major_number;
	}
	deb_debug("registered correctly with major number %d", major_number);

	// Register the device class
	gpio_guest_proxy_class = class_create(THIS_MODULE, CLASS_NAME);
	if (IS_ERR(gpio_guest_proxy_class))
	{ // Check for error and clean up if there is
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to register device class\n");
		return PTR_ERR(gpio_guest_proxy_class); // Correct way to return an error on a pointer
	}
	deb_debug("device class registered correctly\n");

	// Register the device driver
	gpio_guest_proxy_device = device_create(gpio_guest_proxy_class, NULL, MKDEV(major_number, 0), NULL, DEVICE_NAME);
	if (IS_ERR(gpio_guest_proxy_device))
	{								 // Clean up if there is an error
		class_destroy(gpio_guest_proxy_class);
		unregister_chrdev(major_number, DEVICE_NAME);
		deb_error("Failed to create the device\n");
		return PTR_ERR(gpio_guest_proxy_device);
	}
	deb_debug("device class created correctly\n"); // Made it! device was initialized

	// map iomem
	mem_iova = ioremap(gpio_vpa, MEM_SIZE);

	if (!mem_iova) {
        deb_error("ioremap failed\n");
        return -ENOMEM;
  }

	deb_debug("mem_iova: 0x%llx\n", (long long unsigned int)mem_iova);

  is_set_up = true;
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_gpio_guest_init);

// unpreserve_all_tegrachips also does unhooking
extern void unpreserve_all_tegrachips(void);
/*
 * Removes module, sends appropriate message to kernel
 */
int tegra_gpio_guest_cleanup(void)
{
	deb_info("removing module.\n");

	// unmap iomem
	iounmap((void __iomem*)gpio_vpa);

  // clean up shared memory with stock driver and unhook all functions
  unpreserve_all_tegrachips();

	device_destroy(gpio_guest_proxy_class, MKDEV(major_number, 0)); // remove the device
	class_unregister(gpio_guest_proxy_class);           // unregister the device class
	class_destroy(gpio_guest_proxy_class);              // remove the device class
	unregister_chrdev(major_number, DEVICE_NAME);       // unregister the major number
	deb_info("Goodbye from GPIO passthrough!\n");
	unregister_chrdev(major_number, DEVICE_NAME);

	is_set_up = false;
	return 0;
}
EXPORT_SYMBOL_GPL(tegra_gpio_guest_cleanup);

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

static DEFINE_MUTEX(chardev_mutex);

/*
 * Reads from device, displays in userspace, and deletes the read data
 */
static ssize_t read(struct file *filp, char *buf, size_t len, loff_t *offset) {
	int remaining_length = return_size - *offset;

	deb_info("guest: read gpio chardev\n");
	deb_verbose("guest: read op: remaining_length = %d, len = %ld, offset = %lld, return_value = 0x%016llX\n", remaining_length, len, *offset, return_value);
	// hexDump (DEVICE_NAME, "Chardev (guest read) dump buffer", return_buffer, len);
	//deb_verbose("guest: read op: len = %ld, offset = %lld, *return_value = 0x%016llX\n", len, *offset, return_value);
	// hexDump (DEVICE_NAME, "Chardev (guest read) dump buffer", (char *)return_value, return_size);

  if ( *offset == 0 )
    mutex_lock(&chardev_mutex); // activate mutex on entry

  if ( remaining_length < 0 ) {
		deb_error("guest: unrecoverable length *error*, remaining_length = %d\n", remaining_length);
		return -EINVAL;
	}

	if ( len > remaining_length ) {
		deb_error("guest: recoverable length *error*, len = %ld, remaining_length = %d, return_size = %d\n", len, remaining_length, return_size);
		len = remaining_length;
	}

	if (copy_to_user(buf, (char *)return_buffer + *offset, len)) {
		deb_error("guest: failed to copy to user\n");
		return -EFAULT;
	}
	*offset += len;
	remaining_length -= len;

	// Check if all data was copied
	if (remaining_length > len) {
		deb_debug("guest: not all bytes were copied\n");
	}
	else if (remaining_length == 0) {
		// read is complete
		deb_verbose("reset return_size\n");
		return_size = 0;
		*offset = 0;
		memset(return_buffer, 0, RET_SIZE);
 		mutex_unlock(&chardev_mutex);	// allow next message
	}

	// Indicate success by returning the number of bytes read
	return len;
}

/*
 * Writes to the device
 */

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	struct tegra_gpio_pt *kbuf = NULL;
	struct tegra_gpio_pt_ext *kbuf_ext = NULL;
	char *buffer_pos = (char *)buffer;
	void *ret_ptr = NULL;
	uint64_t ret_64;
	int ret_int;  // 32 bits
	int ret;

	_Static_assert( sizeof(ret_ptr) == sizeof(return_value),
               "ret_ptr size does not match return_value" );

  /*
	static struct file *file;
	static struct inode *inode = NULL;
  */
	struct gpio_chip *chip;
  #ifdef GPIO_DEBUG_VERBOSE
	  // struct gpio_chip *chip_alt;
  #endif

	deb_info("## writeing %zu bytes to chardev ##", len);

	if( len != sizeof(struct tegra_gpio_pt) &&
			len != sizeof(struct tegra_gpio_pt_ext) &&
			len != sizeof(struct tegra_readl_writel)  )  {
		deb_error("Illegal chardev data length. Expected %ld, %ld or %ld, but got %ld\n",
				sizeof(struct tegra_gpio_pt),
				sizeof(struct tegra_gpio_pt_ext),
				sizeof(struct tegra_readl_writel),
				len);
		hexDump (DEVICE_NAME, "Chardev (guest) input error", buffer, len);
		return -ENOEXEC; // kbuf not allocated yet
	}

	if(!offset) {
		deb_error("offset pointer is NULL, ignoring offset\n");
	}
	else {
		buffer_pos += (*offset);
	}

	kbuf = kmalloc(len, GFP_KERNEL);
	if ( !kbuf ) {
		deb_error("kbuf memory allocation failed\n");
		len = -ENOMEM;
		goto exit;
	}
	memset(kbuf, 0, len);
	memset(return_buffer, 0, RET_SIZE);

	// Copy header
	if (copy_from_user(kbuf, buffer_pos, len)) {
		deb_error("copy_from_user failed\n");
		len = -ENOMEM;
		goto exit;
	}
	buffer_pos += RETURN_OFF;

	// we are not checking if tegra_gpio_pt_ext is used, we only check for memory allocation
	if( len == sizeof(struct tegra_gpio_pt_ext) ) {
		kbuf_ext = (struct tegra_gpio_pt_ext *)(kbuf);
		deb_verbose("kbuf_ext is set up at kbuf_ext=%p", kbuf_ext);
	}

	// print copied user parameters
	hexDump (DEVICE_NAME, "Chardev input " DEVICE_NAME, kbuf, len);

	// make chardev type call to gpio
	deb_verbose("Passthrough from guest with signal: %c, Chip %d, Offset %d, Level %d", kbuf->signal, kbuf->chipnum, kbuf->offset, kbuf->level);

  chip = find_chip_by_id(kbuf->chipnum);
  /*
	#ifdef GPIO_DEBUG_VERBOSE
		chip_alt = find_chip_by_name(tegra_chiplabel[kbuf->chipnum]);
		if(chip != chip_alt) {
			deb_debug("conflicting chip pointers -- primary 0x%p, alternative 0x%p", chip, chip_alt);
			chip = chip_alt; // we assume find_chip_by_name is more reliable
		}
	#endif
	*/
	if(!chip) {
		deb_error("chip pointer's pvalue is unexpectedly NULL for chip\n");
		len = -ENODEV;
		goto exit;
	}

  switch (kbuf->signal) {
    case GPIO_REQ:
      deb_verbose("GPIO_REQ, using GPIO chip %s, for device %d\n", chip->label, kbuf->chipnum);
      ret_int = chip->request(chip, kbuf->offset);
	    goto retval;
    break;
    case GPIO_FREE:
      deb_verbose("GPIO_FREE\n");
      chip->free(chip, kbuf->offset);
      // chip_alt = NULL;
      goto noretval;
    break;
    case GPIO_GET_DIR:
      deb_verbose("GPIO_GET_DIR\n");
      ret_int = chip->get_direction(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_IN:
      deb_verbose("GPIO_SET_IN\n");
      ret_int = chip->direction_input(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_OUT:
      deb_verbose("GPIO_SET_OUT\n");
      ret_int = chip->direction_output(chip, kbuf->offset, kbuf->level);
      goto retval;
    break;
    case GPIO_GET:
      deb_verbose("GPIO_GET\n");
      ret_int = chip->get(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET:
      deb_verbose("GPIO_SET, set %d at offset 0x%x in gpiochip %s\n", kbuf->level, kbuf->offset, chip->label);
      chip->set(chip, kbuf->offset, kbuf->level);
	    goto noretval;
    break;
    case GPIO_CONFIG:
      deb_verbose("GPIO_CONFIG\n");
      ret_int = chip->set_config(chip, kbuf_ext->base.offset, kbuf_ext->ext.config); // arg mapped to unsigned long config
	    goto retval;
    break;
    case GPIO_TIMESTAMP_CTRL:
      deb_verbose("GPIO_TIMESTAMP_CTRL\n");
      ret_int = chip->timestamp_control(chip, kbuf->offset, kbuf->level);	// mapping level onto enable
      goto retval;
    break;
    case GPIO_TIMESTAMP_READ:
      deb_verbose("GPIO_TIMESTAMP_READ\n");
      ret = chip->timestamp_read(chip, kbuf->offset, &ret_64);	// timestamp is u64, return value as pointer
      deb_verbose("timestamp_read: 0x%016llX\n", ret_64);
      if(ret) {
        deb_error("GPIO_TIMESTAMP_READ error\n");
        ret_64 = ret;
      }
      deb_verbose("timestamp_read: 0x%016llX\n", ret_64);
      goto ret64;
    break;
    case GPIO_SUSPEND_CONF:
      deb_verbose("GPIO_SUSPEND_CONF\n");
      if(!kbuf_ext) {
        deb_error("Parameter error in GPIO_SUSPEND_CONF\n");
        len = -EINVAL;
        goto exit;
      }
      ret_int = chip->suspend_configure(chip, kbuf_ext->base.offset, kbuf_ext->ext.dflags);
      goto retval;
    break;
    case GPIO_ADD_PINRANGES:
      deb_verbose("GPIO_ADD_PINRANGES\n");
      ret_int = chip->add_pin_ranges(chip);
      goto retval;
    break;
    case TEGRA_186_GETBASE:
      deb_verbose("TEGRA_186_GETBASE\n");
      ret_ptr = tegra186_gpio_get_base_redirect(kbuf_ext->base.chipnum, kbuf_ext->ext.pin);
      deb_verbose("tegra186_gpio_get_base_execute, chip: %d, pointer: 0x%p / 0x%016llX\n", kbuf_ext->base.chipnum, ret_ptr, (uint64_t)ret_ptr);
      goto retptr; // 64 bit
    break;
    default:
      deb_error("GPIO, Unknown passthough signal\n");
      len = -EPERM;
      goto exit;
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
      * linehandle_create  -- when userspace requests output (called by gpio_ioctl) -- bypasses the chardev
      * linehandle_ioctl   -- linehandle_ioctl when userspace does actual io (toggles pin)
      *    cmd:
      *    GPIOHANDLE_GET_LINE_VALUES_IOCTL,
      *    GPIOHANDLE_SET_LINE_VALUES_IOCTL,
      *    GPIOHANDLE_SET_CONFIG_IOCTL
      *    arg: user input or output
      */
/*
      // We could want to use the stock gpio chardev (/dev/gpiochip0 and /dev/gpiochip1) /bc userspace functions use it
      // this code is not yet complete and it mey be better to use the stock devices directly.
      case GPIO_CHARDEV_OPEN:	// .open = gpio_chrdev_open
        file = filp_open(tegra_chiplabel[kbuf->chipnum], O_RDWR, 0);
          if (IS_ERR(file)) {
            deb_error("GPIO, failed to open chardev for chip %s: %ld", tegra_chiplabel[kbuf->chipnum], PTR_ERR(file));
            len = -ENOENT;
            goto exit;
          }
        // note: inode and file are static variables
        inode = file->f_path.dentry->d_inode;
        // defined as: static int gpio_chrdev_open(struct inode *inode, struct file *file)
        ret_int = file->f_op->open(inode, file);
        goto retval;
      break;
      case GPIO_CHARDEV_IOCTL:	// .unlocked_ioctl = gpio_ioctl
        // user space triggers gpio_ioctl -- it is .unlocked_ioctl on the chardev
        if( !file ) {
          deb_error("GPIO, chardev file was expected to be open\n");
          len = -ENOENT;
          goto exit;
        }
        // defined as: static long gpio_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
        ret_l = file->f_op->unlocked_ioctl(file, kbuf->cmd, kbuf_ext->ext.arg);	// arg is pointer data which should have been copied from userspace
        goto retlong;
      break;
      case GPIO_CHARDEV_RELEASE: // .release = gpio_chrdev_release
        if( !file ) {
          deb_error("GPIO, chardev file was expected to be open\n");
          len = -ENOENT;
          goto exit;
        }
        // defined as: static int gpio_chrdev_release(struct inode *inode, struct file *file)
        ret_int = file->f_op->release(inode, file);
        goto retval;
      break;
      case GPIO_CHARDEV_POLL: // .poll = lineinfo_watch_poll
        if( !file ) {
          deb_error("GPIO, chardev file was expected to be open\n");
          len = -ENOENT;
          goto exit;
        }
        // defined as: static __poll_t lineinfo_watch_poll(struct file *file, struct poll_table_struct *pollt)
        ret_int = file->f_op->poll(file, kbuf_ext->ext.poll);	// TODO arg is pointer data which should have been copied
        goto retval;	// __poll_t is of size unsigned int
      break;
      case GPIO_CHARDEV_READ: // .read = lineinfo_watch_read
        if( !file ) {
          deb_error("GPIO, chardev file was expected to be open\n");
          len = -ENOENT;
          goto exit;
        }
        // defined as: static ssize_t lineinfo_watch_read(struct file *file, char __user *buf, size_t count, loff_t *off)
        ret = file->f_op->read(file, buffer_pos, kbuf_ext->ext.count, NULL);		//
        if (ret) {
          deb_error("Reading lineinfo returned zero\n");
          len = -EFAULT;
          goto exit;
        }
      break;
      case GPIO_CHARDEV_OWNER: // .owner = THIS_MODULE
        if (copy_to_user(buffer_pos, file->f_op->owner->name, strlen(file->f_op->owner->name)+1)) {
          deb_error("GPIO, copying user return value failed\n");
          len = -EFAULT;
          goto exit;
        }
      break;
      default:
        deb_error("GPIO, Unknown passthough signal\n");
        len = -EPERM;
        goto exit;
      break;
    };
  };
  */

	goto noretval;

	ret64:
	mutex_lock(&chardev_mutex);	// wait for read
	return_size = sizeof(ret_64);
	return_value = ret_64;
	// memcpy(return_buffer, &ret_64, return_size);
	deb_verbose("ret_64 (guest): 0x%p, 0x%016llX", ret_ptr, return_value);
	goto ret_end;
	// goto ret_extra_hack;

	retptr:
	mutex_lock(&chardev_mutex);	// wait for read
	return_size = sizeof(ret_ptr);
	return_value = (uint64_t)ret_ptr;
	// memcpy(return_buffer, &ret_ptr, return_size);
	deb_verbose("retval pointer (guest): 0x%p, 0x%016llX", ret_ptr, return_value);
	goto ret_end;

	retval:
	mutex_lock(&chardev_mutex);	// wait for read
	return_size = sizeof(ret_int);
	memcpy(return_buffer, &ret_int, return_size);
	deb_verbose("retval int (guest): 0x%X", ret_int);
	goto ret_end;

	noretval:
	return_size = 0;
	goto exit;

	/*
	ret_extra_hack:
	// uses memory space used only by long messages, can be used only with short messages (used for GPIO_TIMESTAMP_READ only)
	// we assume ret_end is used normally
	if ( copy_to_user( (char *)buffer_pos - sizeof(uint64_t), &ret, sizeof(ret)) ) {
		deb_error("GPIO, copying user return value in \"hack\" failed: 0x%08X\n", ret);
			len = -EFAULT;
	}
	// no need to update len because it is done in ret_end
	// continue to ret_end
	*/

	ret_end:
	if ( return_size && return_size <= sizeof(return_value) ) {
		if ( MEM_SIZE >= sizeof(return_value) + RETURN_OFF) {
			// if this chardev is closed in write(), we lose the return value in buffer_pos. That's why 'return_buffer' is a static global var.
			if ( (ret = copy_to_user( (char *)buffer_pos, return_buffer, sizeof(return_value))) ) {
				deb_error("GPIO, copying user return value failed: 0x%08X\n", ret);
					len = -EFAULT;
					goto unlock;
			}
			// let Qemu detect we wrote a return value
			len = RETURN_OFF + return_size;
			deb_verbose("return value size %d copied to buffer (guest): 0x%p / 0x%016llX", return_size, (void *)return_value, return_value);
			// hexDump(DEVICE_NAME, "Chardev (host write) dump buffer", return_buffer, MEM_SIZE);
		} else {
			len = -EINVAL; // Buffer too small
			goto unlock;
		}
	}

	unlock:
	mutex_unlock(&chardev_mutex);	// allow next write in case retval is received before chardev close and read() is never called 

	exit:
	kfree(kbuf);
	return len; // return length of read data
}
