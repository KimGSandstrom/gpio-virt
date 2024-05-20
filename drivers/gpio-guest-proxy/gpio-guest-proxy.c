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
  #define deb_info(fmt, ...)     printk(KERN_INFO "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__, __FILE__, ##__VA_ARGS__)
  #define deb_debug(fmt, ...)    printk(KERN_DEBUG "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__ , __FILE__, ##__VA_ARGS__)
  #define deb_error(fmt, ...)    printk(KERN_ERR "GPIO func \'%s\' in file \'%s\' -- " fmt, __func__ , __FILE__, ##__VA_ARGS__)
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

	// deb_debug("\n");
  #ifdef GPIO_DEBUG_VERBOSE
    hexDump(DEVICE_NAME, "msg", &msg, msg_len);
    deb_verbose("passthrough signal is: %c", *(char *)msg);
  #endif

	// Copy msg, to io_buffer
	io_buffer = kmalloc(msg_len, GFP_KERNEL);
	memset(io_buffer, 0, msg_len);
  memcpy(io_buffer, msg, msg_len);

	// Execute the request by copying the io_buffer
	memcpy_toio(mem_iova, io_buffer, msg_len);

	// Read response to io_buffer
	memcpy_fromio(io_buffer, mem_iova, sizeof(*generic_return));

  // check if we expect a return value
  if(generic_return) {
	// Copy reply to io_buffer
	memcpy(generic_return, io_buffer, sizeof(*generic_return));
	deb_verbose("return value %d is copied", *generic_return);
  }

  kfree(io_buffer);
}

// redirect static inline u32 readl(const volatile void __iomem *addr)
inline u32 readl_redirect( void * addr) {
  int ret = 0;
  struct tegra_readl_writel msg;

  msg.signal = GPIO_READL;
  msg.address = addr;
  msg.value = 0;  // value field is not used

  guest_chardev_transfer(&msg, sizeof(msg), &ret);
  return (u32)ret;
}
EXPORT_SYMBOL_GPL(readl_redirect);

// redirect: static inline void writel(u32 value, volatile void __iomem *addr)
inline void writel_redirect( u32 value, void * addr) {
  struct tegra_readl_writel msg;

  msg.signal = GPIO_WRITEL;
  msg.address = addr;
  msg.value = value;

  guest_chardev_transfer(&msg, sizeof(msg), NULL);
}
EXPORT_SYMBOL_GPL(writel_redirect);

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

static bool is_set_up = false;

/**
 * Initializes module at installation
 */
int tegra_gpio_guest_init(void)
{
  // Note: gpio is not referenced, the init is agnostic to which chip triggered this function
  // In an earlier versio of the code we stored the gpio struct pointer in a static table
  // int this_chip_id = gpio.gpio->gpiodev.id;
  // char this_device[12];
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
static ssize_t read(struct file *filep, char *buffer, size_t len, loff_t *offset)
{
	deb_info("read stub");
	return 0;
}

/*
 * Writes to the device
 */

// TODO this is shared code with host proxy include function from there
// look at line 335 of this file
// extern static ssize_t chardev_write(struct file *filep, const char *buffer, size_t len, loff_t *offset)

static ssize_t write(struct file *filep, const char *buffer, size_t len, loff_t *offset)
{
	unsigned int ret;
	unsigned long int ret_l;
	struct tegra_gpio_pt *kbuf = NULL;
	tegra_gpio_pt_extended *kbuf_ext = NULL;

	static struct file *file;
	static struct inode *inode = NULL;
	struct gpio_chip *chip;
  #ifdef GPIO_DEBUG
	  struct gpio_chip *chip_alt;
  #endif

	char *return_buffer = (char *)buffer;
	char *read_buffer = (char *)buffer;

	deb_info("writeing %zu bytes to chardev", len);

  /* removed because condition is covered by susequent checks on 'len'
	if (len > 65535) {
		pr_err("count %zu exceeds max # of bytes allowed, aborting write", len);
		return -EINVAL;
	} */

  // We allow tegra_gpio_pt alone or with tegra_gpio_pt_extended (verify later)
	if( len != sizeof(struct tegra_gpio_pt) && len != sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended) )  {
		pr_err("Illegal chardev data length. Expected %ld or %ld, got %ld", sizeof(struct tegra_gpio_pt), sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended), len);
		return -ENOEXEC;
	}

  if(!offset) {
    pr_err("offset pointer is null, ignoring offset\n");
  }
  else {
	  read_buffer += (*offset);
  }

	kbuf = kmalloc(len, GFP_KERNEL);
	if ( !kbuf ) {
	  pr_err("kbuf memory allocation failed\n");
	  return -ENOMEM;
	}
	memset(kbuf, 0, len);

	// Copy header
  if (copy_from_user(kbuf, read_buffer, sizeof(struct tegra_gpio_pt))) {
    pr_err("copy_from_user failed\n");
    kfree(kbuf);
    return -ENOMEM;
  }
  deb_verbose("kbuf is set up, kbuf=%p", kbuf);

  if( len == (sizeof(struct tegra_gpio_pt) + sizeof(tegra_gpio_pt_extended) ) ) {
    kbuf_ext = (tegra_gpio_pt_extended *)(kbuf + 1);
    deb_verbose("kbuf_ext is set up kbuf_ext=%p", kbuf_ext);
  }

	// print copied user parameters
  hexDump (DEVICE_NAME, "Chardev input", kbuf, len);


  // make gpio-host type call to gpio
	deb_verbose("enter switch with signal: %c, Chip %d, Offset %d, Level %d", kbuf->signal, kbuf->chipnum, kbuf->offset, kbuf->level);

  switch (kbuf->signal) {
    case GPIO_REQ:
      // if(kbuf->chipnum & 0xfe) {    // 0 and 1 are allowed values & mask allows fastcheck, marginal save
      if(kbuf->chipnum >= MAX_CHIP) {  // direct copmparison is more future flexible
        // te!, 
        pr_err("Illegal value for chip number\n");
        kfree(kbuf);
        return -ENODEV;
      }
      chip = find_chip_by_name(kbuf->chipnum);
      #ifdef GPIO_DEBUG_VERBOSE
        chip_alt = find_chip_by_id(tegra_chiplabel[kbuf->chipnum]);
        if(chip != chip_alt)
          deb_debug("conflicting chip pointers -- primary %p, alternative %p", chip, chip_alt);
      #endif
      if(!chip) {
        pr_err("In GPIO_REQ, chip pointer's pvalue is unexpectedly NULL for chip %s\n", tegra_chiplabel[kbuf->chipnum]);
        kfree(kbuf);
        return -ENODEV;
      }

      deb_verbose("GPIO_REQ, using GPIO chip %s, for device %d, pvalue = %p", chip->label, kbuf->chipnum, chip);
      ret = chip->request(chip, kbuf->offset);
	    goto end;
    break;
    case GPIO_FREE:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_FREE\n");
      chip->free(chip, kbuf->offset);
      // chip_alt = NULL;
      goto end;
    break;
    case GPIO_GET_DIR:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_GET_DIR\n");
      ret = chip->get_direction(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_IN:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_SET_IN\n");
      ret = chip->direction_input(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET_OUT:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_SET_OUT, chip pvalue 0 %p", chip);
      ret = chip->direction_output(chip, kbuf->offset, kbuf->level);
      goto retval;
    break;
    case GPIO_GET:
      deb_verbose("GPIO_GET\n");
      chip = find_chip_by_id(kbuf->chipnum);
      ret = chip->get(chip, kbuf->offset);
      goto retval;
    break;
    case GPIO_SET:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_SET, set %d at offset 0x%x in gpiochip %s", kbuf->level, kbuf->offset, chip->label);
      chip->set(chip, kbuf->offset, kbuf->level);
	    goto end;
    break;
    case GPIO_CONFIG:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_CONFIG\n");
      chip->set_config(chip, kbuf->offset, kbuf_ext->config); // arg mapped to unsigned long config
	    goto end;
    break;
    case GPIO_TIMESTAMP_CTRL:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_TIMESTAMP_CTRL\n");
      ret = chip->timestamp_control(chip, kbuf->offset, kbuf->level);	// mapping level onto enable
      goto retval;
    break;
    case GPIO_TIMESTAMP_READ:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_TIMESTAMP_READ\n");
      ret = chip->timestamp_read(chip, kbuf->offset, (u64 *)return_buffer);	// timestamp is u64, return value as pointer
      if(ret) {
        pr_err("GPIO_TIMESTAMP_READ error\n");
        goto end;
      }
      // timestamp_read returns value directly to return_buffer
      goto end;
    break;
    case GPIO_SUSPEND_CONF:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_SUSPEND_CONF\n");
      if(!kbuf_ext) {
        pr_err("Parameter error in GPIO_SUSPEND_CONF\n");
        return -EINVAL;
      }
      ret = chip->suspend_configure(chip, kbuf->offset, kbuf_ext->dflags);
      goto retval;
    break;
    case GPIO_ADD_PINRANGES:
      chip = find_chip_by_id(kbuf->chipnum);
      deb_verbose("GPIO_ADD_PINRANGES\n");
      ret = chip->add_pin_ranges(chip);
      goto retval;
    break;
  };

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

  if(kbuf_ext) {
    switch (kbuf->signal) {
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

      // We could want to use the stock gpio chardev (/dev/gpiochip0 and /dev/gpiochip1) /bc userspace functions use it
      // this code is not yet complete and it mey be better to use the stock devices directly.
      case GPIO_CHARDEV_OPEN:	// .open = gpio_chrdev_open
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
        ret = file->f_op->read(file, return_buffer, kbuf_ext->count, NULL);		//
        if (ret) {
          pr_err("Reading lineinfo returned zero\n");
          kfree(kbuf);
          return -EFAULT;
        }
      return -ENXIO;
      case GPIO_CHARDEV_OWNER: // .owner = THIS_MODULE
        if (copy_to_user(return_buffer, file->f_op->owner->name, strlen(file->f_op->owner->name)+1)) {
          pr_err("GPIO, copying user return value failed\n");
          kfree(kbuf);
          return -EFAULT;
        }
        // ret_sz = strlen(file->f_op->owner->name) + 1;
        // goto generic_ret
      break;
      default:
        pr_err("GPIO, Illegal proxy signal type\n");
        kfree(kbuf);
        return -EPERM;
      break;
    };
  };

	goto end;

	retlong:
	if ( copy_to_user(return_buffer, &ret_l, sizeof(ret_l)) ) {
		pr_err("GPIO, copying int user return value failed\n");
		kfree(kbuf);
		return -EFAULT;
	};

	goto end;

	retval:
	if ( copy_to_user(return_buffer, &ret, sizeof(ret)) ) {
		pr_err("GPIO, copying long int user return value failed\n");
		kfree(kbuf);
		return -EFAULT;
	};

	goto end;

	end:
	kfree(kbuf);
	return len;
}
