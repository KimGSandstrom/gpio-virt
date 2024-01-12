#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>	// For functions related to I/O operations
#include <stddef.h>
#include <linux/gpio/consumer.h>	// for gpiod_flags

enum tegra_gpio_signal {
	GPIO_READ = 'r',
	GPIO_WRITE = 'w',
	GPIO_INPUT = 'i',
	GPIO_TRISTATE = 't',
	GPIO_DIRECTION = 'd',
	GPIO_MUX = 'm'
};

_Static_assert(sizeof(enum tegra_gpio_signal) == 4,
               "Enum size failure\n");

struct tegra_gpio_op {
	void __iomem *io_address;	// Defined by bank and register.
	enum tegra_gpio_signal signal;	// taking values 'r' or 'w' to define desired operation
	u32 value;			// Read or write value
	u32 bank;			// Memory bank
	u32 reg;			// Register (offset)
};

_Static_assert( (sizeof(struct tegra_gpio_op) % 8) == 0,
               "tegra_gpio_io is not aligned to 64 bits\n");

enum tegra_gpio_pt_signal{
	GPIO_CHARDEV_OPEN = '1',	// .open = gpio_chrdev_open
	GPIO_CHARDEV_IOCTL ='2',	// .unlocked_ioctl = gpio_ioctl -- handles IO operation, get linehandle, set direction
	GPIO_CHARDEV_POLL = '3',	// .poll = lineinfo_watch_poll
	GPIO_CHARDEV_READ = '4',	// .read = lineinfo_watch_read
	GPIO_CHARDEV_OWNER = '5',	// .owner = THIS_MODULE
	GPIO_CHARDEV_SEEK = '6',	// .llseek = no_llseek
	GPIO_CHARDEV_RELEASE = '7', // .release = gpio_chrdev_release
	GPIO_SET_VALUE = 's',		// set level
	GPIO_GET_VALUE = 'g',   	// get level
	GPIO_GET_DIR = 'd',     	// get direction
	GPIO_SET_IN = 'i',  		// set direction to input
	GPIO_SET_OUT = 'o', 		// set direction to output
	GPIO_CONFIG = 'c',  		// set config
	GPIO_REQ = 'r',				// generic request
	GPIO_FREE = 'f',			// free
	GPIO_TIMESTAMP_CTRL = 't',	// timestamp control
	GPIO_TIMESTAMP_READ = 'T',	// timestamp read
	GPIO_SUSPEND_CONF = 'S',	// suspend configure
};

_Static_assert(sizeof(enum tegra_gpio_pt_signal) == 4,
               "Enum size failure\n");

#define GPIOCHIP_LABEL 20 // max size of gpio chip's label (a char string)

struct tegra_gpio_pt {
	enum tegra_gpio_pt_signal signal;	// defines operation -- at the moment only 's' for "set"
	char label[GPIOCHIP_LABEL];			// label of gpio chip
	union param1 {
		unsigned int offset;			// gpio register offset
		u32 cmd;						// gpio_ioctl command
		} p1;
	union param2 {
		int level;						// pin level to be set
		unsigned long config;			// pin configuration
		int enable;
		size_t count;					// lineinfo read size
		struct poll_table_struct *poll;
		enum gpiod_flags dflags;
		u64 arg;						// gpio_ioctl argument (this is interpreted as a pointer)
		} p2;
};

#define STRINGIFY(x) #x

_Static_assert( ( sizeof(struct tegra_gpio_pt) % 8 ) == 0,
               "tegra_gpio_pt size is not aligned to 64 bits. Size is: " 
               "sizeof(struct tegra_gpio_pt) = " STRINGIFY(sizeof(struct tegra_gpio_pt)));

#endif
