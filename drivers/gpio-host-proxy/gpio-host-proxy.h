#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>	// For functions related to I/O operations
#include <stddef.h>

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
	GPIO_SET = 's',     // set level
	GPIO_GET = 'g',     // get level
	GPIO_DIR = 'd',     // get direction
	GPIO_SET_IN = 'i',  // set direction to input
	GPIO_SET_OUT = 'o', // set direction to output
	GPIO_CONFIG = 'c',  // set config
	GPIO_REQ = 'r',		// generic request
	GPIO_FREE = 'f'		// free
};

_Static_assert(sizeof(enum tegra_gpio_pt_signal) == 4,
               "Enum size failure\n");

#define GPIOCHIP_PTLABEL 20 // max size of gpio chip's label (a char string)

struct tegra_gpio_pt {
	enum tegra_gpio_pt_signal signal;	// defines operation -- at the moment only 's' for "set"
	char label[GPIOCHIP_PTLABEL];		// label of gpio chip
	unsigned int offset;				// register offset
	int level;			        		// pin level to be set
};

#define STRINGIFY(x) #x

_Static_assert( ( sizeof(struct tegra_gpio_pt) % 8 ) == 0,
               "tegra_gpio_pt size is not aligned to 64 bits. Size is: " 
               "sizeof(struct tegra_gpio_pt) = " STRINGIFY(sizeof(struct tegra_gpio_pt)));

#endif
