#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>	// For functions related to I/O operations

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
               "tegra_gpio_io not aligned to 64 bits\n");

#endif
