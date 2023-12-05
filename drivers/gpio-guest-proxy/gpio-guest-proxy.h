#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>     // For functions related to I/O operations

enum tegra_gpio_signal {
    GPIO_READ = 'r',
    GPIO_WRITE = 'w'
};

struct tegra_gpio_op {
	enum tegra_gpio_signal signal;	// signal tetemining if it's a read or write operation
	void __iomem *io_address;		// this defines the io bank and register. no need to follow the pointer value.
	u32 value;			// either the returned read value or a write value
	u32 bank;
	u32 reg;
};

#endif
