#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>	// For functions related to I/O operations
#include <stddef.h>
#include <linux/gpio/consumer.h>	// for gpiod_flags

// as a workaround this struct is copied here from drivers/gpio/gpiolib.h
// including the original header file does not work becaue proxy drivers are overlay and absolute paths will change
// this copied struct is incomplete and subset of the "real" one
struct gpio_device {
	int			id;
	struct device		dev;
  // a huge number of members are removed here -- do not sizeof this struct !!!
};


/* values to be used as "signal" values in struct tegra_gpio_pt */
#define GPIO_CHARDEV_OPEN    '1'   // .open = gpio_chrdev_open
#define GPIO_CHARDEV_IOCTL   '2'   // .unlocked_ioctl = gpio_ioctl -- handles IO operation, get linehandle, set direction
#define GPIO_CHARDEV_POLL    '3'   // .poll = lineinfo_watch_poll
#define GPIO_CHARDEV_READ    '4'   // .read = lineinfo_watch_read
#define GPIO_CHARDEV_OWNER   '5'   // .owner = THIS_MODULE
#define GPIO_CHARDEV_SEEK    '6'   // .llseek = no_llseek
#define GPIO_CHARDEV_RELEASE '7'   // .release = gpio_chrdev_release

#define GPIO_SET             's'   // set level
#define GPIO_GET             'g'   // get level
#define GPIO_GET_DIR         'd'   // get direction
#define GPIO_SET_IN          'i'   // set direction to input
#define GPIO_SET_OUT         'o'   // set direction to output
#define GPIO_CONFIG          'c'   // set config
#define GPIO_SET_BY_NAME     'n'   // set config

#define GPIO_REQ             'r'   // generic request
#define GPIO_FREE            'f'   // free

#define GPIO_TIMESTAMP_CTRL  'C'   // timestamp control
#define GPIO_TIMESTAMP_READ  'R'   // timestamp read
#define GPIO_SUSPEND_CONF    'S'   // suspend configure
#define GPIO_ADD_PINRANGES   'P'   // add_pin_ranges

// helpers to identify chip
#define TEGRA_GPIO_CHIP       0    // tegra-gpio gpio_main_chip
#define TEGRA_GPIO_AON_CHIP   1    // tegra-gpio-aon gpio_aon_chip
#define TEGRA_GPIO_LABEL      "tegra234-gpio\x00\x00\x00\x00\x00\x00\x00"  // gpio_main_chip / gpiochip0 --padded to 20 bytes
#define TEGRA_GPIO_AON_LABEL  "tegra234-gpio-aon\x00\x00\x00"              // gpio_aon_chip  / gpiochip1 --padded to 20 bytes
#define LABEL_SIZE            20

// struct __attribute__((packed)) tegra_gpio_pt {
struct tegra_gpio_pt {
  unsigned char signal;       // defines operation
  unsigned char chipnum;      // number of gpio chip (gpiochip0 or gpiochip1)
  unsigned char level;        // padding to reach 8 byte word alignment
  unsigned char offset;       // address offset for gpio pin
  u32 cmd;                    // gpio_ioctl command
  // tegra_gpio_pt_extended p2;          // extended parameters -- in second word of struct
};

typedef union extended {
  // int level;                // pin level to be set
  unsigned long config;     // pin configuration
  int enable;
  size_t count;             // lineinfo read size
  struct poll_table_struct *poll;
  enum gpiod_flags dflags ;
  u64 arg;                  // gpio_ioctl argument (this is interpreted as a pointer)
} tegra_gpio_pt_extended;

#define MAX_CHIP 2

_Static_assert( sizeof(struct tegra_gpio_pt) == 8,
               "tegra_gpio_pt size is not 8 bytes." );

_Static_assert( sizeof(tegra_gpio_pt_extended) == 8,
               "tegra_gpio_pt_extended size is not 8 bytes." );

#endif
