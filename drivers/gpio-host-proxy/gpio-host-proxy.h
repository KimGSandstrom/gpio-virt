#ifndef __GPIO_HOST_PROXY__H__
#define __GPIO_HOST_PROXY__H__

#include <linux/types.h>  // For __iomem definition
#include <linux/io.h>	// For functions related to I/O operations
#include <stddef.h>
#include <linux/gpio/consumer.h>	// for gpiod_flags

// as a workaround this struct is copied here from drivers/gpio/gpiolib.h in kernel-5.10
// including the original header file does not work becaue proxy drivers are in an overlay 
// and relative paths will change (kernel-5.10 will be renamed)
// #include ../kernel-5.10/drivers/gpio/gpiolib.h
// this copied struct is incomplete and subset of the "real" one
struct gpio_device {
	int			id;
	struct device		dev;
  // a huge number of members are removed here -- do not "sizeof" this struct !!!
};

// size of qemu iomem. 
// Note: Must be synchronized with value in qemu (hw/misc/nvidia_gpio_guest.c)
#define MEM_SIZE 0x600

/* values to be used as "signal" values in struct tegra_gpio_pt */

// Note: signals with message size of 8 have a signal value greater than or equal to ascii char 'A'
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
#define GPIO_TIMESTAMP_READ  'T'   // timestamp read
#define GPIO_SUSPEND_CONF    'S'   // suspend configure
#define GPIO_ADD_PINRANGES   'P'   // add_pin_ranges

// helpers to identify chip
#define TEGRA_GPIO_CHIP       0    // tegra-gpio gpio_main_chip
#define TEGRA_GPIO_AON_CHIP   1    // tegra-gpio-aon gpio_aon_chip
#define TEGRA_GPIO_LABEL      "tegra234-gpio\x00\x00\x00\x00\x00\x00\x00"  // gpio_main_chip / gpiochip0 --padded to 20 bytes
#define TEGRA_GPIO_AON_LABEL  "tegra234-gpio-aon\x00\x00\x00"              // gpio_aon_chip  / gpiochip1 --padded to 20 bytes
#define LABEL_SIZE            20

// Note: signals with message size of 16 have a signal value less than ascii char 'A'

#define GPIO_READL            '<'
#define GPIO_WRITEL           '>'

#define RWL_STD               '0'   // general readl/writeL
#define RWL_RAW               '1'   // __raw assembler version of readl/writel
#define RWL_RELAXED           '2'   // __relaxed assembler version of readl/writel
                                    //
#define GPIO_CHARDEV_OPEN     '1'   // .open = gpio_chrdev_open
#define GPIO_CHARDEV_IOCTL    '2'   // .unlocked_ioctl = gpio_ioctl -- handles IO operation, get linehandle, set direction
#define GPIO_CHARDEV_POLL     '3'   // .poll = lineinfo_watch_poll
#define GPIO_CHARDEV_READ     '4'   // .read = lineinfo_watch_read
#define GPIO_CHARDEV_OWNER    '5'   // .owner = THIS_MODULE
#define GPIO_CHARDEV_SEEK     '6'   // .llseek = no_llseek
#define GPIO_CHARDEV_RELEASE  '7'   // .release = gpio_chrdev_release


// Note this extern is also in gpio-proxy.h in the kernel source tree (this proxy code might be in an overlay)
extern const unsigned char rwl_std_type;
extern const unsigned char rwl_raw_type;
extern const unsigned char rwl_relaxed_type;

// sizeof is rounded to even 64 bit passhtough writes -- no need to optimise size further on an aarch64
struct tegra_readl_writel {
  unsigned char length;       // shift right one bit, most LSB is ignored
  unsigned char signal;       // Note: 'signal' field is overlapping (based on field offset) with signal in struct tegra_gpio_pt
  unsigned char rwltype;      // type of readl/writel call; std, raw, relaxed
  unsigned char pad[1];       // to get no word wrap at mod 64 bit message size
  u32 value;
  void * address;
};

// struct __attribute__((packed)) tegra_gpio_pt {
struct tegra_gpio_pt {
  unsigned char chipnum;      // lowest bit is number of gpio chip (gpiochip0 or gpiochip1), top 7 bits are message length
  unsigned char signal;       // defines operation
  unsigned char level;        // level to set gpio pin to
  unsigned char offset;       // address offset for gpio pin
  // u32 cmd;                    // gpio_ioctl command
  // tegra_gpio_pt_extended p2;          // extended parameters -- in second word of struct
};

union extended {
  // int level;                // pin level to be set
  unsigned long config;     // pin configuration
  int enable;
  size_t count;             // lineinfo read size
  struct poll_table_struct *poll;
  enum gpiod_flags dflags;
  u64 arg;                  // gpio_ioctl argument (this is interpreted as a pointer)
};

typedef union extended tegra_gpio_pt_extended;

#define MAX_CHIP 2

_Static_assert( sizeof(struct tegra_readl_writel) == 16,
               "tegra_readl_writel size is not 16 bytes." );

_Static_assert( sizeof(struct tegra_gpio_pt) == 4,
               "tegra_gpio_pt size is not 4 bytes." );

_Static_assert( sizeof(tegra_gpio_pt_extended) == 8,
               "tegra_gpio_pt_extended size is not 8 bytes." );
#endif
