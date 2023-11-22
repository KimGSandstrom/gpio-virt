// SPDX-License-Identifier: GPL-2.0-only
/**
 * NVIDIA GPIO Guest Proxy Kernel Module
 * (c) 2023 Unikie, Oy
 * (c) 2023 Kim Sandstrom kim.sandstrom@unikie.com
 *
 **/

#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>

#include <dt-bindings/gpio/tegra186-gpio.h>
#include <dt-bindings/gpio/tegra194-gpio.h>
#include <dt-bindings/gpio/tegra234-gpio.h>
#include <dt-bindings/gpio/tegra239-gpio.h>

/* security registers */
#define TEGRA186_GPIO_CTL_SCR 0x0c
#define TEGRA186_GPIO_CTL_SCR_SEC_WEN BIT(28)
#define TEGRA186_GPIO_CTL_SCR_SEC_REN BIT(27)

#define TEGRA186_GPIO_INT_ROUTE_MAPPING(p, x) (0x14 + (p) * 0x20 + (x) * 4)

#define GPIO_VM_REG				0x00
#define GPIO_VM_RW				0x03
#define GPIO_SCR_REG				0x04
#define GPIO_SCR_DIFF				0x08
#define GPIO_SCR_BASE_DIFF			0x40
#define GPIO_SCR_SEC_WEN			BIT(28)
#define GPIO_SCR_SEC_REN			BIT(27)
#define GPIO_SCR_SEC_G1W			BIT(9)
#define GPIO_SCR_SEC_G1R			BIT(1)
#define GPIO_FULL_ACCESS			(GPIO_SCR_SEC_WEN | \
						 GPIO_SCR_SEC_REN | \
						 GPIO_SCR_SEC_G1R | \
						 GPIO_SCR_SEC_G1W)
#define GPIO_SCR_SEC_ENABLE			(GPIO_SCR_SEC_WEN | \
						 GPIO_SCR_SEC_REN)

/* control registers */
#define TEGRA186_GPIO_ENABLE_CONFIG 0x00
#define TEGRA186_GPIO_ENABLE_CONFIG_ENABLE BIT(0)
#define TEGRA186_GPIO_ENABLE_CONFIG_OUT BIT(1)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_TYPE_NONE (0x0 << 2)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_TYPE_LEVEL (0x1 << 2)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_TYPE_SINGLE_EDGE (0x2 << 2)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_TYPE_DOUBLE_EDGE (0x3 << 2)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_TYPE_MASK (0x3 << 2)
#define TEGRA186_GPIO_ENABLE_CONFIG_TRIGGER_LEVEL BIT(4)
#define TEGRA186_GPIO_ENABLE_CONFIG_DEBOUNCE BIT(5)
#define TEGRA186_GPIO_ENABLE_CONFIG_INTERRUPT BIT(6)
#define TEGRA186_GPIO_ENABLE_CONFIG_TIMESTAMP_FUNC BIT(7)

#define TEGRA186_GPIO_DEBOUNCE_CONTROL 0x04
#define TEGRA186_GPIO_DEBOUNCE_CONTROL_THRESHOLD(x) ((x) & 0xff)

#define TEGRA186_GPIO_INPUT 0x08
#define TEGRA186_GPIO_INPUT_HIGH BIT(0)

#define TEGRA186_GPIO_OUTPUT_CONTROL 0x0c
#define TEGRA186_GPIO_OUTPUT_CONTROL_FLOATED BIT(0)

#define TEGRA186_GPIO_OUTPUT_VALUE 0x10
#define TEGRA186_GPIO_OUTPUT_VALUE_HIGH BIT(0)

#define TEGRA186_GPIO_INTERRUPT_CLEAR 0x14

#define TEGRA186_GPIO_INTERRUPT_STATUS(x) (0x100 + (x) * 4)

/******************** GTE Registers ******************************/

#define GTE_GPIO_TECTRL				0x0
#define GTE_GPIO_TETSCH				0x4
#define GTE_GPIO_TETSCL				0x8
#define GTE_GPIO_TESRC				0xC
#define GTE_GPIO_TECCV				0x10
#define GTE_GPIO_TEPCV				0x14
#define GTE_GPIO_TEENCV				0x18
#define GTE_GPIO_TECMD				0x1C
#define GTE_GPIO_TESTATUS			0x20
#define GTE_GPIO_SLICE0_TETEN			0x40
#define GTE_GPIO_SLICE0_TETDIS			0x44
#define GTE_GPIO_SLICE1_TETEN			0x60
#define GTE_GPIO_SLICE1_TETDIS			0x64
#define GTE_GPIO_SLICE2_TETEN			0x80
#define GTE_GPIO_SLICE2_TETDIS			0x84

#define GTE_GPIO_TECTRL_ENABLE_SHIFT		0
#define GTE_GPIO_TECTRL_ENABLE_MASK		0x1
#define GTE_GPIO_TECTRL_ENABLE_DISABLE		0x0
#define GTE_GPIO_TECTRL_ENABLE_ENABLE		0x1

#define GTE_GPIO_TESRC_SLICE_SHIFT		16
#define GTE_GPIO_TESRC_SLICE_DEFAULT_MASK	0xFF

#define GTE_GPIO_TECMD_CMD_POP			0x1

#define GTE_GPIO_TESTATUS_OCCUPANCY_SHIFT	8
#define GTE_GPIO_TESTATUS_OCCUPANCY_MASK	0xFF

#define AON_GPIO_SLICE1_MAP			0x3000
#define AON_GPIO_SLICE2_MAP			0xFFFFFFF
#define AON_GPIO_SLICE1_INDEX			1
#define AON_GPIO_SLICE2_INDEX			2
#define BASE_ADDRESS_GTE_GPIO_SLICE0		0x40
#define BASE_ADDRESS_GTE_GPIO_SLICE1		0x60
#define BASE_ADDRESS_GTE_GPIO_SLICE2		0x80

#define GTE_GPIO_SLICE_SIZE (BASE_ADDRESS_GTE_GPIO_SLICE1 - \
			     BASE_ADDRESS_GTE_GPIO_SLICE0)

/* AON GPIOS are mapped to only slice 1 and slice 2 */
/* GTE Interrupt connections. For slice 1 */
#define NV_AON_GTE_SLICE1_IRQ_LIC0   0
#define NV_AON_GTE_SLICE1_IRQ_LIC1   1
#define NV_AON_GTE_SLICE1_IRQ_LIC2   2
#define NV_AON_GTE_SLICE1_IRQ_LIC3   3
#define NV_AON_GTE_SLICE1_IRQ_APBERR 4
#define NV_AON_GTE_SLICE1_IRQ_GPIO   5
#define NV_AON_GTE_SLICE1_IRQ_WAKE0  6
#define NV_AON_GTE_SLICE1_IRQ_PMC    7
#define NV_AON_GTE_SLICE1_IRQ_DMIC   8
#define NV_AON_GTE_SLICE1_IRQ_PM     9
#define NV_AON_GTE_SLICE1_IRQ_FPUINT 10
#define NV_AON_GTE_SLICE1_IRQ_AOVC   11
#define NV_AON_GTE_SLICE1_IRQ_GPIO_28 12
#define NV_AON_GTE_SLICE1_IRQ_GPIO_29 13
#define NV_AON_GTE_SLICE1_IRQ_GPIO_30 14
#define NV_AON_GTE_SLICE1_IRQ_GPIO_31 15
#define NV_AON_GTE_SLICE1_IRQ_GPIO_32 16
#define NV_AON_GTE_SLICE1_IRQ_GPIO_33 17
#define NV_AON_GTE_SLICE1_IRQ_GPIO_34 18
#define NV_AON_GTE_SLICE1_IRQ_GPIO_35 19
#define NV_AON_GTE_SLICE1_IRQ_GPIO_36 20
#define NV_AON_GTE_SLICE1_IRQ_GPIO_37 21
#define NV_AON_GTE_SLICE1_IRQ_GPIO_38 22
#define NV_AON_GTE_SLICE1_IRQ_GPIO_39 23
#define NV_AON_GTE_SLICE1_IRQ_GPIO_40 24
#define NV_AON_GTE_SLICE1_IRQ_GPIO_41 25
#define NV_AON_GTE_SLICE1_IRQ_GPIO_42 26
#define NV_AON_GTE_SLICE1_IRQ_GPIO_43 27

/* GTE Interrupt connections. For slice 2 */
#define NV_AON_GTE_SLICE2_IRQ_GPIO_0 0
#define NV_AON_GTE_SLICE2_IRQ_GPIO_1 1
#define NV_AON_GTE_SLICE2_IRQ_GPIO_2 2
#define NV_AON_GTE_SLICE2_IRQ_GPIO_3 3
#define NV_AON_GTE_SLICE2_IRQ_GPIO_4 4
#define NV_AON_GTE_SLICE2_IRQ_GPIO_5 5
#define NV_AON_GTE_SLICE2_IRQ_GPIO_6 6
#define NV_AON_GTE_SLICE2_IRQ_GPIO_7 7
#define NV_AON_GTE_SLICE2_IRQ_GPIO_8 8
#define NV_AON_GTE_SLICE2_IRQ_GPIO_9 9
#define NV_AON_GTE_SLICE2_IRQ_GPIO_10 10
#define NV_AON_GTE_SLICE2_IRQ_GPIO_11 11
#define NV_AON_GTE_SLICE2_IRQ_GPIO_12 12
#define NV_AON_GTE_SLICE2_IRQ_GPIO_13 13
#define NV_AON_GTE_SLICE2_IRQ_GPIO_14 14
#define NV_AON_GTE_SLICE2_IRQ_GPIO_15 15
#define NV_AON_GTE_SLICE2_IRQ_GPIO_16 16
#define NV_AON_GTE_SLICE2_IRQ_GPIO_17 17
#define NV_AON_GTE_SLICE2_IRQ_GPIO_18 18
#define NV_AON_GTE_SLICE2_IRQ_GPIO_19 19
#define NV_AON_GTE_SLICE2_IRQ_GPIO_20 20
#define NV_AON_GTE_SLICE2_IRQ_GPIO_21 21
#define NV_AON_GTE_SLICE2_IRQ_GPIO_22 22
#define NV_AON_GTE_SLICE2_IRQ_GPIO_23 23
#define NV_AON_GTE_SLICE2_IRQ_GPIO_24 24
#define NV_AON_GTE_SLICE2_IRQ_GPIO_25 25
#define NV_AON_GTE_SLICE2_IRQ_GPIO_26 26
#define NV_AON_GTE_SLICE2_IRQ_GPIO_27 27


/**************************************************************/

struct tegra_gpio_port {
	const char *name;
	unsigned int bank;
	unsigned int port;
	unsigned int pins;
};

struct tegra186_pin_range {
	unsigned int offset;
	const char *group;
};

struct tegra_gpio_soc {
	const struct tegra_gpio_port *ports;
	unsigned int num_ports;
	const char *name;
	unsigned int instance;
	unsigned int num_irqs_per_bank;
	bool is_hw_ts_sup;
	bool do_vm_check;
	const struct tegra186_pin_range *pin_ranges;
	unsigned int num_pin_ranges;
	const char *pinmux;
	const struct tegra_gte_info *gte_info;
	int gte_npins;
};

struct tegra_gpio_saved_register {
	bool restore_needed;
	u32 val;
	u32 conf;
	u32 out;
};

struct tegra_gpio {
	struct gpio_chip gpio;
	struct irq_chip intc;
	unsigned int num_irq;
	unsigned int *irq;

	const struct tegra_gpio_soc *soc;
	unsigned int num_irqs_per_bank;
	unsigned int num_banks;
	unsigned int gte_enable;
	bool use_timestamp;

	void __iomem *secure;
	void __iomem *base;
	void __iomem *gte_regs;
	struct tegra_gpio_saved_register *gpio_rval;
};
//

// virtualisation memory (for: tegra_gpio *gpio)
extern uint64_t gpio_vpa;
extern const int vpamem;


// TODO: we need to double these elements, one set of elements for each gpio chip.
extern void * (*get_tegra186_gpio_driver)(struct platform_driver *);	// TODO we need to copy driver for gpiochip1 and gpiochip0
extern void * (*get_preset_gpio)(struct tegra_gpio *, int);

extern struct semaphore *copy_sem_gpio, *copy_sem_driver;	// notifies initialisation done in stock driver
extern uint64_t gpio_ready_flag, driver_ready_flag;
static struct tegra_gpio preset_gpio[2];			// will be initialised to values set by stock driver in host

// static volatile void __iomem *mem_iova = NULL;

extern void __iomem *secure_vpa;
extern void __iomem *base_vpa;
extern void __iomem *gte_regs_vpa;

// this var is used temproarily while we have no true passthrough
static struct platform_driver tegra186_gpio_guest_proxy;

/*************************** GTE related code ********************/

// remove duplicate code (code also in stock driver)


static const struct of_device_id tegra186_pmc_of_match[] = {
	{ .compatible = "nvidia,tegra186-pmc" },
	{ .compatible = "nvidia,tegra194-pmc" },
	{ .compatible = "nvidia,tegra234-pmc" },
	{ /* sentinel */ }
};

static int tegra186_gpio_probe(struct platform_device *pdev)
{
//	unsigned int i, j, offset;
//	struct gpio_irq_chip *irq;
	struct tegra_gpio *gpio;
//	struct device_node *np;
//	struct resource *res;
//	char **names;
//	int err;
//	int ret;
//	int value;
//	void __iomem *base;

	printk(KERN_DEBUG "Debug guest gpio %s, file %s", __func__, __FILE__);

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	// we shall virtualise 'gpio' and transfer the memory segment between guest and host using 'preset_gpio'
	//
	// we are copying hosts initialisation from passthough preset_gpio into gpio
	// temporary debug workaround (while no true passthrough)

	// Use memcpy to initialise struct tegra_gpio *gpio
//TODO: we have two gpio chips
	memcpy(gpio, &preset_gpio[0],sizeof(preset_gpio));

	// we should copy during operation?
	// readl() and writel() functions
	//
	// A completely different issue:
	// this how it was done in bpmp
	// mem_iova = ioremap(gpio_vpa, MEM_SIZE);
        // if (!mem_iova) {
        // 	deb_error("ioremap failed\n");
        // return -ENOMEM;

// debug test
gpio_vpa = 0;

	// this is guest code !!!
	// gpio is already initialised in the host

	// [...]
	// TODO some initialisatons are removed ... its probable that theys should not
	// maybe we only should passthrough *secure, *base, *gte_regs


	// [...]
	// TODO


	// gpio already set up by host -- gte-function
	// for now we let that be while we do fake passthrough
	//if (gpio->use_timestamp)
	//	tegra_gte_setup(gpio);
        printk(KERN_DEBUG "Debug gpio %s, label=%s", __func__, gpio->gpio.label);
	printk(KERN_DEBUG "Debug gpio %s, initialised gpio at %p", __func__, gpio);
	printk(KERN_DEBUG "Debug gpio %s, initialised gpio->secure at %p", __func__, gpio->secure);
	printk(KERN_DEBUG "Debug gpio %s, initialised gpio->base at %p", __func__, gpio->base);
	printk(KERN_DEBUG "Debug gpio %s, initialised gpio->gte_regs at %p", __func__, gpio->gte_regs);

	return 0;

	// there is an error of thought here:
	// the local variable 'gpio' here in the guest driver
	// must be initialised for guest's (non-io) memory space
	// We cannot simply copy it from host in its entirety.
	//
	// We must identify which parts of 'gpio' can/must be passed through.
	// these __iomem variables seem critical:
	// void __iomem *secure;
	// void __iomem *base;
	// void __iomem *gte_regs;
	//
	// what is more: that pointer data is at the moment not passed though  -- only the gpio struct
}

static int tegra186_gpio_remove(struct platform_device *pdev)
{
	return 0;
}

// Initialization function
static int __init copymemory(void) {
	int err;
	// use values from stock code in gpio-tegra186.c

	// initialise platform_driver
	while (!driver_ready_flag)
		if((err=down_interruptible(copy_sem_driver)))
			printk(KERN_DEBUG "Debug semaphore error %d in %s", err, __func__);
	cpy_tegra186_gpio_driver(&tegra186_gpio_guest_proxy);
	up(copy_sem_driver);

	tegra186_gpio_guest_proxy.driver.name = "tegra186-guest-gpio";
	tegra186_gpio_guest_proxy.probe = tegra186_gpio_probe;
	tegra186_gpio_guest_proxy.remove = tegra186_gpio_remove;

	// initialise preset_gpio
	for (n=0, n<=1, n++) {
		while (!gpio_ready_flag & n)
			if((err=down_interruptible(copy_sem_gpio)))
				printk(KERN_DEBUG "Debug semaphore error %d in %s", err, __func__);
		get_preset_gpio(&preset_gpio[n], n);
		up(copy_sem_gpio);
	}

	return 0;
}

module_init(copymemory);
builtin_platform_driver(tegra186_gpio_guest_proxy);

MODULE_DESCRIPTION("NVIDIA Tegra186 GPIO guest driver");
MODULE_AUTHOR("Kim Sandström <kim.sandstrom@unikie.com>");
MODULE_LICENSE("GPL v2");
