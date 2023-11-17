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
#include <linux/uaccess.h>

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

// variables for virtualisaton
extern uint64_t gpio_vpa;
struct tegra_gpio *tegra_gpio_host;
EXPORT_SYMBOL_GPL(tegra_gpio_host);

/*************************** GTE related code ********************/

static inline u32 tegra_gte_readl(struct tegra_gpio *tgi, u32 reg)
{
	printk(KERN_DEBUG "Debug host gpio %s, file %s", __func__, __FILE__);

	return __raw_readl(tgi->gte_regs + reg);
}

static inline void tegra_gte_writel(struct tegra_gpio *tgi, u32 reg,
		u32 val)
{
	printk(KERN_DEBUG "Debug host gpio %s, file %s", __func__, __FILE__);

	__raw_writel(val, tgi->gte_regs + reg);
}

// most tegra_*() functions removed.
// Makes sense to use functions in stock driver ?

/*****************************************************************/

// not used functions removed
// we assume functions will be called in stock driver code gpio-tegra186.c

// extern int _tegra186_gpio_probe(struct platform_device *pdev);

// seems we must export io memory instead of whole struct
// TODO: export irq
struct tegra_gpio *tegra_gpio_virtual = NULL;
void __iomem *secure_virtual = NULL;
void __iomem *base_virtual = NULL;
void __iomem *gte_regs_virtual = NULL;
unsigned int *irq_virtual = NULL;
EXPORT_SYMBOL_GPL(tegra_gpio_virtual);
EXPORT_SYMBOL_GPL(secure_virtual);
EXPORT_SYMBOL_GPL(base_virtual);
EXPORT_SYMBOL_GPL(gte_regs_virtual);
EXPORT_SYMBOL_GPL(irq_virtual);

static int tegra186_gpio_probe(struct platform_device *pdev)
{
/* unused variables
	unsigned int i, j, offset;
	struct gpio_irq_chip *irq;
*/
	struct tegra_gpio *gpio;
/*
	struct device_node *np;
	struct resource *res;
	char **names;
*/
	int err;
//	int ret;
//	int value;
//	void __iomem *base;

	printk(KERN_DEBUG "Debug host gpio %s, file %s", __func__, __FILE__);

	gpio = devm_kzalloc(&pdev->dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	// we shall virtualise 'gpio'
	// this is host code !!!
	// host has been set up by stock tegra driver (initialisations, irq, everything)

	// this copying is not fully correct
	// the physical hardware is accessed from the original addresses
	// 1) can we export the original devm?
	// 2) can we provide guest with 'struct tegra_gpio *gpio' only -- physical gpio handled here in host
	// 3) can it be handled via copy at interaction?

	gpio = tegra_gpio_host;
	err = copy_to_user(tegra_gpio_virtual, gpio, sizeof(tegra_gpio_host));		// the full struct will contain needed irq data ?
	if ( err )  printk(KERN_DEBUG "Debug host gpio %s, 1 copy_to_user error %d", __func__, err);
	err = copy_to_user(secure_virtual, gpio->secure, gpio->soc->num_ports * 0x1000 + 0x1800);
	if ( err )  printk(KERN_DEBUG "Debug host gpio %s, 2 copy_to_user error %d", __func__, err);
	// err = copy_to_user(base_virtual, gpio->base, xxx);		// for now I can't find size -- maybe pointer alone is enough?
	// if ( err )  printk(KERN_DEBUG "Debug host gpio %s, copy_to_user error %d", __func__, err);
	// err = copy_to_user(gte_regs_virtual, gpio->gte_regs, xxx); // for now I can't find size -- maybe pointer alone is enough?
	// if ( err )  printk(KERN_DEBUG "Debug host gpio %s, copy_to_user error %d", __func__, err);
	err = copy_to_user(irq_virtual, gpio->irq, gpio->num_irq * sizeof(*gpio->irq));
	if ( err )  printk(KERN_DEBUG "Debug host gpio %s, 3 copy_to_user error %d", __func__, err);

	tegra_gpio_virtual->secure = secure_virtual;
	tegra_gpio_virtual->base = base_virtual;
	tegra_gpio_virtual->gte_regs = gte_regs_virtual;
	tegra_gpio_virtual->irq = irq_virtual;

	// since this is host code all values and pointers of gpio should be correct
	// gpio shoud be set up already -- with tegra_gte_setup(gpio);
        printk(KERN_DEBUG "Debug host gpio %s, label=%s", __func__, gpio->gpio.label);
	printk(KERN_DEBUG "Debug host gpio %s, initialised gpio at %p", __func__, gpio);
	printk(KERN_DEBUG "Debug host gpio %s, initialised gpio->secure at %p", __func__, gpio->secure);
	printk(KERN_DEBUG "Debug host gpio %s, initialised gpio->base at %p", __func__, gpio->base);
	printk(KERN_DEBUG "Debug host gpio %s, initialised gpio->gte_regs at %p", __func__, gpio->gte_regs);

	return 0;

	// -----------------------------------------------

/* this code would never be executed any how -- to be removed

	gpio->soc = of_device_get_match_data(&pdev->dev);
	gpio->gpio.label = gpio->soc->name;
	gpio->gpio.parent = &pdev->dev;

	gpio->secure = devm_platform_ioremap_resource_byname(pdev, "security");
	if (IS_ERR(gpio->secure))
		return PTR_ERR(gpio->secure);
*/
	/* count the number of banks in the controller */
/*
 	for (i = 0; i < gpio->soc->num_ports; i++)
		if (gpio->soc->ports[i].bank > gpio->num_banks)
			gpio->num_banks = gpio->soc->ports[i].bank;

	gpio->num_banks++;

	gpio->base = devm_platform_ioremap_resource_byname(pdev, "gpio");
	if (IS_ERR(gpio->base))
		return PTR_ERR(gpio->base);

	gpio->gpio_rval = devm_kzalloc(&pdev->dev, gpio->soc->num_ports * 8 *
				      sizeof(*gpio->gpio_rval), GFP_KERNEL);
	if (!gpio->gpio_rval)
		return -ENOMEM;

	np = pdev->dev.of_node;
	if (!np) {
		dev_err(&pdev->dev, "No valid device node, probe failed\n");
		return -EINVAL;
	}

	gpio->use_timestamp = of_property_read_bool(np, "use-timestamp");

	if (gpio->use_timestamp) {
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "gte");
		if (!res) {
			dev_err(&pdev->dev, "Missing gte MEM resource\n");
			return -ENODEV;
		}
		gpio->gte_regs = devm_ioremap_resource(&pdev->dev, res);
		if (IS_ERR(gpio->gte_regs)) {
			ret = PTR_ERR(gpio->gte_regs);
			dev_err(&pdev->dev,
				"Failed to iomap for gte: %d\n", ret);
			return ret;
		}
	}

	err = platform_irq_count(pdev);
	if (err < 0)
		return err;

	gpio->num_irq = err;

	err = tegra186_gpio_irqs_per_bank(gpio);
	if (err < 0)
		return err;

	gpio->irq = devm_kcalloc(&pdev->dev, gpio->num_irq, sizeof(*gpio->irq),
				 GFP_KERNEL);
	if (!gpio->irq)
		return -ENOMEM;

	for (i = 0; i < gpio->num_irq; i++) {
		err = platform_get_irq(pdev, i);
		if (err < 0)
			return err;

		gpio->irq[i] = err;
	}

	gpio->gpio.request = gpiochip_generic_request;
	gpio->gpio.free = gpiochip_generic_free;
	gpio->gpio.get_direction = tegra186_gpio_get_direction;
	gpio->gpio.direction_input = tegra186_gpio_direction_input;
	gpio->gpio.direction_output = tegra186_gpio_direction_output;
	gpio->gpio.get = tegra186_gpio_get,
	gpio->gpio.set = tegra186_gpio_set;
	gpio->gpio.set_config = tegra186_gpio_set_config;
	gpio->gpio.timestamp_control = tegra_gpio_timestamp_control;
	gpio->gpio.timestamp_read = tegra_gpio_timestamp_read;
	gpio->gpio.suspend_configure = tegra_gpio_suspend_configure;
	gpio->gpio.add_pin_ranges = tegra186_gpio_add_pin_ranges;

	gpio->gpio.base = -1;

	for (i = 0; i < gpio->soc->num_ports; i++)
		gpio->gpio.ngpio += gpio->soc->ports[i].pins;

	names = devm_kcalloc(gpio->gpio.parent, gpio->gpio.ngpio,
			     sizeof(*names), GFP_KERNEL);
	if (!names)
		return -ENOMEM;

	for (i = 0, offset = 0; i < gpio->soc->num_ports; i++) {
		const struct tegra_gpio_port *port = &gpio->soc->ports[i];
		char *name;

		for (j = 0; j < port->pins; j++) {
			name = devm_kasprintf(gpio->gpio.parent, GFP_KERNEL,
					      "P%s.%02x", port->name, j);
			if (!name)
				return -ENOMEM;

			names[offset + j] = name;
			// printk(KERN_DEBUG "Debug host gpio %s, name=%s", __func__, name);
		}

		offset += port->pins;
	}

	gpio->gpio.names = (const char * const *)names;

	gpio->gpio.of_node = pdev->dev.of_node;
	gpio->gpio.of_gpio_n_cells = 2;
	gpio->gpio.of_xlate = tegra186_gpio_of_xlate;

	gpio->intc.name = pdev->dev.of_node->name;
	gpio->intc.irq_ack = tegra186_irq_ack;
	gpio->intc.irq_mask = tegra186_irq_mask;
	gpio->intc.irq_unmask = tegra186_irq_unmask;
	gpio->intc.irq_set_type = tegra186_irq_set_type;
	gpio->intc.irq_set_wake = tegra186_irq_set_wake;

	irq = &gpio->gpio.irq;
	irq->chip = &gpio->intc;
	irq->fwnode = of_node_to_fwnode(pdev->dev.of_node);
	irq->child_to_parent_hwirq = tegra186_gpio_child_to_parent_hwirq;
	irq->populate_parent_alloc_arg = tegra186_gpio_populate_parent_fwspec;
	irq->child_offset_to_irq = tegra186_gpio_child_offset_to_irq;
	irq->child_irq_domain_ops.translate = tegra186_gpio_irq_domain_translate;
	irq->handler = handle_simple_irq;
	irq->default_type = IRQ_TYPE_NONE;
	irq->parent_handler = tegra186_gpio_irq;
	irq->parent_handler_data = gpio;
	irq->num_parents = gpio->num_irq;
*/

	/*
	* To simplify things, use a single interrupt per bank for now. Some
	* chips support up to 8 interrupts per bank, which can be useful to
	* distribute the load and decrease the processing latency for GPIOs
	* but it also requires a more complicated interrupt routing than we
	* currently program.
	*/
/* this code would never be executed any how -- to be removed
	if (gpio->num_irqs_per_bank > 1) {
		irq->parents = devm_kcalloc(&pdev->dev, gpio->num_banks,
						sizeof(*irq->parents), GFP_KERNEL);
		if (!irq->parents)
			return -ENOMEM;

		for (i = 0; i < gpio->num_banks; i++)
			irq->parents[i] = gpio->irq[i * gpio->num_irqs_per_bank];

		irq->num_parents = gpio->num_banks;
	} else {
		irq->num_parents = gpio->num_irq;
		irq->parents = gpio->irq;
	}

	if (gpio->soc->num_irqs_per_bank > 1)
		tegra186_gpio_init_route_mapping(gpio);

	np = of_find_matching_node(NULL, tegra186_pmc_of_match);
	if (!of_device_is_available(np))
		np = of_irq_find_parent(pdev->dev.of_node);

	if (of_device_is_available(np)) {
		irq->parent_domain = irq_find_host(np);
		of_node_put(np);

		if (!irq->parent_domain)
			return -EPROBE_DEFER;
	}


	irq->map = devm_kcalloc(&pdev->dev, gpio->gpio.ngpio,
				sizeof(*irq->map), GFP_KERNEL);
	if (!irq->map)
		return -ENOMEM;

	for (i = 0, offset = 0; i < gpio->soc->num_ports; i++) {
		const struct tegra_gpio_port *port = &gpio->soc->ports[i];

		for (j = 0; j < port->pins; j++)
			irq->map[offset + j] = irq->parents[port->bank];

		offset += port->pins;
	}

	platform_set_drvdata(pdev, gpio);

	err = devm_gpiochip_add_data(&pdev->dev, &gpio->gpio, gpio);
	if (err < 0)
		return err;

	if (gpio->soc->is_hw_ts_sup) {
		for (i = 0, offset = 0; i < gpio->soc->num_ports; i++) {
			const struct tegra_gpio_port *port =
							&gpio->soc->ports[i];

			for (j = 0; j < port->pins; j++) {
				base = tegra186_gpio_get_base(gpio, offset + j);
				if (WARN_ON(base == NULL))
					return -EINVAL;

				value = readl(base +
					      TEGRA186_GPIO_ENABLE_CONFIG);
				value |=
				TEGRA186_GPIO_ENABLE_CONFIG_TIMESTAMP_FUNC;
				writel(value,
				       base + TEGRA186_GPIO_ENABLE_CONFIG);
			}
			offset += port->pins;
		}
	}

	if (gpio->use_timestamp)
		tegra_gte_setup(gpio);

	return 0;
*/
}
/* functions removed as probably unecessary
 * we can call the implementation in the stock driver
 *
#ifdef CONFIG_PM_SLEEP
static int tegra_gpio_resume_early(struct device *dev)
{
	struct tegra_gpio *gpio = dev_get_drvdata(dev);
	struct tegra_gpio_saved_register *regs;
	unsigned offset = 0U;
	void __iomem *base;
	int i;

	base = tegra186_gpio_get_base(gpio, offset);
	if (WARN_ON(base == NULL))
		return -EINVAL;

	for (i = 0; i < gpio->gpio.ngpio; i++) {
		regs = &gpio->gpio_rval[i];
		if (!regs->restore_needed)
			continue;

		regs->restore_needed = false;

		writel(regs->val,  base + TEGRA186_GPIO_OUTPUT_VALUE);
		writel(regs->out,  base + TEGRA186_GPIO_OUTPUT_CONTROL);
		writel(regs->conf, base + TEGRA186_GPIO_ENABLE_CONFIG);
	}

	return 0;
}

static int tegra_gpio_suspend_late(struct device *dev)
{
	struct tegra_gpio *gpio = dev_get_drvdata(dev);

	return of_gpiochip_suspend(&gpio->gpio);
}

static const struct dev_pm_ops tegra_gpio_pm = {
	.suspend_late = tegra_gpio_suspend_late,
	.resume_early = tegra_gpio_resume_early,
};
#define TEGRA_GPIO_PM		&tegra_gpio_pm
#else
#define TEGRA_GPIO_PM		NULL
#endif
*/

static int tegra186_gpio_remove(struct platform_device *pdev)
{
	return 0;
}
/*
#define TEGRA186_MAIN_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA186_MAIN_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra186_main_ports[] = {
	TEGRA186_MAIN_GPIO_PORT( A, 2, 0, 7),
	TEGRA186_MAIN_GPIO_PORT( B, 3, 0, 7),
	TEGRA186_MAIN_GPIO_PORT( C, 3, 1, 7),
	TEGRA186_MAIN_GPIO_PORT( D, 3, 2, 6),
	TEGRA186_MAIN_GPIO_PORT( E, 2, 1, 8),
	TEGRA186_MAIN_GPIO_PORT( F, 2, 2, 6),
	TEGRA186_MAIN_GPIO_PORT( G, 4, 1, 6),
	TEGRA186_MAIN_GPIO_PORT( H, 1, 0, 7),
	TEGRA186_MAIN_GPIO_PORT( I, 0, 4, 8),
	TEGRA186_MAIN_GPIO_PORT( J, 5, 0, 8),
	TEGRA186_MAIN_GPIO_PORT( K, 5, 1, 1),
	TEGRA186_MAIN_GPIO_PORT( L, 1, 1, 8),
	TEGRA186_MAIN_GPIO_PORT( M, 5, 3, 6),
	TEGRA186_MAIN_GPIO_PORT( N, 0, 0, 7),
	TEGRA186_MAIN_GPIO_PORT( O, 0, 1, 4),
	TEGRA186_MAIN_GPIO_PORT( P, 4, 0, 7),
	TEGRA186_MAIN_GPIO_PORT( Q, 0, 2, 6),
	TEGRA186_MAIN_GPIO_PORT( R, 0, 5, 6),
	TEGRA186_MAIN_GPIO_PORT( T, 0, 3, 4),
	TEGRA186_MAIN_GPIO_PORT( X, 1, 2, 8),
	TEGRA186_MAIN_GPIO_PORT( Y, 1, 3, 7),
	TEGRA186_MAIN_GPIO_PORT(BB, 2, 3, 2),
	TEGRA186_MAIN_GPIO_PORT(CC, 5, 2, 4),
};

static const struct tegra_gpio_soc tegra186_main_soc = {
	.num_ports = ARRAY_SIZE(tegra186_main_ports),
	.ports = tegra186_main_ports,
	.name = "tegra186-gpio",
	.instance = 0,
	.num_irqs_per_bank = 1,
};

#define TEGRA186_AON_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA186_AON_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra186_aon_ports[] = {
	TEGRA186_AON_GPIO_PORT( S, 0, 1, 5),
	TEGRA186_AON_GPIO_PORT( U, 0, 2, 6),
	TEGRA186_AON_GPIO_PORT( V, 0, 4, 8),
	TEGRA186_AON_GPIO_PORT( W, 0, 5, 8),
	TEGRA186_AON_GPIO_PORT( Z, 0, 7, 4),
	TEGRA186_AON_GPIO_PORT(AA, 0, 6, 8),
	TEGRA186_AON_GPIO_PORT(EE, 0, 3, 3),
	TEGRA186_AON_GPIO_PORT(FF, 0, 0, 5),
};

static const struct tegra_gpio_soc tegra186_aon_soc = {
	.num_ports = ARRAY_SIZE(tegra186_aon_ports),
	.ports = tegra186_aon_ports,
	.name = "tegra186-gpio-aon",
	.instance = 1,
	.num_irqs_per_bank = 1,
};

#define TEGRA194_MAIN_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA194_MAIN_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra194_main_ports[] = {
	TEGRA194_MAIN_GPIO_PORT( A, 1, 2, 8),
	TEGRA194_MAIN_GPIO_PORT( B, 4, 7, 2),
	TEGRA194_MAIN_GPIO_PORT( C, 4, 3, 8),
	TEGRA194_MAIN_GPIO_PORT( D, 4, 4, 4),
	TEGRA194_MAIN_GPIO_PORT( E, 4, 5, 8),
	TEGRA194_MAIN_GPIO_PORT( F, 4, 6, 6),
	TEGRA194_MAIN_GPIO_PORT( G, 4, 0, 8),
	TEGRA194_MAIN_GPIO_PORT( H, 4, 1, 8),
	TEGRA194_MAIN_GPIO_PORT( I, 4, 2, 5),
	TEGRA194_MAIN_GPIO_PORT( J, 5, 1, 6),
	TEGRA194_MAIN_GPIO_PORT( K, 3, 0, 8),
	TEGRA194_MAIN_GPIO_PORT( L, 3, 1, 4),
	TEGRA194_MAIN_GPIO_PORT( M, 2, 3, 8),
	TEGRA194_MAIN_GPIO_PORT( N, 2, 4, 3),
	TEGRA194_MAIN_GPIO_PORT( O, 5, 0, 6),
	TEGRA194_MAIN_GPIO_PORT( P, 2, 5, 8),
	TEGRA194_MAIN_GPIO_PORT( Q, 2, 6, 8),
	TEGRA194_MAIN_GPIO_PORT( R, 2, 7, 6),
	TEGRA194_MAIN_GPIO_PORT( S, 3, 3, 8),
	TEGRA194_MAIN_GPIO_PORT( T, 3, 4, 8),
	TEGRA194_MAIN_GPIO_PORT( U, 3, 5, 1),
	TEGRA194_MAIN_GPIO_PORT( V, 1, 0, 8),
	TEGRA194_MAIN_GPIO_PORT( W, 1, 1, 2),
	TEGRA194_MAIN_GPIO_PORT( X, 2, 0, 8),
	TEGRA194_MAIN_GPIO_PORT( Y, 2, 1, 8),
	TEGRA194_MAIN_GPIO_PORT( Z, 2, 2, 8),
	TEGRA194_MAIN_GPIO_PORT(FF, 3, 2, 2),
	TEGRA194_MAIN_GPIO_PORT(GG, 0, 0, 2)
};

static const struct tegra_gpio_soc tegra194_main_soc = {
	.num_ports = ARRAY_SIZE(tegra194_main_ports),
	.ports = tegra194_main_ports,
	.name = "tegra194-gpio",
	.instance = 0,
	.num_irqs_per_bank = 8,
	.do_vm_check = true,
};

#define TEGRA194_AON_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA194_AON_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra194_aon_ports[] = {
	TEGRA194_AON_GPIO_PORT(AA, 0, 3, 8),
	TEGRA194_AON_GPIO_PORT(BB, 0, 4, 4),
	TEGRA194_AON_GPIO_PORT(CC, 0, 1, 8),
	TEGRA194_AON_GPIO_PORT(DD, 0, 2, 3),
	TEGRA194_AON_GPIO_PORT(EE, 0, 0, 7)
};

static const struct tegra_gpio_soc tegra194_aon_soc = {
	.num_ports = ARRAY_SIZE(tegra194_aon_ports),
	.ports = tegra194_aon_ports,
	.name = "tegra194-gpio-aon",
	.gte_info = tegra194_gte_info,
	.gte_npins = ARRAY_SIZE(tegra194_gte_info),
	.instance = 1,
	.num_irqs_per_bank = 8,
	.is_hw_ts_sup = true,
	.do_vm_check = false,
};

#define TEGRA234_MAIN_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA234_MAIN_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra234_main_ports[] = {
	TEGRA234_MAIN_GPIO_PORT(A, 0, 0, 8),
	TEGRA234_MAIN_GPIO_PORT(B, 0, 3, 1),
	TEGRA234_MAIN_GPIO_PORT(C, 5, 1, 8),
	TEGRA234_MAIN_GPIO_PORT(D, 5, 2, 4),
	TEGRA234_MAIN_GPIO_PORT(E, 5, 3, 8),
	TEGRA234_MAIN_GPIO_PORT(F, 5, 4, 6),
	TEGRA234_MAIN_GPIO_PORT(G, 4, 0, 8),
	TEGRA234_MAIN_GPIO_PORT(H, 4, 1, 8),
	TEGRA234_MAIN_GPIO_PORT(I, 4, 2, 7),
	TEGRA234_MAIN_GPIO_PORT(J, 5, 0, 6),
	TEGRA234_MAIN_GPIO_PORT(K, 3, 0, 8),
	TEGRA234_MAIN_GPIO_PORT(L, 3, 1, 4),
	TEGRA234_MAIN_GPIO_PORT(M, 2, 0, 8),
	TEGRA234_MAIN_GPIO_PORT(N, 2, 1, 8),
	TEGRA234_MAIN_GPIO_PORT(P, 2, 2, 8),
	TEGRA234_MAIN_GPIO_PORT(Q, 2, 3, 8),
	TEGRA234_MAIN_GPIO_PORT(R, 2, 4, 6),
	TEGRA234_MAIN_GPIO_PORT(X, 1, 0, 8),
	TEGRA234_MAIN_GPIO_PORT(Y, 1, 1, 8),
	TEGRA234_MAIN_GPIO_PORT(Z, 1, 2, 8),
	TEGRA234_MAIN_GPIO_PORT(AC, 0, 1, 8),
	TEGRA234_MAIN_GPIO_PORT(AD, 0, 2, 4),
	TEGRA234_MAIN_GPIO_PORT(AE, 3, 3, 2),
	TEGRA234_MAIN_GPIO_PORT(AF, 3, 4, 4),
	TEGRA234_MAIN_GPIO_PORT(AG, 3, 2, 8)
};

static const struct tegra_gpio_soc tegra234_main_soc = {
	.num_ports = ARRAY_SIZE(tegra234_main_ports),
	.ports = tegra234_main_ports,
	.name = "tegra234-gpio",
	.instance = 0,
	.num_irqs_per_bank = 8,
	.do_vm_check = true,
};

#define TEGRA234_AON_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA234_AON_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra234_aon_ports[] = {
	TEGRA234_AON_GPIO_PORT(AA, 0, 4, 8),
	TEGRA234_AON_GPIO_PORT(BB, 0, 5, 4),
	TEGRA234_AON_GPIO_PORT(CC, 0, 2, 8),
	TEGRA234_AON_GPIO_PORT(DD, 0, 3, 3),
	TEGRA234_AON_GPIO_PORT(EE, 0, 0, 8),
	TEGRA234_AON_GPIO_PORT(GG, 0, 1, 1)
};

static const struct tegra_gpio_soc tegra234_aon_soc = {
	.num_ports = ARRAY_SIZE(tegra234_aon_ports),
	.ports = tegra234_aon_ports,
	.name = "tegra234-gpio-aon",
	.instance = 1,
	.num_irqs_per_bank = 8,
	.is_hw_ts_sup = true,
	.do_vm_check = false,
};

#define TEGRA239_MAIN_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA239_MAIN_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra239_main_ports[] = {
	TEGRA239_MAIN_GPIO_PORT(A, 0, 0, 8),
	TEGRA239_MAIN_GPIO_PORT(B, 0, 1, 5),
	TEGRA239_MAIN_GPIO_PORT(C, 0, 2, 8),
	TEGRA239_MAIN_GPIO_PORT(D, 0, 3, 8),
	TEGRA239_MAIN_GPIO_PORT(E, 0, 4, 4),
	TEGRA239_MAIN_GPIO_PORT(F, 0, 5, 8),
	TEGRA239_MAIN_GPIO_PORT(G, 0, 6, 8),
	TEGRA239_MAIN_GPIO_PORT(H, 0, 7, 6),
	TEGRA239_MAIN_GPIO_PORT(J, 1, 0, 8),
	TEGRA239_MAIN_GPIO_PORT(K, 1, 1, 4),
	TEGRA239_MAIN_GPIO_PORT(L, 1, 2, 8),
	TEGRA239_MAIN_GPIO_PORT(M, 1, 3, 8),
	TEGRA239_MAIN_GPIO_PORT(N, 1, 4, 3),
	TEGRA239_MAIN_GPIO_PORT(P, 1, 5, 8),
	TEGRA239_MAIN_GPIO_PORT(Q, 1, 6, 3),
	TEGRA239_MAIN_GPIO_PORT(R, 2, 0, 8),
	TEGRA239_MAIN_GPIO_PORT(S, 2, 1, 8),
	TEGRA239_MAIN_GPIO_PORT(T, 2, 2, 8),
	TEGRA239_MAIN_GPIO_PORT(U, 2, 3, 6),
	TEGRA239_MAIN_GPIO_PORT(V, 2, 4, 2),
	TEGRA239_MAIN_GPIO_PORT(W, 3, 0, 8),
	TEGRA239_MAIN_GPIO_PORT(X, 3, 1, 2)
};

static const struct tegra_gpio_soc tegra239_main_soc = {
	.num_ports = ARRAY_SIZE(tegra239_main_ports),
	.ports = tegra239_main_ports,
	.name = "tegra239-gpio",
	.instance = 0,
	.num_irqs_per_bank = 8,
	.do_vm_check = true,
};

#define TEGRA239_AON_GPIO_PORT(_name, _bank, _port, _pins)	\
	[TEGRA239_AON_GPIO_PORT_##_name] = {			\
		.name = #_name,					\
		.bank = _bank,					\
		.port = _port,					\
		.pins = _pins,					\
	}

static const struct tegra_gpio_port tegra239_aon_ports[] = {
	TEGRA239_AON_GPIO_PORT(AA, 0, 0, 8),
	TEGRA239_AON_GPIO_PORT(BB, 0, 1, 1),
	TEGRA239_AON_GPIO_PORT(CC, 0, 2, 8),
	TEGRA239_AON_GPIO_PORT(DD, 0, 3, 8),
	TEGRA239_AON_GPIO_PORT(EE, 0, 4, 6),
	TEGRA239_AON_GPIO_PORT(FF, 0, 5, 8),
	TEGRA239_AON_GPIO_PORT(GG, 0, 6, 8),
	TEGRA239_AON_GPIO_PORT(HH, 0, 7, 4)
};

static const struct tegra_gpio_soc tegra239_aon_soc = {
	.num_ports = ARRAY_SIZE(tegra239_aon_ports),
	.ports = tegra239_aon_ports,
	.name = "tegra239-gpio-aon",
	.instance = 1,
	.num_irqs_per_bank = 8,
	.is_hw_ts_sup = true,
	.do_vm_check = false,
};

static const struct of_device_id tegra186_gpio_of_match[] = {
	{
		.compatible = "nvidia,tegra186-gpio",
		.data = &tegra186_main_soc
	}, {
		.compatible = "nvidia,tegra186-gpio-aon",
		.data = &tegra186_aon_soc
	}, {
		.compatible = "nvidia,tegra194-gpio",
		.data = &tegra194_main_soc
	}, {
		.compatible = "nvidia,tegra194-gpio-aon",
		.data = &tegra194_aon_soc
	}, {
		.compatible = "nvidia,tegra234-gpio",
		.data = &tegra234_main_soc
	}, {
		.compatible = "nvidia,tegra234-gpio-aon",
		.data = &tegra234_aon_soc
	}, {
		.compatible = "nvidia,tegra239-gpio",
		.data = &tegra239_main_soc
	}, {
		.compatible = "nvidia,tegra239-gpio-aon",
		.data = &tegra239_aon_soc
	}, {
*/
		/* sentinel */
/*
	}
};
MODULE_DEVICE_TABLE(of, tegra186_gpio_of_match);
*/

static struct platform_driver tegra186_gpio_host_proxy;

// Initialization function
static int __init copymemory(void) {
	// use values from stock code in gpio-tegra186.c
	extern void * (*cpy_tegra186_gpio_driver)(struct platform_driver *);

	// Use memcpy to initialise tegra186_gpio_host_proxy
	cpy_tegra186_gpio_driver(&tegra186_gpio_host_proxy);

	tegra186_gpio_host_proxy.driver.name = "tegra186-host-gpio";
	tegra186_gpio_host_proxy.probe = tegra186_gpio_probe;
	tegra186_gpio_host_proxy.remove = tegra186_gpio_remove;

	return 0;
}

module_init(copymemory);
builtin_platform_driver(tegra186_gpio_host_proxy);

MODULE_DESCRIPTION("NVIDIA Tegra186 GPIO host driver");
MODULE_AUTHOR("Kim Sandström <kim.sandstrom@unikie.com>");
MODULE_LICENSE("GPL v2");
