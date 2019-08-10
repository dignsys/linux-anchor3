/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#ifndef __PINCTRL_NEXELL_H
#define __PINCTRL_NEXELL_H

#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/machine.h>
#include <linux/gpio.h>
#include <dt-bindings/pinctrl/nexell.h>

/**
 * enum pincfg_type - possible pin configuration types supported.
 * @PINCFG_TYPE_FUNC: Function configuration.
 * @PINCFG_TYPE_DAT: Pin value configuration.
 * @PINCFG_TYPE_PULL: Pull up/down configuration.
 * @PINCFG_TYPE_DRV: Drive strength configuration.
 * @PINCFG_TYPE_DIR: pin direction for GPIO mode.
 */
enum pincfg_type {
	PINCFG_TYPE_FUNC,
	PINCFG_TYPE_DAT,
	PINCFG_TYPE_PULL,
	PINCFG_TYPE_DRV,
	PINCFG_TYPE_DIR,

	PINCFG_TYPE_NUM
};

/*
 * pin configuration (pull up/down and drive strength) type and its value are
 * packed together into a 16-bits. The upper 8-bits represent the configuration
 * type and the lower 8-bits hold the value of the configuration type.
 */
#define PINCFG_TYPE_MASK		0xFF
#define PINCFG_VALUE_SHIFT		8
#define PINCFG_VALUE_MASK		(0xFF << PINCFG_VALUE_SHIFT)
#define PINCFG_PACK(type, value)	(((value) << PINCFG_VALUE_SHIFT) | type)
#define PINCFG_UNPACK_TYPE(cfg)		((cfg) & PINCFG_TYPE_MASK)
#define PINCFG_UNPACK_VALUE(cfg)	(((cfg) & PINCFG_VALUE_MASK) >> \
						PINCFG_VALUE_SHIFT)
/**
 * enum eint_type - possible external interrupt types.
 * @EINT_TYPE_NONE: bank does not support external interrupts
 * @EINT_TYPE_GPIO: bank supportes external gpio interrupts
 * @EINT_TYPE_WKUP: bank supportes external wakeup interrupts
 *
 * GPIO controller groups all the available pins into banks. The pins
 * in a pin bank can support external gpio interrupts or external wakeup
 * interrupts or no interrupts at all. From a software perspective, the only
 * difference between external gpio and external wakeup interrupts is that
 * the wakeup interrupts can additionally wakeup the system if it is in
 * suspended state.
 */
enum eint_type {
	EINT_TYPE_NONE,
	EINT_TYPE_GPIO,
	EINT_TYPE_WKUP,
};

/* maximum length of a pin in pin descriptor (example: "gpioa-30") */
#define PIN_NAME_LENGTH	10

struct nexell_pinctrl_drv_data;

/**
 * struct nexell_pin_bank: represent a controller pin-bank.
 * @virt_base: base address of the pin-bank registers.
 * @pin_base: starting pin number of the bank.
 * @nr_pins: number of pins included in this bank.
 * @eint_type: type of the external interrupt supported by the bank.
 * @name: name to be prefixed for each pin in this pin bank.
 * @of_node: OF node of the bank.
 * @drvdata: link to controller driver data
 * @irq_domain: IRQ domain of the bank.
 * @gpio_chip: GPIO chip of the bank.
 * @grange: linux gpio pin range supported by this bank.
 */
struct nexell_pin_bank {
	void __iomem	*virt_base;
	u32		pin_base;
	u8		nr_pins;
	enum eint_type	eint_type;
	char		*name;
	int		irq;
	struct device_node *of_node;
	struct nexell_pinctrl_drv_data *drvdata;
	struct irq_domain *irq_domain;
	struct gpio_chip gpio_chip;
	struct pinctrl_gpio_range grange;
};

/**
 * struct nexell_pin_ctrl: represent a pin controller.
 * @pin_banks: list of pin banks included in this controller.
 * @nr_banks: number of pin banks.
 * @base: starting system wide pin number.
 * @nr_pins: number of pins supported by the controller.
 * @eint_gpio_init: platform specific callback to setup the external gpio
 *	interrupts for the controller.
 * @label: for debug information.
 */
struct nexell_pin_ctrl {
	struct nexell_pin_bank	*pin_banks;
	u32		nr_banks;

	u32		base;
	u32		nr_pins;

	int		(*base_init)(struct nexell_pinctrl_drv_data *);
	int		(*gpio_irq_init)(struct nexell_pinctrl_drv_data *);
	int		(*alive_irq_init)(struct nexell_pinctrl_drv_data *);
	void		(*suspend)(struct nexell_pinctrl_drv_data *);
	void		(*resume)(struct nexell_pinctrl_drv_data *);
};

/**
 * struct nexell_pinctrl_drv_data: wrapper for holding driver data together.
 * @node: global list node
 * @dev: device instance representing the controller.
 * @irq: interrpt number used by the controller to notify gpio interrupts.
 * @ctrl: pin controller instance managed by the driver.
 * @pctl: pin controller descriptor registered with the pinctrl subsystem.
 * @pctl_dev: cookie representing pinctrl device instance.
 * @pin_groups: list of pin groups available to the driver.
 * @nr_groups: number of such pin groups.
 * @pmx_functions: list of pin functions available to the driver.
 * @nr_functions: number of such pin functions.
 * @pwr: list of powerdown pin available to the driver.
 * @nr_pwrs: number of such powerdown pin.
 */
struct nexell_pinctrl_drv_data {
	struct list_head		node;
	struct device			*dev;
	int				irq;

	struct nexell_pin_ctrl		*ctrl;
	struct pinctrl_desc		pctl;
	struct pinctrl_dev		*pctl_dev;

	const struct nexell_pin_group	*pin_groups;
	unsigned int			nr_groups;
	const struct nexell_pmx_func	*pmx_functions;
	unsigned int			nr_functions;
	const struct nexell_pwr_func	*pwr_functions;
	unsigned int			nr_pwr_groups;
};

/**
 * struct nexell_pin_group: represent group of pins of a pinmux function.
 * @name: name of the pin group, used to lookup the group.
 * @pins: the pins included in this group.
 * @num_pins: number of pins included in this group.
 * @func: the function number to be programmed when selected.
 */
struct nexell_pin_group {
	const char		*name;
	const unsigned int	*pins;
	u8			num_pins;
	u8			func;
};

/**
 * struct nexell_pmx_func: represent a pin function.
 * @name: name of the pin function, used to lookup the function.
 * @groups: one or more names of pin groups that provide this function.
 * @num_groups: number of groups included in @groups.
 */
struct nexell_pmx_func {
	const char		*name;
	const char		**groups;
	u8			num_groups;
	u32			val;
};

/**
 * struct nexell_pin_pwr: represent powerdown state of pins.
 * @name: name of the powerdown pin, used to lookup the powerdown pin.
 * @drive: powerdown pin drive state.
 * @val: powerdown pin value state.
 */
struct nexell_pin_pwr {
	const char		*name;
	unsigned int		pin;
	u32			drive;
	u32			val;
	u32			pullsel;
};

/**
 * struct nexell_pwr_func: represent a powerdown pin.
 * @domain: number of the powerdown domain.
 * @groups: one or more names of powerdown pin groups
 * @num_groups: number of groups included in @groups.
 */
struct nexell_pwr_func {
	unsigned int		domain;
	struct nexell_pin_pwr	*groups;
	u8			num_groups;
};

/* list of all exported SoC specific data */
extern const struct nexell_pin_ctrl nxp3220_pin_ctrl[];

/*
 * nx_soc functions
 */
void nx_soc_gpio_set_io_func(unsigned int io, unsigned int func);
int nx_soc_gpio_get_altnum(unsigned int io);
unsigned int nx_soc_gpio_get_io_func(unsigned int io);
void nx_soc_gpio_set_io_dir(unsigned int io, int out);
int nx_soc_gpio_get_io_dir(unsigned int io);
void nx_soc_gpio_set_io_pull(unsigned int io, int val);
int nx_soc_gpio_get_io_pull(unsigned int io);
void nx_soc_gpio_set_io_drv(int gpio, int mode);
int nx_soc_gpio_get_io_drv(int gpio);
void nx_soc_gpio_set_out_value(unsigned int io, int high);
int nx_soc_gpio_get_value(unsigned int io);
int nx_soc_is_gpio_pin(unsigned int io);

#endif	/* __PINCTRL_NEXELL_H */
