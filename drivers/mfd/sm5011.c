// SPDX-License-Identifier: GPL-2.0+
/*
 * Silicon Mitus SM5011 MFD driver
 *
 */

#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mfd/core.h>
#include <linux/mfd/sm5011.h>
#include <linux/mfd/sm5011-private.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

static const struct mfd_cell sm5011_devs[] = {
	{ .name = "sm5011-pmic", },
	/* TODO: Add RTC / CLK(37.768KHz) mfd cells */
};

static const struct regmap_config sm5011_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regmap_irq sm5011_irqs[] = {
	/* INT1 interrupts */
	{ .reg_offset = 0, .mask = SM5011_INT1_MICDET_MSK, },
	{ .reg_offset = 0, .mask = SM5011_INT1_VBAT_VALID_MSK, },
	{ .reg_offset = 0, .mask = SM5011_INT1_MANUALRST, },
	{ .reg_offset = 0, .mask = SM5011_INT1_LONGKEY_MRSTB, },
	{ .reg_offset = 0, .mask = SM5011_INT1_LONGKEY_CHGON, },
	{ .reg_offset = 0, .mask = SM5011_INT1_LONGKEY_nONKEY, },
	{ .reg_offset = 0, .mask = SM5011_INT1_SHORTKEY, },
	/* INT2 interrupts */
	{ .reg_offset = 1, .mask = SM5011_INT2_ALARM2_ON, },
	{ .reg_offset = 1, .mask = SM5011_INT2_ALARM1_ON, },
	{ .reg_offset = 1, .mask = SM5011_INT2_WDTMEROUT, },
};

static const struct regmap_irq_chip sm5011_irq_chip = {
	.name			= "sm5011-pmic",
	.status_base		= SM5011_REG_INT1,
	.mask_base		= SM5011_REG_INTMSK1,
	.num_regs		= 2,
	.irqs			= sm5011_irqs,
	.num_irqs		= ARRAY_SIZE(sm5011_irqs),
};

static const struct of_device_id sm5011_pmic_dt_match[] = {
	{
		.compatible = "sm,sm5011",
		.data = NULL,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, sm5011_pmic_dt_match);

static int sm5011_i2c_probe(struct i2c_client *i2c)
{
	struct sm5011_dev *sm5011 = NULL;
	unsigned int data;
	int ret = 0;
	const struct regmap_config *config;
	const struct regmap_irq_chip *irq_chip;
	const struct mfd_cell *cells;
	int n_devs;

	sm5011 = devm_kzalloc(&i2c->dev,
				sizeof(struct sm5011_dev), GFP_KERNEL);
	if (!sm5011)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sm5011);
	sm5011->dev = &i2c->dev;
	sm5011->i2c = i2c;

	sm5011->irq = i2c->irq;

	config = &sm5011_regmap_config;
	irq_chip = &sm5011_irq_chip;
	cells =  sm5011_devs;
	n_devs = ARRAY_SIZE(sm5011_devs);

	sm5011->regmap = devm_regmap_init_i2c(i2c, config);
	if (IS_ERR(sm5011->regmap)) {
		ret = PTR_ERR(sm5011->regmap);
		dev_err(sm5011->dev, "Failed to allocate register map: %d\n",
				ret);
		return ret;
	}

	ret = regmap_read(sm5011->regmap, SM5011_REG_DEVICEID, &data);
	if (ret < 0) {
		dev_err(sm5011->dev,
			"device not found on this channel (this is not an error)\n");
		return -ENODEV;
	}

	ret = devm_regmap_add_irq_chip(&i2c->dev, sm5011->regmap,
				       sm5011->irq,
				       IRQF_TRIGGER_FALLING | IRQF_ONESHOT |
				       IRQF_SHARED, 0, irq_chip,
				       &sm5011->irq_data);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to add PMIC irq chip: %d\n", ret);
		return ret;
	}

	ret = devm_mfd_add_devices(sm5011->dev, -1, cells, n_devs, NULL,
				   0, NULL);
	if (ret < 0) {
		dev_err(&i2c->dev, "failed to add MFD devices: %d\n", ret);
		return ret;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sm5011_suspend(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct sm5011_dev *sm5011 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		enable_irq_wake(sm5011->irq);

	disable_irq(sm5011->irq);

	return 0;
}

static int sm5011_resume(struct device *dev)
{
	struct i2c_client *i2c = to_i2c_client(dev);
	struct sm5011_dev *sm5011 = i2c_get_clientdata(i2c);

	if (device_may_wakeup(dev))
		disable_irq_wake(sm5011->irq);

	enable_irq(sm5011->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sm5011_pm, sm5011_suspend, sm5011_resume);

static struct i2c_driver sm5011_i2c_driver = {
	.driver = {
		   .name = "sm5011",
		   .pm = &sm5011_pm,
		   .of_match_table = of_match_ptr(sm5011_pmic_dt_match),
	},
	.probe_new = sm5011_i2c_probe,
};

static int __init sm5011_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&sm5011_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register wm831x I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(sm5011_i2c_init);

static void __exit sm5011_i2c_exit(void)
{
	i2c_del_driver(&sm5011_i2c_driver);
}
module_exit(sm5011_i2c_exit);

MODULE_DESCRIPTION("SM5011 multi-function core driver");
MODULE_AUTHOR("Chanho Park <chanho61.park@samsung.com>");
MODULE_LICENSE("GPL");
