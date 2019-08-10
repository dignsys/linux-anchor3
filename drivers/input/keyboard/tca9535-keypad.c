// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for keys on TCA9535 I2C IO expander
 * Derived from drivers/input/keyboard/tca6416-keypad.c
 *
 * Copyright (C) 2018  Nexell Co., Ltd.
 * Author: Bon-gyu, KOO <freestyle@nexell.co.kr>
 */

#include <linux/types.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/workqueue.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/of_irq.h>
#include <linux/tca9535_keypad.h>

#define TCA9535_INPUT          0
#define TCA9535_OUTPUT         1
#define TCA9535_INVERT         2
#define TCA9535_DIRECTION      3

static const struct i2c_device_id tca9535_id[] = {
	{ "tca9535-keys", 16, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, tca9535_id);

struct tca9535_keypad_chip {
	uint16_t reg_output;
	uint16_t reg_direction;
	uint16_t reg_input;

	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work dwork;
	int io_size;
	int irqnum;
	u16 pinmask;
	bool use_polling;
	struct tca9535_button buttons[0];
	unsigned long driver_data;
};

static struct tca9535_keys_platform_data *
gpio_keys_get_devtree_pdata(struct device *dev)
{
	struct tca9535_keys_platform_data *pdata;
	struct tca9535_button *buttons;
	struct fwnode_handle *child;
	int nbuttons;
	u32 *ports;
	int i;

	nbuttons = device_get_child_node_count(dev);
	if (nbuttons == 0)
		return ERR_PTR(-ENODEV);

	pdata = devm_kzalloc(dev,
			     sizeof(*pdata) + nbuttons * sizeof(*buttons),
			     GFP_KERNEL);
	if (!pdata)
		return ERR_PTR(-ENOMEM);

	buttons = (struct tca9535_button *)(pdata + 1);

	pdata->buttons = buttons;
	pdata->nbuttons = nbuttons;

	pdata->rep = device_property_read_bool(dev, "autorepeat");
	pdata->use_polling = device_property_read_bool(dev, "use-polling");

	ports = devm_kzalloc(dev, nbuttons * sizeof(*ports), GFP_KERNEL);
	if (!ports)
		return ERR_PTR(-ENOMEM);
	if (device_property_read_u32_array(dev, "ports", ports, nbuttons) < 0) {
		dev_err(dev, "Ports configurations are not proper\n");
		return ERR_PTR(-EINVAL);
	}

	for (i = 0; i < nbuttons; i++)
		pdata->pinmask |= 1 << (ports[i] > 10 ? ports[i] - 2: ports[i]);

	device_for_each_child_node(dev, child) {
		if (fwnode_property_read_u32(child, "linux,code",
					     &buttons->code)) {
			dev_err(dev, "Button without keycode\n");
			fwnode_handle_put(child);
			return ERR_PTR(-EINVAL);
		}

		fwnode_property_read_string(child, "label", &buttons->desc);

		if (fwnode_property_read_u32(child, "linux,input-type",
					     &buttons->type))
			buttons->type = EV_KEY;

		buttons++;
	}

	return pdata;
}

static int tca9535_write_reg(struct tca9535_keypad_chip *chip, int reg, u16 val)
{
	int error;

	error = chip->io_size > 8 ?
		i2c_smbus_write_word_data(chip->client, reg << 1, val) :
		i2c_smbus_write_byte_data(chip->client, reg, val);
	if (error < 0) {
		dev_err(&chip->client->dev,
			"%s failed, reg: %d, val: %d, error: %d\n",
			__func__, reg, val, error);
		return error;
	}

	return 0;
}

static int tca9535_read_reg(struct tca9535_keypad_chip *chip, int reg, u16 *val)
{
	int retval;

	retval = chip->io_size > 8 ?
		 i2c_smbus_read_word_data(chip->client, reg << 1) :
		 i2c_smbus_read_byte_data(chip->client, reg);
	if (retval < 0) {
		dev_err(&chip->client->dev, "%s failed, reg: %d, error: %d\n",
			__func__, reg, retval);
		return retval;
	}
	*val = (u16)retval;

	return 0;
}

static void tca9535_keys_scan(struct tca9535_keypad_chip *chip)
{
	struct input_dev *input = chip->input;
	u16 reg_val, val;
	int error, i, pinnum;

	error = tca9535_read_reg(chip, TCA9535_INPUT, &reg_val);
	if (error)
		return;

	reg_val &= chip->pinmask;

	/* Figure out which lines have changed */
	val = reg_val ^ chip->reg_input;
	chip->reg_input = reg_val;

	for (i = 0, pinnum = 0; i < 16; i++) {
		if (val & (1 << i)) {
			struct tca9535_button *button = &chip->buttons[pinnum];
			unsigned int type = button->type ?: EV_KEY;
			int state = ((reg_val & (1 << i)) ? 1 : 0)
						^ button->active_low;

			input_event(input, type, button->code, !!state);
			input_sync(input);
		}

		if (chip->pinmask & (1 << i))
			pinnum++;
	}
}

/*
 * This is threaded IRQ handler and this can (and will) sleep.
 */
static irqreturn_t tca9535_keys_isr(int irq, void *dev_id)
{
	struct tca9535_keypad_chip *chip = dev_id;

	tca9535_keys_scan(chip);

	return IRQ_HANDLED;
}

static void tca9535_keys_work_func(struct work_struct *work)
{
	struct tca9535_keypad_chip *chip =
		container_of(work, struct tca9535_keypad_chip, dwork.work);

	tca9535_keys_scan(chip);
	schedule_delayed_work(&chip->dwork, msecs_to_jiffies(100));
}

static int tca9535_keys_open(struct input_dev *dev)
{
	struct tca9535_keypad_chip *chip = input_get_drvdata(dev);

	/* Get initial device state in case it has switches */
	tca9535_keys_scan(chip);

	if (chip->use_polling)
		schedule_delayed_work(&chip->dwork, msecs_to_jiffies(100));
	else
		enable_irq(chip->irqnum);

	return 0;
}

static void tca9535_keys_close(struct input_dev *dev)
{
	struct tca9535_keypad_chip *chip = input_get_drvdata(dev);

	if (chip->use_polling)
		cancel_delayed_work_sync(&chip->dwork);
	else
		disable_irq(chip->irqnum);
}

static int tca9535_setup_registers(struct tca9535_keypad_chip *chip)
{
	int error;

	error = tca9535_read_reg(chip, TCA9535_OUTPUT, &chip->reg_output);
	if (error)
		return error;

	error = tca9535_read_reg(chip, TCA9535_DIRECTION, &chip->reg_direction);
	if (error)
		return error;

	/* ensure that keypad pins are set to input */
	error = tca9535_write_reg(chip, TCA9535_DIRECTION,
				  chip->reg_direction | chip->pinmask);
	if (error)
		return error;

	error = tca9535_read_reg(chip, TCA9535_DIRECTION, &chip->reg_direction);
	if (error)
		return error;

	error = tca9535_read_reg(chip, TCA9535_INPUT, &chip->reg_input);
	if (error)
		return error;

	chip->reg_input &= chip->pinmask;

	return 0;
}

static const struct of_device_id pca9535_of_match[];

static int tca9535_keypad_probe(struct i2c_client *client,
		const struct i2c_device_id *i2c_id)
{
	struct tca9535_keys_platform_data *pdata;
	struct tca9535_keypad_chip *chip;
	struct input_dev *input;
	int error;
	int i;

	/* Check functionality */
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev, "%s adapter not supported\n",
			dev_driver_string(&client->adapter->dev));
		return -ENODEV;
	}

	pdata = dev_get_platdata(&client->dev);
	if (!pdata) {
		pdata = gpio_keys_get_devtree_pdata(&client->dev);
		if (IS_ERR(pdata)) {
			dev_dbg(&client->dev, "no platform data\n");
			return -EINVAL;
		}
	}

	chip = devm_kzalloc(&client->dev, sizeof(struct tca9535_keypad_chip) +
			pdata->nbuttons * sizeof(struct tca9535_button),
			GFP_KERNEL);
	input = input_allocate_device();
	if (!chip || !input) {
		error = -ENOMEM;
		goto fail1;
	}

	if (i2c_id) {
		chip->driver_data = i2c_id->driver_data;
	} else {
		const struct of_device_id *match;

		match = of_match_device(pca9535_of_match, &client->dev);
		if (match) {
			chip->driver_data = (int)(uintptr_t)match->data;
		} else {
			error = -ENODEV;
			goto fail1;
		}
	}

	chip->client = client;
	chip->input = input;
	chip->io_size = chip->driver_data;
	chip->pinmask = pdata->pinmask;
	chip->use_polling = pdata->use_polling;

	INIT_DELAYED_WORK(&chip->dwork, tca9535_keys_work_func);

	input->phys = "tca9535-keys/input0";
	input->name = client->name;
	input->dev.parent = &client->dev;

	input->open = tca9535_keys_open;
	input->close = tca9535_keys_close;

	input->id.bustype = BUS_HOST;
	input->id.vendor = 0x0001;
	input->id.product = 0x0002;
	input->id.version = 0x0101;

	/* Enable auto repeat feature of Linux input subsystem */
	if (pdata->rep)
		__set_bit(EV_REP, input->evbit);

	for (i = 0; i < pdata->nbuttons; i++) {
		unsigned int type;

		chip->buttons[i] = pdata->buttons[i];
		type = (pdata->buttons[i].type) ?: EV_KEY;
		input_set_capability(input, type, pdata->buttons[i].code);
	}

	input_set_drvdata(input, chip);

	/*
	 * Initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	error = tca9535_setup_registers(chip);
	if (error)
		goto fail1;

	if (!chip->use_polling) {
		if (pdata->irq_is_gpio)
			chip->irqnum = gpio_to_irq(client->irq);
		else
			chip->irqnum = client->irq;

		error = request_threaded_irq(chip->irqnum, NULL,
					     tca9535_keys_isr,
					     IRQF_TRIGGER_RISING |
						IRQF_TRIGGER_FALLING |
						IRQF_ONESHOT,
					     "tca9535-keypad", chip);
		if (error) {
			dev_dbg(&client->dev,
				"Unable to claim irq %d; error %d\n",
				chip->irqnum, error);
			goto fail1;
		}
		disable_irq(chip->irqnum);
	}

	error = input_register_device(input);
	if (error) {
		dev_dbg(&client->dev,
			"Unable to register input device, error: %d\n", error);
		goto fail2;
	}

	i2c_set_clientdata(client, chip);
	device_init_wakeup(&client->dev, 1);

	return 0;

fail2:
	if (!chip->use_polling) {
		free_irq(chip->irqnum, chip);
		enable_irq(chip->irqnum);
	}
fail1:
	input_free_device(input);
	kfree(chip);
	return error;
}

static int tca9535_keypad_remove(struct i2c_client *client)
{
	struct tca9535_keypad_chip *chip = i2c_get_clientdata(client);

	if (!chip->use_polling) {
		free_irq(chip->irqnum, chip);
		enable_irq(chip->irqnum);
	}

	input_unregister_device(chip->input);
	kfree(chip);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int tca9535_keypad_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tca9535_keypad_chip *chip = i2c_get_clientdata(client);

	if (device_may_wakeup(dev))
		enable_irq_wake(chip->irqnum);

	return 0;
}

static int tca9535_keypad_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct tca9535_keypad_chip *chip = i2c_get_clientdata(client);

	if (device_may_wakeup(dev))
		disable_irq_wake(chip->irqnum);

	return 0;
}
#endif

static const struct of_device_id pca9535_of_match[] = {
	{ .compatible = "tca9535-key", .data = (void *)16, },
	{ }
};

MODULE_DEVICE_TABLE(of, pca9535_of_match);

static SIMPLE_DEV_PM_OPS(tca9535_keypad_dev_pm_ops,
			 tca9535_keypad_suspend, tca9535_keypad_resume);

static struct i2c_driver tca9535_keypad_driver = {
	.driver = {
		.name	= "tca9535-keypad",
		.pm	= &tca9535_keypad_dev_pm_ops,
		.of_match_table = pca9535_of_match,
	},
	.probe		= tca9535_keypad_probe,
	.remove		= tca9535_keypad_remove,
	.id_table	= tca9535_id,
};

static int __init tca9535_keypad_init(void)
{
	return i2c_add_driver(&tca9535_keypad_driver);
}
#ifdef CONFIG_DEFERRED_KEYPAD
deferred_module_init(tca9535_keypad_init)
#else
module_init(tca9535_keypad_init);
#endif

static void __exit tca9535_keypad_exit(void)
{
	i2c_del_driver(&tca9535_keypad_driver);
}
module_exit(tca9535_keypad_exit);

MODULE_AUTHOR("Bon-gyu, KOO <freestyle@nexell.co.kr>");
MODULE_DESCRIPTION("Keypad driver over tca9535 IO expander");
MODULE_LICENSE("GPL v2");
