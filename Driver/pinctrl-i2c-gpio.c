// SPDX-License-Identifier: GPL-2.0+
/*
 * MOD-I2C-GPIO support for pinctl driver.
 *
 * Copyright (C) 2018 Olimex Ltd.
 *   Author: Stefan Mavrodiev <stefan@olimex.com>
 */
#define DEBUG

#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio/driver.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include "core.h"
#include "devicetree.h"
#include "pinconf.h"
#include "pinctrl-utils.h"

#define I2C_GPIO_DEVICE_ID_REG		0x00
#define I2C_GPIO_DEVICE_ID		0x43

#define I2C_GPIO_FIRMWARE_VER_REG	0x01
#define I2C_GPIO_SERIAL_REG		0x02

#define I2C_GPIO_DIRECTION_REG		0x06	// 0 - output, 1 - input
#define I2C_GPIO_INPUT_REG		0x07	// 0 - low, 1- high
#define I2C_GPIO_OUTPUT_REG		0x08	// 0 -low, 1 -high
#define I2C_GPIO_PULLUP_REG		0x09	// 0 - disable, 1 - enable
#define I2C_GPIO_MODE_REG		0x0A	// 0 - push-pull, 1 - open-drain
#define I2C_GPIO_BUFFER_REG		0x0B	// 0 -TTL, 1 - CMOS
#define I2C_GPIO_SLEW_REG		0x0C	// 0 - unlimited, 1 - limited
#define I2C_GPIO_IRQ_EN_REG		0x0D	// 1 - enabled, 0 - disabled

#define I2C_GPIO_IRQ_SENSE_HI_REG	0x0E
#define I2C_GPIO_IRQ_SENSE_LO_REG	0x0F
#define I2C_GPIO_IRQ_EDGE_NONE			(0)
#define I2C_GPIO_IRQ_EDGE_RISING		(1)
#define I2C_GPIO_IRQ_EDGE_FALLING		(2)
#define I2C_GPIO_IRQ_EDGE_BOTH			(3)

#define I2C_GPIO_IRQ_STATUS_REG		0x10

struct i2c_gpio_pinctrl {
	struct device		*dev;
	struct i2c_client	*client;
	struct pinctrl_dev	*pinctrl_dev;
	struct pinctrl_desc	pinctrl_desc;

	struct gpio_chip	gpio_chip;
	struct regmap		*regmap;

	struct irq_chip		irq_chip;
	struct mutex		irq_lock;
	u8 			irq_mask;
	u16			irq_sense;
	u8			irq_status;
};

static const struct pinctrl_pin_desc i2c_gpio_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
};

/*----------------------------------------------------------------------------*/
static const struct regmap_range i2c_gpio_wr_ranges[] = {
	{
		.range_min = I2C_GPIO_DIRECTION_REG,
		.range_max = I2C_GPIO_DIRECTION_REG,
	},
	{
		.range_min = I2C_GPIO_OUTPUT_REG,
		.range_max = I2C_GPIO_IRQ_SENSE_LO_REG,
	},
};

static const struct regmap_access_table i2c_gpio_wr_table = {
	.yes_ranges = i2c_gpio_wr_ranges,
	.n_yes_ranges = ARRAY_SIZE(i2c_gpio_wr_ranges),
};

static const struct regmap_range i2c_gpio_volatile_ranges[] = {
	{
		.range_min = I2C_GPIO_INPUT_REG,
		.range_max = I2C_GPIO_INPUT_REG,
	},
	{
		.range_min = I2C_GPIO_IRQ_STATUS_REG,
		.range_max = I2C_GPIO_IRQ_STATUS_REG,
	},
};

static const struct regmap_access_table i2c_gpio_volatile_table = {
	.yes_ranges = i2c_gpio_volatile_ranges,
	.n_yes_ranges = ARRAY_SIZE(i2c_gpio_volatile_ranges),
};

static const struct regmap_range i2c_gpio_precious_ranges[] = {
	{
		.range_min = I2C_GPIO_IRQ_STATUS_REG,
		.range_max = I2C_GPIO_IRQ_STATUS_REG,
	},
};

static const struct regmap_access_table i2c_gpio_precious_table = {
	.yes_ranges = i2c_gpio_precious_ranges,
	.n_yes_ranges = ARRAY_SIZE(i2c_gpio_precious_ranges),
};

static int i2c_gpio_regmap_reg_read(void *context, unsigned int reg,
				    unsigned int *val)
{
	struct i2c_gpio_pinctrl *pctl = context;
	struct i2c_client *i2c = pctl->client;
	int ret;

	ret = i2c_smbus_read_byte_data(i2c, reg);
	if (ret < 0)
		return ret;

	*val = ret;
	return 0;
}

static int i2c_gpio_regmap_reg_write(void *context, unsigned int reg,
				     unsigned int val)
{
	struct i2c_gpio_pinctrl *pctl = context;
	struct i2c_client *i2c = pctl->client;
	int ret;

	ret = i2c_smbus_write_byte_data(i2c, reg, val);
	if (ret < 0)
		return ret;

	return 0;
}

static const struct reg_default i2c_gpio_reg_defaults[] = {
	{.reg = I2C_GPIO_DIRECTION_REG,		.def = 0xFF},
	{.reg = I2C_GPIO_OUTPUT_REG,		.def = 0x00},
	{.reg = I2C_GPIO_PULLUP_REG,		.def = 0xFF},
	{.reg = I2C_GPIO_MODE_REG,		.def = 0x00},
	{.reg = I2C_GPIO_BUFFER_REG,		.def = 0xFF},
	{.reg = I2C_GPIO_SLEW_REG,		.def = 0xFF},
	{.reg = I2C_GPIO_IRQ_EN_REG,		.def = 0x00},
	{.reg = I2C_GPIO_IRQ_SENSE_HI_REG,	.def = 0x00},
	{.reg = I2C_GPIO_IRQ_SENSE_LO_REG,	.def = 0x00},
};

static const struct regmap_config i2c_gpio_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.cache_type = REGCACHE_RBTREE,

	.max_register = I2C_GPIO_IRQ_STATUS_REG,

	.reg_read = i2c_gpio_regmap_reg_read,
	.reg_write = i2c_gpio_regmap_reg_write,

	.wr_table = &i2c_gpio_wr_table,
	.volatile_table = &i2c_gpio_volatile_table,
	.precious_table = &i2c_gpio_precious_table,

	.reg_defaults = i2c_gpio_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(i2c_gpio_reg_defaults),
};

/*----------------------------------------------------------------------------*/

static int i2c_gpio_pinctrl_get_groups_count(struct pinctrl_dev *pinctrl_dev)
{
	return 0;
}

static const char *i2c_gpio_pinctrl_get_group_name(struct pinctrl_dev *pinctrl_dev,
						   unsigned int group)
{
	return NULL;
}

static int i2c_gpio_pinctrl_get_group_pins(struct pinctrl_dev *pinctrl_dev,
					   unsigned int group,
					   const unsigned int **pins,
					   unsigned int *num_pins)
{
	return -ENOTSUPP;
}

static const struct pinctrl_ops i2c_gpio_pinctrl_ops = {
	.get_groups_count = i2c_gpio_pinctrl_get_groups_count,
	.get_group_name = i2c_gpio_pinctrl_get_group_name,
	.get_group_pins = i2c_gpio_pinctrl_get_group_pins,
#ifdef CONFIG_OF
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
#endif
};

/*----------------------------------------------------------------------------*/

static int i2c_gpio_get_direction(struct gpio_chip *chip,
				  unsigned int offset)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(pctl->regmap, I2C_GPIO_DIRECTION_REG, &value);
	if (ret < 0)
		return ret;

	return !!(value & BIT(offset));
}

static int i2c_gpio_direction_input(struct gpio_chip *chip,
				    unsigned int offset)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	return regmap_write_bits(pctl->regmap,
				 I2C_GPIO_DIRECTION_REG,
				 BIT(offset),
				 BIT(offset));
}

static int i2c_gpio_direction_output(struct gpio_chip *chip,
				     unsigned int offset, int value)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	int ret;

	ret = regmap_write_bits(pctl->regmap,
				I2C_GPIO_OUTPUT_REG,
				BIT(offset),
				value ? BIT(offset) : 0);
	if (ret < 0)
		return ret;

	return regmap_write_bits(pctl->regmap,
				 I2C_GPIO_DIRECTION_REG,
				 BIT(offset),
				 0);
}

static int i2c_gpio_get(struct gpio_chip *chip,
		        unsigned int offset)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);
	unsigned int value;
	int ret;

	ret = regmap_read(pctl->regmap, I2C_GPIO_INPUT_REG, &value);
	if (ret < 0)
		return ret;

	return !!(value & BIT(offset));
}

static void i2c_gpio_set(struct gpio_chip *chip,
			 unsigned offset, int value)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	regmap_write_bits(pctl->regmap,
			  I2C_GPIO_OUTPUT_REG,
			  BIT(offset),
			  value ? BIT(offset) : 0);
}

static void i2c_gpio_set_multiple(struct gpio_chip *chip,
				  unsigned long *mask,
				  unsigned long *bits)
{
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(chip);

	regmap_write_bits(pctl->regmap, I2C_GPIO_OUTPUT_REG, *mask, *bits);
}

/*----------------------------------------------------------------------------*/

static int i2c_gpio_pin_config_get(struct pinctrl_dev *pinctrl_dev,
				   unsigned pin,
				   unsigned long *config)
{
	struct i2c_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pinctrl_dev);
	unsigned int param = pinconf_to_config_param(*config);
	unsigned int value;
	unsigned int arg;
	int ret;

	dev_dbg(pctl->dev, "%s: pin %d, param %d\n", __func__, pin, param);

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		ret = regmap_read(pctl->regmap, I2C_GPIO_PULLUP_REG, &value);
		if (ret < 0)
			return ret;

		arg = !(value & BIT(pin));
		break;

	case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
	case PIN_CONFIG_BIAS_PULL_UP:
		ret = regmap_read(pctl->regmap, I2C_GPIO_PULLUP_REG, &value);
		if (ret < 0)
			return ret;

		arg = !!(value & BIT(pin));
		break;

	case PIN_CONFIG_INPUT_ENABLE:
		ret = regmap_read(pctl->regmap, I2C_GPIO_DIRECTION_REG, &value);
		if (ret < 0)
			return ret;

		arg = !(value & BIT(pin));
		break;

	case PIN_CONFIG_OUTPUT:
		ret = regmap_read(pctl->regmap, I2C_GPIO_DIRECTION_REG, &value);
		if (ret < 0)
			return ret;

		arg = !!(value & BIT(pin));
		break;

	case PIN_CONFIG_DRIVE_PUSH_PULL:
		ret = regmap_read(pctl->regmap, I2C_GPIO_MODE_REG, &value);
		if (ret < 0)
			return ret;

		arg = !!(value & BIT(pin));
		break;


	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		ret = regmap_read(pctl->regmap, I2C_GPIO_MODE_REG, &value);
		if (ret < 0)
			return ret;

		arg = !(value & BIT(pin));
		break;

	default:
		dev_err(pctl->dev, "Property %u not supported\n", param);
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int i2c_gpio_pin_config_set(struct pinctrl_dev *pinctrl_dev,
				   unsigned pin,
				   unsigned long *config,
				   unsigned num_configs)
{
	struct i2c_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pinctrl_dev);

	unsigned int param, arg;
	int cfg;

	for (cfg = 0; cfg < num_configs; cfg++) {
		param = pinconf_to_config_param(config[cfg]);
		arg = pinconf_to_config_argument(config[cfg]);

		dev_dbg(pctl->dev, "%s: pin %d, param %d, arg %d\n", __func__, pin, param, arg);

		switch(param) {
		case PIN_CONFIG_BIAS_DISABLE:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_PULLUP_REG,
					  BIT(pin), 0);
			break;

		case PIN_CONFIG_BIAS_PULL_PIN_DEFAULT:
		case PIN_CONFIG_BIAS_PULL_UP:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_PULLUP_REG,
					  BIT(pin), BIT(pin));
			break;

		case PIN_CONFIG_INPUT_ENABLE:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_DIRECTION_REG,
					  BIT(pin), BIT(pin));
			break;

		case PIN_CONFIG_OUTPUT:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_OUTPUT_REG,
					  BIT(pin),
					  arg ? BIT(pin) : 0);

			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_DIRECTION_REG,
					  BIT(pin), 0);
			break;

		case PIN_CONFIG_DRIVE_PUSH_PULL:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_MODE_REG,
					  BIT(pin), 0);
			break;

		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			regmap_write_bits(pctl->regmap,
					  I2C_GPIO_MODE_REG,
					  BIT(pin), BIT(pin));
			break;

		default:
			dev_err(pctl->dev, "Property %u not supported\n", param);
			return -ENOTSUPP;
		}
	}

	return 0;
}

static const struct pinconf_ops i2c_gpio_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = i2c_gpio_pin_config_get,
	.pin_config_set = i2c_gpio_pin_config_set,
};

/*----------------------------------------------------------------------------*/

static void i2c_gpio_irq_mask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(gc);

	dev_dbg(pctl->dev, "%s: GPIO%lu\n", __func__, data->hwirq);
	pctl->irq_mask &= ~BIT(data->hwirq);
}

static void i2c_gpio_irq_unmask(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(gc);

	dev_dbg(pctl->dev, "%s: GPIO%lu\n", __func__, data->hwirq);
	pctl->irq_mask |= BIT(data->hwirq);
}

static void i2c_gpio_irq_bus_lock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(gc);

	dev_dbg(pctl->dev, "%s\n", __func__);
	mutex_lock(&pctl->irq_lock);
}

static void i2c_gpio_irq_bus_sync_unlock(struct irq_data *data)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(gc);

	regmap_write(pctl->regmap, I2C_GPIO_IRQ_SENSE_LO_REG, pctl->irq_sense >> 8);
	regmap_write(pctl->regmap, I2C_GPIO_IRQ_SENSE_HI_REG, pctl->irq_sense & 0xFF);
	regmap_write(pctl->regmap, I2C_GPIO_IRQ_EN_REG, pctl->irq_mask);

	dev_dbg(pctl->dev, "%s\n", __func__);
	mutex_unlock(&pctl->irq_lock);
}

static int i2c_gpio_irq_set_type(struct irq_data *data, unsigned int flow_type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(data);
	struct i2c_gpio_pinctrl *pctl = gpiochip_get_data(gc);
	unsigned int pos = data->hwirq;

	dev_dbg(pctl->dev, "%s: pos: %u, type: %02x\n", __func__, pos, flow_type);

	switch(flow_type & IRQ_TYPE_SENSE_MASK) {
	case IRQ_TYPE_NONE:
		pctl->irq_sense &= ~(I2C_GPIO_IRQ_EDGE_BOTH << (pos * 2));
		break;
	case IRQ_TYPE_EDGE_RISING:
		pctl->irq_sense &= ~(I2C_GPIO_IRQ_EDGE_BOTH << (pos * 2));
		pctl->irq_sense |= I2C_GPIO_IRQ_EDGE_RISING << (pos * 2);
		break;
	case IRQ_TYPE_EDGE_FALLING:
		pctl->irq_sense &= ~(I2C_GPIO_IRQ_EDGE_BOTH << (pos * 2));
		pctl->irq_sense |= I2C_GPIO_IRQ_EDGE_FALLING << (pos * 2);
		break;
	case IRQ_TYPE_EDGE_BOTH:
		pctl->irq_sense |= I2C_GPIO_IRQ_EDGE_BOTH << (pos * 2);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static irqreturn_t i2c_gpio_irq_thread_fn(int irq, void *dev_id)
{
	struct i2c_gpio_pinctrl *pctl = (struct i2c_gpio_pinctrl *)dev_id;
	long unsigned int n, status;
	int err;
	u32 val;

	err = regmap_read(pctl->regmap, I2C_GPIO_IRQ_STATUS_REG, &val);
	if (err < 0)
		return IRQ_NONE;
	status = val;

	dev_dbg(pctl->dev, "irq: %02lx\n", status);

	for_each_set_bit(n, &status, pctl->gpio_chip.ngpio)
		handle_nested_irq(irq_find_mapping(pctl->gpio_chip.irq.domain, n));

	return IRQ_HANDLED;
}

/*----------------------------------------------------------------------------*/
static const struct i2c_device_id i2c_gpio_id[] = {
	{ "i2c-gpio", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, i2c_gpio_id);

#ifdef CONFIG_OF
static const struct of_device_id i2c_gpio_of_match[] = {
	{ .compatible = "olimex,i2c-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, i2c_gpio_of_match);
#endif

static int i2c_gpio_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct i2c_gpio_pinctrl *pctl;
	u32 irq_flags;
	u32 serial;
	u8 fw;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENOSYS;

	/* Check device ID */
	ret = i2c_smbus_read_byte_data(client, I2C_GPIO_DEVICE_ID_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to check device ID: %d\n", ret);
		return ret;
	}

	if ((u8)ret != I2C_GPIO_DEVICE_ID) {
		dev_err(dev, "Device ID is not valid: %02x\n", ret);
		return -ENODEV;
	}

	/* Get device FW revision */
	ret = i2c_smbus_read_byte_data(client, I2C_GPIO_FIRMWARE_VER_REG);
	if (ret < 0) {
		dev_err(dev, "Failed to get firmware revision: %d\n", ret);
		return ret;
	}
	fw = ret;

	ret = i2c_smbus_read_i2c_block_data(client,
					    I2C_GPIO_SERIAL_REG,
					    4, (u8 *)&serial);
	if (ret < 0) {
		dev_err(dev, "Failed to get serial number: %d\n", ret);
		return ret;
	}

	dev_info(dev, "Found MOD-I2C-GPIO, firmware revision: %02x, serial number: %08x\n", fw, serial);

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	i2c_set_clientdata(client, pctl);

	pctl->dev = dev;
	pctl->client = client;

	/* Initialize regmap */
	pctl->regmap = devm_regmap_init(dev, NULL, pctl,
					&i2c_gpio_regmap_config);
	if (IS_ERR(pctl->regmap)) {
		ret = PTR_ERR(pctl->regmap);
		dev_err(dev, "Failed to initialize regmap\n");
		return ret;
	}

	/* Initialize pinctrl part */
	pctl->pinctrl_desc.name = "i2c-gpio-pintrl";
	pctl->pinctrl_desc.pins = i2c_gpio_pins;
	pctl->pinctrl_desc.npins = ARRAY_SIZE(i2c_gpio_pins);
	pctl->pinctrl_desc.pctlops = &i2c_gpio_pinctrl_ops;
	pctl->pinctrl_desc.confops = &i2c_gpio_pinconf_ops;
	pctl->pinctrl_desc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pctl->pinctrl_desc,
					     pctl, &pctl->pinctrl_dev);
	if (ret) {
		dev_err(pctl->dev, "Failed to register pinctrl\n");
		return ret;
	}

	ret = pinctrl_enable(pctl->pinctrl_dev);
	if (ret) {
		dev_err(dev, "Failed to enable pinctrl\n");
		return ret;
	}

	/* Initialize gpio controller */
	pctl->gpio_chip.label = devm_kstrdup(dev, client->name, GFP_KERNEL);
	pctl->gpio_chip.base = -1;
	pctl->gpio_chip.ngpio = ARRAY_SIZE(i2c_gpio_pins);
	pctl->gpio_chip.get_direction = i2c_gpio_get_direction;
	pctl->gpio_chip.direction_input = i2c_gpio_direction_input;
	pctl->gpio_chip.direction_output = i2c_gpio_direction_output;
	pctl->gpio_chip.get = i2c_gpio_get;
	pctl->gpio_chip.set = i2c_gpio_set;
	pctl->gpio_chip.set_multiple = i2c_gpio_set_multiple;
	pctl->gpio_chip.set_config = gpiochip_generic_config;
	pctl->gpio_chip.parent = dev;
#ifdef CONFIG_OF_GPIO
	pctl->gpio_chip.of_node = dev->of_node;
#endif
	pctl->gpio_chip.can_sleep = true;

	ret = devm_gpiochip_add_data(dev, &pctl->gpio_chip, pctl);
	if (ret)
		return ret;

	/* Initialize irq controler part */
	if (client->irq) {
		mutex_init(&pctl->irq_lock);

		pctl->irq_chip.name = devm_kstrdup(dev, client->name,
						   GFP_KERNEL);
		pctl->irq_chip.irq_mask = i2c_gpio_irq_mask;
		pctl->irq_chip.irq_unmask = i2c_gpio_irq_unmask;
		pctl->irq_chip.irq_set_type = i2c_gpio_irq_set_type;
		pctl->irq_chip.irq_bus_lock = i2c_gpio_irq_bus_lock;
		pctl->irq_chip.irq_bus_sync_unlock = i2c_gpio_irq_bus_sync_unlock;

		pctl->irq_mask = 0;
		pctl->irq_sense = 0;

		ret = gpiochip_irqchip_add_nested(&pctl->gpio_chip,
						  &pctl->irq_chip, 0,
						  handle_bad_irq,
						  IRQ_TYPE_NONE);
		if (ret < 0) {
			dev_err(dev, "Could not connect irqchip to gpiochip\n");
			return ret;
		}

		irq_flags = irq_get_trigger_type(client->irq);
		if (irq_flags == IRQF_TRIGGER_NONE)
			irq_flags |= IRQF_TRIGGER_RISING;
		irq_flags |= IRQF_ONESHOT | IRQF_SHARED;

		ret = devm_request_threaded_irq(dev, client->irq, NULL,
						i2c_gpio_irq_thread_fn,
						irq_flags,
						dev_name(dev), pctl);
		if (ret < 0) {
			dev_err(dev, "Failed to request irq: %d\n", ret);
			return ret;
		}

		gpiochip_set_nested_irqchip(&pctl->gpio_chip,
					    &pctl->irq_chip,
					    client->irq);

		dev_info(dev, "Registered irq controller part\n");
	} else {
		dev_dbg(dev, "Interrupts are not enabled\n");
	}




	return 0;
}

static void i2c_gpio_shutdown(struct i2c_client *client)
{
	/* Make all inputs */
}

static struct i2c_driver i2c_gpio_driver = {
	.driver = {
		.name = "i2c-gpio-pinctl",
		.of_match_table = of_match_ptr(i2c_gpio_of_match),
	},
	.probe = i2c_gpio_probe,
	.shutdown = i2c_gpio_shutdown,
	.id_table = i2c_gpio_id,
};

static int __init i2c_gpio_init(void)
{
	return i2c_add_driver(&i2c_gpio_driver);
}
/* Register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
subsys_initcall(i2c_gpio_init);

static void __exit i2c_gpio_exit(void)
{
	i2c_del_driver(&i2c_gpio_driver);
}
module_exit(i2c_gpio_exit);

MODULE_DESCRIPTION("MOD-I2C-GPIO pin control driver");
MODULE_AUTHOR("Stefan Mavrodiev <stefan@olimex.com>");
MODULE_LICENSE("GPL v2");
