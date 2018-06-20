// SPDX-License-Identifier: GPL-2.0+
/*
 * MOD-I2C-GPIO support for pinctl driver.
 *
 * Copyright (C) 2018 Olimex Ltd.
 *   Author: Stefan Mavrodiev <stefan@olimex.com>
 */
#define DEBUG

#include <linux/device.h>
#include <linux/i2c.h>
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
#define I2C_GPIO_DIRECTION_REG		0x02	// 0 - output, 1 - input
#define I2C_GPIO_INPUT_REG		0x03	// 0 - low, 1- high
#define I2C_GPIO_OUTPUT_REG		0x04	// 0 -low, 1 -high
#define I2C_GPIO_PULLUP_REG		0x05	// 0 - disable, 1 - enable
#define I2C_GPIO_MODE_REG		0x06	// 0 - push-pull, 1 - open-drain
#define I2C_GPIO_BUFFER_REG		0x07	// 0 -TTL, 1 - CMOS
#define I2C_GPIO_SLEW_REG		0x08	// 0 - unlimited, 1 - limited
#define I2C_GPIO_IRQ_EN_REG		0x09
#define I2C_GPIO_IRQ_SENSE_HI_REG	0x0A
#define I2C_GPIO_IRQ_SENSE_LO_REG	0x0B
#define I2C_GPIO_IRQ_STATUS_REG		0x0C

struct i2c_gpio_pinctrl {
	struct device		*dev;
	struct i2c_client	*client;
	struct pinctrl_dev	*pctldev;
	struct pinctrl_desc	pinctrl_desc;
	struct gpio_chip	gpio;
	struct regmap		*regmap;
	struct mutex		lock;
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
		.range_min = I2C_GPIO_DIRECTION_REG,
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
	// TODO: Fill with IRQ registerst
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

static int i2c_gpio_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return 0;
}

static const char *i2c_gpio_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						   unsigned int group)
{
	return NULL;
}

static int i2c_gpio_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
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

static int i2c_gpio_pin_config_get(struct pinctrl_dev *pctldev,
				   unsigned pin,
				   unsigned long *config)
{
	struct i2c_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
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

static int i2c_gpio_pin_config_set(struct pinctrl_dev *pctldev,
				   unsigned pin,
				   unsigned long *config,
				   unsigned num_configs)
{
	struct i2c_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);

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
static int i2c_gpio_pinctrl_init(struct i2c_gpio_pinctrl *pctl)
{
	int ret;

	pctl->pinctrl_desc.name = "i2c-gpio-pintrl";
	pctl->pinctrl_desc.pins = i2c_gpio_pins;
	pctl->pinctrl_desc.npins = ARRAY_SIZE(i2c_gpio_pins);
	pctl->pinctrl_desc.pctlops = &i2c_gpio_pinctrl_ops;
	pctl->pinctrl_desc.confops = &i2c_gpio_pinconf_ops;
	pctl->pinctrl_desc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(pctl->dev, &pctl->pinctrl_desc,
					     pctl, &pctl->pctldev);
	if (ret) {
		dev_err(pctl->dev, "Failed to register pinctrl device\n");
		return ret;
	}

	ret = pinctrl_enable(pctl->pctldev);
	if (ret) {
		dev_err(pctl->dev, "Failed to enable pinctrl device\n");
		return ret;
	}

	return 0;
}

static int i2c_gpio_gpiochip_init(struct i2c_gpio_pinctrl *pctl)
{
	int ret;

	pctl->gpio.label = devm_kstrdup(pctl->dev, pctl->client->name, GFP_KERNEL);
	pctl->gpio.base = -1;
	pctl->gpio.ngpio = ARRAY_SIZE(i2c_gpio_pins);
	pctl->gpio.get_direction = i2c_gpio_get_direction;
	pctl->gpio.direction_input = i2c_gpio_direction_input;
	pctl->gpio.direction_output = i2c_gpio_direction_output;
	pctl->gpio.get = i2c_gpio_get;
	pctl->gpio.set = i2c_gpio_set;
	pctl->gpio.set_multiple = i2c_gpio_set_multiple;
	pctl->gpio.set_config = gpiochip_generic_config;
	pctl->gpio.parent = pctl->dev;
#ifdef CONFIG_OF_GPIO
	pctl->gpio.of_node = pctl->dev->of_node;
#endif
	pctl->gpio.can_sleep = true;

	ret = devm_gpiochip_add_data(pctl->dev, &pctl->gpio, pctl);
	if (ret)
		return ret;

	return 0;
}

static int i2c_gpio_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct i2c_gpio_pinctrl *pctl;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WRITE_WORD_DATA))
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

	dev_info(dev, "Found MOD-I2C-GPIO, firmware revision: %02x\n", ret);

	pctl = devm_kzalloc(dev, sizeof(*pctl), GFP_KERNEL);
	if (!pctl)
		return -ENOMEM;

	i2c_set_clientdata(client, pctl);

	pctl->dev = dev;
	pctl->client = client;

	pctl->regmap = devm_regmap_init(dev, NULL, pctl,
					&i2c_gpio_regmap_config);
	if (IS_ERR(pctl->regmap)) {
		ret = PTR_ERR(pctl->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", ret);
		return ret;
	}

	mutex_init(&pctl->lock);

	/* Initialize pinctrl part */
	ret = i2c_gpio_pinctrl_init(pctl);
	if (ret)
		return ret;
	dev_info(dev, "Registered pinctrl part\n");

	/* Initialize gpio controler part */
	ret = i2c_gpio_gpiochip_init(pctl);
	if (ret)
		return ret;
	dev_info(dev, "Registered gpio controller part\n");

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
