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
#include <linux/module.h>
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
#define I2C_GPIO_FIRMWARE_VER_REG	0x01
#define I2C_GPIO_DIRECTION_REG		0x02
#define I2C_GPIO_VALUE_REG		0x03
#define I2C_GPIO_PULLUP_REG		0x04
#define I2C_GPIO_MODE_REG		0x05
#define I2C_GPIO_BUFFER_REG		0x06
#define I2C_GPIO_SLEW_REG		0x07
#define I2C_GPIO_IRQ_EN_REG		0x08
#define I2C_GPIO_IRQ_SENSE_HI_REG	0x09
#define I2C_GPIO_IRQ_SENSE_LO_REG	0x0A
#define I2C_GPIO_IRQ_STATUS_REG		0x0B

struct i2c_gpio_pinctrl {
	struct device *dev;
	struct i2c_client *client;
	struct pinctrl_dev *pctldev;
	struct pinctrl_desc pinctrl_desc;
	struct regmap *regmap;
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

static const struct of_device_id i2c_gpio_of_match[] = {
	{ .compatible = "olimex,i2c-gpio"},
};

static const struct regmap_range i2c_gpio_wr_ranges[] = {
	{
		.range_min = I2C_GPIO_DIRECTION_REG,
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
		.range_max = I2C_GPIO_DIRECTION_REG,
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
	dev_dbg(pctl->dev, "Read 0x%02x from register 0x%02x\n", *val, reg);
	return 0;
}

static int i2c_gpio_regmap_reg_write(void *context, unsigned int reg,
				     unsigned int val)
{
	struct i2c_gpio_pinctrl *pctl = context;
	dev_dbg(pctl->dev, "Writing 0x%02x into register 0x%02x\n", val, reg);
	return 0;
}



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
	.dt_node_to_map = pinconf_generic_dt_node_to_map_pin,
	.dt_free_map = pinctrl_utils_free_map,
};

static int i2c_gpio_pinconf_get(struct pinctrl_dev *pctldev,
				unsigned pin,
				unsigned long *config)
{
	struct i2c_gpio_pinctrl *pctl = pinctrl_dev_get_drvdata(pctldev);
	unsigned int param = pinconf_to_config_param(*config);

	dev_dbg(pctl->dev, "%s, pin: %d, config: %d\n", __func__, pin, param);

	switch (param) {

	/* Enable/Dissable pullup */
	case PIN_CONFIG_BIAS_DISABLE:
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		break;

	/* OD/PP configuration */
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		break;
	case PIN_CONFIG_DRIVE_PUSH_PULL:
		break;


	/* By default chip uses CMOS ST */
	case PIN_CONFIG_INPUT_ENABLE:
	case PIN_CONFIG_INPUT_SCHMITT:
		break;

	/* Switch between ST CMOS/TTL */
	case PIN_CONFIG_INPUT_SCHMITT_ENABLE:
		break;

	/* Configure as output */
	case PIN_CONFIG_OUTPUT:
		break;

	/* Enable/Disable slew rate */
	case PIN_CONFIG_SLEW_RATE:
		break;

	default:
		return -ENOTSUPP;


	}
	return 0;
}

static int i2c_gpio_pinconf_set(struct pinctrl_dev *pctldev,
				unsigned pin,
				unsigned long *config,
				unsigned num_configs)
{
	return 0;
}

static const struct pinconf_ops i2c_gpio_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = i2c_gpio_pinconf_get,
	.pin_config_set = i2c_gpio_pinconf_set,
};

static int i2c_gpio_probe(struct i2c_client *client,
			  const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct i2c_gpio_pinctrl *pctl;
	int ret;

	dev_dbg(dev, "Probing device....\n");

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_WRITE_WORD_DATA))
		return -ENOSYS;

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

	/* Pinctrl description */
	pctl->pinctrl_desc.name = "i2c-gpio-pintrl";
	pctl->pinctrl_desc.pins = i2c_gpio_pins;
	pctl->pinctrl_desc.npins = ARRAY_SIZE(i2c_gpio_pins);
	pctl->pinctrl_desc.pctlops = &i2c_gpio_pinctrl_ops;
	pctl->pinctrl_desc.confops = &i2c_gpio_pinconf_ops;
	pctl->pinctrl_desc.owner = THIS_MODULE;

	ret = devm_pinctrl_register_and_init(dev, &pctl->pinctrl_desc,
					     pctl, &pctl->pctldev);
	if (ret) {
		dev_err(dev, "Failed to register pinctrl device\n");
		return ret;
	}

	ret = pinctrl_enable(pctl->pctldev);
	if (ret) {
		dev_err(dev, "Failed to enable pinctrl device\n");
		return ret;
	}

	return 0;
}

static struct i2c_driver i2c_gpio_driver = {
	.driver = {
		.name = "i2c-gpio-pinctl",
		.of_match_table = of_match_ptr(i2c_gpio_of_match),
	},
	.probe = i2c_gpio_probe,
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
