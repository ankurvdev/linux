// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm PMI8998 fuel gauge (FG Gen3) driver
 *
 * Copyright (c) 2020, The Linux Foundation. All rights reserved.
 *
 * Based on the pmi8998_fg driver by Joel Selvaraj, Casey Connolly,
 * Barnabás Czémán and Yassine Oudjana, as posted by David Heidelberg in
 * "[PATCH RFC 2/8] power: supply: Add driver for Qualcomm PMI8998 fuel
 * gauge" (November 2025), and on the downstream qpnp-fg-gen3 driver.
 *
 * The driver never writes to the PMIC. Everything it reports comes from
 * FG_BATT_SOC and FG_BATT_INFO registers that the fuel gauge keeps up to
 * date on its own. The FG SRAM behind FG_MEM_IF (battery profile, learned
 * capacity, algorithm settings) is not accessed, and no fuel gauge
 * interrupt is used (enabling one would write its interrupt configuration
 * registers), so changes are found by polling.
 */

#include <linux/bits.h>
#include <linux/device.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#include <asm/byteorder.h>

/* Present in every PMIC peripheral */
#define PERPH_SUBTYPE			0x05

/* FG_BATT_SOC peripheral */
#define FG_BATT_SOC_SUBTYPE		0x10
#define BATT_SOC_FG_MONOTONIC_SOC	0x09	/* 0x0a is a shadow copy */
#define BATT_SOC_INT_RT_STS		0x10
#define  MSOC_EMPTY_BIT			BIT(5)

/* FG_BATT_INFO peripheral */
#define FG_BATT_INFO_SUBTYPE		0x11
#define BATT_INFO_INT_RT_STS		0x10
#define  BT_MISS_BIT			BIT(3)
#define BATT_INFO_BATT_TEMP_LSB		0x50	/* MSB at 0x51 */
#define  BATT_TEMP_MSB_MASK		GENMASK(2, 0)
#define BATT_INFO_JEITA_TOO_COLD	0x62
#define BATT_INFO_JEITA_COLD		0x63
#define BATT_INFO_JEITA_HOT		0x64
#define BATT_INFO_JEITA_TOO_HOT		0x65
#define BATT_INFO_VBATT_LSB		0xa0	/* MSB at 0xa1 */
#define BATT_INFO_IBATT_LSB		0xa2	/* MSB at 0xa3 */

#define MSOC_RAW_FULL			255
#define MSOC_READ_TRIES			5

/* Units of the BATT_INFO voltage and current registers, in nV and nA */
#define VBATT_NV_PER_LSB		122070
#define IBATT_NA_PER_LSB		488281

#define PMI8998_FG_POLL_INTERVAL	msecs_to_jiffies(30000)

struct pmi8998_fg {
	struct device *dev;
	struct regmap *regmap;
	u32 batt_soc_base;
	u32 batt_info_base;

	struct power_supply *psy;
	struct delayed_work poll_work;

	/* Values last announced with power_supply_changed() */
	int last_capacity;
	int last_status;
	int last_present;
};

static int pmi8998_fg_read_le16(struct pmi8998_fg *fg, u16 reg, u16 *val)
{
	__le16 raw;
	int ret;

	ret = regmap_bulk_read(fg->regmap, fg->batt_info_base + reg, &raw,
			       sizeof(raw));
	if (ret)
		return ret;

	*val = le16_to_cpu(raw);

	return 0;
}

static int pmi8998_fg_get_voltage(struct pmi8998_fg *fg, int *uv)
{
	u16 raw;
	int ret;

	ret = pmi8998_fg_read_le16(fg, BATT_INFO_VBATT_LSB, &raw);
	if (ret)
		return ret;

	*uv = div_u64((u64)raw * VBATT_NV_PER_LSB, 1000);

	return 0;
}

static int pmi8998_fg_get_current(struct pmi8998_fg *fg, int *ua)
{
	u16 raw;
	int ret;

	ret = pmi8998_fg_read_le16(fg, BATT_INFO_IBATT_LSB, &raw);
	if (ret)
		return ret;

	/*
	 * The fuel gauge reports discharge current as positive, the power
	 * supply ABI wants it negative.
	 */
	*ua = -div_s64((s64)(s16)raw * IBATT_NA_PER_LSB, 1000);

	return 0;
}

static int pmi8998_fg_get_temp(struct pmi8998_fg *fg, int *decidegc)
{
	u16 raw;
	int ret;

	ret = pmi8998_fg_read_le16(fg, BATT_INFO_BATT_TEMP_LSB, &raw);
	if (ret)
		return ret;

	/* 11-bit value in units of 0.25 K */
	raw &= (BATT_TEMP_MSB_MASK << 8) | 0xff;
	*decidegc = DIV_ROUND_CLOSEST((int)raw * 25 - 27315, 10);

	return 0;
}

static int pmi8998_fg_get_jeita(struct pmi8998_fg *fg, u16 reg, int *decidegc)
{
	unsigned int raw;
	int ret;

	ret = regmap_read(fg->regmap, fg->batt_info_base + reg, &raw);
	if (ret)
		return ret;

	/* 0.5 degC per LSB, starting at -30 degC */
	*decidegc = (int)raw * 5 - 300;

	return 0;
}

static int pmi8998_fg_get_present(struct pmi8998_fg *fg, int *present)
{
	unsigned int sts;
	int ret;

	ret = regmap_read(fg->regmap, fg->batt_info_base + BATT_INFO_INT_RT_STS,
			  &sts);
	if (ret)
		return ret;

	*present = !(sts & BT_MISS_BIT);

	return 0;
}

static int pmi8998_fg_get_msoc_raw(struct pmi8998_fg *fg, unsigned int *raw)
{
	u8 soc[2];
	int i, ret;

	/*
	 * The monotonic SOC register and its shadow copy are updated one
	 * after the other, so retry until both agree (like downstream does).
	 * If they keep differing, the update is still in flight and either
	 * value is current; take the lower one.
	 */
	for (i = 0; i < MSOC_READ_TRIES; i++) {
		ret = regmap_bulk_read(fg->regmap,
				       fg->batt_soc_base + BATT_SOC_FG_MONOTONIC_SOC,
				       soc, sizeof(soc));
		if (ret)
			return ret;

		if (soc[0] == soc[1])
			break;
	}

	*raw = min(soc[0], soc[1]);

	return 0;
}

static int pmi8998_fg_get_charger_status(struct power_supply *psy)
{
	union power_supply_propval val;
	int ret;

	ret = power_supply_get_property_from_supplier(psy,
						      POWER_SUPPLY_PROP_STATUS,
						      &val);
	if (ret)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	return val.intval;
}

static int pmi8998_fg_get_status(struct pmi8998_fg *fg,
				 struct power_supply *psy, int *status)
{
	unsigned int msoc_raw;
	int ret, ua;

	*status = pmi8998_fg_get_charger_status(psy);
	if (*status != POWER_SUPPLY_STATUS_UNKNOWN)
		return 0;

	/* Without the charger, go by the state of charge and the current */
	ret = pmi8998_fg_get_msoc_raw(fg, &msoc_raw);
	if (ret)
		return ret;

	if (msoc_raw == MSOC_RAW_FULL) {
		*status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	}

	ret = pmi8998_fg_get_current(fg, &ua);
	if (ret)
		return ret;

	if (ua > 0)
		*status = POWER_SUPPLY_STATUS_CHARGING;
	else if (ua < 0)
		*status = POWER_SUPPLY_STATUS_DISCHARGING;
	else
		*status = POWER_SUPPLY_STATUS_NOT_CHARGING;

	return 0;
}

static int pmi8998_fg_get_capacity(struct pmi8998_fg *fg,
				   struct power_supply *psy, int *capacity)
{
	const struct power_supply_battery_info *info = psy->battery_info;
	unsigned int msoc_raw, sts;
	int ret, uv;

	ret = pmi8998_fg_get_msoc_raw(fg, &msoc_raw);
	if (ret)
		return ret;

	/*
	 * Same scaling as downstream: 0 and 255 are the endpoints, 1..254
	 * map to 1..99 so that 100% is only reported when really full.
	 */
	if (msoc_raw == 0 || msoc_raw == MSOC_RAW_FULL)
		*capacity = msoc_raw * 100 / MSOC_RAW_FULL;
	else
		*capacity = DIV_ROUND_CLOSEST((msoc_raw - 1) * 98,
					      MSOC_RAW_FULL - 2) + 1;

	/*
	 * Like downstream, report an empty battery once the fuel gauge flags
	 * its SOC as empty and the voltage is below the cutoff voltage.
	 */
	if (info && info->voltage_min_design_uv > 0) {
		ret = regmap_read(fg->regmap,
				  fg->batt_soc_base + BATT_SOC_INT_RT_STS, &sts);
		if (ret)
			return ret;

		if (sts & MSOC_EMPTY_BIT) {
			ret = pmi8998_fg_get_voltage(fg, &uv);
			if (ret)
				return ret;

			if (uv < info->voltage_min_design_uv) {
				*capacity = 0;
				return 0;
			}
		}
	}

	/*
	 * Downstream "hold-soc-while-full": once the charger has terminated
	 * with the SOC at 99% or more, report 100%. Downstream also writes a
	 * full SOC into the FG SRAM at that point; this driver doesn't.
	 */
	if (DIV_ROUND_CLOSEST(msoc_raw * 100, MSOC_RAW_FULL) >= 99 &&
	    pmi8998_fg_get_charger_status(psy) == POWER_SUPPLY_STATUS_FULL)
		*capacity = 100;

	return 0;
}

static const enum power_supply_property pmi8998_fg_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TEMP_MIN,
	POWER_SUPPLY_PROP_TEMP_MAX,
	POWER_SUPPLY_PROP_TEMP_ALERT_MIN,
	POWER_SUPPLY_PROP_TEMP_ALERT_MAX,
};

static int pmi8998_fg_get_property(struct power_supply *psy,
				   enum power_supply_property psp,
				   union power_supply_propval *val)
{
	struct pmi8998_fg *fg = power_supply_get_drvdata(psy);

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		return pmi8998_fg_get_status(fg, psy, &val->intval);
	case POWER_SUPPLY_PROP_PRESENT:
		return pmi8998_fg_get_present(fg, &val->intval);
	case POWER_SUPPLY_PROP_CAPACITY:
		return pmi8998_fg_get_capacity(fg, psy, &val->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return pmi8998_fg_get_voltage(fg, &val->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return pmi8998_fg_get_current(fg, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return pmi8998_fg_get_temp(fg, &val->intval);
	/* JEITA thresholds: charging stops outside the hard ones */
	case POWER_SUPPLY_PROP_TEMP_MIN:
		return pmi8998_fg_get_jeita(fg, BATT_INFO_JEITA_TOO_COLD,
					    &val->intval);
	case POWER_SUPPLY_PROP_TEMP_MAX:
		return pmi8998_fg_get_jeita(fg, BATT_INFO_JEITA_TOO_HOT,
					    &val->intval);
	case POWER_SUPPLY_PROP_TEMP_ALERT_MIN:
		return pmi8998_fg_get_jeita(fg, BATT_INFO_JEITA_COLD,
					    &val->intval);
	case POWER_SUPPLY_PROP_TEMP_ALERT_MAX:
		return pmi8998_fg_get_jeita(fg, BATT_INFO_JEITA_HOT,
					    &val->intval);
	default:
		return -EINVAL;
	}
}

static void pmi8998_fg_external_power_changed(struct power_supply *psy)
{
	struct pmi8998_fg *fg = power_supply_get_drvdata(psy);

	mod_delayed_work(system_freezable_power_efficient_wq, &fg->poll_work, 0);
}

static int pmi8998_fg_psy_init(struct power_supply *psy)
{
	struct pmi8998_fg *fg = power_supply_get_drvdata(psy);

	/* Set before the supply is visible, for the poll work */
	fg->psy = psy;

	return 0;
}

static const struct power_supply_desc pmi8998_fg_psy_desc = {
	.name = "qcom-battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = pmi8998_fg_props,
	.num_properties = ARRAY_SIZE(pmi8998_fg_props),
	.get_property = pmi8998_fg_get_property,
	.external_power_changed = pmi8998_fg_external_power_changed,
	.init = pmi8998_fg_psy_init,
};

static void pmi8998_fg_poll_work(struct work_struct *work)
{
	struct pmi8998_fg *fg = container_of(work, struct pmi8998_fg,
					     poll_work.work);
	int capacity = -1, status = -1, present = -1;

	/* A failed read leaves -1, which also counts as a change */
	pmi8998_fg_get_capacity(fg, fg->psy, &capacity);
	pmi8998_fg_get_status(fg, fg->psy, &status);
	pmi8998_fg_get_present(fg, &present);

	if (capacity != fg->last_capacity || status != fg->last_status ||
	    present != fg->last_present) {
		fg->last_capacity = capacity;
		fg->last_status = status;
		fg->last_present = present;
		power_supply_changed(fg->psy);
	}

	queue_delayed_work(system_freezable_power_efficient_wq, &fg->poll_work,
			   PMI8998_FG_POLL_INTERVAL);
}

static void pmi8998_fg_disable_poll(void *data)
{
	struct pmi8998_fg *fg = data;

	disable_delayed_work_sync(&fg->poll_work);
}

static int pmi8998_fg_check_subtype(struct pmi8998_fg *fg, u32 base,
				    unsigned int expected)
{
	unsigned int subtype;
	int ret;

	ret = regmap_read(fg->regmap, base + PERPH_SUBTYPE, &subtype);
	if (ret)
		return dev_err_probe(fg->dev, ret,
				     "Failed to read subtype at 0x%04x\n", base);

	if (subtype != expected)
		return dev_err_probe(fg->dev, -ENODEV,
				     "Unexpected subtype 0x%02x at 0x%04x\n",
				     subtype, base);

	return 0;
}

static int pmi8998_fg_probe(struct platform_device *pdev)
{
	struct power_supply_config psy_cfg = {};
	struct device *dev = &pdev->dev;
	struct power_supply *psy;
	struct pmi8998_fg *fg;
	u32 base[2];
	int ret;

	fg = devm_kzalloc(dev, sizeof(*fg), GFP_KERNEL);
	if (!fg)
		return -ENOMEM;

	fg->dev = dev;
	fg->last_capacity = -1;
	fg->last_status = -1;
	fg->last_present = -1;

	fg->regmap = dev_get_regmap(dev->parent, NULL);
	if (!fg->regmap)
		return dev_err_probe(dev, -ENODEV, "Failed to get regmap\n");

	/* FG_BATT_SOC and FG_BATT_INFO; FG_MEM_IF (third entry) is unused */
	ret = device_property_read_u32_array(dev, "reg", base,
					     ARRAY_SIZE(base));
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read reg\n");

	fg->batt_soc_base = base[0];
	fg->batt_info_base = base[1];

	ret = pmi8998_fg_check_subtype(fg, fg->batt_soc_base,
				       FG_BATT_SOC_SUBTYPE);
	if (ret)
		return ret;

	ret = pmi8998_fg_check_subtype(fg, fg->batt_info_base,
				       FG_BATT_INFO_SUBTYPE);
	if (ret)
		return ret;

	INIT_DELAYED_WORK(&fg->poll_work, pmi8998_fg_poll_work);

	psy_cfg.drv_data = fg;
	psy_cfg.fwnode = dev_fwnode(dev);

	psy = devm_power_supply_register(dev, &pmi8998_fg_psy_desc, &psy_cfg);
	if (IS_ERR(psy))
		return dev_err_probe(dev, PTR_ERR(psy),
				     "Failed to register battery\n");

	/* Runs before the supply is unregistered */
	ret = devm_add_action_or_reset(dev, pmi8998_fg_disable_poll, fg);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, fg);

	queue_delayed_work(system_freezable_power_efficient_wq, &fg->poll_work,
			   0);

	return 0;
}

static int pmi8998_fg_resume(struct device *dev)
{
	struct pmi8998_fg *fg = dev_get_drvdata(dev);

	mod_delayed_work(system_freezable_power_efficient_wq, &fg->poll_work, 0);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(pmi8998_fg_pm_ops, NULL, pmi8998_fg_resume);

static const struct of_device_id pmi8998_fg_of_match[] = {
	{ .compatible = "qcom,pmi8998-fg" },
	{ }
};
MODULE_DEVICE_TABLE(of, pmi8998_fg_of_match);

static struct platform_driver pmi8998_fg_driver = {
	.probe = pmi8998_fg_probe,
	.driver = {
		.name = "pmi8998-fg",
		.of_match_table = pmi8998_fg_of_match,
		.pm = pm_sleep_ptr(&pmi8998_fg_pm_ops),
	},
};
module_platform_driver(pmi8998_fg_driver);

MODULE_AUTHOR("Casey Connolly <casey@connolly.tech>");
MODULE_AUTHOR("David Heidelberg <david@ixit.cz>");
MODULE_AUTHOR("Joel Selvaraj <foss@joelselvaraj.com>");
MODULE_AUTHOR("Yassine Oudjana <y.oudjana@protonmail.com>");
MODULE_DESCRIPTION("Qualcomm PMI8998 fuel gauge driver");
MODULE_LICENSE("GPL");
