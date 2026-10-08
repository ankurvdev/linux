// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm MSM8998 GFX memory accelerator (mem-acc) select
 *
 * The memories on the MSM8998 graphics rail (VDD_GFX) have an accelerator
 * select bit in TCSR. Qualcomm's GFX CPR driver sets it to 1 for the
 * 180-342 MHz GPU corners and to 0 for the 414-710 MHz corners. It selects
 * 0 only after raising VDD_GFX to a 414 MHz+ corner and selects 1 before
 * lowering VDD_GFX below one, so 0 is the setting that needs the higher
 * voltage; downstream also keeps it while the GPU clock already runs at a
 * lower corner (clock first, then voltage when scaling down) and when it
 * raises the VDD_GFX floor to the 515 MHz corner in the cold.
 *
 * Mainline has no GFX CPR driver. A board that pins VDD_GFX at or above
 * the 414 MHz corner voltage enables this node to select 0 at probe and
 * after system resume, which the 414 MHz and faster GPU OPPs need.
 */

#include <linux/bits.h>
#include <linux/mfd/syscon.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>

#define GFX_ACC_SEL		BIT(0)

struct qcom_gfx_mem_acc {
	struct regmap *tcsr;
	unsigned int offset;
};

/* Select the 414 MHz+ setting; returns the previous bit or -errno */
static int qcom_gfx_mem_acc_select(struct qcom_gfx_mem_acc *acc)
{
	unsigned int val;
	int ret;

	ret = regmap_read(acc->tcsr, acc->offset, &val);
	if (ret)
		return ret;

	if (val & GFX_ACC_SEL) {
		ret = regmap_update_bits(acc->tcsr, acc->offset, GFX_ACC_SEL, 0);
		if (ret)
			return ret;
	}

	return val & GFX_ACC_SEL;
}

static int qcom_gfx_mem_acc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct qcom_gfx_mem_acc *acc;
	int old;

	acc = devm_kzalloc(dev, sizeof(*acc), GFP_KERNEL);
	if (!acc)
		return -ENOMEM;

	acc->tcsr = syscon_regmap_lookup_by_phandle_args(dev->of_node,
							 "qcom,tcsr", 1,
							 &acc->offset);
	if (IS_ERR(acc->tcsr))
		return dev_err_probe(dev, PTR_ERR(acc->tcsr),
				     "failed to get the TCSR regmap\n");

	old = qcom_gfx_mem_acc_select(acc);
	if (old < 0)
		return dev_err_probe(dev, old, "failed to select mem-acc\n");

	platform_set_drvdata(pdev, acc);

	dev_info(dev, "GFX mem-acc select %d -> 0\n", old);

	return 0;
}

static int qcom_gfx_mem_acc_resume(struct device *dev)
{
	struct qcom_gfx_mem_acc *acc = dev_get_drvdata(dev);
	int old;

	old = qcom_gfx_mem_acc_select(acc);
	if (old < 0)
		return old;

	if (old)
		dev_info(dev, "GFX mem-acc select was reset to 1 in suspend\n");

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(qcom_gfx_mem_acc_pm_ops, NULL,
				qcom_gfx_mem_acc_resume);

static const struct of_device_id qcom_gfx_mem_acc_of_match[] = {
	{ .compatible = "qcom,msm8998-gfx-mem-acc" },
	{ }
};
MODULE_DEVICE_TABLE(of, qcom_gfx_mem_acc_of_match);

static struct platform_driver qcom_gfx_mem_acc_driver = {
	.probe = qcom_gfx_mem_acc_probe,
	.driver = {
		.name = "qcom-gfx-mem-acc",
		.of_match_table = qcom_gfx_mem_acc_of_match,
		.pm = pm_sleep_ptr(&qcom_gfx_mem_acc_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(qcom_gfx_mem_acc_driver);

MODULE_DESCRIPTION("Qualcomm MSM8998 GFX memory accelerator select");
MODULE_LICENSE("GPL");
