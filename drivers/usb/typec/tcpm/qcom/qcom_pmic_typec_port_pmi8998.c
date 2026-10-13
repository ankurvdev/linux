// SPDX-License-Identifier: GPL-2.0
/*
 * Type-C port for the SMB2 charger of the Qualcomm PMI8998 (and PM660).
 *
 * Unlike PM8150B/PMI632, the Type-C state machine of these PMICs is part of
 * the charger's USBIN peripheral: status in TYPE_C_STATUS_1..5, role and
 * VCONN control in TYPE_C_INTRPT_ENB_SOFTWARE_CTRL, and a single
 * "type-c-change" interrupt gated by TYPE_C_INTRPT_ENB. The hardware runs
 * the attach/detach state machine itself; this backend reports its result to
 * TCPM and sets the requested role.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/string_choices.h>
#include <linux/usb/tcpm.h>
#include <linux/workqueue.h>

#include "qcom_pmic_typec.h"
#include "qcom_pmic_typec_port.h"

/* Offsets from the USBIN base (0x1300 on PMI8998) */
#define TYPE_C_STATUS_1_REG			0x0b	/* sink: Rp seen */
#define UFP_TYPEC_RDSTD				BIT(7)
#define UFP_TYPEC_RD1P5				BIT(6)
#define UFP_TYPEC_RD3P0				BIT(5)

#define TYPE_C_STATUS_2_REG			0x0c	/* source: what is attached */
#define DFP_TYPEC_MASK				GENMASK(3, 0)
#define DFP_RD_OPEN				BIT(3)
#define DFP_RD_RA_VCONN				BIT(2)
#define DFP_RD_RD				BIT(1)
#define DFP_RA_RA				BIT(0)

#define TYPE_C_STATUS_4_REG			0x0e
#define UFP_DFP_MODE_STATUS			BIT(7)	/* set: we are the source */
#define TYPEC_VBUS_STATUS			BIT(6)
#define TYPEC_VBUS_ERROR_STATUS			BIT(5)
#define TYPEC_DEBOUNCE_DONE_STATUS		BIT(4)
#define TYPEC_VCONN_OVERCURR_STATUS		BIT(2)
#define CC_ORIENTATION				BIT(1)	/* set: CC2 */
#define CC_ATTACHED				BIT(0)

#define TYPE_C_INTRPT_ENB_REG			0x67
#define TYPEC_CCOUT_DETACH_INT_EN		BIT(7)
#define TYPEC_CCOUT_ATTACH_INT_EN		BIT(6)
#define TYPEC_VBUS_ERROR_INT_EN			BIT(5)
#define TYPEC_DEBOUNCE_DONE_INT_EN		BIT(3)
#define TYPEC_CCSTATE_CHANGE_INT_EN		BIT(2)
#define TYPEC_VBUS_DEASSERT_INT_EN		BIT(1)
#define TYPEC_VBUS_ASSERT_INT_EN		BIT(0)

#define TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG	0x68
#define VCONN_EN_ORIENTATION			BIT(6)
#define VCONN_EN_SRC				BIT(4)
#define VCONN_EN_VALUE				BIT(3)
#define TYPEC_POWER_ROLE_CMD_MASK		GENMASK(2, 0)
#define UFP_EN_CMD				BIT(2)
#define DFP_EN_CMD				BIT(1)
#define TYPEC_DISABLE_CMD			BIT(0)

/*
 * Downstream's "PBS workaround" (smb-lib.c, TYPEC_PBS_WA_BIT): a test register
 * in the charger's MISC peripheral that has to read 0xa5 unless the port is
 * forced to sink. Like all registers from 0xa0 up it is write protected:
 * each write has to be preceded by an unlock of the peripheral.
 */
#define PMI8998_MISC_SEC_ACCESS			0x16d0
#define PMI8998_TM_IO_DTEST4_SEL		0x16e9

struct pmic_typec_port_pmi8998 {
	struct device		*dev;
	struct tcpm_port	*tcpm_port;
	struct regmap		*regmap;
	u32			base;
	int			irq;

	struct regulator	*vdd_vbus;
	bool			vbus_enabled;
	struct mutex		vbus_lock;	/* VBUS state serialization */

	int			cc;
	bool			debouncing_cc;
	struct delayed_work	cc_debounce_dwork;

	spinlock_t		lock;		/* Register atomicity */
};

static struct pmic_typec_port_pmi8998 *to_port(struct tcpc_dev *tcpc)
{
	return (struct pmic_typec_port_pmi8998 *)tcpc_to_tcpm(tcpc)->pmic_typec_port;
}

static int pmi8998_read(struct pmic_typec_port_pmi8998 *port, u32 reg, u32 *val)
{
	return regmap_read(port->regmap, port->base + reg, val);
}

static int pmi8998_set_role(struct pmic_typec_port_pmi8998 *port, u32 role)
{
	int ret;

	/* PBS workaround: 0 when forced to sink, 0xa5 otherwise */
	ret = regmap_write(port->regmap, PMI8998_MISC_SEC_ACCESS, 0xa5);
	if (ret)
		return ret;

	ret = regmap_write(port->regmap, PMI8998_TM_IO_DTEST4_SEL,
			   role == UFP_EN_CMD ? 0 : 0xa5);
	if (ret)
		return ret;

	return regmap_update_bits(port->regmap,
				  port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				  TYPEC_POWER_ROLE_CMD_MASK, role);
}

static void pmi8998_cc_debounce(struct work_struct *work)
{
	struct pmic_typec_port_pmi8998 *port =
		container_of(work, struct pmic_typec_port_pmi8998, cc_debounce_dwork.work);
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	port->debouncing_cc = false;
	spin_unlock_irqrestore(&port->lock, flags);

	/*
	 * The interrupt only fires on CC changes: when a partner is already
	 * attached (e.g. at boot, or after a role change that keeps the same
	 * state), TCPM would wait for one forever. Have it re-read the state.
	 */
	if (port->tcpm_port)
		tcpm_cc_change(port->tcpm_port);
}

static void pmi8998_set_cc_debounce(struct pmic_typec_port_pmi8998 *port)
{
	port->debouncing_cc = true;
	schedule_delayed_work(&port->cc_debounce_dwork, msecs_to_jiffies(20));
}

static irqreturn_t pmi8998_typec_isr(int irq, void *dev_id)
{
	struct pmic_typec_port_pmi8998 *port = dev_id;

	/* One interrupt for every Type-C event: let TCPM re-read everything */
	tcpm_vbus_change(port->tcpm_port);
	if (!port->debouncing_cc)
		tcpm_cc_change(port->tcpm_port);

	return IRQ_HANDLED;
}

static int pmi8998_get_vbus(struct tcpc_dev *tcpc)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	u32 stat4 = 0;
	int ret;

	mutex_lock(&port->vbus_lock);
	if (pmi8998_read(port, TYPE_C_STATUS_4_REG, &stat4))
		stat4 = 0;
	ret = port->vbus_enabled || !!(stat4 & TYPEC_VBUS_STATUS);
	mutex_unlock(&port->vbus_lock);

	return ret;
}

static int pmi8998_set_vbus(struct tcpc_dev *tcpc, bool on, bool sink)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	u32 stat4;
	int ret = 0;

	mutex_lock(&port->vbus_lock);
	if (port->vbus_enabled == on)
		goto done;

	ret = on ? regulator_enable(port->vdd_vbus) :
		   regulator_disable(port->vdd_vbus);
	if (ret)
		goto done;

	/* No vSafe0V/vSafe5V state here: wait for the VBUS status to follow */
	ret = regmap_read_poll_timeout(port->regmap,
				       port->base + TYPE_C_STATUS_4_REG, stat4,
				       !!(stat4 & TYPEC_VBUS_STATUS) == on,
				       100, 250000);
	if (ret)
		dev_warn(port->dev, "VBUS did not turn %s\n", str_on_off(on));
	ret = 0;

	port->vbus_enabled = on;
	tcpm_vbus_change(port->tcpm_port);
done:
	mutex_unlock(&port->vbus_lock);

	return ret;
}

static int pmi8998_get_cc(struct tcpc_dev *tcpc, enum typec_cc_status *cc1,
			  enum typec_cc_status *cc2)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	u32 stat4, val;
	int ret;

	*cc1 = TYPEC_CC_OPEN;
	*cc2 = TYPEC_CC_OPEN;

	if (port->debouncing_cc)
		return -EBUSY;

	ret = pmi8998_read(port, TYPE_C_STATUS_4_REG, &stat4);
	if (ret)
		return ret;

	if (!(stat4 & CC_ATTACHED))
		goto done;

	if (stat4 & UFP_DFP_MODE_STATUS) {
		/* We are the source: what does the partner present? */
		ret = pmi8998_read(port, TYPE_C_STATUS_2_REG, &val);
		if (ret)
			return ret;

		switch (val & DFP_TYPEC_MASK) {
		case DFP_RA_RA:
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			goto done;
		case DFP_RD_RA_VCONN:
			*cc1 = TYPEC_CC_RA;
			*cc2 = TYPEC_CC_RA;
			val = TYPEC_CC_RD;
			break;
		case DFP_RD_OPEN:
		case DFP_RD_RD:
		default:
			val = TYPEC_CC_RD;
			break;
		}
	} else {
		/* We are the sink: which Rp does the source advertise? */
		ret = pmi8998_read(port, TYPE_C_STATUS_1_REG, &val);
		if (ret)
			return ret;

		if (val & UFP_TYPEC_RD3P0)
			val = TYPEC_CC_RP_3_0;
		else if (val & UFP_TYPEC_RD1P5)
			val = TYPEC_CC_RP_1_5;
		else
			val = TYPEC_CC_RP_DEF;
	}

	if (stat4 & CC_ORIENTATION)
		*cc2 = val;
	else
		*cc1 = val;
done:
	dev_dbg(port->dev, "get_cc: stat4 %#04x cc1 %d cc2 %d\n", stat4, *cc1, *cc2);
	return 0;
}

static int pmi8998_set_cc(struct tcpc_dev *tcpc, enum typec_cc_status cc)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	unsigned long flags;
	u32 role;
	int ret;

	switch (cc) {
	case TYPEC_CC_RD:
		role = UFP_EN_CMD;
		break;
	case TYPEC_CC_OPEN:
	case TYPEC_CC_RP_DEF:
	case TYPEC_CC_RP_1_5:
	case TYPEC_CC_RP_3_0:
		/* The Rp current is left at the hardware default (80 uA) */
		role = DFP_EN_CMD;
		break;
	default:
		return -EINVAL;
	}

	spin_lock_irqsave(&port->lock, flags);
	ret = pmi8998_set_role(port, role);
	if (!ret) {
		port->cc = cc;
		pmi8998_set_cc_debounce(port);
	}
	spin_unlock_irqrestore(&port->lock, flags);

	dev_dbg(port->dev, "set_cc: %d role %#x ret %d\n", cc, role, ret);
	return ret;
}

static int pmi8998_set_polarity(struct tcpc_dev *tcpc, enum typec_cc_polarity pol)
{
	/* The QMP PHY / SBU mux get the orientation from the Type-C switch */
	return 0;
}

static int pmi8998_set_vconn(struct tcpc_dev *tcpc, bool on)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	u32 stat4, mask, value;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&port->lock, flags);
	ret = pmi8998_read(port, TYPE_C_STATUS_4_REG, &stat4);
	if (ret)
		goto done;

	/* VCONN goes on the CC line that is not used for communication */
	if (on) {
		mask = VCONN_EN_ORIENTATION | VCONN_EN_VALUE | VCONN_EN_SRC;
		value = (stat4 & CC_ORIENTATION ? 0 : VCONN_EN_ORIENTATION) |
			VCONN_EN_VALUE | VCONN_EN_SRC;
	} else {
		mask = VCONN_EN_VALUE;
		value = 0;
	}

	ret = regmap_update_bits(port->regmap,
				 port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 mask, value);
done:
	spin_unlock_irqrestore(&port->lock, flags);
	dev_dbg(port->dev, "set_vconn %s ret %d\n", str_on_off(on), ret);
	return ret;
}

static int pmi8998_start_toggling(struct tcpc_dev *tcpc,
				  enum typec_port_type port_type,
				  enum typec_cc_status cc)
{
	struct pmic_typec_port_pmi8998 *port = to_port(tcpc);
	unsigned long flags;
	u32 role;
	int ret;

	switch (port_type) {
	case TYPEC_PORT_SRC:
		role = DFP_EN_CMD;
		break;
	case TYPEC_PORT_SNK:
		role = UFP_EN_CMD;
		break;
	case TYPEC_PORT_DRP:
	default:
		role = 0;	/* dual role: the hardware toggles */
		break;
	}

	spin_lock_irqsave(&port->lock, flags);
	pmi8998_set_cc_debounce(port);
	ret = pmi8998_set_role(port, role);
	spin_unlock_irqrestore(&port->lock, flags);

	dev_dbg(port->dev, "start_toggling: type %d role %#x ret %d\n",
		port_type, role, ret);
	return ret;
}

static int pmi8998_port_start(struct pmic_typec *tcpm, struct tcpm_port *tcpm_port)
{
	struct pmic_typec_port_pmi8998 *port = (void *)tcpm->pmic_typec_port;
	int ret;

	/* Interrupt on CC state, debounce and VBUS changes */
	ret = regmap_write(port->regmap, port->base + TYPE_C_INTRPT_ENB_REG,
			   TYPEC_CCSTATE_CHANGE_INT_EN | TYPEC_DEBOUNCE_DONE_INT_EN |
			   TYPEC_VBUS_ASSERT_INT_EN | TYPEC_VBUS_DEASSERT_INT_EN |
			   TYPEC_VBUS_ERROR_INT_EN);
	if (ret)
		return ret;

	/* VCONN under software control, off */
	ret = regmap_update_bits(port->regmap,
				 port->base + TYPE_C_INTRPT_ENB_SOFTWARE_CTRL_REG,
				 VCONN_EN_SRC | VCONN_EN_VALUE, VCONN_EN_SRC);
	if (ret)
		return ret;

	port->tcpm_port = tcpm_port;
	enable_irq(port->irq);

	return 0;
}

static void pmi8998_port_stop(struct pmic_typec *tcpm)
{
	struct pmic_typec_port_pmi8998 *port = (void *)tcpm->pmic_typec_port;

	disable_irq(port->irq);
	disable_delayed_work_sync(&port->cc_debounce_dwork);
}

static int qcom_pmic_typec_port_pmi8998_probe(struct platform_device *pdev,
					      struct pmic_typec *tcpm,
					      const struct pmic_typec_port_resources *res,
					      struct regmap *regmap, u32 base)
{
	struct device *dev = &pdev->dev;
	struct pmic_typec_port_pmi8998 *port;
	struct fwnode_handle *connector;
	int ret;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	mutex_init(&port->vbus_lock);
	spin_lock_init(&port->lock);
	INIT_DELAYED_WORK(&port->cc_debounce_dwork, pmi8998_cc_debounce);

	connector = device_get_named_child_node(dev, "connector");
	if (!connector)
		return -EINVAL;

	port->vdd_vbus = devm_of_regulator_get_optional(dev, to_of_node(connector),
							"vbus");
	fwnode_handle_put(connector);
	if (port->vdd_vbus == ERR_PTR(-ENODEV))
		port->vdd_vbus = devm_regulator_get(dev, "vdd-vbus");
	if (IS_ERR(port->vdd_vbus))
		return PTR_ERR(port->vdd_vbus);

	port->dev = dev;
	port->base = base;
	port->regmap = regmap;

	port->irq = platform_get_irq_byname(pdev, "type-c-change");
	if (port->irq < 0)
		return port->irq;

	ret = devm_request_threaded_irq(dev, port->irq, NULL, pmi8998_typec_isr,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					"type-c-change", port);
	if (ret)
		return ret;

	tcpm->pmic_typec_port = (struct pmic_typec_port *)port;

	tcpm->tcpc.get_vbus = pmi8998_get_vbus;
	tcpm->tcpc.set_vbus = pmi8998_set_vbus;
	tcpm->tcpc.set_cc = pmi8998_set_cc;
	tcpm->tcpc.get_cc = pmi8998_get_cc;
	tcpm->tcpc.set_polarity = pmi8998_set_polarity;
	tcpm->tcpc.set_vconn = pmi8998_set_vconn;
	tcpm->tcpc.start_toggling = pmi8998_start_toggling;

	tcpm->port_start = pmi8998_port_start;
	tcpm->port_stop = pmi8998_port_stop;

	return 0;
}

const struct pmic_typec_port_resources pmi8998_port_res = {
	.probe = qcom_pmic_typec_port_pmi8998_probe,
};
